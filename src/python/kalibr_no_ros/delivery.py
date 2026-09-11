"""Named calibration deliveries; private solver files never pollute a shared output directory."""

from pathlib import Path
import json
import os
import re
import shutil
import tempfile
from .rolling_shutter import CAMERA_IMU_JOBS


def _ids(task):
    from .task import load_yaml, resolve_task_path
    if task['job'] == 'camera_calibration':
        return [c.get('id', 'cam{}'.format(i)) for i, c in enumerate(task['cameras'])]
    value = task['camera_calibration']
    path = value.get('path') if isinstance(value, dict) else value
    return [c['id'] for c in load_yaml(resolve_task_path(task, path))['cameras']]


def result_name(task):
    from .task import TaskError
    name = task['output'].get('name')
    if not name:
        ids = _ids(task)
        if task['job'] in CAMERA_IMU_JOBS:
            ids += [c.get('id', 'imu{}'.format(i)) for i, c in enumerate(task['imus'])]
        name = task['job'] + '_' + '_'.join(ids)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,159}', name):
        raise TaskError('result name is not a safe filename; set output.name explicitly')
    return name


def discover_camera_result(task, output):
    """Never guess between multiple models/runs or silently replace an explicit input."""
    from .task import TaskError, load_yaml, resolve_task_path
    if task['job'] not in CAMERA_IMU_JOBS or task.get('camera_calibration'):
        return
    expected = None
    if task['dataset']['type'] == 'directory':
        from kalibr_bag_io.directory import DirectoryReader
        reader = DirectoryReader(resolve_task_path(task, task['dataset']['path']))
        expected = [c['id'] for c in reader.manifest.get('cameras', [])]
    matches = []
    for path in sorted(Path(output).glob('*.yaml')):
        if path.is_symlink() or not path.is_file():
            continue
        try:
            value = load_yaml(path)
        except (OSError, ValueError):
            continue
        if value.get('schema_version') != '1.0.0' or value.get('kind') != 'calibration_result' or value.get('calibration_type') != 'cameras':
            continue
        cameras = value.get('cameras', [])
        if not isinstance(cameras, list) or len(cameras) < 2 or not all(isinstance(c, dict) for c in cameras):
            continue
        ids = [c.get('id') for c in cameras]
        if any(not isinstance(i, str) for i in ids) or len(set(ids)) != len(ids):
            continue
        if expected is not None and set(ids) != set(expected):
            continue
        try:
            from .validation import load_cameras
            candidate_task = dict(task, camera_calibration={'path': str(path.resolve())})
            load_cameras(candidate_task)
        except (ValueError, OSError, KeyError, TypeError):
            continue
        matches.append(path.resolve())
    if len(matches) != 1:
        reason = 'none found' if not matches else ', '.join(p.name for p in matches)
        raise TaskError('camera_calibration.path omitted: expected one matching stereo/multicamera result in output directory; {}. Specify camera_calibration.path explicitly.'.format(reason))
    task['camera_calibration'] = {'path': str(matches[0])}


def apply_camera_ids(task, artifacts, result):
    """Keep the solver's list order while restoring public sensor identities."""
    ids = _ids(task)
    frame_ids = {}
    for i, (camera, public) in enumerate(zip(artifacts['cameras'], result['cameras'])):
        camera['id'] = public['id'] = ids[i]
        for frame in camera.get('frames', []):
            previous = frame['frame_id']
            current = ids[i] + ':' + previous.split(':', 1)[1]
            frame_ids[previous] = current
            frame['frame_id'] = current
        public.pop('from_camera', None)
    for view in artifacts.get('views', []):
        view['frame_ids'] = [frame_ids.get(v, v) for v in view.get('frame_ids', [])]
    for event in artifacts.get('events', []):
        if 'frame_id' in event:
            event['frame_id'] = frame_ids.get(event['frame_id'], event['frame_id'])


def enrich_result(result, metrics):
    result.pop('transform_convention', None)
    for i, camera in enumerate(result['cameras']):
        camera.pop('from_camera', None)
        values = metrics['cameras'][camera['id']]['reprojection']
        camera['rms'] = values.get('rms_px')
        if i and 'T_cn_cnm1' in camera:
            previous = result['cameras'][i-1]['id']
            pair = metrics.get('stereo_pairs', {}).get(previous + '_' + camera['id'], {})
            camera['alignment'] = (pair.get('alignment') or {}).get('rms_px')


def locate_result(source):
    from .task import TaskError, load_yaml
    source = Path(source).resolve()
    if source.is_file():
        return source
    legacy = source / 'calibration.yaml'
    if legacy.is_file():
        return legacy
    candidates = []
    for p in sorted(source.glob('*.yaml')):
        try:
            if load_yaml(p).get('kind') == 'calibration_result':
                candidates.append(p)
        except (ValueError, OSError):
            continue
    if len(candidates) != 1:
        raise TaskError('select an explicit result YAML; directory contains {} result candidates'.format(len(candidates)))
    return candidates[0]


def _destinations(stage, output, name, options, failed=False):
    files = {}
    main = {'calibration.yaml': name + '.yaml', 'report.html': name + '.report.html',
            'report.pdf': name + '.report.pdf'}
    if options.get('export_text'):
        main['results.txt'] = name + '.results.txt'
    for source, dest in main.items():
        if not failed and (stage / source).is_file():
            files[source] = dest
    if failed and options.get('save_diagnostics') and (stage / 'calibration.yaml').is_file():
        from .task import load_yaml
        try:
            # Preserve a finite solve after report failure, clearly inside failed diagnostics.
            json.dumps(load_yaml(stage / 'calibration.yaml'), allow_nan=False)
            files['calibration.yaml'] = name + '/result.yaml'
        except (ValueError, OSError):
            pass
    for p in stage.rglob('*'):
        if not p.is_file():
            continue
        relative = p.relative_to(stage).as_posix()
        if relative in main:
            continue
        retain = p.parts[len(stage.parts)] in {'observations', 'visualizations', 'images', 'opencv'}
        retain |= relative == 'timing.json'
        retain |= options.get('export_poses', False) and relative == 'poses.csv'
        retain |= options.get('save_metrics', False) and relative == 'metrics.json'
        retain |= options.get('save_diagnostics', False)
        if retain and relative != 'calibration.yaml':
            files[relative] = name + '/' + relative
    return files


def _check_owned(output, name, force):
    """Inspect only this delivery; other tasks and arbitrary sibling files are untouched."""
    from .task import TaskError
    main = [output / (name + suffix) for suffix in ('.yaml', '.report.html', '.report.pdf', '.results.txt')]
    side = output / name
    existing = [p for p in main + [side] if p.exists() or p.is_symlink()]
    if existing and not force:
        raise TaskError('output for {} already exists; choose output.name or use --force'.format(name))
    for p in existing:
        if p.is_symlink():
            raise TaskError('output must not contain symlinks: {}'.format(p))
        if p != side and not p.is_file():
            raise TaskError('expected output file: {}'.format(p))
    if side.exists():
        inventory = side / '.inventory.json'
        if not inventory.is_file() or inventory.is_symlink():
            raise TaskError('cannot overwrite unregistered output directory: {}'.format(side))
        registered = json.loads(inventory.read_text())
        actual = {p.relative_to(side).as_posix() for p in side.rglob('*')}
        if any(p.is_symlink() for p in side.rglob('*')) or actual != set(registered) | {'.inventory.json'}:
            raise TaskError('output directory contains unregistered or missing entries: {}'.format(side))
    return existing


def published_paths(stage, output, name, options):
    """List only artifacts that survived delivery selection and renaming."""
    return sorted(relative for relative in _destinations(stage, output, name, options).values()
                  if (Path(output) / relative).is_file())


def publish(stage, output, name, options, force=False, failed=False):
    """All copies and link rewrites complete before old owned files are replaced."""
    from .task import dump_yaml, load_yaml
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    existing = _check_owned(output, name, force)
    mapping = _destinations(stage, output, name, options, failed)
    with tempfile.TemporaryDirectory(prefix='.delivery-', dir=output) as folder:
        prepared = Path(folder)
        for source, relative in mapping.items():
            target = prepared / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if source == 'report.html':
                text = (stage / source).read_text()
                for old, new in mapping.items():
                    text = text.replace('href="' + old + '"', 'href="' + new + '"')
                    text = text.replace('src="' + old + '"', 'src="' + new + '"')
                # Report links list only delivered artifacts.
                import re as regex
                text = regex.sub(r'<li><a href="([^"]+)">.*?</a></li>',
                    lambda m: m.group(0) if m.group(1) in mapping.values() else '', text)
                target.write_text(text)
            else:
                shutil.copy2(stage / source, target)
        side = prepared / name
        if side.exists():
            init = side / 'initialization_report.yaml'
            if init.is_file():
                value = load_yaml(init)
                result_path = prepared / mapping.get('calibration.yaml', '__absent__')
                if value.get('result') and result_path.is_file():
                    from .task import _sha256
                    value['result'] = {'path': 'result.yaml' if failed else '../' + name + '.yaml', 'sha256': _sha256(result_path)}
                    dump_yaml(value, init)
            manifest = side / 'run_manifest.json'
            if manifest.is_file():
                # Inventory is local to the optional evidence directory.
                value = json.loads(manifest.read_text())
                value['files'] = [{'path': p.relative_to(side).as_posix(), 'size_bytes': p.stat().st_size, 'schema_version': '1.0.0'}
                                  for p in side.rglob('*') if p.is_file() and p != manifest]
                value['directories'] = [p.relative_to(side).as_posix() for p in side.rglob('*') if p.is_dir()]
                manifest.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
            entries = sorted(p.relative_to(side).as_posix() for p in side.rglob('*'))
            (side / '.inventory.json').write_text(json.dumps(entries, indent=2) + '\n')
        existing = _check_owned(output, name, force)
        for path in existing:
            if path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()
        for path in prepared.iterdir():
            os.replace(path, output / path.name)
    return output


def run_delivery(prefix, config, output_dir, expected_job, force, overrides):
    from .task import TaskError, load_task, resolve_task_path, _run_task_staged
    task = load_task(config, expected_job)
    output = Path(output_dir).expanduser().resolve()
    dataset = resolve_task_path(task, task['dataset']['path'])
    if output in {Path('/'), Path.home(), Path.cwd()} or output == dataset or dataset in output.parents or output in dataset.parents:
        raise TaskError('output directory must be separate from the input dataset and workspace root')
    discover_camera_result(task, output)
    name = result_name(task)
    _check_owned(output, name, force)
    inputs = [Path(config).resolve(), *([Path(overrides['initialization']).resolve()] if overrides.get('initialization') else [])]
    for block in [task.get('target'), task.get('camera_calibration'), task.get('initialization'), *task.get('imus', [])]:
        value = block.get('path') if isinstance(block, dict) else block
        if value:
            source = resolve_task_path(task, value)
            inputs.append(source)
    replaced = {output / (name + suffix) for suffix in ('.yaml', '.report.html', '.report.pdf', '.results.txt')}
    for source in inputs:
        if source in replaced or output / name in source.parents:
            raise TaskError('delivery would overwrite its input: {}'.format(source))
    with tempfile.TemporaryDirectory(prefix='kalibr-delivery-') as temporary:
        stage = Path(temporary) / 'run'
        run_overrides = dict(overrides)
        if run_overrides.get('timing_json'):
            run_overrides['timing_json'] = str(stage / 'timing.json')
        try:
            _run_task_staged(prefix, config, stage, expected_job, _task=task, **run_overrides)
        except Exception as original_error:
            # Failed attempts never replace a previous successful delivery.
            if task['output'].get('save_diagnostics') and stage.exists():
                from uuid import uuid4
                failed_name = name + '_failed'
                while (output / failed_name).exists() or (output / failed_name).is_symlink():
                    failed_name = name + '_failed_' + uuid4().hex[:12]
                try:
                    publish(stage, output, failed_name, task['output'], False, failed=True)
                except Exception as diagnostic_error:
                    import sys
                    print('Could not preserve failure diagnostics: {}'.format(diagnostic_error), file=sys.stderr)
            raise
        return publish(stage, output, name, task['output'], force)
