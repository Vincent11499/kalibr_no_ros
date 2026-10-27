"""Named calibration deliveries; private solver files never pollute a shared output directory."""

from pathlib import Path
import json
import math
import os
import re
import shutil
import tempfile
from .filesystem import file_sha256
from .rolling_shutter import CAMERA_IMU_JOBS, CAMERA_CALIBRATION_JOBS
from .jobs import CAMERA_BASELINK_JOB


def _ids(task):
    from .task import load_yaml, resolve_task_path
    if task['job'] in CAMERA_CALIBRATION_JOBS or task['job'] == CAMERA_BASELINK_JOB:
        return [c.get('id', 'cam{}'.format(i)) for i, c in enumerate(task['cameras'])]
    value = task['camera_calibration']
    path = value.get('path') if isinstance(value, dict) else value
    return [c['id'] for c in load_yaml(resolve_task_path(task, path))['cameras']]


def result_name(task, rs_solver_backend='system'):
    from .task import TaskError
    name = task['output'].get('name')
    if not name:
        ids = _ids(task)
        if task['job'] in CAMERA_IMU_JOBS:
            ids += [c.get('id', 'imu{}'.format(i)) for i, c in enumerate(task['imus'])]
        name = task['job'] + '_' + '_'.join(ids)
    if rs_solver_backend == 'native' and not name.startswith('native_'):
        name = 'native_' + name
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


def _rolling_shutter_pair_quality(pair, field, pair_id):
    """Return the public scalar/null value without inventing evidence.

    Detailed unavailability reasons remain in metrics.json and the reports; the
    calibration-result contract keeps these fields scalar (or null).
    """
    from .task import TaskError

    metric = pair.get(field)
    label = '{} for {}'.format(field, pair_id)
    if metric is None:
        return None
    if not isinstance(metric, dict):
        raise TaskError('{} metric must be a mapping'.format(label))

    status = metric.get('status')
    value = metric.get('rms_px')
    count = metric.get('count')
    if status == 'available':
        if (type(count) is not int or count <= 0
                or isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0.0):
            raise TaskError(
                '{} marked available requires a positive sample count and a '
                'nonnegative finite rms_px'.format(label))
        return float(value)
    if status != 'unavailable':
        raise TaskError('{} status must be available or unavailable'.format(label))
    if type(count) is not int or count != 0 or value is not None:
        raise TaskError(
            '{} marked unavailable requires count 0 and rms_px null'.format(
                label))
    return None


def enrich_result(result, metrics):
    from .task import TaskError

    result.pop('transform_convention', None)
    for i, camera in enumerate(result['cameras']):
        camera.pop('from_camera', None)
        values = metrics['cameras'][camera['id']]['reprojection']
        rms = values.get('rms_px')
        if camera.get('shutter') and rms is None:
            raise TaskError(
                'rolling-shutter result requires final pixel residuals for {}'.format(
                    camera['id']))
        camera['rms'] = rms
        if i and 'T_cn_cnm1' in camera:
            previous = result['cameras'][i-1]['id']
            pair_id = previous + '_' + camera['id']
            pair = metrics.get('stereo_pairs', {}).get(pair_id, {})
            if camera.get('shutter'):
                camera['alignment'] = _rolling_shutter_pair_quality(
                    pair, 'alignment', pair_id)
                camera['rs_compensated_pair_residual'] = (
                    _rolling_shutter_pair_quality(
                        pair, 'rs_compensated_pair_residual', pair_id))
            else:
                camera['alignment'] = (
                    (pair.get('alignment') or {}).get('rms_px'))


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


def _inspect_sidecar(side):
    from .task import TaskError

    if not side.is_dir() or side.is_symlink():
        raise TaskError('completed delivery sidecar is missing: {}'.format(side))
    inventory = side / '.inventory.json'
    if not inventory.is_file() or inventory.is_symlink():
        raise TaskError('managed delivery inventory is missing: {}'.format(inventory))
    try:
        registered = json.loads(inventory.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise TaskError(
            'invalid managed delivery inventory: {}'.format(error)) from error
    if (not isinstance(registered, list)
            or any(not isinstance(value, str) for value in registered)
            or len(set(registered)) != len(registered)):
        raise TaskError('managed delivery inventory must be a list of paths')
    descendants = list(side.rglob('*'))
    if any(path.is_symlink() for path in descendants):
        raise TaskError(
            'managed delivery must not contain symlinks: {}'.format(side))
    actual = {
        path.relative_to(side).as_posix()
        for path in descendants if path != inventory
    }
    if actual != set(registered):
        raise TaskError(
            'managed delivery inventory does not match: {}'.format(side))
    return inventory, descendants


def _read_completed_manifest(manifest_path):
    from .task import TaskError

    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise TaskError(
            'invalid delivery run_manifest.json: {}'.format(error)) from error
    if not isinstance(manifest, dict) or manifest.get('status') != 'completed':
        raise TaskError('delivery run manifest is not completed')
    records = (
        manifest.get('files'),
        manifest.get('directories'),
        manifest.get('delivery_files'),
    )
    if any(not isinstance(value, list) for value in records):
        raise TaskError('delivery run manifest lacks its file inventory')
    return manifest, records


def _validate_file_records(records, expected, *, published=False):
    from .task import TaskError

    invalid = (
        'delivery run manifest has an invalid published file record'
        if published else 'delivery run manifest has an invalid file record'
    )
    mismatch = (
        'published delivery files do not match the run manifest'
        if published else 'delivery files do not match the run manifest'
    )
    recorded = {}
    for record in records:
        path = record.get('path') if isinstance(record, dict) else None
        if (not isinstance(record, dict)
                or not isinstance(path, str)
                or published and Path(path).name != path
                or type(record.get('size_bytes')) is not int
                or record.get('size_bytes') < 0
                or not isinstance(record.get('sha256'), str)
                or not re.fullmatch(r'[0-9a-f]{64}', record['sha256'])
                or path in recorded):
            raise TaskError(invalid)
        recorded[path] = record
    if set(recorded) != set(expected) or any(
            expected[path].stat().st_size != record['size_bytes']
            or file_sha256(expected[path]) != record['sha256']
            for path, record in recorded.items()):
        raise TaskError(mismatch)


def inspect_managed_delivery(output, name):
    """Read-only integrity check for one successfully published delivery.

    This does not decide whether the calibration is suitable for a caller.  It
    only proves that the named YAML/reports and evidence directory form one
    intact delivery whose run manifest says ``completed``.
    """
    from .task import TaskError

    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,159}', name):
        raise TaskError('managed delivery name is not a safe filename')
    output = Path(output).resolve()
    paths = {
        'result': output / (name + '.yaml'),
        'html': output / (name + '.report.html'),
        'pdf': output / (name + '.report.pdf'),
        'sidecar': output / name,
    }
    for role in ('result', 'html', 'pdf'):
        path = paths[role]
        if not path.is_file() or path.is_symlink():
            raise TaskError(
                'completed delivery is missing {}: {}'.format(role, path))
    side = paths['sidecar']
    inventory, descendants = _inspect_sidecar(side)

    manifest_path = side / 'run_manifest.json'
    task_path = side / 'task_resolved.yaml'
    if (not manifest_path.is_file() or manifest_path.is_symlink()
            or not task_path.is_file() or task_path.is_symlink()):
        raise TaskError('completed delivery lacks workflow provenance')
    manifest, records = _read_completed_manifest(manifest_path)
    file_records, directory_records, delivery_records = records
    expected_files = {
        path.relative_to(side).as_posix(): path
        for path in descendants
        if path.is_file() and path not in {inventory, manifest_path}
    }
    _validate_file_records(file_records, expected_files)
    expected_directories = {
        path.relative_to(side).as_posix()
        for path in descendants if path.is_dir()
    }
    if (any(not isinstance(value, str) for value in directory_records)
            or len(set(directory_records)) != len(directory_records)
            or set(directory_records) != expected_directories):
        raise TaskError('delivery directories do not match the run manifest')

    expected_delivery = {
        paths[role].name: paths[role]
        for role in ('result', 'html', 'pdf')
    }
    optional_text = output / (name + '.results.txt')
    if optional_text.exists() or optional_text.is_symlink():
        if not optional_text.is_file() or optional_text.is_symlink():
            raise TaskError(
                'completed delivery has an invalid text result: {}'.format(
                    optional_text))
        expected_delivery[optional_text.name] = optional_text
    _validate_file_records(
        delivery_records, expected_delivery, published=True)
    return {
        **paths,
        'manifest_path': manifest_path,
        'task_resolved': task_path,
        'manifest': manifest,
    }


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
        retain |= relative == 'rotation_summary.csv'
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
                    value['result'] = {'path': 'result.yaml' if failed else '../' + name + '.yaml', 'sha256': file_sha256(result_path)}
                    dump_yaml(value, init)
            manifest = side / 'run_manifest.json'
            if manifest.is_file():
                # Inventory is local to the optional evidence directory.
                value = json.loads(manifest.read_text())
                value['files'] = [{'path': p.relative_to(side).as_posix(), 'size_bytes': p.stat().st_size, 'sha256': file_sha256(p), 'schema_version': '1.0.0'}
                                  for p in side.rglob('*') if p.is_file() and p != manifest]
                value['directories'] = [p.relative_to(side).as_posix() for p in side.rglob('*') if p.is_dir()]
                value['delivery_files'] = [
                    {
                        'path': p.name,
                        'size_bytes': p.stat().st_size,
                        'sha256': file_sha256(p),
                        'schema_version': '1.0.0',
                    }
                    for p in sorted(prepared.iterdir()) if p.is_file()
                ]
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
    rs_solver_backend = overrides.get('_rs_solver_backend', 'system')
    name = result_name(task, rs_solver_backend)
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
        except Exception:
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
        published = publish(stage, output, name, task['output'], force)
        print('[kalibr] result={}'.format(output / (name + '.yaml')))
        print('[kalibr] report_html={}'.format(
            output / (name + '.report.html')))
        print('[kalibr] report_pdf={}'.format(
            output / (name + '.report.pdf')))
        return published
