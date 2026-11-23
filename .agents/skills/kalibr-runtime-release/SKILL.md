---
name: kalibr-runtime-release
description: Build, validate, and publish this repository's versioned kalibr_cedarobo runtime while preserving installed configuration and writing a version comparison README. Use for formal runtime releases or overwriting an installed release; do not use for Docker images, Git tags, or ordinary development builds.
---

# Kalibr 运行包发布

发布目标是一个可复现、可回滚检查的 Release 运行包。先阅读仓库根 `AGENTS.md`、
`README.md`、`tools/release/README_ZH.md` 和目标版本更新说明。

## 发布授权与版本

- 只有用户明确要求发布或覆盖发布时才修改安装目录。仅咨询、评审或准备说明不构成发布授权。
- 版本号必须由用户明确给出，格式为 `X.Y.Z`；不得从源码、现有目录或上次发布自行决定。
- 默认安装根为 `/home/gs/kalibr_cedarobo/install/`，唯一目标目录为
  `kalibr_cedarobo_vX.Y.Z`。除非用户明确指定其他安装根，不得创建、改用或顺带更新
  其他版本目录。
- 如果发布请求未给版本号，停止并询问版本；不得默认覆盖“最新版本”。
- 发布不授权提交、推送、创建 tag 或发布 Docker 镜像。

## 上一版本与安装根说明

发布前确定比较基线。优先使用用户明确指定的上一版本；否则使用默认安装根下版本号低于
目标版本的最高 SemVer 目录。没有候选、候选含义不明确或无法取得可信差异证据时，先询问
用户，不能猜测。

仓库可维护详细的 `docs/RELEASE_NOTES_VX.Y.Z_ZH.md`，记录完整差异和验证证据；目标安装
根的 `README_ZH.md` 只写面向使用者的简短发版声明，不得原样复制长篇内部说明。内容
来自可信 Git 差异、版本化文档和实际检查结果，采用以下结构：

```markdown
# Kalibr Cedarobot vX.Y.Z

发布日期：YYYY-MM-DD
上一版本：vA.B.C

## 本版更新

- 3～8 条用户可感知的主要变化。

## 兼容性与升级

- 简述配置、schema、缓存和结果是否兼容，以及用户是否需要操作。
- 用一句话说明标定算法、默认值、数据选择和坐标语义是否改变。

## 使用

- 给出主要启动命令和详细文档入口。

## 验证

- 只写构建、检查和关键冒烟结果的简短结论。

## 已知问题

- 仅在确有用户需要知道的问题时保留本节。
```

不要在安装根 README 中罗列源码 commit、工作树状态、绝对本机路径、修改文件清单、
内部实现细节、完整测试日志、详细用例数量或真实数据逐项记录；这些内容留在仓库详细
版本说明和发布完成报告中。没有兼容性操作或已知问题时应直接写“无需迁移”或省略空节，
避免模板化废话。

覆盖同一版本时也要重新生成或核对 README，不能沿用与当前源码不一致的旧说明。README
必须是目标根内的普通文件，不得是符号链接；写入失败则发布未完成，不能报告成功。

## 发布前检查

1. 运行 `git status --short`，保留并报告既有修改；工作树不要求干净，但不得遗漏当前
   构建来自未提交源码这一事实。
2. 运行 `git diff --check`。
3. 如果目标已存在，确认目标不是符号链接；递归记录 `config/` 的相对路径和 SHA-256，
   并拒绝配置中的符号链接。
4. 正式发布使用以下两个既有 preset，所有构建和检查最多 4 个并行 job：

   ```bash
   cmake --preset project-profile
   cmake --build --preset project-profile --parallel 4
   cmake --build --preset project-profile --target check --parallel 4
   cmake --preset release
   cmake --build --preset release --parallel 4
   ```

   修改过 Python 时，两个 `cmake --preset ...` 都必须重新执行；不能用
   `ninja: no work to do` 代替 configure。`release` 必须关闭 profiling，且不安装测试。

检查或构建失败立即停止，不覆盖既有安装。

## 覆盖与配置保护

使用仓库正式发布器：

```bash
python3 tools/release/install_runtime.py \
  --build-dir build/release \
  --prefix /home/gs/kalibr_cedarobo/install/kalibr_cedarobo_vX.Y.Z \
  --replace
```

- 已存在目标必须通过 `--replace` 原子替换，不能手工删除后复制。
- 覆盖时完整保留原目标 `config/` 的文件集合与内容；只有用户明确点名要求更新的配置
  才能变更，而且只能变更指定文件。`--user-config` 只用于用户明确要求替换
  `config/user.yaml` 的情况。
- 新版本目录不存在时使用构建提供的默认 `config/`。
- 不使用开发安装 `cmake --install build/release` 代替精简运行包发布。
- 安装包只应包含 `bin/`、`lib/`、`libexec/`、`config/`、`LICENSE`、`licenses/` 和
  根 `README_ZH.md`。发布工具目录、构建树和测试不能进入安装包。
- 版本说明应紧接运行包发布写入；使用同目录临时文件和原子替换，避免留下半写 README。

## 发布后验收

- 目标目录名与用户指定版本完全一致；两个 CLI 的 `--version` 都报告该版本。
- `kalibr-cedarobot machine --help`、`kalibr-noros tools --help` 和本次新增/变更入口可启动。
- 安装 `_build_info.py` 显示 `BUILD_TYPE=Release`、`PROFILING_ENABLED=False`。
- 既有目标的 `config/` 相对路径和 SHA-256 与发布前完全一致，除非用户明确授权了指定
  配置变更；其他配置不能变化。
- `README_ZH.md` 是简短的普通软件发版声明，准确写明当前版本、上一版本、主要变化、
  兼容性和验证结论，不包含内部审计明细。
- 安装根没有对应版本的 staging/backup 残留，`git diff --check` 仍通过。

完成时报告目标路径、版本、配置是否原样保留、README 比较基线、构建/检查结果、冒烟
结果、真实数据是否执行及原因、未提交源码状态。没有用户明确要求时，不提交、不推送、
不创建 tag。
