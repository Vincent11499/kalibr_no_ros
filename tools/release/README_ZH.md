# 构建与交付工具

本目录只保存安装前运行的构建、发布和 Docker 宿主工具，不随精简运行包安装，也不属于
`kalibr-noros tools`：

| 入口 | 用途 |
|---|---|
| `bootstrap_deps.sh` | 在仓库 `.deps/` 准备私有构建与运行依赖 |
| `install_runtime.py` | 从已配置的构建目录发布精简运行包 |
| `kalibr_docker.sh` | 构建依赖镜像、检查安装包并启动机器标定容器 |
| `docker/` | Docker 运行依赖和长期用户镜像定义 |

随安装包交付、可由用户直接运行的诊断和研究工具位于
`src/python/kalibr_tools/`，统一通过 `kalibr-noros tools --help` 查看。

正式运行包发布遵循仓库 skill `kalibr-runtime-release`：版本号由用户明确指定，默认安装
到 `/home/gs/kalibr_cedarobo/install/kalibr_cedarobo_vX.Y.Z/`；覆盖时保留现有
`config/`，并在安装根写入比较上一版本的 `README_ZH.md`。本目录脚本只负责构建依赖、
原子组装和替换运行包，不自行决定版本号或生成版本差异说明。安装根 README 采用简洁的
软件发版声明格式；完整 Git、测试和真实数据证据保留在仓库文档与发布完成报告中。
