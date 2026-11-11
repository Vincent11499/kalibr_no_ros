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
