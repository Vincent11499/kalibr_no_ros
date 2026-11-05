# Kalibr Cedarobot Docker 离线安装与使用说明

本文档适用于交付目录 `kalibr-cedarobot-docker`。交付包已经包含：

- Ubuntu 22.04 amd64 Docker 运行镜像；
- 完整的 `kalibr_cedarobo_v1.0.0` 安装目录；
- Docker 运行脚本和镜像构建文件；
- 可编辑的 `config/user.yaml`；
- 文件完整性校验清单。

正常安装不需要在 Ubuntu 上安装 NumPy、OpenCV、Boost、TBB 或 Python 虚拟环境，也不需要重新构建 Docker 镜像。

## 1. Windows 交付目录结构

```text
D:\WSL\kalibr-cedarobot-docker\
├── README_ZH.md
├── UBUNTU22_QUICKSTART_ZH.md
├── MACHINE_WORKFLOW_PROTECTION_BACKLOG_ZH.md
├── BUNDLE_SHA256.txt
└── kalibr-cedarobot-docker_bundle.tar.gz
```

`kalibr-cedarobot-docker_bundle.tar.gz` 是完整离线包，内部结构为：

```text
kalibr-cedarobot-docker/
├── README_ZH.md
├── UBUNTU22_QUICKSTART_ZH.md
├── MACHINE_WORKFLOW_PROTECTION_BACKLOG_ZH.md
├── LICENSE
├── SHA256SUMS
├── config/
│   └── user.yaml
├── docker-image/
│   └── kalibr-cedarobot-runtime_ubuntu22.04_amd64.tar.gz
├── install/
│   └── kalibr_cedarobo_v1.0.0/
└── tools/
    ├── kalibr_docker.sh
    └── docker/
        └── Dockerfile.runtime
```

完整包内已经包含展开后的 `install/kalibr_cedarobo_v1.0.0`。使用 tar 包是为了保留 `install/lib` 中 Linux 动态库的权限和符号链接；不要使用 Windows 资源管理器单独解压、复制或修改 `install`。

对外传递时请复制整个 Windows 交付目录，不要只复制 Docker 镜像。

## 2. 运行环境要求

- x86_64/amd64 计算机；
- Ubuntu 22.04，或 Windows 10/11 + WSL2 Ubuntu；
- Docker Engine，或启用了 WSL2 Integration 的 Docker Desktop；
- Linux 文件系统中至少预留 4 GiB 可用空间；
- 数据集所在磁盘具有足够的结果输出空间。

检查系统和 Docker：

```bash
uname -m
docker version
docker info
```

`uname -m` 应输出 `x86_64`。如果 `docker version` 无法连接 daemon，Windows 用户应先启动 Docker Desktop，并在 Docker Desktop 的 WSL Integration 设置中启用当前 Ubuntu 发行版。

## 3. 校验并解压到 Ubuntu

不要从 Windows 的 `/mnt/c` 或 `/mnt/d` 直接运行程序。应在 WSL Ubuntu 中把完整包解压到 Linux 文件系统，数据集仍可保留在 Windows 磁盘。

假设交付包位于 Windows：

```text
D:\WSL\kalibr-cedarobot-docker
```

先校验 Windows 交付文件：

```bash
cd /mnt/d/WSL/kalibr-cedarobot-docker
sha256sum -c BUNDLE_SHA256.txt
```

应显示：

```text
kalibr-cedarobot-docker_bundle.tar.gz: OK
```

然后解压：

```bash
mkdir -p "$HOME/kalibr-cedarobot-docker"
tar -xzf \
  /mnt/d/WSL/kalibr-cedarobot-docker/kalibr-cedarobot-docker_bundle.tar.gz \
  -C "$HOME/kalibr-cedarobot-docker"
cd "$HOME/kalibr-cedarobot-docker"
chmod +x tools/kalibr_docker.sh
```

如果交付包通过 U 盘或网络传输，请把 tar 包路径替换成实际路径。不要把 tar 包解压到 Windows NTFS 目录后再复制 `install`。

## 4. 校验交付文件

进入交付目录后执行：

```bash
cd "$HOME/kalibr-cedarobot-docker"
sha256sum -c SHA256SUMS
```

所有条目都应显示 `OK`。如果出现 `FAILED`，不要继续使用该交付包，应重新复制损坏的文件。

## 5. 导入 Docker 镜像

只需在每台电脑上导入一次：

```bash
cd "$HOME/kalibr-cedarobot-docker"
docker load -i \
  docker-image/kalibr-cedarobot-runtime_ubuntu22.04_amd64.tar.gz
docker image inspect kalibr-cedarobot-runtime:ubuntu22.04 \
  --format '{{.Id}}'
```

镜像名称应为：

```text
kalibr-cedarobot-runtime:ubuntu22.04
```

不需要执行 `docker build`。`tools/docker/Dockerfile.runtime` 仅用于以后确实需要更新系统依赖时重新构建镜像。

## 6. 检查外挂安装包

执行：

```bash
cd "$HOME/kalibr-cedarobot-docker"

INSTALL_DIR="$PWD/install/kalibr_cedarobo_v1.0.0"

tools/kalibr_docker.sh check "$INSTALL_DIR"
```

成功时最后会输出：

```text
Kalibr Docker runtime check passed.
```

该检查会完成以下工作：

- 以只读方式将 `install` 挂入临时容器；
- 检查安装包中所有 ELF 文件是否缺少动态库；
- 导入 NumPy、OpenCV、SciPy、Matplotlib 和 PyYAML；
- 检查 `kalibr-noros` 与 `kalibr-cedarobot` 能否启动。

## 7. 准备机器批次目录

批次目录必须位于机器目录的下一层：

```text
<数据根目录>/<机器型号-编号>/<批次>/
```

例如：

```text
/mnt/d/machine_data/EVT312-02/1104-docker/
```

机器目录名必须为 `<machine_version>-<machine_num>`，例如 `EVT312-02`。批次名可使用字母、数字、下划线和连字符。

批次中按实际任务放置一个或多个数据集：

```text
1104-docker/
├── stereo_YYMMDD_HHMM/
├── stereo_imu_YYMMDD_HHMM/
├── stereo_base_link_YYMMDD_HHMM/
├── usb_YYMMDD_HHMM/
└── usb_base_link_YYMMDD_HHMM/
```

不要求五类数据集全部存在。每个目录数据集必须包含项目规定的 `dataset.yaml`、时间戳/IMU CSV 和图像文件。

当前交付是算法测试版本，允许软件版本、主机/Docker路径、模式、profile、BaseLink配置和
同名数据变化后在原批次重算。生产防护恢复计划见
`MACHINE_WORKFLOW_PROTECTION_BACKLOG_ZH.md`。仍建议独立试验使用独立批次，便于结果对比。

重要要求：

- 新建批次时不要复制其他批次的 `result/.kalibr-cedarobot` 状态；
- 标定过程中不得修改、重命名或重新保存采集文件；
- 查看图像时使用不会写入元数据、缩略图或 sidecar 文件的只读查看方式；
- 不要手工修改 workflow 状态绕过仍保留的硬安全检查。

机器名和批次名一致时，当前测试版本允许旧批次在原生 Ubuntu 与容器稳定路径之间迁移；
切换后会按当前 provenance 判断并重算受影响阶段。

## 8. 配置 `user.yaml`

交付包提供独立的可编辑配置：

```text
config/user.yaml
```

根据机器任务检查其中的：

- `targets`；
- `camera.mode`；
- `camera_imu.mode`；
- `camera_imu.recompute_extrinsics`；
- BaseLink 相机与平移参数。

运行脚本会在容器临时目录中把 `machine_dir` 自动改成稳定的容器路径，因此不需要为每台电脑手工修改配置中的主机绝对路径。原始 `config/user.yaml` 不会被容器修改。

## 9. 运行机器标定

先设置三个绝对路径：

```bash
cd "$HOME/kalibr-cedarobot-docker"

INSTALL_DIR="$PWD/install/kalibr_cedarobo_v1.0.0"
USER_CONFIG="$PWD/config/user.yaml"
BATCH_DIR="/mnt/d/machine_data/EVT312-02/1104-docker"
```

确认批次路径：

```bash
test -d "$BATCH_DIR"
find "$BATCH_DIR" -maxdepth 1 -mindepth 1 -type d -printf '%f\n'
```

开始运行：

```bash
tools/kalibr_docker.sh machine \
  "$INSTALL_DIR" \
  "$USER_CONFIG" \
  "$BATCH_DIR"
```

运行时的挂载关系：

| 主机内容 | 容器路径 | 权限 |
|---|---|---|
| `install/kalibr_cedarobo_v1.0.0` | `/opt/kalibr` | 只读 |
| `config/user.yaml` | `/config/source-user.yaml` | 只读 |
| 整个机器批次目录 | `/data/<机器>/<批次>` | 读写 |

容器使用当前 Ubuntu 用户的 UID/GID 写文件。容器退出后自动删除，但输入数据和结果都保留在主机批次目录。

## 10. 查看结果与继续运行

结果位于：

```text
<BATCH_DIR>/result/
```

查看：

```bash
find "$BATCH_DIR/result" -maxdepth 3 -type f -printf '%p\n' | sort
```

如果进程被正常中断，可使用完全相同的 `INSTALL_DIR`、`USER_CONFIG` 和 `BATCH_DIR` 命令继续。继续前不要修改输入数据、配置、安装包或已有受管结果。

## 11. 升级 Kalibr 程序

Docker 镜像只包含运行依赖，Kalibr 程序始终来自主机外挂的 `install` 目录。因此，依赖 ABI 没有变化时，升级程序不需要重新构建或导入 Docker 镜像。

升级步骤：

1. 确认没有标定任务正在运行；
2. 备份交付包顶层的 `config/user.yaml`；
3. 用发布方提供的新内容覆盖 `install/kalibr_cedarobo_v1.0.0`，按发布要求保留其现有 `config/`；
4. 重新执行：

```bash
tools/kalibr_docker.sh check \
  "$HOME/kalibr-cedarobot-docker/install/kalibr_cedarobo_v1.0.0"
```

只有当 `check` 报告新的缺失动态库、Python ABI 或系统依赖不兼容时，才需要发布方提供更新后的 Docker 镜像。

## 12. 常见问题

### 12.1 `runtime image not found`

尚未导入镜像，重新执行：

```bash
docker load -i \
  docker-image/kalibr-cedarobot-runtime_ubuntu22.04_amd64.tar.gz
```

### 12.2 `permission denied: tools/kalibr_docker.sh`

执行：

```bash
chmod +x tools/kalibr_docker.sh
```

### 12.3 `Cannot connect to the Docker daemon`

启动 Docker Engine/Docker Desktop。WSL 用户还要确认 Docker Desktop 已启用当前 Ubuntu 发行版的 WSL Integration。

### 12.4 出现 NumPy 2.x 或 `_ARRAY_API not found`

这通常表示误用了主机 Python。不要在主机上直接运行 `install/.../bin/kalibr-*`，应先确认镜像已加载，再通过 `tools/kalibr_docker.sh` 运行。

### 12.5 `dataset changed during calibration`

求解期间输入文件发生了变化。停止会写文件的图像查看器或同步软件，并使用新的批次目录重新采集或复制确认不再变化的数据。

### 12.6 `workflow provenance changed`

当前测试包已关闭该严格阻断。出现此错误通常表示仍在运行旧安装包；重新执行 `check`，
并确认外挂 `install` 来自当前离线包。不要手工编辑或删除部分 workflow 状态。

### 12.7 优化不收敛

Docker 只提供稳定的运行依赖，不会改变求解算法、迭代次数或收敛条件。应继续检查采集运动、角点覆盖、同步、Rolling Shutter 模式及初始化参数。

## 13. 每台电脑的最短操作清单

```bash
cd /mnt/d/WSL/kalibr-cedarobot-docker
sha256sum -c BUNDLE_SHA256.txt
mkdir -p "$HOME/kalibr-cedarobot-docker"
tar -xzf kalibr-cedarobot-docker_bundle.tar.gz \
  -C "$HOME/kalibr-cedarobot-docker"
cd "$HOME/kalibr-cedarobot-docker"
sha256sum -c SHA256SUMS
docker load -i docker-image/kalibr-cedarobot-runtime_ubuntu22.04_amd64.tar.gz
tools/kalibr_docker.sh check "$PWD/install/kalibr_cedarobo_v1.0.0"
tools/kalibr_docker.sh machine \
  "$PWD/install/kalibr_cedarobo_v1.0.0" \
  "$PWD/config/user.yaml" \
  "/绝对路径/机器型号-编号/新批次"
```
