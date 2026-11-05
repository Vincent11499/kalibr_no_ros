# Kalibr Cedarobot：Ubuntu 22.04 简化操作说明

适用环境：x86_64 Ubuntu 22.04，已经安装 Docker，并且执行 `docker ps` 不需要 `sudo`。

## 1. 校验并解压

将收到的 `kalibr-cedarobot-docker` 文件夹复制到 Ubuntu，然后执行：

```bash
PACKAGE_DIR="/实际路径/kalibr-cedarobot-docker"
DEST="$HOME/kalibr-cedarobot-docker"

cd "$PACKAGE_DIR"
sha256sum -c BUNDLE_SHA256.txt

mkdir -p "$DEST"
tar -xzf kalibr-cedarobot-docker_bundle.tar.gz -C "$DEST"
```

校验结果必须显示：

```text
kalibr-cedarobot-docker_bundle.tar.gz: OK
```

## 2. 导入 Docker 镜像

```bash
cd "$DEST"

docker load -i \
  docker-image/kalibr-cedarobot-runtime_ubuntu22.04_amd64.tar.gz
```

每台电脑只需导入一次。

## 3. 检查 Kalibr

```bash
chmod +x tools/kalibr_docker.sh

tools/kalibr_docker.sh check \
  "$PWD/install/kalibr_cedarobo_v1.0.0"
```

成功时最后显示：

```text
Kalibr Docker runtime check passed.
```

## 4. 准备批次

使用新的批次目录，例如：

```text
/data/machine_data/EVT312-02/1104-docker/
└── stereo_261104_1000/
```

要求：

- 机器目录名格式为 `型号-编号`，例如 `EVT312-02`；
- 当前测试版本允许软件版本、Docker路径、模式、profile和同名数据变化后在原批次重算；
- 新建批次时不要复制其他批次的 `result/.kalibr-cedarobot`；
- 标定过程中不要修改、重命名或重新保存采集文件；
- 如有需要，运行前编辑 `$DEST/config/user.yaml` 中的模式、标定板和 BaseLink 参数；
- `machine_dir` 不需要手工修改，运行脚本会自动设置。

## 5. 开始标定

```bash
cd "$DEST"

INSTALL_DIR="$PWD/install/kalibr_cedarobo_v1.0.0"
USER_CONFIG="$PWD/config/user.yaml"
BATCH_DIR="/data/machine_data/EVT312-02/1104-docker"

tools/kalibr_docker.sh machine \
  "$INSTALL_DIR" \
  "$USER_CONFIG" \
  "$BATCH_DIR"
```

结果保存在：

```text
<BATCH_DIR>/result/
```

以后只升级 Kalibr 程序时，替换 `install/kalibr_cedarobo_v1.0.0`，然后重新执行第 3 步；依赖没有变化时不需要重新制作 Docker 镜像。

当前测试版本不应再报告通用 `workflow provenance changed`；若出现，通常说明正在运行旧
`install`。`dataset changed during calibration` 仍会保留，因为求解期间变化的数据不能
生成有效结果。不要手工修改 workflow 状态绕过检查。
