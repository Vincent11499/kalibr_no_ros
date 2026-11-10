# Kalibr 长期 Docker 容器简明操作

本文档适用于以下固定目录：

```text
/home/sti/kalibr_cedarobo/
├── install/kalibr_cedarobo_v1.0.0/
├── data/
└── tools/
```

## 1. 镜像、程序与工具

- `kalibr-cedarobot-runtime:ubuntu22.04` 只包含 Ubuntu、Python、NumPy、
  OpenCV、Boost、TBB 等运行依赖，不包含 Kalibr 项目代码或安装包。
- `kalibr-cedarobot-runtime:ubuntu22.04-gs` 在依赖镜像上只增加 UID/GID 1000
  的 `gs` 用户、主目录和默认工作目录，也不包含 Kalibr 安装包。
- Kalibr 程序来自主机的 `install/kalibr_cedarobo_v1.0.0`，通过绑定挂载
  出现在容器的 `/opt/cedarobot/kalibr_cedarobo_v1.0.0`。
- `docker-image/` 是离线镜像归档。执行 `docker load` 后，正在运行的容器不再读取
  该目录；保留它可以在 Docker 镜像丢失时离线恢复。
- `tools/kalibr_docker.sh` 用于构建依赖镜像、检查安装包，以及启动用完即删的临时
  标定容器。当前长期容器运行时不依赖该脚本。
- `tools/docker/Dockerfile.runtime` 用于重建依赖镜像；
  `tools/docker/Dockerfile.persistent-user` 用于生成带 `gs` 用户的镜像。

## 2. 当前长期容器配置

```text
容器名称：kalibr-cedarobot
主机名：kalibr
用户：gs（UID/GID 1000）
默认目录：/opt/cedarobot
重启策略：unless-stopped
```

挂载关系：

| 主机路径 | 容器路径 | 权限 |
|---|---|---|
| `/home/sti/kalibr_cedarobo/install/kalibr_cedarobo_v1.0.0` | `/opt/cedarobot/kalibr_cedarobo_v1.0.0` | 读写 |
| `/home/sti/kalibr_cedarobo/data` | `/data` | 读写 |

查看当前配置，不修改容器：

```bash
docker container inspect kalibr-cedarobot --format \
  '状态={{.State.Status}} 镜像={{.Config.Image}} 用户={{.Config.User}} 目录={{.Config.WorkingDir}} 重启={{.HostConfig.RestartPolicy.Name}}'

docker container inspect kalibr-cedarobot --format \
  '{{range .Mounts}}{{.Source}} -> {{.Destination}} RW={{.RW}}{{println}}{{end}}'
```

## 3. 日常使用

启动并进入容器：

```bash
docker start kalibr-cedarobot
docker exec -it kalibr-cedarobot bash
```

提示符和目录应为：

```text
gs@kalibr:/opt/cedarobot$
```

以 `EVT320-01/1102` 为例，设置数据目录并运行：

```bash
sed -i.bak \
  's#^machine_dir:.*#machine_dir: /data/EVT320-01/1102#' \
  kalibr_cedarobo_v1.0.0/config/user.yaml

kalibr_cedarobo_v1.0.0/bin/kalibr-cedarobot machine \
  --config kalibr_cedarobo_v1.0.0/config/user.yaml
```

结果同时出现在主机：

```text
/home/sti/kalibr_cedarobo/data/EVT320-01/1102/result/
```

常用管理命令：

```bash
docker ps --filter name=kalibr-cedarobot
docker stop kalibr-cedarobot
docker start kalibr-cedarobot
docker restart kalibr-cedarobot
docker logs kalibr-cedarobot
```

## 4. 不要删除当前容器

正常使用和升级安装包都不需要删除容器。不要执行：

```bash
docker rm -f kalibr-cedarobot
docker container prune
docker system prune -a
docker image rm kalibr-cedarobot-runtime:ubuntu22.04-gs
```

也不要删除主机上的 `install/` 和 `data/`。升级 Kalibr 时，在没有标定进程运行的
情况下覆盖主机的 `install/kalibr_cedarobo_v1.0.0`；容器会直接看到新内容。

## 5. 仅在容器确实不存在时创建

先检查 `gs` 用户镜像。镜像不存在时才构建：

```bash
cd /home/sti/kalibr_cedarobo

docker image inspect kalibr-cedarobot-runtime:ubuntu22.04-gs >/dev/null 2>&1 || \
docker build --pull=false \
  --file tools/docker/Dockerfile.persistent-user \
  --tag kalibr-cedarobot-runtime:ubuntu22.04-gs \
  tools/docker
```

下面的命令会保护现有容器：存在时只启动，不会删除或重建；只有不存在时才创建。

```bash
if docker container inspect kalibr-cedarobot >/dev/null 2>&1; then
  echo "kalibr-cedarobot 已存在，只启动现有容器"
  docker start kalibr-cedarobot
else
  docker run -d \
    --name kalibr-cedarobot \
    --hostname kalibr \
    --restart unless-stopped \
    --shm-size 2g \
    --env HOME=/home/gs \
    --env MPLCONFIGDIR=/tmp/matplotlib \
    --mount type=bind,src=/home/sti/kalibr_cedarobo/install/kalibr_cedarobo_v1.0.0,dst=/opt/cedarobot/kalibr_cedarobo_v1.0.0 \
    --mount type=bind,src=/home/sti/kalibr_cedarobo/data,dst=/data \
    --workdir /opt/cedarobot \
    --entrypoint /bin/bash \
    kalibr-cedarobot-runtime:ubuntu22.04-gs \
    -lc 'exec sleep infinity'
fi
```

创建或启动后验证：

```bash
docker exec kalibr-cedarobot bash -lc \
  'whoami; id; pwd; test -w /opt/cedarobot/kalibr_cedarobo_v1.0.0 && echo INSTALL_RW_OK; test -w /data && echo DATA_RW_OK'
```
