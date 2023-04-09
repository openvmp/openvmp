# OpenVMP

Use the following commands to build the container where all the necessary tools work no matter what OS/version is used on the host:

```bash
cd openvmp
docker build -t openvmp-docker-dev -f deployment/docker/dev/Dockerfile .
docker run -d -it -p 5900:5900/tcp \
  --mount "type=bind,source=$(pwd),target=/root/openvmp" \
  --name "openvmp" openvmp-docker-dev
```

Graphical UI access to the container from the host:

```bash
vncviewer localhost:5900
```

Terminal access to the container from the host:

```bash
docker attach openvmp
```