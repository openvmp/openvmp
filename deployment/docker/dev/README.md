# OpenVMP

Use the following commands to build the container where all the necessary tools work no matter what OS/version is used on the host:

```bash
cd openvmp
docker build -t openvmp-docker-dev -f deployment/docker/dev/Dockerfile .
docker run -d -it \
  --mount "type=bind,source=$(pwd),target=/root/openvmp" \
  --name "openvmp" openvmp-docker-dev
```

Access the container from the host:

```bash
vncviewer -encodings 'copyrect tight zrle hextile' localhost:5900
```
