## Use `Docker` build

```shell
# pull
docker pull ros:humble-ros-base

# build
docker build \
    --network=host \
    --build-arg https_proxy="127.0.0.1:7890" \
    --build-arg HTTPS_PROXY="127.0.0.1:7890" \
    -t rm_auto_aim:infantry .

# run
docker run \
    --privileged \
    --device=/dev/dri:/dev/dri \
    --device=/dev/ttyACM0:/dev/ttyACM0 \
    --volume=/dev/bus/usb:/dev/bus/usb \
    --volume=/dev/ttyACM0:/dev/ttyACM0 \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix: \
    --name rm_auto_aim \
    rm_auto_aim \
    /rm_auto_aim/src/WatchDog infantry_CS004
```

## Configure program to run at system startup
```shell
echo -e "[Unit]
Description="2024 RoboMaster auto aim program"
Requires=docker.service
After=docker.service

[Service]
Restart=always
ExecStart=docker start rm_auto_aim
ExecStop=docker stop rm_auto_aim

[Install]
WantedBy=multi-user.target
" | sudo tee /etc/systemd/system/rm_auto_aim.service
sudo systemctl daemon-reload
sudo systemctl enable rm_auto_aim
```

