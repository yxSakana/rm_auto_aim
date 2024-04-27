## Use `Docker`

```shell
# pull dokcer images
docker pull yxsakana/rm_auto_aim:latest

# OR Use Dockerfile to build images
docker build \
    --network=host \
    --build-arg https_proxy="127.0.0.1:7890" \
    --build-arg HTTPS_PROXY="127.0.0.1:7890" \
    -t rm_auto_aim:<TAG> .

# run
docker run \
	--network host \
    --privileged \
    --device=/dev/dri:/dev/dri \
    --device=/dev/ttyACM0:/dev/ttyACM0 \
    --volume=/dev/bus/usb:/dev/bus/usb \
    --volume=/dev/ttyACM0:/dev/ttyACM0 \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --volume=./rm_auto_aim/log:/rm_auto_aim/runtime_log \
    --name rm_auto_aim \
    rm_auto_aim \
    /rm_auto_aim/src/WatchDog infantry_CS016
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

