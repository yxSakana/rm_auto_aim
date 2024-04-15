FROM osrf/ros:humble-desktop

# Create word directory
RUN mkdir -p /rm_auto_aim
WORKDIR /rm_auto_aim

# init
RUN apt-get update && \
    apt-get install vim wget curl git cmake ninja-build kmod -y && \
    touch /etc/profile.d/rm_auto_aim-path.sh && \
    echo ". /opt/ros/humble/setup.sh" >> /etc/profile.d/rm_auto_aim-path.sh

# Install OpenVINO
RUN cd /tmp && \
    git clone https://github.com/openvinotoolkit/openvino.git --depth=1 && cd openvino && \
    chmod +x scripts/submodule_update_with_gitee.sh && \
    ./scripts/submodule_update_with_gitee.sh && \
    cmake -B build -G "Ninja" && cmake --build build -j4 && cmake --build build --target install -j3 && \
    echo ". /usr/local/setupvars.sh" >> /etc/profile.d/rm_auto_aim-path.sh && \
    apt-get install gpg-agent -y && \
    wget https://repositories.intel.com/gpu/ubuntu/dists/jammy/lts/2350/intel-gpu-ubuntu-jammy-2350.run && \
    chmod +x intel-gpu-ubuntu-jammy-2350.run && \
    ./intel-gpu-ubuntu-jammy-2350.run && \
    cd /rm_auto_aim

# Clone repository
RUN git clone https://github.com/yxSakana/rm_auto_aim.git . --depth=1 && \
    cd /rm_auto_aim/src && chmod +x hik_camera/hikSDK/runtime/configure && ./hik_camera/hikSDK/runtime/configure && \
    cd /rm_auto_aim && apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /etc/profile.d/rm_auto_aim-path.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo ". /rm_auto_aim/install/setup.sh" >> /etc/profile.d/rm_auto_aim-path.sh && \
    . /etc/profile.d/rm_auto_aim-path.sh

# Cmd
CMD ["ros2", "launch", "auto_aim", "rm_auto_aim"]
