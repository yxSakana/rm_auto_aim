FROM osrf/ros:humble-desktop

ARG https_proxy="127.0.0.1:7890"
ARG HTTPS_PROXY="127.0.0.1:7890"
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

WORKDIR /rm_auto_aim

# init
RUN apt-get update && \
    apt-get instal wget curl git cmake ninja-build kmod ros-humble-foxglove-bridge -y

# apt-get install gpg-agent -y && \
# wget https://repositories.intel.com/gpu/ubuntu/dists/jammy/lts/2350/intel-gpu-ubuntu-jammy-2350.run && \
# chmod +x intel-gpu-ubuntu-jammy-2350.run && \
# ./intel-gpu-ubuntu-jammy-2350.run && \
# chmod +x scripts/submodule_update_with_gitee.sh && \
# ./scripts/submodule_update_with_gitee.sh && \

# Install OpenVINO
RUN cd /tmp && \
    git clone https://github.com/openvinotoolkit/openvino.git --depth=1 && cd openvino && \
    git submodule update --init --recursive && \
    chmod +x install_build_dependencies.sh && ./install_build_dependencies.sh && \
    cmake -B build -G "Ninja" && cmake --build build -j$(nproc) && cmake --build build --target install -j$(nproc) && \
    rm -Rf /tmp/openvino && cd /rm_auto_aim

# From GPU
RUN mkdir /tmp/gpu_deps && cd /tmp/gpu_deps && \
    curl -L -O https://github.com/intel/compute-runtime/releases/download/23.05.25593.11/libigdgmm12_22.3.0_amd64.deb && \
    curl -L -O https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13700.14/intel-igc-core_1.0.13700.14_amd64.deb && \
    curl -L -O https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.13700.14/intel-igc-opencl_1.0.13700.14_amd64.deb && \
    curl -L -O https://github.com/intel/compute-runtime/releases/download/23.13.26032.30/intel-opencl-icd_23.13.26032.30_amd64.deb && \
    curl -L -O https://github.com/intel/compute-runtime/releases/download/23.13.26032.30/libigdgmm12_22.3.0_amd64.deb && \
    dpkg -i ./*.deb && rm -Rf /tmp/gpu_deps

# Clone repository
RUN cd /rm_auto_aim && git clone https://github.com/yxSakana/rm_auto_aim.git . --depth=1 && \
    apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/humble/setup.sh && . /usr/local/setupvars.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    sed --in-place --expression \
        '$isource "/rm_auto_aim/install/setup.bash"' \
        /ros_entrypoint.sh && \
    rm -Rf /var/lib/apt/lists/*
