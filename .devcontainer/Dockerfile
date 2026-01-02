FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set environment variable to indicate docker execution (used by ubuntu.sh logic if we were to run it, but helpful generally)
ENV RUNS_IN_DOCKER=true

# Initial dependencies and common tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    gosu \
    lsb-release \
    software-properties-common \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# PX4 General Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle \
    build-essential \
    ccache \
    cmake \
    cppcheck \
    file \
    g++ \
    gcc \
    gdb \
    git \
    lcov \
    libssl-dev \
    libxml2-dev \
    libxml2-utils \
    make \
    ninja-build \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    rsync \
    shellcheck \
    unzip \
    zip \
    && rm -rf /var/lib/apt/lists/*

# NuttX Toolchain Dependencies (x86_64)
RUN apt-get update && apt-get install -y --no-install-recommends \
    automake \
    binutils-dev \
    bison \
    build-essential \
    curl \
    clang \
    clang-tidy \
    clang-format \
    flex \
    gdb-multiarch \
    genromfs \
    gettext \
    gperf \
    kconfig-frontends \
    libelf-dev \
    libexpat-dev \
    libgmp-dev \
    libisl-dev \
    libmpc-dev \
    libmpfr-dev \
    libncurses-dev \
    libncurses6 \
    libncursesw6 \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    libtool \
    libunwind-dev \
    lldb \
    lld \
    pkg-config \
    screen \
    texinfo \
    u-boot-tools \
    util-linux \
    vim-common \
    g++-multilib \
    gcc-arm-none-eabi \
    gcc-multilib \
    && rm -rf /var/lib/apt/lists/*

# Simulation Tools Dependencies
# Set up Gazebo repository (Gazebo Harmonic for Ubuntu 22.04)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    libunwind-dev \
    bc \
    dmidecode \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libeigen3-dev \
    libgstreamer-plugins-base1.0-dev \
    libimage-exiftool-perl \
    libopencv-dev \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

# Setup a non-root user 'vscode' with sudo access
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG dialout $USERNAME

# Python Dependencies
# Note: requirements.txt was not found in the workspace, so skipping explicit pip install.
# If you have a requirements.txt, uncomment the following lines and ensure the file exists.
# COPY requirements.txt /tmp/requirements.txt
# RUN pip3 install --break-system-packages -r /tmp/requirements.txt

# Set temporary workspace
WORKDIR /workspace

USER $USERNAME
