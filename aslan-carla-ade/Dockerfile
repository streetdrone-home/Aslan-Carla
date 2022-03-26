FROM ros:melodic

# Install Aslan dependencies
COPY install-deps.sh install-deps.sh
RUN apt-get update && rosdep update \
    && (yes | bash ./install-deps.sh) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

# Install Carla dependencies
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    clang-8 \
    lld-8 \
    g++-7 \
    cmake \
    ninja-build \
    libvulkan1 \
    openssh-client \
    python \
    python-pip \
    python-dev \
    python3-dev \
    python3-pip \
    libpng-dev \
    libtiff5-dev \
    libjpeg-dev \
    tzdata \
    sed \
    curl \
    unzip \
    autoconf \
    libtool \
    rsync \
    libxml2-dev \
    git \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 100
RUN update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 100

RUN pip install \
    transforms3d

RUN cd /opt && wget https://github.com/carla-simulator/scenario_runner/archive/refs/tags/v0.9.12.tar.gz && tar xzf v0.9.12.tar.gz

RUN cd /opt/scenario_runner-0.9.12 && pip3 install -r requirements.txt

# Finish setup for ADE
RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

COPY env.sh /etc/profile.d/ade_env.sh
COPY entrypoint /ade_entrypoint

ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
