ARG ROS_DISTRO=rolling
FROM ros:$ROS_DISTRO-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/ws_ros

# Update to be your project's name
ENV PROJECT_NAME=reach

COPY . src/$PROJECT_NAME

# Install apt packages
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    git \
    sudo \
    clang \
    python3-pip \
    python3-dev \
    python3-venv \
    apt-utils \
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install GStreamer
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    gir1.2-gstreamer-1.0 \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libcairo2-dev \
    libgif-dev \
    libjpeg-dev \
    python3-pip \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Configure the ubuntu non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G dialout $USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to the ubuntu user
USER $USERNAME
ENV USER=$USERNAME

ENV USER_WORKSPACE=/home/$USERNAME/ws_ros
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/$PROJECT_NAME

# Create a new virtual environment for Python
ENV VIRTUAL_ENV=$USER_WORKSPACE/.venv/$PROJECT_NAME
RUN python3 -m venv --system-site-packages $VIRTUAL_ENV \
    && echo "source ${VIRTUAL_ENV}/bin/activate" >> /home/$USERNAME/.bashrc \
    && touch .venv/COLCON_IGNORE \
    && echo "\n# Ensure colcon is run in the venv\nalias colcon='python3 -m colcon'" >> /home/$USERNAME/.bashrc
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install all ROS dependencies
WORKDIR $USER_WORKSPACE
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && echo "source ${USER_WORKSPACE}/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

FROM base AS desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV USER_WORKSPACE=/home/$USERNAME/ws_ros
WORKDIR $USER_WORKSPACE

# Install Gazebo
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y \
    ros-$ROS_DISTRO-ros-gz \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install debugging/linting Python packages
RUN pip install \
    pre-commit

# Install debugging/linting C++ packages
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y \
    clang-format-18 \
    clang-tidy \
    clang-tools \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

FROM desktop AS desktop-nvidia

# Install NVIDIA software
RUN sudo apt-get update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"
