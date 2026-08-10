FROM ros:jazzy

# Add arguments for user ID and group ID
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=robot

RUN apt-get update -y && apt-get install -y \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    python3-pip \
    python3-tk \
    openssh-client \
    ros-${ROS_DISTRO}-control-msgs \
    sudo

# Create a user with the same UID/GID as the host user. The base image may
# already ship a user/group at that UID/GID (e.g. ros:jazzy's "ubuntu"), so
# reuse and rename it instead of failing on a conflict.
RUN set -eux; \
    if getent group "$GROUP_ID" >/dev/null; then \
        EXISTING_GROUP=$(getent group "$GROUP_ID" | cut -d: -f1); \
        [ "$EXISTING_GROUP" = "$USERNAME" ] || groupmod -n "$USERNAME" "$EXISTING_GROUP"; \
    else \
        groupadd --gid "$GROUP_ID" "$USERNAME"; \
    fi; \
    if getent passwd "$USER_ID" >/dev/null; then \
        EXISTING_USER=$(getent passwd "$USER_ID" | cut -d: -f1); \
        if [ "$EXISTING_USER" = "$USERNAME" ]; then \
            usermod -g "$GROUP_ID" "$USERNAME"; \
        else \
            usermod -l "$USERNAME" -g "$GROUP_ID" -d "/home/$USERNAME" -m "$EXISTING_USER"; \
        fi; \
    else \
        useradd --uid "$USER_ID" --gid "$GROUP_ID" --create-home --shell /bin/bash "$USERNAME"; \
    fi; \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Make sure user can use sudo
RUN adduser $USERNAME sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/nopasswd && chmod 0440 /etc/sudoers.d/nopasswd

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

COPY . /home/robot/ros2_ws

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --from-paths micro_drive --ignore-src -r -y"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev --packages-select micro_drive"

# Set up ROS environment for the new user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash && [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc