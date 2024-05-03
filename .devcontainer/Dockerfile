FROM osrf/ros:humble-desktop

# Install custom packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    sudo \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # Add sudo support [Optional]
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set the default user
USER $USERNAME
ENV HOME /home/$USERNAME
# WORKDIR $HOME

# Setup auto-source of workspace for ros user
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> $HOME/.bashrc

# Set up virtual environment for this devcontainer
# RUN python3 -m venv $HOME/env
# ENV PATH="$HOME/env/bin:$PATH"

