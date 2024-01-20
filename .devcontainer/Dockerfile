FROM ros:humble

ARG USERNAME=angora
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update all packages
RUN apt upgrade -y

# Install Git
RUN apt install -y git
RUN apt install -y pip

# Install all the necessary packages
COPY ./requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# Disable python setup warning
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"

# Rosdep update
RUN rosdep update
#RUN rosdep install --from-paths ./ -r -y

# Change user
USER $USERNAME

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
