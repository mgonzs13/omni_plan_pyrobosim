# Use omniplan as base image (Ubuntu 24)
FROM omniplan:latest

# Set working directory
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]

# Set Qt to use offscreen platform (no GUI needed)
ENV QT_QPA_PLATFORM=offscreen

# Install system dependencies (minimal, only what's necessary)
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    wget \
    curl \
    psmisc \
    && rm -rf /var/lib/apt/lists/*

# Copy the current ROS 2 project code into the workspace source directory
COPY . /root/ros2_ws/src/

# Clone pyrobosim repository
RUN cd /root/ros2_ws/src && git clone https://github.com/mgonzs13/pyrobosim_ros.git

# Install pyrobosim using pip3
RUN pip3 install pyrobosim==4.3.4 numpy==1.26.4 --break-system-packages

# Install ROS dependencies using rosdep
RUN rosdep update && \
    rosdep install --from-paths /root/ros2_ws/src --ignore-src -r -y

# Build the ws (colcon)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /root/ros2_ws/install/setup.bash 2>/dev/null || true && \
    colcon build --symlink-install

# Create results directory
RUN mkdir -p /root/ros2_ws/src/results

CMD ["bash"]