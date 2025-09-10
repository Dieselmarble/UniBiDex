FROM ros:humble

# Language setup
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

WORKDIR /server

# Install common packages
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
  build-essential \
  gfortran \
  libblas-dev \
  liblapack-dev \
  libatlas-base-dev \
  python3-dev \
  python3-pip \
  curl \
  git \
  wget \
  vim \
  unzip \
  net-tools \
  iputils-ping \
&& rm -rf /var/lib/apt/lists/*

COPY ./banana_teleop_server /server/banana_teleop_server

RUN pip install --no-cache-dir numpy grpcio pytransform3d

RUN pip install --no-cache-dir \
      torch --extra-index-url https://download.pytorch.org/whl/cpu \
      git+https://github.com/colcon/colcon-clean

# 6) Install your package (will pull wheels from core_requirements, and only build if not available)
WORKDIR /server/banana_teleop_server
RUN pip install --no-cache-dir --prefer-binary .

# Add necessary environment variable to bashrc
RUN echo '\
source /opt/ros/humble/setup.bash \n\
if [ -d "/server/banana_teleop_server" ] \n\
then \n\
  source /server/banana_teleop_server/ros/install/local_setup.bash \n\
fi \n' >> $HOME/.bashrc

 # Clone & chmod your package
RUN chmod +x /server/banana_teleop_server/banana_teleop_server/nodes/bimanual_detector_node.py \
    /server/banana_teleop_server/banana_teleop_server/nodes/bimanual_monitor_node.py


# Build ROS and chmod the file
RUN /bin/bash -c ". /opt/ros/humble/setup.bash && cd /server/banana_teleop_server/ros && colcon clean workspace -y && colcon build"

