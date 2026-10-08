# K1 EcoWalk (k1-mp-ecowalk-public) + ROS 2 Jazzy teleop sim, x86_64
#   docker build -f docker/Dockerfile -t k1-ecowalk:jazzy .        (run in the k1_ecowalk_ros folder)
# Python: Ubuntu 24.04's python3.12 + apt numpy 1.26 (kept, ROS python packages are built against it);
# MuJoCo and CPU PyTorch go into a venv that sees the system packages. Verified combination:
# python 3.12 / numpy 1.26.4 / mujoco 3.14.0 / torch 2.14.1.
FROM osrf/ros:jazzy-desktop

ARG ECOWALK_REPO=https://github.com/Takeyuki-K/k1-mp-ecowalk-public.git
ARG ECOWALK_REF=main
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rosbridge-server ros-jazzy-teleop-twist-keyboard \
      python3-venv python3-pip python3-numpy python3-colcon-common-extensions \
      git libgl1 libegl1 libosmesa6 mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# venv with the system site-packages (rclpy, colcon, numpy 1.26 stay the apt versions)
RUN python3 -m venv --system-site-packages /opt/k1venv \
    && python3 -c 'import numpy; print("numpy==" + numpy.__version__)' > /opt/k1venv/constraints.txt \
    && /opt/k1venv/bin/pip install --no-cache-dir -c /opt/k1venv/constraints.txt \
         torch --index-url https://download.pytorch.org/whl/cpu \
    && /opt/k1venv/bin/pip install --no-cache-dir -c /opt/k1venv/constraints.txt mujoco==3.14.0 pytest \
    && /opt/k1venv/bin/python -c "import numpy, mujoco, torch; print('numpy', numpy.__version__, 'mujoco', mujoco.__version__, 'torch', torch.__version__)"

# trained policies and environments (mount your own clone over it with run.sh ECOWALK_DIR=...)
RUN git clone --depth 1 --branch ${ECOWALK_REF} ${ECOWALK_REPO} /opt/k1-mp-ecowalk-public
ENV ECOWALK_ROOT=/opt/k1-mp-ecowalk-public

# ROS 2 package, built with the venv python so the node's shebang points to the venv
COPY . /ws/src/k1_ecowalk_ros
RUN rm -rf /ws/src/k1_ecowalk_ros/docker \
    && . /opt/ros/jazzy/setup.sh && cd /ws \
    && /opt/k1venv/bin/python -m colcon build --event-handlers console_direct- \
    && head -1 /ws/install/k1_ecowalk_ros/lib/k1_ecowalk_ros/k1_sim | grep -q /opt/k1venv \
    && echo "k1_sim uses the venv python"

# self-test at build time: the wrapper must reproduce REPORT_TURN.md numbers (no display needed)
RUN cd /ws/src/k1_ecowalk_ros && PYTHONPATH=. /opt/k1venv/bin/python -m pytest -q test/test_sim_core.py

COPY docker/k1-launch /usr/local/bin/k1-launch
RUN chmod +x /usr/local/bin/k1-launch \
    && echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && export PATH=/opt/k1venv/bin:$PATH' >> /root/.bashrc

# keep DDS traffic on this machine by default (the host may run Humble robots on the same LAN)
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_DOMAIN_ID=77

CMD ["k1-launch"]
