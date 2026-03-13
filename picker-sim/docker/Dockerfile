# Picker V2 ROS 2 仿真环境 (ARM64 原生, 轻量版)
# 先跑 RViz2 可视化 + 关节控制，Gazebo 后续再加

FROM ros:humble

# 使用清华镜像源加速（避免 Ubuntu ports 502 错误）
RUN sed -i 's|http://ports.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# 只装可视化必需包（不装 Gazebo，省时间）
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-xacro \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
RUN mkdir -p /ros2_ws/src/picker_sim
WORKDIR /ros2_ws

# 复制文件
COPY urdf/ /ros2_ws/src/picker_sim/urdf/
COPY launch/ /ros2_ws/src/picker_sim/launch/
COPY config/ /ros2_ws/src/picker_sim/config/
COPY meshes/ /ros2_ws/src/picker_sim/meshes/
COPY kinematics.py /ros2_ws/src/picker_sim/

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && \
    echo '=== Picker V2 仿真环境就绪 ===' && \
    echo '运行: ros2 launch /ros2_ws/src/picker_sim/launch/display.launch.py' && \
    bash"]
