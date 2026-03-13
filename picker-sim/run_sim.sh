#!/bin/bash
# Picker V2 仿真启动脚本
# 用法: ./run_sim.sh [display|gazebo|bash]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DOCKER=/Applications/Docker.app/Contents/Resources/bin/docker

# 检查 Docker
if ! $DOCKER info > /dev/null 2>&1; then
    echo "❌ Docker 未运行，请先启动 Docker Desktop"
    exit 1
fi

# 检查镜像
if ! $DOCKER images picker-sim --format '{{.Repository}}' | grep -q picker-sim; then
    echo "📦 首次运行，构建 Docker 镜像..."
    cd "$SCRIPT_DIR"
    $DOCKER build -t picker-sim -f docker/Dockerfile .
fi

MODE=${1:-bash}

# macOS GUI 转发（需要 XQuartz）
# 如果没装 XQuartz: brew install --cask xquartz
# 然后重启，在 XQuartz 偏好设置中启用 "允许从网络客户端连接"
DISPLAY_ARGS=""
if command -v xhost > /dev/null 2>&1; then
    xhost +localhost > /dev/null 2>&1
    DISPLAY_ARGS="-e DISPLAY=host.docker.internal:0"
fi

case $MODE in
    display)
        echo "🤖 启动 RViz2 可视化..."
        $DOCKER run -it --rm \
            $DISPLAY_ARGS \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            picker-sim \
            bash -c "source /opt/ros/humble/setup.bash && \
                     ros2 launch /ros2_ws/src/picker_sim/launch/display.launch.py"
        ;;
    gazebo)
        echo "🤖 启动 Gazebo 仿真..."
        $DOCKER run -it --rm \
            $DISPLAY_ARGS \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            picker-sim \
            bash -c "source /opt/ros/humble/setup.bash && \
                     ros2 launch /ros2_ws/src/picker_sim/launch/gazebo.launch.py"
        ;;
    bash)
        echo "🤖 进入仿真环境 (交互模式)..."
        $DOCKER run -it --rm \
            $DISPLAY_ARGS \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v "$SCRIPT_DIR":/ros2_ws/src/picker_sim \
            picker-sim
        ;;
    *)
        echo "用法: $0 [display|gazebo|bash]"
        echo "  display  - RViz2 可视化（滑条控制关节）"
        echo "  gazebo   - Gazebo 物理仿真"
        echo "  bash     - 进入容器交互模式"
        exit 1
        ;;
esac
