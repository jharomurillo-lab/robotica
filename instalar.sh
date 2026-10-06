#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

sudo apt update || true
sudo apt install -y   git python3-colcon-common-extensions python3-rosdep python3-vcstool python3-numpy   ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-robot-state-publisher   ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui   ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-launch-param-builder

mkdir -p src
if [ ! -d "src/xarm_ros2" ]; then
  cd src
  git clone --depth 1 --branch jazzy --filter=blob:none --sparse https://github.com/xArm-Developer/xarm_ros2.git
  cd xarm_ros2
  git sparse-checkout set xarm_description uf_ros_lib
  cd ../..
fi

rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true
colcon build --symlink-install --packages-up-to grupo01_xarm6_bringup grupo01_xarm6_kinematics

cat > entorno.sh <<'ENV'
#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$(pwd)/install/setup.bash" ]; then
  source "$(pwd)/install/setup.bash"
fi
ENV
chmod +x entorno.sh
echo "¡Instalación y compilación completadas!"
