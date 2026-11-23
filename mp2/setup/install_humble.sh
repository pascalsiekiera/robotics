#!/usr/bin/env bash
# One-time setup of the ROS 2 Humble environment for MP2 (Ubuntu 22.04, e.g. the WSL distro
# "Ubuntu-22.04"). Installs ROS 2 Humble, Nav2, TurtleBot3 + Gazebo Classic 11, the Python
# packages of the mp2 scripts and the dependencies of the source packages in ~/ros2_ws/src
# (teb_local_planner, costmap_converter - clone them before running this - and the link
# rrt_planner -> mp2/rrt_planner, see README).
# Usage (inside Ubuntu 22.04):
#     sudo bash install_humble.sh
# Afterwards, as normal user:
#     cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
USER_NAME="${SUDO_USER:?run with sudo from your normal user}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"
. /etc/os-release
[ "$VERSION_CODENAME" = jammy ] || { echo "ROS 2 Humble needs Ubuntu 22.04 (this is $PRETTY_NAME)"; exit 1; }

echo "=== locale, universe, ROS apt source"
apt-get update
apt-get install -y locales software-properties-common curl
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F tag_name | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.jammy_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
fi
apt-get update
apt-get upgrade -y

echo "=== ROS 2 Humble, Nav2, TurtleBot3, Gazebo Classic"
apt-get install -y \
  ros-humble-desktop ros-dev-tools \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-simulations \
  ros-humble-gazebo-ros-pkgs

echo "=== Python packages and tools of the mp2 scripts"
apt-get install -y python3-numpy python3-scipy python3-matplotlib python3-yaml python3-pil \
  ffmpeg x11-utils fonts-dejavu-core

echo "=== dependencies of the source packages in $USER_HOME/ros2_ws/src"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
sudo -u "$USER_NAME" -H rosdep update --rosdistro humble
# "rosdep install" as root would use root's own (empty) database, and as the user it would ask
# for the password again -> resolve the packages as the user (-s only prints them), install as root
DEPS=$(sudo -u "$USER_NAME" -H rosdep install --rosdistro humble \
         --from-paths "$USER_HOME/ros2_ws/src" --ignore-src -y -s \
       | sed -n 's/^ *sudo -H apt-get install -y //p' | tr '\n' ' ')
echo "rosdep dependencies: ${DEPS:-(none)}"
[ -z "$DEPS" ] || apt-get install -y $DEPS

echo "=== repo of the 'Ubuntu' (24.04, Jazzy) distro under /mnt/robotics, also at every start"
# one copy of the repo: scripts run here, results land in the git repo of the other distro
REPO_UNC='\\wsl.localhost\Ubuntu\home\maya\ros2_ws\src\robotics'
cat > /usr/local/sbin/mount-robotics <<EOF
#!/bin/sh
mkdir -p /mnt/robotics
mountpoint -q /mnt/robotics || mount -t drvfs '$REPO_UNC' /mnt/robotics -o uid=1000,gid=1000
EOF
chmod 755 /usr/local/sbin/mount-robotics
touch /etc/wsl.conf
if ! grep -q mount-robotics /etc/wsl.conf; then
  if grep -q '^\[boot\]' /etc/wsl.conf; then
    sed -i '/^\[boot\]/a command = /usr/local/sbin/mount-robotics' /etc/wsl.conf
  else
    printf '\n[boot]\ncommand = /usr/local/sbin/mount-robotics\n' >> /etc/wsl.conf
  fi
fi
/usr/local/sbin/mount-robotics && ls /mnt/robotics/mp2 >/dev/null \
  && echo "repo mounted" || echo "mount failed - try again later: sudo mount-robotics"

echo "=== environment in $USER_HOME/.bashrc"
BASHRC="$USER_HOME/.bashrc"
grep -q '# MP2 Humble' "$BASHRC" || cat >> "$BASHRC" <<'EOF'
# MP2 Humble
source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=waffle_pi
EOF

echo "=== done. Next (as $USER_NAME): cd ~/ros2_ws && colcon build --symlink-install"
