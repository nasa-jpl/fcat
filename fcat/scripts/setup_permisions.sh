

sudo cp scripts/fcat /opt/ros/$ROS_DISTRO/bin/fcat
sudo chmod 750 /opt/ros/$ROS_DISTRO/bin/fcat
echo "%$USER ALL=(ALL:ALL) NOPASSWD:/opt/ros/$ROS_DISTRO/bin/fcat" | sudo tee /etc/sudoers.d/fcat