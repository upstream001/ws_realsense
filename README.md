source install/setup.bash && ros2 launch straw strawberry_world.launch.py

cd /home/tianqi/ws_realsense
colcon build --packages-select straw

pkill -f ign
pkill -f ros