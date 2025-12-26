cd /home/tianqi/ws_realsense
colcon build --packages-select straw


pkill -f ign
pkill -f ros

ros2 run straw simple_image_capture

source /home/tianqi/miniconda3/bin/activate pointAttn2

rm -rf build/ install/ log/ && colcon build

ign service -s /world/strawberry_world/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 5000 --req 'name: "realsense_d405" position { x: 0.5 y: 2.0 z: 1.0 } orientation { w: 1 x: 0 y: 0 z: 0 }'
