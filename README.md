# GoCT_Sabertooth
Sabertooth 2x25 is connected to USB port. Differential drive motors are controlled by Sabertooth. Flow of current execution is  teleOpKeyboard -> cmd_vel_to_diff_value -> Sabertooth_node



ros2 pkg create --build-type ament_python py_sabertooth

~/ros2_ws$ colcon build --packages-select py_sabertooth
source install/local_setup.bash 

ros2 run py_sabertooth cmd_vel_to_diff

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run py_sabertooth diff_to_motor


