在linux平台用ros2的方式编译:
    (1) 打开终端执行：
        source /opt/ros/foxy/setup.bash
    (2) 进入/root/app/xqdriver_ros2/out，执行：
        colcon build --merge-install --base-path ..
    (3) 打开新的终端启动launch文件：
    source /root/app/ros2ex/xqdriver_ros2/out/install/setup.bash
    ros2 launch xqdriver_ros2 xqdriver.launch.py
	
