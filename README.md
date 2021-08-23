在linux平台用ros2的方式编译:
    (1) 打开终端执行：
        source /opt/ros/foxy/setup.bash
    (2) 进入/root/app/XQDriverROS2/out，执行：
        colcon build --merge-install --base-path .. --cmake-args -DCOMMON_CMAKE=/root/app/include/cscv/cmaker/cscv.cmake
    (3) 打开新的终端启动launch文件：
    source /root/app/ros2ex/XQDriverROS2/out/install/setup.bash
    ros2 launch xqdriver_ros2 xqdriver.launch.py
	
