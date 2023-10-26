source /opt/ros/$ROS_DISTRO/setup.bash
case $1 in
    python)
        ros2 pkg create --build-type ament_python $2
        python3 $PWD/buildNode.py $2
    ;;
    c++)
        ros2 pkg create --build-type ament_cmake $2
    ;;
    *)
        echo select python or c++
    ;;
esac