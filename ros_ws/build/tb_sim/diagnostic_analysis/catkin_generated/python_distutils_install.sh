#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/natalia/Scrivania/tb-simulation/ros_ws/src/tb_sim/diagnostic_analysis"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/natalia/Scrivania/tb-simulation/ros_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/natalia/Scrivania/tb-simulation/ros_ws/install/lib/python2.7/dist-packages:/home/natalia/Scrivania/tb-simulation/ros_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/natalia/Scrivania/tb-simulation/ros_ws/build" \
    "/usr/bin/python2" \
    "/home/natalia/Scrivania/tb-simulation/ros_ws/src/tb_sim/diagnostic_analysis/setup.py" \
    egg_info --egg-base /home/natalia/Scrivania/tb-simulation/ros_ws/build/tb_sim/diagnostic_analysis \
    build --build-base "/home/natalia/Scrivania/tb-simulation/ros_ws/build/tb_sim/diagnostic_analysis" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/natalia/Scrivania/tb-simulation/ros_ws/install" --install-scripts="/home/natalia/Scrivania/tb-simulation/ros_ws/install/bin"
