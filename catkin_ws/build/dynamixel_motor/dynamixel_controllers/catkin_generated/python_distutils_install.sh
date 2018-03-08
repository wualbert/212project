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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/albertwu/me212project/catkin_ws/src/dynamixel_motor/dynamixel_controllers"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/albertwu/me212project/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/albertwu/me212project/catkin_ws/install/lib/python2.7/dist-packages:/home/albertwu/me212project/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/albertwu/me212project/catkin_ws/build" \
    "/usr/bin/python" \
    "/home/albertwu/me212project/catkin_ws/src/dynamixel_motor/dynamixel_controllers/setup.py" \
    build --build-base "/home/albertwu/me212project/catkin_ws/build/dynamixel_motor/dynamixel_controllers" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/albertwu/me212project/catkin_ws/install" --install-scripts="/home/albertwu/me212project/catkin_ws/install/bin"
