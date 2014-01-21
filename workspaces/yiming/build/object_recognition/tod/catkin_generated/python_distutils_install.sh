#!/bin/sh -x

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

cd "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/tod"

# todo --install-layout=deb per platform
# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/yiming/ros_workspace/youbot/workspaces/yiming/install/lib/python2.7/dist-packages:/home/yiming/ros_workspace/youbot/workspaces/yiming/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yiming/ros_workspace/youbot/workspaces/yiming/build" \
    "/usr/bin/python" \
    "/home/yiming/ros_workspace/youbot/workspaces/yiming/src/object_recognition/tod/setup.py" \
    build --build-base "/home/yiming/ros_workspace/youbot/workspaces/yiming/build/object_recognition/tod" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/yiming/ros_workspace/youbot/workspaces/yiming/install" --install-scripts="/home/yiming/ros_workspace/youbot/workspaces/yiming/install/bin"
