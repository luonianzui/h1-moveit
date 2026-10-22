#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3COMPAT

exec rviz2 --ogre-log
