#!/usr/bin/env bash

# Source this file before running ROS/Gazebo TurtleBot3 demos.
# It removes Conda from PATH for the current shell because ROS Humble's
# Gazebo tools expect the system Python at /usr/bin/python3.

if [ -n "${CONDA_PREFIX:-}" ]; then
  unset CONDA_DEFAULT_ENV
  unset CONDA_EXE
  unset CONDA_PREFIX
  unset CONDA_PROMPT_MODIFIER
  unset CONDA_PYTHON_EXE
  unset CONDA_SHLVL
fi

case ":${PATH}:" in
  *":/home/pl/miniconda3/bin:"*)
    PATH="${PATH//\/home\/pl\/miniconda3\/bin:/}"
    PATH="${PATH//:\/home\/pl\/miniconda3\/bin/}"
    ;;
esac

case ":${PATH}:" in
  *":/home/pl/miniconda3/condabin:"*)
    PATH="${PATH//\/home\/pl\/miniconda3\/condabin:/}"
    PATH="${PATH//:\/home\/pl\/miniconda3\/condabin/}"
    ;;
esac

case ":${PATH}:" in
  *":/home/pl/miniconda3/envs/agv_system/bin:"*)
    PATH="${PATH//\/home\/pl\/miniconda3\/envs\/agv_system\/bin:/}"
    PATH="${PATH//:\/home\/pl\/miniconda3\/envs\/agv_system\/bin/}"
    ;;
esac

case ":${PATH}:" in
  *":/home/pl/miniconda3/envs/crawl/bin:"*)
    PATH="${PATH//\/home\/pl\/miniconda3\/envs\/crawl\/bin:/}"
    PATH="${PATH//:\/home\/pl\/miniconda3\/envs\/crawl\/bin/}"
    ;;
esac

export PATH
hash -r 2>/dev/null || true

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-}"

echo "ROS_DISTRO=${ROS_DISTRO}"
echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "python3=$(command -v python3)"
