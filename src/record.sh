#!/bin/bash
if [ -z ${1+x} ]; then
    sensor_id=0
else
    sensor_id=$1
fi
if [ -z ${2+x} ]; then
    num_buffers=1080000
else
    num_buffers=$2
fi
if [ -z ${3+x} ]; then
    filename="default_recording.mp4"
elif [ ${3} = 'auto' ]; then
    filename=$(date +"%y%m%d_%H%M%S_cam_${sensor_id}.mp4")
else
    filename=$3
fi
if [ -z ${4+x} ]; then
    FR=60
else
    FR=$4
fi
if [ -z ${5+x} ]; then
    WIDTH=3840
else
    WIDTH=$5
fi
if [ -z ${6+x} ]; then
    HEIGHT=2160
else
    HEIGHT=$6
fi
if [ -z ${7+x} ]; then
    path="/xavier_ssd/footage/video/"
else
    path=$7
fi

# If destination is missing, create it!
mkdir -p ${path} || exit $?

echo "cam[${sensor_id}] Start recording"
echo "cam[${sensor_id}] num_buffers: ${num_buffers}"
echo "cam[${sensor_id}] path: ${path}"
echo "cam[${sensor_id}] filename: ${filename}"
echo "cam[${sensor_id}] FR: ${FR}"
echo "cam[${sensor_id}] WIDTH: ${WIDTH}"
echo "cam[${sensor_id}] HEIGHT: ${HEIGHT}"

gst-launch-1.0 nvarguscamerasrc sensor-id=$sensor_id num-buffers=$num_buffers do-timestamp=true ! "video/x-raw(memory:NVMM), width=(int)$WIDTH, height=(int)$HEIGHT, format=(string)NV12, framerate=(fraction)$FR/1" ! nvv4l2h265enc maxperf-enable=1 control-rate=0 ! h265parse ! qtmux ! filesink location=$path$filename sync=1 -e
