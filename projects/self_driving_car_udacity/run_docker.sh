#! /bin/sh

docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ -v $PWD/../data/rosbag:/capstone/ros/rosbag --rm -it capstone
