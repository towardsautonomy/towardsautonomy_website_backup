#!/bin/bash

export KERAS_BACKEND=tensorflow
python yad2k.py model_data/yolov2.cfg model_data/yolov2.weights model_data/yolov2.h5
## Use yolov2_coreml.cfg if planning to convert to a coreml model
