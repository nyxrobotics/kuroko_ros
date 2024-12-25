#!/bin/bash

## Get script directory
SCRIPT_DIR=$(cd $(dirname $0); pwd)

cd $SCRIPT_DIR
wget https://huggingface.co/datasets/nyxrobotics/roboone_auto_dataset/resolve/main/export_yolo/train_output/final_train_results/weights/best.pt
