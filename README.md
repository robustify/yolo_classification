# Object Classification Using Darknet

This package uses Darknet CNN and the COCO database to classify objects in an RGB image. See details here: [https://pjreddie.com/darknet/yolo/](https://pjreddie.com/darknet/yolo/)

To use the `yolo_classification` package in this repository, first clone Dataspeed's fork of the Darknet repository:
[https://github.com/DataspeedInc/darknet](https://github.com/DataspeedInc/darknet)

The fork adds a ROS package that builds Darknet and exports the shared C library and headers, as well as the Python interface wrapper for use in other ROS packages. This package makes use of these exported components.

## Local Configuration

By default, Darknet doesn't compile with GPU support, which allows it to compile successfully even if you don't have CUDA installed. However, it will run extremely slowly unless you enable GPU support. To set up a machine to compile Darknet with GPU support:

- Install Cuda (version 10.2 has been tested) by following the instructions here: [https://docs.nvidia.com/cuda/archive/10.2/cuda-installation-guide-linux/index.html](https://docs.nvidia.com/cuda/archive/10.2/cuda-installation-guide-linux/index.html)
- Open the `Makefile` in the Darknet repository clone and set `GPU=1`.
- Compile ROS workspace

Before running Darknet, you will need to download the CNN weights file ([https://pjreddie.com/media/files/yolov3.weights](https://pjreddie.com/media/files/yolov3.weights)) and put it in the root of the Darknet clone folder.

## `yolo_classification.launch`

This package contains a single launch file called `yolo_classification.launch`, which runs the following two nodes:

- `yolo_classification`: This node runs the Darknet CNN on a camera image and output bounding box and YOLO label data.
- `synced_yolo_data`: This node synchronizes images with YOLO labels and outputs an image with YOLO label data overlaid on it.

The launch file accepts the following arguments:

- `run_darknet`: If true, run `yolo_classification`, else don't
- `camera_name`: Namespace where camera images are published
- `run_image_proc`: Apply image rectification if necessary
- `image_topic`: Namespaced topic name to run classification
