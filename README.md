# Object Classification Using Darknet

This package uses Darknet CNN and the COCO database to classify objects in an RGB image. See details here: [https://pjreddie.com/darknet/yolo/](https://pjreddie.com/darknet/yolo/)

To use the `yolo_classification` package in this repository, first clone Dataspeed's fork of the Darknet repository:
[https://github.com/DataspeedInc/darknet](https://github.com/DataspeedInc/darknet)

The fork adds a ROS package that builds Darknet and exports the shared C library and headers, as well as the Python interface wrapper for use in other ROS packages. This package makes use of these exported components.

## Local Configuration

By default, Darknet doesn't compile with GPU support, which allows it to compile successfully even if you don't have CUDA installed. However, it will run extremely slowly unless you enable GPU support. To set up a machine to compile Darknet with GPU support, first install CUDA:

- Determine the latest CUDA version supported by your NVIDIA graphics card. To do this, type `nvidia-smi` into a terminal and you should see something like this. If you don't, then something is wrong with the driver setup for your graphics card.
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 510.73.05    Driver Version: 510.73.05    CUDA Version: 11.6     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:06:00.0  On |                  N/A |
| 24%   45C    P0    41W / 235W |    926MiB /  8192MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
```
- Download the latest CUDA version that matches the `CUDA Version:` you see from `nvidia-smi`. The complete list of current and older versions can be found here: [https://developer.nvidia.com/cuda-toolkit-archive](https://developer.nvidia.com/cuda-toolkit-archive)
- After clicking on the correct version, select the target platform options Linux -> x86_64 -> Ubuntu -> 20.04 -> deb (local).
- After selecting platform options, copy the installation instructions it gives you one at a time into a terminal to install CUDA.
- Finally, run this command to put the CUDA installation directory on the `$PATH` environment variable every time you open a terminal. Be sure to replace the CUDA version number with the one you actually installed.
```
echo "export PATH=\$PATH:/usr/local/cuda-11.6/bin" >> $HOME/.bashrc
```

Once CUDA is installed, finally
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
