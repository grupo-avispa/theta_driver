# theta_driver

## Getting started

You need to install the libuvc and it's dependencies before using this package.

1. <https://github.com/ricohapi/libuvc-theta>

You will also need the sample for the Theta-Z1

2. <https://github.com/ricohapi/libuvc-theta-sample>

## To install the package:

```bash
cd colcon_workspace/src
git clone https://github.com/grupo-avispa/theta_driver
cd theta_driver
git submodule init
git submodule update
colcon build  --symlink-install
```

## Configure parameters:
You can configure `image_topic`, `camera_frame` or `use4k` in `params.yaml` inside `config` folder

## Try it with:

```bash
source install/setup.bash
ros2 launch theta_driver theta.launch.py 
```

And then you can use image_view package, rqt or rviz2 to see the published image.
