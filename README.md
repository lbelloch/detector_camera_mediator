# detector_camera_mediator

ROS 1 node that subscribes to configured `sensor_msgs/Image` camera topics,
queues frames per camera, and processes them round-robin. Old queued frames are
dropped when a camera queue is full. When at least one configured model is
READY, frames are published on the single configured `output_topic`.

## Configure

Edit `config/detector_camera_mediator.yaml` and list camera topics in order:

```yaml
camera_topics:
  - /camera/image_raw
```

Set `frame_queue_size` and `output_topic` as needed.
`model_status_interfaces` is a list of `robotnik_msgs/State` topics, one per
model. Frames are forwarded while at least one listed model reports
`READY_STATE` (300); individual model input topics are not required.

## Build and run

From the root of a ROS 1 catkin workspace containing this package:

```bash
catkin_make
source devel/setup.bash
roslaunch detector_camera_mediator detector_camera_mediator.launch
```

The node does not start or stop models.
