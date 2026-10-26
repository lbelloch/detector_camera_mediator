#!/usr/bin/env python3

from collections import deque
import threading

import rospy
from sensor_msgs.msg import Image


class DetectorCameraMediator:
    """Subscribe to configured camera topics and queue incoming frames."""

    def __init__(self):
        """Load camera settings and create one subscriber per topic."""
        camera_topics = rospy.get_param("~camera_topics", [])
        self.camera_topics = list(dict.fromkeys(camera_topics))
        self.queue_size = int(rospy.get_param("~frame_queue_size", 2))

        if self.queue_size < 1:
            raise ValueError("~frame_queue_size must be at least 1")

        self._queues = {
            topic: deque() for topic in self.camera_topics
        }
        self._condition = threading.Condition()
        self._next_camera_index = 0
        self._subscribers = [
            rospy.Subscriber(
                topic,
                Image,
                self._make_callback(topic),
                queue_size=1,
            )
            for topic in self.camera_topics
        ]

        if not self.camera_topics:
            rospy.logwarn("No camera topics configured")

    def _make_callback(self, camera_topic):
        """Return a callback that enqueues frames for one camera."""
        def enqueue_frame(frame):
            with self._condition:
                camera_queue = self._queues[camera_topic]
                if len(camera_queue) >= self.queue_size:
                    camera_queue.popleft()
                    rospy.logwarn_throttle(
                        5.0,
                        "Dropping oldest queued frame from %s", camera_topic
                    )
                camera_queue.append(frame)
                self._condition.notify()

        return enqueue_frame

    def _next_frame(self):
        """Return the next available frame in round-robin camera order."""
        with self._condition:
            while not rospy.is_shutdown():
                topic_count = len(self.camera_topics)
                for offset in range(topic_count):
                    index = (self._next_camera_index + offset) % topic_count
                    topic = self.camera_topics[index]
                    if self._queues[topic]:
                        self._next_camera_index = (index + 1) % topic_count
                        return topic, self._queues[topic].popleft()

                self._condition.wait(timeout=0.1)

        return None

    def run(self):
        """Drain queued frames until ROS shuts down."""
        while not rospy.is_shutdown():
            next_frame = self._next_frame()
            if next_frame is not None:
                topic, _frame = next_frame
                rospy.logdebug("Received frame from %s", topic)


def main():
    """Initialize ROS and run the camera frame queue."""
    rospy.init_node("detector_camera_mediator")
    mediator = DetectorCameraMediator()
    mediator.run()


if __name__ == "__main__":
    main()