#!/usr/bin/env python3

from collections import deque
import threading

import rospy
from robotnik_msgs.msg import State
from sensor_msgs.msg import Image


class ActiveModelDiscovery:
    """Expose active model input topics from the status interface."""

    def __init__(self):
        """Subscribe to each model state topic and track READY models."""
        status_topics = rospy.get_param("~model_status_interfaces", [])
        if not isinstance(status_topics, list):
            raise ValueError("~model_status_interfaces must be a list")
        if any(not isinstance(topic, str) or not topic for topic in status_topics):
            raise ValueError("~model_status_interfaces entries must be topic strings")

        self.status_topics = tuple(dict.fromkeys(status_topics))
        self._active_model_topics = set()
        self._lock = threading.Lock()
        self._subscribers = [
            rospy.Subscriber(
                topic,
                State,
                self._make_status_callback(topic),
                queue_size=1,
            )
            for topic in self.status_topics
        ]

        if self.status_topics:
            rospy.logwarn(
                "Tracking READY model states; frames forward when any model is active"
            )
        else:
            rospy.loginfo("No model state topics configured")

    def _make_status_callback(self, status_topic):
        """Return a callback that updates activity for one model."""
        def update(status_message):
            self.update_from_status(status_topic, status_message)

        return update

    def has_active_models(self):
        """Return whether at least one tracked model is currently READY."""
        with self._lock:
            return bool(self._active_model_topics)

    def update_from_status(self, status_topic, status_message):
        """Mark a model active only while its state is READY_STATE."""
        with self._lock:
            if status_message.state == State.READY_STATE:
                self._active_model_topics.add(status_topic)
            else:
                self._active_model_topics.discard(status_topic)


class DetectorCameraMediator:
    """Subscribe to configured camera topics and queue incoming frames."""

    def __init__(self):
        """Load camera settings and create one subscriber per topic."""
        camera_topics = rospy.get_param("~camera_topics", [])
        self.camera_topics = list(dict.fromkeys(camera_topics))
        self.queue_size = int(rospy.get_param("~frame_queue_size", 2))
        self.output_topic = rospy.get_param(
            "~output_topic", "/robot/detectors/image_raw"
        )

        if self.queue_size < 1:
            raise ValueError("~frame_queue_size must be at least 1")

        self._queues = {
            topic: deque() for topic in self.camera_topics
        }
        self.model_discovery = ActiveModelDiscovery()
        self._image_publisher = rospy.Publisher(
            self.output_topic, Image, queue_size=1
        )
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

    def _publish_frame(self, frame):
        """Publish one frame when at least one model is active."""
        if not self.model_discovery.has_active_models():
            return False
        self._image_publisher.publish(frame)
        return True

    def run(self):
        """Drain queued frames until ROS shuts down."""
        while not rospy.is_shutdown():
            next_frame = self._next_frame()
            if next_frame is not None:
                topic, frame = next_frame
                self._publish_frame(frame)
                rospy.logdebug("Processed frame from %s", topic)


def main():
    """Initialize ROS and run the camera frame queue."""
    rospy.init_node("detector_camera_mediator")
    mediator = DetectorCameraMediator()
    mediator.run()


if __name__ == "__main__":
    main()