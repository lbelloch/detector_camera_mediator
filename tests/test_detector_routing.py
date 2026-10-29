import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


def load_mediator_module():
    rospy = types.ModuleType("rospy")
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.loginfo = lambda *args, **kwargs: None
    robotnik_msgs = types.ModuleType("robotnik_msgs")
    robotnik_msgs_msg = types.ModuleType("robotnik_msgs.msg")

    class State:
        READY_STATE = 300

    robotnik_msgs_msg.State = State
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = type("Image", (), {})
    sys.modules.update({
        "rospy": rospy,
        "robotnik_msgs": robotnik_msgs,
        "robotnik_msgs.msg": robotnik_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    })

    script_path = Path(__file__).parents[1] / "scripts" / "detector_camera_mediator_node.py"
    spec = importlib.util.spec_from_file_location("mediator_under_test", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, State


class DetectorRoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mediator_module, cls.State = load_mediator_module()

    def test_frame_fans_out_to_fresh_ready_detectors_only(self):
        registry = [
            {"name": "first", "state_topic": "/first/state", "image_topic": "/first/image"},
            {"name": "second", "state_topic": "/second/state", "image_topic": "/second/image"},
        ]
        discovery = self.mediator_module.ActiveModelDiscovery(registry, 5.0)
        ready = types.SimpleNamespace(state=self.State.READY_STATE)

        first_publisher = types.SimpleNamespace(published=[])
        first_publisher.publish = first_publisher.published.append
        second_publisher = types.SimpleNamespace(published=[])
        second_publisher.publish = second_publisher.published.append
        mediator = self.mediator_module.DetectorCameraMediator.__new__(
            self.mediator_module.DetectorCameraMediator
        )
        mediator.model_discovery = discovery
        mediator._image_publishers = {"first": first_publisher, "second": second_publisher}

        with patch.object(self.mediator_module.time, "monotonic", side_effect=[10.0, 10.0, 10.1, 16.0]):
            discovery.update_from_status("first", ready)
            discovery.update_from_status("second", ready)
            self.assertTrue(mediator._publish_frame("frame-1"))
            self.assertEqual(first_publisher.published, ["frame-1"])
            self.assertEqual(second_publisher.published, ["frame-1"])
            self.assertFalse(mediator._publish_frame("frame-2"))

    def test_non_ready_state_removes_detector(self):
        registry = [{"name": "first", "state_topic": "/first/state", "image_topic": "/first/image"}]
        discovery = self.mediator_module.ActiveModelDiscovery(registry, 5.0)
        discovery.update_from_status("first", types.SimpleNamespace(state=self.State.READY_STATE))
        discovery.update_from_status("first", types.SimpleNamespace(state=0))

        self.assertEqual(discovery.ready_detectors(), ())


if __name__ == "__main__":
    unittest.main()