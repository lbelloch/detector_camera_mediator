#!/usr/bin/env python3

import rospy


def main():
    rospy.init_node("detector_camera_mediator")
    rospy.loginfo("detector_camera_mediator node started")
    rospy.spin()


if __name__ == "__main__":
    main()