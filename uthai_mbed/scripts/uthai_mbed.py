#!/usr/bin/env python

import rospy
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException

if __name__ == "__main__":
    rospy.init_node("uthai_mbed")
    rospy.loginfo("ROS Serial to Uthai Robot")

    port_name = rospy.get_param('~port', '/dev/ttyACM0')
    baudrate = int(rospy.get_param('~baud', '57600'))
    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %s baud." % (port_name, baudrate))
        try:
            client = SerialClient(port_name, baudrate)
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            rospy.sleep(0.1)
            continue
        except OSError:
            rospy.sleep(0.1)
            continue
