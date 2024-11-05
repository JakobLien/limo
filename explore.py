import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Create a publisher for the /cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

cmd_vel = Twist()

# Callback function that is triggered each time a message is received on the /scan topic
def scan_callback(msg):
    # print(msg)
    # ranges = msg.ranges[210:220]
    # dist = sum(ranges)/len(ranges)
    # print(dist)

    ranges = msg.ranges[int(430*1/4):int(430*3/4)]

    if min(ranges) <= 0.1:
        print("Not moving due to proximity", list(map(lambda r: r[0], filter(lambda r: r[1] <= 0.1, enumerate(ranges)))))
        cmd_vel.linear.x = 0
        cmd_vel_pub.publish(cmd_vel)
        return

    # Fremre halvdel av dem
    print("moving")
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = ranges.index(max(ranges)) / (430/2)
    cmd_vel_pub.publish(cmd_vel)

# Initialize the ROS node
rospy.init_node('scan_listener')

# Subscribe to the /scan topic
rospy.Subscriber('/scan', LaserScan, scan_callback)

# Keep the node running to listen for messages
rospy.spin()
