import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Create a publisher for the /cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Set up a Twist message with forward linear velocity
cmd_vel = Twist()
cmd_vel.linear.x = 0.2  # Set forward speed (e.g., 0.2 m/s)

# Publish the Twist message at a set rate
# rate = rospy.Rate(10)  # 10 Hz
# while not rospy.is_shutdown():
    # rate.sleep()

cmd_vel = Twist()

# Callback function that is triggered each time a message is received on the /scan topic
def scan_callback(msg):
    # print(msg)
    ranges = msg.ranges[210:220]
    dist = sum(ranges)/len(ranges)
    print(dist)

    cmd_vel.linear.x = (dist - 0.5) / 2
    cmd_vel_pub.publish(cmd_vel)


    # rospy.signal_shutdown(0)

# Initialize the ROS node
rospy.init_node('scan_listener')

# Subscribe to the /scan topic
rospy.Subscriber('/scan', LaserScan, scan_callback)

# Keep the node running to listen for messages
rospy.spin()

