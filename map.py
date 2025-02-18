import pygame
import math

import os
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

from geometry import Point, getUnitPointFromAngle, splitAndMerge
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# pygame setup
pygame.init()
screen = pygame.display.set_mode((600, 600))
clock = pygame.time.Clock()
running = True

ranges = [0 for i in range(430)]

# Callback function that is triggered each time a message is received on the /scan topic
def scan_callback(msg):
    global ranges
    ranges = [r if msg.range_min < r < msg.range_max else None for r in msg.ranges]

# Initialize the ROS node
rospy.init_node('listener')

# Subscribe to the /scan topic på 10 hz
rospy.Subscriber('/scan', LaserScan, scan_callback)

robotPos = [0, 0, 0] # x, y og vinkel der x e fram, og radian vinkel (positiv venstre)

def odom_callback(msg):
    global robotPos
    robotPos[2] += msg.twist.twist.angular.z / 50 * 0.6 # Manuell korrigering
    robotPos[1] += msg.twist.twist.linear.x / 50 * math.sin(robotPos[2]) * 1
    robotPos[0] += msg.twist.twist.linear.x / 50 * math.cos(robotPos[2]) * 1
    # print(robotPos)

# Subscribe to the /odom topic på 50 hz
rospy.Subscriber('/odom', Odometry, odom_callback)

# Create a publisher for the /cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

cmd_vel = Twist()

isLine = False
frameCount = 0
touchCord = (0, 0)

firstPoints = []
pointIndexes = []

while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        break

    # Draw background to clear last frame
    screen.fill("white")

    pygame.draw.polygon(screen, "black", [
        Point(0.2, 0).toScreen().xy(), # .fromReferenceFrame(robotPos)
        Point(-0.1, 0.1).toScreen().xy(),
        Point(-0.1, -0.1).toScreen().xy()
    ], 5)

    points = []
    for i, r in enumerate(ranges):
        if r == None:
            continue
        # Skaff punktet i den globale referanseramma
        points.append(getUnitPointFromAngle(i).scale(r).add(Point(0.2, 0)).fromReferenceFrame(robotPos))

    for i, point in enumerate(points):
        pygame.draw.circle(screen, "black", point.toReferenceFrame(robotPos).toScreen().xy(), 3)

    if len(firstPoints) > 1:
        for point in firstPoints:
            pygame.draw.circle(screen, "gray", point.toReferenceFrame(robotPos).toScreen().xy(), 3)
        pygame.draw.lines(screen, 'orange', False, [p.toReferenceFrame(robotPos).toScreen().xy() for i, p in enumerate(firstPoints) if i in pointIndexes], 2)

    f = pygame.font.Font(size=32)

    # Skriv robotPos til skjermen
    screen.blit(f.render(f'({round(robotPos[0], 2)}, {round(robotPos[1], 2)}, {round(robotPos[2], 2)})', True, "black", "white"), (30, 10))

    if pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "blue", (pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1]), 30)
        touchCord = ((pygame.mouse.get_pos()[0] - 300) / 300, (pygame.mouse.get_pos()[1] - 300) / 300)
    else:
        touchCord = (touchCord[0] * 0.9, touchCord[1] * 0.9)

    # Dette konvertere fra turning velocity (grader) te faktisk wheel position, så e smoothar for dette:)
    cmd_vel.linear.x = -touchCord[1]
    cmd_vel.angular.z = -touchCord[0] * -touchCord[1]

    # # Kjør sakte, hardt venstre
    # cmd_vel.linear.x = 0.1
    # cmd_vel.angular.z = 100

    # # Fram og tilbake
    # cmd_vel.angular.z = 0
    # if frameCount % 50 < 25:
    #     cmd_vel.linear.x = 0.05
    # else:
    #     cmd_vel.linear.x = -0.05

    cmd_vel_pub.publish(cmd_vel)

    if frameCount % 50 == 10:
        firstPoints = [point for point in points]
        pointIndexes = splitAndMerge(points, distanceParameter=0.03)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
