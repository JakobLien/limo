import pygame
import math

import os
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

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

robotPos = [0, 0, 0] # x og y med x fram, og radian vinkel

def odom_callback(msg):
    global robotPos
    robotPos[2] += msg.twist.twist.angular.z / 50 * 0.6 # Manuell korrigering
    robotPos[1] += msg.twist.twist.linear.x / 50 * math.sin(robotPos[2]) * 1
    robotPos[0] += msg.twist.twist.linear.x / 50 * math.cos(robotPos[2]) * 1
    # print(robotPos)

# Subscribe to the /odom topic på 50 hz
rospy.Subscriber('/odom', Odometry, odom_callback)

def getAngle(i, theta=0):
    'Returns a (x, y) touple on the unit circle from an index in the Lidar list'
    return rotateVector(1, 0, -2 * math.pi * (i + 215) / 430) # Den første målingen e rett bakover

def getAverage(*points):
    return [sum([p[0] for p in points])/len(points), sum([p[1] for p in points])/len(points)]

def pointDistance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def pointBetween(p1, p2, p3):
    # if min(pointDistance(p1, p2), pointDistance(p2, p3)) < 0.01:
    #     # Om nån av punktan e under en cm fra hverandre telle vi det
    #     print('Override')
    #     return True

    # Versjon av følgende omstokket for å unngå deling på 0: (p3[0] - p1[0]) / (p2[0] - p1[0]) - (p3[1] - p1[1]) / (p2[1] - p1[1]) < 0.1
    return (p3[0] - p1[0]) * (p2[1] - p1[1]) - (p3[1] - p1[1]) * (p2[0] - p1[0]) < 10**6 * (p2[1] - p1[1]) * (p2[0] - p1[0])

def rotateVector(x, y, theta):
    x_new = x * math.cos(theta) + y * math.sin(theta)
    y_new = -x * math.sin(theta) + y * math.cos(theta)
    return x_new, y_new

def addVector(x1, y1, x2, y2):
    'GITT SAMME REFERANSERAMME!!!'
    return x1+x2, y1+y2

def toScreen(x, y):
    'Konvertere fra robotens referanseramme der x e framover, til skjermen der negativ y e framover, der vi også sentrere på bilen'
    return 300 - y * 100, 300 - x * 100

def scaleVector(x, y, factor):
    return x * factor, y * factor

# Fleir program her:
# Det utkommenterte e at den kjøre mot den lengst unna lidar målinga den finn
# Så e koden som ligg her no at man kan styr hjulan sin hastighet (opp/ned) og svinging(høyre/venstre) ved å rør på skjermen
# også vises også koordinaten av fingern øverst te venstre, og vi tegne en stor blå sirkel under fingern. 

# X e framover og positiv rotasjon e te venstre. 

# Create a publisher for the /cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

cmd_vel = Twist()

isLine = False
frameCount = 0
firstPoints = []
while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        break

    # Draw background to clear last frame
    screen.fill("white")

    pygame.draw.polygon(screen, "black", [(300, 280), (310, 310), (290, 310)], 5)

    # pygame.draw.circle(screen, "green", toScreen(*scaleVector(*robotPos[:2], -1)), 10)

    pygame.draw.circle(screen, "green", toScreen(*addVector(*rotateVector(0, 0, robotPos[2]), *scaleVector(*robotPos[:2], -1))), 3)

    points = []
    for i, r in enumerate(ranges):
        if r == None:
            continue
        points.append(scaleVector(getAngle(i)[0], getAngle(i)[1], r))

    for i, point in enumerate(points):
        pygame.draw.circle(screen, "black", toScreen(*point), 3)

    firstPoints = [addVector(*rotateVector(*p, -robotPos[2]), *scaleVector(*robotPos[:2], 1)) for p in points]
    for i, point in enumerate(firstPoints):
        pygame.draw.circle(screen, "gray", toScreen(*addVector(*rotateVector(*point, robotPos[2]), *scaleVector(*robotPos[:2], 1))), 3)

    f = pygame.font.Font(size=32)

    screen.blit(f.render(f'({round(robotPos[0], 2)}, {round(robotPos[1], 2)}, {round(robotPos[2], 2)})', True, "black", "white"), (30, 10))
    # print(f'({round(robotPos[0], 2)}, {round(robotPos[1], 2)}, {round(robotPos[2], 2)})')

    touchCord = (0, 0) # Merk at dette e bare koordinatan på skjermen. 
    if pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "blue", (40, pygame.mouse.get_pos()[1]), 30)
        touchCord = ((pygame.mouse.get_pos()[0] - 300) / 300, (pygame.mouse.get_pos()[1] - 300) / 300)
        # print(relCord)
        # sliderPos = -(pygame.mouse.get_pos()[1] - 300) / 300

    # if points:
    #     furthest_point = max([p for p in points if p[1] > abs(p[0])], key=lambda p: p[0]**2 + p[1]**2)
    #     pygame.draw.circle(screen, "red", (300 + int(100 * furthest_point[0]), 300 - int(100 * furthest_point[1])), 5)

    #     cmd_vel.linear.x = 0.05 * math.sqrt(furthest_point[0]**2 + furthest_point[1]**2)
    #     cmd_vel.angular.z = -furthest_point[0] / 20
    #     cmd_vel_pub.publish(cmd_vel)
    # else:
    #     print('points empty')

    # cmd_vel.linear.x = -touchCord[1]
    # cmd_vel.angular.z = -touchCord[0] * -touchCord[1] # Dette konvertere fra turning velocity (grader) te faktisk wheel position, så e smoothar for dette:)
    # cmd_vel.linear.x = 0.1

    # Fram og tilbake
    if frameCount % 50 < 25:
        cmd_vel.linear.x = 0.1
    else:
        cmd_vel.linear.x = -0.1

    cmd_vel.angular.z = 100
    cmd_vel_pub.publish(cmd_vel)

    if frameCount % 5 == 0:
        # robotPos = [0, 0, 0]
        # Transformer til global posisjon. 
        firstPoints = [addVector(*rotateVector(*p, -robotPos[2]), *scaleVector(*robotPos[:2], 1)) for p in points]

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
