import pygame
import math

import os
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# MERK at denne fila e gammel og bruke referanserammen der Y e framover. 

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
rospy.init_node('scan_listener')

# Subscribe to the /scan topic
rospy.Subscriber('/scan', LaserScan, scan_callback)

# Create a publisher for the /cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

cmd_vel = Twist()

def getAngle(i):
    'Returns a (x, y) tuple on the unit circle from an index in the Lidar list'
    vinkel = 2 * math.pi * i / 430
    return (math.sin(vinkel), -math.cos(vinkel))

def getAverage(*points):
    return [sum([p[0] for p in points])/len(points), sum([p[1] for p in points])/len(points)]

# Fleir program her:
# Det utkommenterte e at den kjøre mot den lengst unna lidar målinga den finn
# Så e koden som ligg her no at man kan styr hjulan sin hastighet (opp/ned) og svinging(høyre/venstre) ved å rør på skjermen
# også vises også koordinaten av fingern øverst te venstre, og vi tegne en stor blå sirkel under fingern. 

relCord = (0, 0)
while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        break

    # Draw background to clear last frame
    screen.fill("white")

    pygame.draw.circle(screen, "green", [300, 300], 10)

    points = []
    for i, r in enumerate(ranges):
        if r == None:
            continue
        points.append(((r + 0.1) * getAngle(i)[0], (r + 0.1) * getAngle(i)[1]))

    for point in points:
        # if 2 < i < 428 and abs(getAverage(*points[(i-2)%430:(i+2)%430])[0] - points[i][0]) + abs(getAverage(*points[(i-2)%430:(i+2)%430])[1] - points[i][1]) > 0.1:
        #     continue
        # intensity = int((255 - (intensities[i] / 1024 * 256)) / 2)
        # color = (intensity, intensity, intensity)

        # Lysintensiteten e et godt mål på om det e en god måling, virke som om alle ekte målinga har lysintensitet over 1000 (av 1024 anntar e)
        # Fortsatt en par feilmålinger med dette, men det kan vi fjern effektivt via fleir målinger på rad. 

        pygame.draw.circle(screen, "black", (300 + int(100 * point[0]), 300 - int(100 * point[1])), 3)

    pygame.draw.circle(screen, "pink", relCord, 3)

    f = pygame.font.Font(size=32)
    
    screen.blit(f.render(f'({round(relCord[0], 2)}, {round(relCord[1], 2)})', True, "black", "white"), (30, 10))

    if pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "blue", pygame.mouse.get_pos(), 30)
        relCord = ((pygame.mouse.get_pos()[0] - 300) / 300, (pygame.mouse.get_pos()[1] - 300) / 300)
        # print(relCord)

    # if points:
    #     furthest_point = max([p for p in points if p[1] > abs(p[0])], key=lambda p: p[0]**2 + p[1]**2)
    #     pygame.draw.circle(screen, "red", (300 + int(100 * furthest_point[0]), 300 - int(100 * furthest_point[1])), 5)

    #     cmd_vel.linear.x = 0.05 * math.sqrt(furthest_point[0]**2 + furthest_point[1]**2)
    #     cmd_vel.angular.z = -furthest_point[0] / 20
    #     cmd_vel_pub.publish(cmd_vel)
    # else:
    #     print('points empty')

    cmd_vel.linear.x = -relCord[1]
    cmd_vel.angular.z = -relCord[0] * -relCord[1] # Dette konvertere fra turning velocity (grader) te faktisk wheel position, så e smoothar for dette:)
    cmd_vel_pub.publish(cmd_vel)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    relCord = (relCord[0] / 1.1, relCord[1] / 1.1)

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
