import datetime
import pygame
import math
import os

from UI import Button
from consts import ROBOT_SPEED
from driving import AStar, TurnManager
from map import LidarScan, GlobalMap
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

from benchmark import Benchmark
from geometry import Point, circularWheelDriver
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# pygame setup
pygame.init()
screen = pygame.display.set_mode((600, 600))
clock = pygame.time.Clock()
f = pygame.font.Font(size=32)

ranges = []
rangesLength = 0

# Callback function that is triggered each time a message is received on the /scan topic
def scan_callback(msg):
    global ranges, rangesLength
    ranges = [r if msg.range_min < r < msg.range_max else None for r in msg.ranges]
    if len(ranges) != rangesLength:
        rangesLength = len(ranges)
        print('Ranges:', rangesLength)

# Initialize the ROS node
rospy.init_node('listener')

# Subscribe to the /scan topic på 10 hz
rospy.Subscriber('/scan', LaserScan, scan_callback)

globalMap = GlobalMap()

odomModifierX = 1
odomModifierZ = 0.6

def odom_callback(msg):
    global globalMap
    globalMap.robotPos.x += msg.twist.twist.linear.x / 50 * math.cos(globalMap.robotPos.angle) * odomModifierX
    globalMap.robotPos.y += msg.twist.twist.linear.x / 50 * math.sin(globalMap.robotPos.angle) * odomModifierX
    globalMap.robotPos.angle += msg.twist.twist.angular.z / 50 * odomModifierZ

# Subscribe to the /odom topic på 50 hz
rospy.Subscriber('/odom', Odometry, odom_callback)

# Create a publisher for the /cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

cmd_vel = Twist()

isLine = False
frameCount = 0
touchCord = Point(0, 0)

pointIndexes = []
target = None

turns = []
turnStart = None
# followAroundPoint = Point(1, 0)
# followAroundIndex = rangesLength / 2

clearMapButton = Button('ClearMap')
stopButton = Button('Stop')
returnButton = Button('Return')
queueButton = Button('Queue')
queueButton.queue = []

buttons = [clearMapButton, stopButton, returnButton, queueButton]

while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        # Stopp bilen
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        cmd_vel_pub.publish(cmd_vel)
        break

    benchmark = Benchmark()
    # benchmark.start('Point transform')

    # INPUT TRANSFORMASJON
    scan = LidarScan(ranges, globalMap.robotPos, benchmark=benchmark)

    benchmark.start('Draw screen')

    # TEGNING PÅ SKJERMEN
    # Draw background to clear last frame
    screen.fill("white")

    # Tegn inn en bil på midta
    pygame.draw.polygon(screen, "black", [
        Point(0.2, 0).toScreen().xy(),
        Point(-0.1, 0.1).toScreen().xy(),
        Point(-0.1, -0.1).toScreen().xy()
    ], 5)

    for point in scan.points[::5]: # Tegn hvert 5. punkt for å spar tid
        pygame.draw.circle(screen, "grey", point.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 2)

    # # Skriv robotPos til skjermen
    # screen.blit(f.render(f'robotPos: ({round(globalMap.robotPos.x, 2)}, {round(globalMap.robotPos.y, 2)}, {round(globalMap.robotPos.angle, 2)})', True, "black", "white"), (10, 10))

    benchmark.start('UI and Drive')

    # BRUKER INPUT
    if pygame.mouse.get_pressed()[0]:
        clickPoint = Point(pygame.mouse.get_pos())
        touchCord = clickPoint.fromScreen().fromReferenceFrame(globalMap.robotPos)
        if clearMapButton.isClick(clickPoint):
            globalMap = GlobalMap()
        elif stopButton.isClick(clickPoint):
            target, turns, turnStart = None, None, None
            queueButton.queue = []
        elif returnButton.isClick(clickPoint):
            target, turns, turnStart = Point(0, 0), None, None
            queueButton.queue = []
            pygame.draw.circle(screen, "blue", target.toScreen().xy(), 3)
        elif queueButton.isClick(clickPoint):
            if queueButton.label == 'Queue' and not target:
                # Klikket på queue
                queueButton.label = 'Drive (0)'
                queueButton.queue = []
            elif queueButton.queue:
                # Klikket på Drive, dersom queue ikke er tom
                target = queueButton.queue.pop(0)
                queueButton.label = 'Queue'
        elif queueButton.label.startswith('Drive'):
            if not queueButton.queue or queueButton.queue[-1].distance(touchCord) > 0.5:
                pygame.draw.circle(screen, "blue", clickPoint.xy(), 30)
                queueButton.queue.append(touchCord)
                queueButton.label = 'Drive (' + str(int(''.join([c for c in queueButton.label if c.isnumeric()]))+1) + ')'
                print(queueButton.queue)
            else:
                pygame.draw.circle(screen, "green", clickPoint.xy(), 30)
        else:
            # Kjør til punktet
            pygame.draw.circle(screen, "blue", clickPoint.xy(), 30)
            target = touchCord
            turns, turnStart = None, None
            pygame.draw.circle(screen, "blue", target.toScreen().xy(), 3)
    elif target and not turns:
        turns = AStar(globalMap.robotPos, target, scan.points + globalMap.getLinePoints(0.2))
        if turns:
            turns = TurnManager(turns)
        else:
            print('Found no path')
            target = None
        turnStart = None
    
    benchmark.start('Drawing buttons')

    for button in buttons:
        button.draw(screen)

    if turns:
        benchmark.start('Drawing turns')
        for t in turns.turns:
            pygame.draw.circle(screen, "green", t.endPos.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 5)
            if t.turnCenter:
                # Tegn på veian den kjøre
                pygame.draw.circle(screen, "pink", t.turnCenter.toReferenceFrame(globalMap.robotPos).toScreen().xy(), abs(t.turnRadius)*100 - 20, width=1)
                pygame.draw.circle(screen, "red", t.turnCenter.toReferenceFrame(globalMap.robotPos).toScreen().xy(), abs(t.turnRadius)*100, width=1)
                pygame.draw.circle(screen, "pink", t.turnCenter.toReferenceFrame(globalMap.robotPos).toScreen().xy(), abs(t.turnRadius)*100 + 20, width=1)

    benchmark.start('Driving logic')

    # KJØRING
    if turns and not pygame.mouse.get_pressed()[0]:
        if not turnStart:
            turnStart = datetime.datetime.now()

        currDist = (datetime.datetime.now() - turnStart).total_seconds() * ROBOT_SPEED

        if currDist > turns.distance:
            # Stopp kjøringa når vi har komme fram
            target, turns, turnStart = None, None, None

            # Plukk fra queueButton om vi kan
            if queueButton.queue:
                target = queueButton.queue.pop(0)
        else:
            pass
            # Utfør planen ved å følg et punkt (for å korriger)
            partway = turns.guidePoint(currDist, 0.3).toReferenceFrame(globalMap.robotPos)
            pygame.draw.circle(screen, "blue", partway.toScreen().xy(), 3)

            turnRadius = circularWheelDriver(partway)

            distanceSpeed = (partway.distance(Point(0, 0)) - 0.20) * 10 * ROBOT_SPEED
            cmd_vel.linear.x = min(max(-ROBOT_SPEED-0.05, math.copysign(max(0, distanceSpeed), partway.x)), ROBOT_SPEED+0.05)
            cmd_vel.angular.z = cmd_vel.linear.x / turnRadius

            # # TODO: Dette funke ikkje
            # # Collision avoidance, gitt at vi kjøre slik som no 30 cm til. 
            # # Burda gjør et unntak her for om den nåværende turnen slutte like før, men dette funke foreløpig. 
            # if not Turn(globalMap.robotPos, turnRadius, math.copysign(0.3, cmd_vel.linear.x)).free(scan.points, margin=0.15)[0]:
            #     print('COLISSION AVOIDANCE')
            #     problemPoint = Turn(globalMap.robotPos, turnRadius, math.copysign(0.3, cmd_vel.linear.x)).free(scan.points, margin=0.15)[1]
            #     pygame.draw.circle(screen, "red", problemPoint.fromReferenceFrame(globalMap.robotPos).toScreen().xy(), 10)
            #     turns = None
            #     # turns = AStar(globalMap.robotPos, target, scan.points)
            #     # turns = TurnManager(turns)
            #     turnStart = None
            #     cmd_vel.linear.x = 0
            #     cmd_vel.angular.z = 0
    else:
        # cmd_vel.linear.x = 0.1
        # cmd_vel.angular.z = 100
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
    cmd_vel_pub.publish(cmd_vel)

    # benchmark.start('Map maintenance and localization')
    globalMap.addLinesAndLocalize(scan, localize=cmd_vel.linear.x != 0, benchmark=benchmark)

    benchmark.start('Map drawing')
    globalMap.draw(screen)

    print(benchmark)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
