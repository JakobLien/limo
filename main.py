import datetime
import random
import pygame
import math
import os

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
running = True

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

while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        # Stopp bilen
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        cmd_vel_pub.publish(cmd_vel)
        break

    benchmark = Benchmark()
    benchmark.start('Point transform')

    # INPUT TRANSFORMASJON
    scan = LidarScan(ranges, globalMap.robotPos)

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

    for point in scan.points:
        pygame.draw.circle(screen, "grey", point.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 1)

    # for i, (p1, p2) in enumerate(zip(scan.featurePoints, scan.featurePoints[1:])):
    #     pygame.draw.circle(screen, "black", p1.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 1)
    #     if scan.linesBool[i]:
    #         pygame.draw.line(screen, "gray", p1.toReferenceFrame(globalMap.robotPos).toScreen().xy(), p2.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 2)

    # Skriv robotPos til skjermen
    f = pygame.font.Font(size=32)
    screen.blit(f.render(f'robotPos: ({round(globalMap.robotPos.x, 2)}, {round(globalMap.robotPos.y, 2)}, {round(globalMap.robotPos.angle, 2)})', True, "black", "white"), (10, 10))

    # offsetSumInRobotFrame = Point(offsetSum)

    # screen.blit(f.render(f'offset: ({round(offsetSumInRobotFrame.x, 2)}, {round(offsetSumInRobotFrame.y, 2)}, {round(offsetSum[2], 2)})', True, "black", "white"), (10, 70))
    # # screen.blit(f.render(f'odomModifierX: {round(odomModifierX, 2)}', True, "black", "white"), (10, 30))
    # # screen.blit(f.render(f'odomModifierZ: {round(odomModifierZ, 2)}', True, "black", "white"), (10, 50))

    benchmark.start('UI and Drive')

    # BRUKER INPUT
    if pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "blue", Point(pygame.mouse.get_pos()).xy(), 30)
        touchCord = Point(pygame.mouse.get_pos()).fromScreen()

        target = touchCord.fromReferenceFrame(globalMap.robotPos)
        turns, turnStart = None, None

        pygame.draw.circle(screen, "blue", target.toScreen().xy(), 3)

        # # Debugging av wheelDrivers
        # turnRadius = circularWheelDriver(target)
        # pygame.draw.circle(screen, "red", Point(0, turnRadius).toScreen().xy(), 4)
        # pygame.draw.circle(screen, "red", Point(0, turnRadius).toScreen().xy(), abs(turnRadius)*100, width=1)
        # pygame.draw.circle(screen, "red", Point(0.2, 0).toScreen().xy(), 4)
        # # cmd_vel.linear.x = target.distance(Point(0, 0)) / 20
        # # cmd_vel.angular.z = 0.1 / turnRadius
        # # cmd_vel_pub.publish(cmd_vel)
        # pygame.draw.line(screen, "green", Point(0.2, 0).toScreen().xy(), Point(0, turnRadius).toScreen().xy(), 2)
        # pygame.draw.line(screen, "green", Point(0.2, 0).toScreen().xy(), target.toScreen().xy(), 2)


        # # En potensiell miss, spørs på obscurement
        # pInRobotFrameAngle = target.origoAngle()

        # # Binærsøk fram til rett måling
        # bestPointIndexMin, bestPointIndexMax = 0, len(scan.points) - 1
        # while bestPointIndexMax - bestPointIndexMin > 1:
        #     bestPointIndex = int((bestPointIndexMax + bestPointIndexMin) / 2)
        #     print(bestPointIndex)
        #     # Merk at Lidaren gir oss punktan fra stor til lav vinkel (venstre mot høyre)
        #     if scan.points[bestPointIndex].toReferenceFrame(globalMap.robotPos).origoAngle() > pInRobotFrameAngle:
        #         bestPointIndexMax = bestPointIndex
        #     else:
        #         bestPointIndexMin = bestPointIndex
        # measurementPoint = scan.points[int((bestPointIndexMax + bestPointIndexMin) / 2)].toReferenceFrame(globalMap.robotPos)

        # pygame.draw.circle(screen, "red", measurementPoint.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 5)
    elif target and not turns:
        turns = AStar(globalMap.robotPos, target, scan.points)
        if turns:
            turns = TurnManager(turns)
        else:
            print('Found no path')
            target = None
        turnStart = None
    
    # # TODO: Dette skaffe nærmste punktet til målpunktet heller enn punktet som e nærmast roboten. 
    # # Problemet med dette e at den ikkje recovere om punktet havne på veggen. 
    # # Kanskje kombiner metodan? 
    # if scan.points:
    #     closest, closestDist = None, math.inf
    #     for p in scan.points:
    #         if not closest or p.distance(followAroundPoint) < closestDist:
    #             closest, closestDist = p, p.distance(followAroundPoint)
        
    #     followAroundPoint = closest

    #     pygame.draw.circle(screen, "blue", followAroundPoint.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 5)

    # if scan.points:
    #     while scan.points[followAroundIndex].distance(Point(0, 0)) > scan.points[followAroundIndex+1].distance(Point(0, 0)):
    #         followAroundIndex += 1
    #     while scan.points[followAroundIndex].distance(Point(0, 0)) > scan.points[followAroundIndex-11].distance(Point(0, 0)):
    #         followAroundIndex -= 1

    #     followAroundIndex = min(max(rangesLength/4, followAroundIndex), rangesLength*3/4)

    #     pygame.draw.circle(screen, "blue", scan.points[followAroundIndex].toReferenceFrame(globalMap.robotPos).toScreen().xy(), 5)

    # if len(scan.points) > 100:
    #     pointsInRobotFrame = [p.toReferenceFrame(globalMap.robotPos) for p in scan.points]

    #     followAroundPointIndex = pointIndexClosestToAngle(pointsInRobotFrame, followAroundAngle)

    #     for i in range(4, 0, -1):
    #         if pointsInRobotFrame[followAroundPointIndex].distance(Point(0, 0)) > pointsInRobotFrame[followAroundPointIndex-2**i].distance(Point(0, 0)):
    #             followAroundPointIndex -= 2**i
    #         if pointsInRobotFrame[followAroundPointIndex].distance(Point(0, 0)) > pointsInRobotFrame[followAroundPointIndex+2**i].distance(Point(0, 0)):
    #             followAroundPointIndex += 2**i

    #     followAroundAngle = pointsInRobotFrame[followAroundPointIndex].origoAngle()

    if turns:
        for t in turns.turns:
            pygame.draw.circle(screen, "green", t.endPos.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 5)
            if t.turnCenter:
                # Tegn på veian den kjøre
                pygame.draw.circle(screen, "pink", t.turnCenter.toReferenceFrame(globalMap.robotPos).toScreen().xy(), abs(t.turnRadius)*100 - 20, width=1)
                pygame.draw.circle(screen, "red", t.turnCenter.toReferenceFrame(globalMap.robotPos).toScreen().xy(), abs(t.turnRadius)*100, width=1)
                pygame.draw.circle(screen, "pink", t.turnCenter.toReferenceFrame(globalMap.robotPos).toScreen().xy(), abs(t.turnRadius)*100 + 20, width=1)

    # KJØRING
    if turns and not pygame.mouse.get_pressed()[0]:
        if not turnStart:
            turnStart = datetime.datetime.now()

        currDist = (datetime.datetime.now() - turnStart).total_seconds() * ROBOT_SPEED

        if currDist > turns.distance:
            target, turns, turnStart = None, None, None
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

    benchmark.stop()
    print(benchmark)

    # if frameCount % 10 == 0:
    #     print(benchmark)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
