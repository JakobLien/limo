import datetime
import random
import pygame
import math
import os

from driving import AStar, Turn, TurnManager
from map import LocalizationMap
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

from benchmark import Benchmark
from geometry import Orientation, Point, angleFromPoints, closestPoint, closestPointIndex, getUnitPointFromAngle, pointIndexClosestToAngle, splitAndMerge, tryTranslations, straightWheelDriver, circularWheelDriver, unitCirclePoint
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

robotPos = Orientation(0.0, 0.0, 0.0) # x, y og vinkel der x e fram, og radian vinkel (positiv venstre)

odomModifierX = 1
odomModifierZ = 0.6

def odom_callback(msg):
    global robotPos
    robotPos.x += msg.twist.twist.linear.x / 50 * math.cos(robotPos.angle) * odomModifierX
    robotPos.y += msg.twist.twist.linear.x / 50 * math.sin(robotPos.angle) * odomModifierX
    robotPos.angle += msg.twist.twist.angular.z / 50 * odomModifierZ

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

benchmark = Benchmark()
localizationMap = LocalizationMap()

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

    benchmark.start('Point transform')

    # INPUT TRANSFORMASJON
    points = []
    for i, r in enumerate(ranges):
        if r == None or i < 30 or i > len(ranges)-30:
            # Det e enkelte unyttige målinger rett bakover som catches av dette.
            continue
        # Skaff punktet i den globale referanseramma
        # Her korrigere vi 27 grader unna midta, usikker på koffor vi må det. 
        p = getUnitPointFromAngle(i, rangesLength, correction=-math.pi * 0/180).scale(r).add(Point(0.2, 0)).fromReferenceFrame(robotPos)
        # if p.distance(Point(0, 0)) < 0.1:
        #     print(f'Hmmm, {i} {r}')
        points.append(p)

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

    for point in points:
        pygame.draw.circle(screen, "grey", point.toReferenceFrame(robotPos).toScreen().xy(), 1)

    # Skriv robotPos til skjermen
    f = pygame.font.Font(size=32)
    screen.blit(f.render(f'robotPos: ({round(robotPos.x, 2)}, {round(robotPos.y, 2)}, {round(robotPos.angle, 2)})', True, "black", "white"), (10, 10))

    # offsetSumInRobotFrame = Point(offsetSum)

    # screen.blit(f.render(f'offset: ({round(offsetSumInRobotFrame.x, 2)}, {round(offsetSumInRobotFrame.y, 2)}, {round(offsetSum[2], 2)})', True, "black", "white"), (10, 70))
    # # screen.blit(f.render(f'odomModifierX: {round(odomModifierX, 2)}', True, "black", "white"), (10, 30))
    # # screen.blit(f.render(f'odomModifierZ: {round(odomModifierZ, 2)}', True, "black", "white"), (10, 50))

    benchmark.start('UI and Drive')

    # BRUKER INPUT
    if pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "blue", Point(pygame.mouse.get_pos()).xy(), 30)
        touchCord = Point(pygame.mouse.get_pos()).fromScreen()

        target = touchCord.fromReferenceFrame(robotPos)
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
        # bestPointIndexMin, bestPointIndexMax = 0, len(points) - 1
        # while bestPointIndexMax - bestPointIndexMin > 1:
        #     bestPointIndex = int((bestPointIndexMax + bestPointIndexMin) / 2)
        #     print(bestPointIndex)
        #     # Merk at Lidaren gir oss punktan fra stor til lav vinkel (venstre mot høyre)
        #     if points[bestPointIndex].toReferenceFrame(robotPos).origoAngle() > pInRobotFrameAngle:
        #         bestPointIndexMax = bestPointIndex
        #     else:
        #         bestPointIndexMin = bestPointIndex
        # measurementPoint = points[int((bestPointIndexMax + bestPointIndexMin) / 2)].toReferenceFrame(robotPos)

        # pygame.draw.circle(screen, "red", measurementPoint.toReferenceFrame(robotPos).toScreen().xy(), 5)
    elif target and not turns:
        turns = AStar(robotPos, target, points)
        if turns:
            turns = TurnManager(turns)
        else:
            print('Found no path')
            target = None
        turnStart = None
    
    # # TODO: Dette skaffe nærmste punktet til målpunktet heller enn punktet som e nærmast roboten. 
    # # Problemet med dette e at den ikkje recovere om punktet havne på veggen. 
    # # Kanskje kombiner metodan? 
    # if points:
    #     closest, closestDist = None, math.inf
    #     for p in points:
    #         if not closest or p.distance(followAroundPoint) < closestDist:
    #             closest, closestDist = p, p.distance(followAroundPoint)
        
    #     followAroundPoint = closest

    #     pygame.draw.circle(screen, "blue", followAroundPoint.toReferenceFrame(robotPos).toScreen().xy(), 5)

    # if points:
    #     while points[followAroundIndex].distance(Point(0, 0)) > points[followAroundIndex+1].distance(Point(0, 0)):
    #         followAroundIndex += 1
    #     while points[followAroundIndex].distance(Point(0, 0)) > points[followAroundIndex-11].distance(Point(0, 0)):
    #         followAroundIndex -= 1

    #     followAroundIndex = min(max(rangesLength/4, followAroundIndex), rangesLength*3/4)

    #     pygame.draw.circle(screen, "blue", points[followAroundIndex].toReferenceFrame(robotPos).toScreen().xy(), 5)

    # if len(points) > 100:
    #     pointsInRobotFrame = [p.toReferenceFrame(robotPos) for p in points]

    #     followAroundPointIndex = pointIndexClosestToAngle(pointsInRobotFrame, followAroundAngle)

    #     for i in range(4, 0, -1):
    #         if pointsInRobotFrame[followAroundPointIndex].distance(Point(0, 0)) > pointsInRobotFrame[followAroundPointIndex-2**i].distance(Point(0, 0)):
    #             followAroundPointIndex -= 2**i
    #         if pointsInRobotFrame[followAroundPointIndex].distance(Point(0, 0)) > pointsInRobotFrame[followAroundPointIndex+2**i].distance(Point(0, 0)):
    #             followAroundPointIndex += 2**i

    #     followAroundAngle = pointsInRobotFrame[followAroundPointIndex].origoAngle()

    if turns:
        for t in turns.turns:
            pygame.draw.circle(screen, "green", t.endPos.toReferenceFrame(robotPos).toScreen().xy(), 5)
            if t.turnCenter:
                # Tegn på veian den kjøre
                pygame.draw.circle(screen, "pink", t.turnCenter.toReferenceFrame(robotPos).toScreen().xy(), abs(t.turnRadius)*100 - 20, width=1)
                pygame.draw.circle(screen, "red", t.turnCenter.toReferenceFrame(robotPos).toScreen().xy(), abs(t.turnRadius)*100, width=1)
                pygame.draw.circle(screen, "pink", t.turnCenter.toReferenceFrame(robotPos).toScreen().xy(), abs(t.turnRadius)*100 + 20, width=1)

    speed = 0.1

    # KJØRING
    if turns and not pygame.mouse.get_pressed()[0]:
        if not turnStart:
            turnStart = datetime.datetime.now()

        currDist = (datetime.datetime.now() - turnStart).total_seconds() * speed

        if currDist > turns.distance:
            target, turns, turnStart = None, None, None
        else:
            # Utfør planen ved å følg et punkt (for å korriger)
            partway = turns.guidePoint(currDist, 0.3).toReferenceFrame(robotPos)
            pygame.draw.circle(screen, "blue", partway.toScreen().xy(), 3)

            # TODO: Om to påfølgende turns e samme retning (framover eller bakover kjøring), burda vi istedet 
            # return det punktet som følge den andre turnen, (høyre/venstre) !!! 
            # Men om det e motsatt framover/bakover burda mellompunktet istedet gå bort fra roboten. 
            # Eller omformulert, mellompunktet burda alltid gå bort fra roboten egtl. 

            turnRadius = circularWheelDriver(partway)

            # Hold den 40 cm bak punktet (forhjulan beregnes som 20 cm foran bakre aksling), og kjør maks speed + 0.1 m/s
            # Dette funke ihvertfall begge veia:)

            # Vi burda kunna regn det ut slik at om vi skrur opp hastigheten kjem den te å ha den hastigheten på 30 cm unna. 
            # Nøyaktige stigningsgraden har ikkje så my å si, men det må vær rett på ekvilibriumet for at den ska hold seg rett distanse fra punktet, 
            # som e viktig for å kunna følg en path som bytte på framover og bakover kjøring. 
            distanceSpeed = (partway.distance(Point(0, 0)) - 0.20) * 10 * speed
            cmd_vel.linear.x = min(max(-speed-0.05, math.copysign(max(0, distanceSpeed), partway.x)), speed+0.05)
            cmd_vel.angular.z = cmd_vel.linear.x / turnRadius

        # TODO: Dette funke ikkje
        # Collision avoidance, gitt at vi kjøre slik som no 30 cm til. 
        # Burda gjør et unntak her for om den nåværende turnen slutte like før, men dette funke foreløpig. 
        if not Turn(robotPos, turnRadius, math.copysign(0.3, cmd_vel.linear.x)).free(points, margin=0.15):
            print('COLISSION AVOIDANCE')
            turns = AStar(robotPos, target, points)
            turns = TurnManager(turns)
            turnStart = None
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
    else:
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
    cmd_vel_pub.publish(cmd_vel)

    benchmark.start('Localization maintenence')
    robotPos = localizationMap.addPointsAndLocalize(robotPos, points, localize=cmd_vel.linear.x != 0)

    benchmark.start('Localization drawing')
    localizationMap.draw(robotPos, screen)

    benchmark.stop()

    # if frameCount % 10 == 0:
    #     print(benchmark)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
