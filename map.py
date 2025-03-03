import pygame
import math
import os
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

from geometry import Point, closestPoint, closestPointIndex, getUnitPointFromAngle, splitAndMerge, tryTranslations
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

robotPos = [0.0, 0.0, 0.0] # x, y og vinkel der x e fram, og radian vinkel (positiv venstre)

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
touchCord = Point(0, 0)

# firstPoints = []
pointIndexes = []
target = None

# Liste av points som e hjørne punkt
# Sida det bli veldig få punkt kan vi slepp å tenk på oppslagstid foreløpig
globalMap = []

while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        break


    # INPUT TRANSFORMASJON
    points = []
    for i, r in enumerate(ranges):
        if r == None:
            continue
        # Skaff punktet i den globale referanseramma
        points.append(getUnitPointFromAngle(i).scale(r).add(Point(0.2, 0)).fromReferenceFrame(robotPos))


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
        pygame.draw.circle(screen, "black", point.toReferenceFrame(robotPos).toScreen().xy(), 1)

    # Skriv robotPos til skjermen
    f = pygame.font.Font(size=32)
    screen.blit(f.render(f'robotPos: ({round(robotPos[0], 2)}, {round(robotPos[1], 2)}, {round(robotPos[2], 2)})', True, "black", "white"), (10, 10))


    # BRUKER INPUT
    if pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "blue", Point(pygame.mouse.get_pos()).fromScreen().toScreen().xy(), 30)
        touchCord = Point(pygame.mouse.get_pos()).fromScreen()
        target = touchCord.fromReferenceFrame(robotPos)

    if target:
        print(target.angle())

    # KJØRING
    if target and not pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "green", target.toReferenceFrame(robotPos).toScreen().xy(), 5)
        # print(target.distance(Point(robotPos[:2])))

        if target.distance(Point(robotPos[:2])) < 0.2:
            target = None
        else:
            targetForRobot = target.toReferenceFrame(robotPos)
            cmd_vel.linear.x = 0.1 if targetForRobot.x > 0 else -0.1
            cmd_vel.angular.z = targetForRobot.y * 4 * (1 if targetForRobot.x > 0 else -1)
    else:
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
    cmd_vel_pub.publish(cmd_vel)


    # POSISJON KORRIGERING OG MAP MAINTENANCE
    newPoints = splitAndMerge(points)

    for point in globalMap:
        if point.data.get('miss') > 5:
            pygame.draw.circle(screen, "red", point.toReferenceFrame(robotPos).toScreen().xy(), 3)
        else:
            pygame.draw.circle(screen, "gray", point.toReferenceFrame(robotPos).toScreen().xy(), 3)

    # Korriger posisjon dersom vi e i bevegelse. 
    offset = [0, 0, 0]
    if cmd_vel.linear.x != 0:
        offset = tryTranslations(globalMap, newPoints)
        robotPos = [t1-t2 for t1, t2 in zip(robotPos, offset)]

    # Vedlikehold kartet
    hitIndexes = []
    for newPoint in [p.toReferenceFrame(offset) for p in newPoints]:
        closest = closestPoint(newPoint, globalMap)
        if not closest or newPoint.distance(closest) > 0.05:
            # Nytt punkt
            globalMap.append(newPoint.setData('miss', -1))
        else:
            # Eksisterende pukt
            closestIndex = globalMap.index(closest)
            globalMap[closestIndex] = closest.toward(newPoint).setData('miss', closest.data['miss'])
            if closestIndex not in hitIndexes:
                hitIndexes.append(closestIndex)

    for i, p in enumerate(globalMap):
        if i in hitIndexes:
            # En mindre miss
            globalMap[i] = p.setData('miss', max(p.data['miss'] - 1, 0))
        else:
            # En potensiell miss, spørs på obscurement
            pInRobotFrame = p.toReferenceFrame(robotPos)
            pInRobotFrameAngle = pInRobotFrame.angle()

            bestPointIndexMax = len(points) - 1
            bestPointIndexMin = 0
            while bestPointIndexMax - bestPointIndexMin > 1:
                bestPointIndex = int((bestPointIndexMax + bestPointIndexMin) / 2)
                # Merk at Lidaren gir oss punktan fra stor til lav vinkel (venstre mot høyre)
                if points[bestPointIndex].toReferenceFrame(robotPos).angle() > pInRobotFrameAngle:
                    bestPointIndexMax = bestPointIndex
                else:
                    bestPointIndexMin = bestPointIndex
            measurementPoint = points[int((bestPointIndexMax + bestPointIndexMin) / 2)].toReferenceFrame(robotPos)

            if abs(measurementPoint.angle() - pInRobotFrame.angle()) > 2 * math.pi / 430:
                # Om den nærmeste målingens vinkel ikkje stemme godt overrens, ikkje mink usikkerheten
                continue

            if measurementPoint.distance(Point(0, 0)) > pInRobotFrame.distance(Point(0, 0)):
                # Det e bak roboten, minke sikkerheten
                globalMap[i] = p.setData('miss', p.data['miss'] + 1)
            else:
                # Det e foran roboten, minke ikke sikkerheten
                pass

    # Slett punkt med mange misses
    globalMap = [p for p in globalMap if p.data['miss'] < 10]

    # if frameCount % 50 == 10:
    #     firstPoints = splitAndMerge(points)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
