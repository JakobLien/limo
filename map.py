import pygame
import math
import os
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

from benchmark import Benchmark
from geometry import Orientation, Point, closestPoint, closestPointIndex, getUnitPointFromAngle, splitAndMerge, tryTranslations
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

# Liste av points som e hjørne punkt
# Sida det bli veldig få punkt kan vi slepp å tenk på oppslagstid foreløpig
globalMap = []
# offset = [0, 0, 0]

# offsetSum = [0, 0, 0]

benchmark = Benchmark()

while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        break

    benchmark.start('Point transform')

    # INPUT TRANSFORMASJON
    points = []
    for i, r in enumerate(ranges):
        if r == None:
            continue
        if r < 0.25:
            continue
        # Skaff punktet i den globale referanseramma
        points.append(getUnitPointFromAngle(i).scale(r).add(Point(0.2, 0)).fromReferenceFrame(robotPos))

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


    # KJØRING
    if target and not pygame.mouse.get_pressed()[0]:
        pygame.draw.circle(screen, "green", target.toReferenceFrame(robotPos).toScreen().xy(), 5)
        # print(target.distance(robotPos.point()))

        if target.distance(robotPos.point()) < 0.2:
            target = None
        else:
            targetForRobot = target.toReferenceFrame(robotPos)
            cmd_vel.linear.x = 0.1 if targetForRobot.x > 0 else -0.1
            cmd_vel.angular.z = targetForRobot.y * 4 * (1 if targetForRobot.x > 0 else -1)
    else:
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
    cmd_vel_pub.publish(cmd_vel)

    benchmark.start('Split and merge')

    # POSISJON KORRIGERING OG MAP MAINTENANCE
    newPoints = splitAndMerge(points)

    benchmark.start('Position correction')

    # Korriger posisjon dersom vi e i bevegelse. 
    offset = Orientation(0.0, 0.0, 0.0)
    if cmd_vel.linear.x != 0:
        offset = tryTranslations(globalMap, newPoints)

        # # TODO: Jobb meir med å juster odometri dataen, det virke skikkelig nyttig å kunna gjør, og e trur det skal vær mulig å gjør!
        # offsetInRobotFrame = [*Point(offset).toReferenceFrame(Orientation(0, 0, robotPos.angle)).xy(), offset[2]]

        # offsetSum = [t1+t2 for t1, t2 in zip(offsetSum, offsetInRobotFrame)]

        # # Juster odometridataen basert på dette
        # pygame.draw.circle(screen, "green", Point(offsetInRobotFrame).scale(100).toScreen().xy(), 10)

        # # # Om du korrigere deg framover, øk den den
        # # odomModifierX += Point(offset[:2]).toReferenceFrame(Orientation(0, 0, robotPos.angle)).x / 2

        # # # Om du svinge venstre og korrigere til venstre, øk den
        # # odomModifierZ += cmd_vel.angular.z * offset[2] / 2

        robotPos = robotPos.toReferenceFrame(offset)

    benchmark.start('Maintain map: Alter points')

    # Vedlikehold kartet
    hitIndexes = []
    for newPoint in [p.toReferenceFrame(offset) for p in newPoints]:
        closest = closestPoint(newPoint, globalMap)
        if not closest or newPoint.distance(closest) > 0.05:
            # Nytt punkt
            globalMap.append(newPoint.setData('miss', 8))
        else:
            # Eksisterende pukt
            closestIndex = globalMap.index(closest)
            globalMap[closestIndex] = closest.toward(newPoint).setData('miss', closest.data['miss'])
            if closestIndex not in hitIndexes:
                hitIndexes.append(closestIndex)

    benchmark.start('Maintain map: Obscurement')

    for i, p in enumerate(globalMap):
        if i in hitIndexes:
            # En mindre miss
            globalMap[i] = p.setData('miss', max(p.data['miss'] - 1, 0))
        else:
            # En potensiell miss, spørs på obscurement
            pInRobotFrame = p.toReferenceFrame(robotPos)
            pInRobotFrameAngle = pInRobotFrame.origoAngle()

            bestPointIndexMax = len(points) - 1
            bestPointIndexMin = 0
            while bestPointIndexMax - bestPointIndexMin > 1:
                bestPointIndex = int((bestPointIndexMax + bestPointIndexMin) / 2)
                # Merk at Lidaren gir oss punktan fra stor til lav vinkel (venstre mot høyre)
                if points[bestPointIndex].toReferenceFrame(robotPos).origoAngle() > pInRobotFrameAngle:
                    bestPointIndexMax = bestPointIndex
                else:
                    bestPointIndexMin = bestPointIndex
            measurementPoint = points[int((bestPointIndexMax + bestPointIndexMin) / 2)].toReferenceFrame(robotPos)

            if abs(measurementPoint.origoAngle() - pInRobotFrame.origoAngle()) > 2 * math.pi / 430:
                # Om næmreste målingens vinkel stemme dårlig overrens, bare senk sikkerheten om det e et rødt punkt. 
                if p.data['miss'] > 5:
                    globalMap[i] = p.setData('miss', p.data['miss'] + 1)

            if measurementPoint.distance(Point(0, 0)) > pInRobotFrame.distance(Point(0, 0)):
                # Det e bak roboten, minke sikkerheten
                globalMap[i] = p.setData('miss', p.data['miss'] + 2)
            elif p.data['miss'] > 5:
                # Det e foran roboten, miss kun dersom om punktet e usikkert te å begynn med. 
                # Dette for å unngå at når vi flytte noko mot roboten får vi veldig mange usikre punkt. 
                globalMap[i] = p.setData('miss', p.data['miss'] + 1)

    # Slett punkt med mange misses
    globalMap = [p for p in globalMap if p.data['miss'] < 10]

    benchmark.start('Draw features')

    for point in globalMap:
        # if point.data.get('miss') < 3:
        #     pygame.draw.circle(screen, "black", point.toReferenceFrame(robotPos).toScreen().xy(), 3)

        if globalMap.index(point) in hitIndexes:
            pygame.draw.circle(screen, "green", point.toReferenceFrame(robotPos).toScreen().xy(), 3)
        elif point.data.get('miss') > 5:
            pygame.draw.circle(screen, "red", point.toReferenceFrame(robotPos).toScreen().xy(), 3)
        else:
            pygame.draw.circle(screen, "black", point.toReferenceFrame(robotPos).toScreen().xy(), 3)

    benchmark.stop()

    if frameCount % 10 == 0:
        print(benchmark)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
