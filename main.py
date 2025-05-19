import pygame
import math
import os

from UI import Button
from driving import Driver, Turn
from map import LidarScan, GlobalMap
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

from benchmark import Benchmark
from geometry import Point, closestPoint
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
driver = Driver()

odomModifierX = 1
odomModifierZ = 0.8

def odom_callback(msg):
    global globalMap
    globalMap.odomRobotPos.x += msg.twist.twist.linear.x / 50 * math.cos(globalMap.odomRobotPos.angle) * odomModifierX
    globalMap.odomRobotPos.y += msg.twist.twist.linear.x / 50 * math.sin(globalMap.odomRobotPos.angle) * odomModifierX
    globalMap.odomRobotPos.angle += msg.twist.twist.angular.z / 50 * odomModifierZ

# Subscribe to the /odom topic på 50 hz
rospy.Subscriber('/odom', Odometry, odom_callback)

# Create a publisher for the /cmd_vel topic
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

cmd_vel = Twist()

clearMapButton = Button('ClearMap')
stopButton = Button('Stop')
returnButton = Button('Return')
queueButton = Button('Queue', 'Drive')
queueButton.queue = []
queueButton.labelAppend = lambda: f' ({len(queueButton.queue)})' if queueButton.queue else ''
avoidanceButton = Button('Avoid: off', 'Avoid: stop', 'Avoid: replan')
avoidanceButton.labelIndex = 2
backingButton = Button('Backing: 0', 'Backing: 1', 'Backing: 2', 'Backing: inf')
backingButton.labelIndex = driver.backLimit = 1
exploreButton = Button('Explore: Off', 'Explore: On')

buttons = [clearMapButton, stopButton, returnButton, queueButton, avoidanceButton, backingButton, exploreButton]

while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        # Stopp bilen
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        cmd_vel_pub.publish(cmd_vel)
        break

    if ranges == []:
        # Sjekk 1000 gong i sekundet i main thread om vi har fått en ny scan
        clock.tick(1000)
        continue

    globalMap.robotPos = globalMap.odomRobotPos.copy()

    benchmark = Benchmark()
    # benchmark.start('Point transform')

    # INPUT TRANSFORMASJON
    scan = LidarScan(ranges, globalMap.robotPos, benchmark=benchmark)

    benchmark.start('Draw screen')
    # Draw background to clear last frame
    screen.fill("white")

    # Tegn inn en bil på midta
    pygame.draw.polygon(screen, "black", [
        Point(0.28, 0).toScreen().xy(),
        Point(-0.02, 0.1).toScreen().xy(),
        Point(-0.02, -0.1).toScreen().xy()
    ], 3)

    for point in scan.points[::5]: # Tegn hvert 5. punkt for å spar tid
        pygame.draw.circle(screen, "black", point.toReferenceFrame(globalMap.robotPos).toScreen().xy(), 1)

    benchmark.start('UI logic')
    if not pygame.mouse.get_pressed()[0]:
        # Å sett dette "manuelt" her (utenfor button klassen) e den enklaste tilnærminga. 
        for button in buttons:
            button.clicked = False
    else:
        clickPoint = Point(pygame.mouse.get_pos())
        touchCord = clickPoint.fromScreen().fromReferenceFrame(globalMap.robotPos)
        target = None
        if clearMapButton.isNewClick(clickPoint):
            globalMap = GlobalMap()
        elif stopButton.isNewClick(clickPoint):
            driver.stop()
            queueButton.queue = []
        elif returnButton.isNewClick(clickPoint):
            target = Point(0, 0)
        elif queueButton.isNewClick(clickPoint):
            if queueButton.labelIndex == 1:
                queueButton.queue = []
                driver.stop()
        elif avoidanceButton.isNewClick(clickPoint):
            pass # Treng ikkje gjør nå spessielt, state e labelIndex
        elif backingButton.isNewClick(clickPoint):
            digit = ''.join([c for c in backingButton.label() if c.isdigit()])
            driver.backLimit = int(digit) if digit else None
        elif exploreButton.isNewClick(clickPoint):
            if exploreButton.labelIndex == 0:
                driver.stop()
        elif not any([b.isClick(clickPoint) for b in buttons]):
            # Om man ikkje klikke på nån knappa. 
            target = touchCord

        # Gitt et kjøremål, håndter det med queue eller vanlig. Også tegn knapper og gjør at trykk på roboten er stopp. 
        if target:
            targetOnScreen = target.toReferenceFrame(globalMap.robotPos).toScreen()
            if queueButton.labelIndex == 0: # Vanlig modus
                if target.distance(globalMap.robotPos) < 0.5:
                    # Klikk nær roboten skal stopp kjøringa
                    driver.stop()
                else:
                    # Kjør til det stedet. 
                    driver.setTarget(target, globalMap.robotPos, scan.points + globalMap.getLinePoints(0.2))
                    pygame.draw.circle(screen, "green" if driver.target else 'red', targetOnScreen.xy(), 30)
            else:
                # Legg til i queue om det e langt nok unna forrige mål. 
                if not queueButton.queue or queueButton.queue[-1].distance(target) > 0.5:
                    queueButton.queue.append(target)
                    pygame.draw.circle(screen, "green", targetOnScreen.xy(), 30)
                else:
                    pygame.draw.circle(screen, "blue", targetOnScreen.xy(), 30)

    benchmark.start('Drawing buttons')
    for button in buttons:
        button.draw(screen)

    benchmark.start('Driving')
    if not driver.target:
        target = None
        if queueButton.labelIndex == 0 and queueButton.queue:
            target = queueButton.queue.pop(0)
        elif exploreButton.labelIndex == 1 and globalMap.frontierLines:
            frontierLinesCenter = [globalMap.points[i1].toward(globalMap.points[i2]) for (i1, i2) in globalMap.frontierLines]
            target = closestPoint(globalMap.robotPos, frontierLinesCenter)[1]

        if target and target.distance(globalMap.robotPos) > 0.5:
            driver.setTarget(target, globalMap.robotPos, scan.points + globalMap.getLinePoints(0.2))

    if pygame.mouse.get_pressed()[0]:
        # Ikke kjør om nån trykke på skjermen
        cmd_vel.linear.x, cmd_vel.angular.z = 0, 0 # 0.1, 100
    else:
        cmd_vel.linear.x, cmd_vel.angular.z = driver.motion(globalMap.robotPos)

    # Kommenter ut dette for å stopp bilen fra å kjør. 
    cmd_vel_pub.publish(cmd_vel)

    # Kun lokaliser når vi bevege oss, for å unngå stillestående drift. 
    globalMap.addLinesAndLocalize(scan, localize=cmd_vel.linear.x != 0, benchmark=benchmark)

    if avoidanceButton.labelIndex != 0 and cmd_vel.linear.x != 0:
        benchmark.start('Avoidance')
        obstacles = scan.points + globalMap.getLinePoints(0.2)

        # Denne collision avoidancen e intensjonelt litt konservativ, slik at den ikkje skal avbryte nyttige ting. 
        currTurn = Turn(globalMap.robotPos, math.inf if cmd_vel.angular.z == 0 else cmd_vel.linear.x / cmd_vel.angular.z, min(cmd_vel.linear.x * 2.5, driver.directionDistance()))
        currTurn.draw(screen, globalMap.robotPos, width=0.15)

        # Vi sett forwardMargin=False for å gjør grensan av kor vi ser etter obstacles korrekt. 
        if not currTurn.free(obstacles, margin=0.15, forwardMargin=False)[0]:
            if avoidanceButton.labelIndex == 1:
                driver.stop()
            if avoidanceButton.labelIndex == 2:
                driver.replan(globalMap.robotPos, obstacles)

    benchmark.start('Map drawing')
    # scan.draw(screen, globalMap.robotPos)
    globalMap.draw(screen)
    driver.draw(screen, globalMap.robotPos)

    # print(benchmark)

    pygame.display.flip() # Draw the screen

    ranges = []


# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
