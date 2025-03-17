import datetime
import random
import pygame
import math
import os

from driving import AStar, Turn
os.environ["DISPLAY"] = ":0" # Gjør at pygame åpne vindu på roboten heller enn over SSH

from benchmark import Benchmark
from geometry import Orientation, Point, closestPoint, closestPointIndex, getUnitPointFromAngle, splitAndMerge, tryTranslations, unitCirclePoint
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

turns = []
turnStart = None

while True:
    # Kryss ut eller CTRL + C for å stopp programmet
    if any([e.type == pygame.QUIT for e in pygame.event.get()]) or rospy.is_shutdown():
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        cmd_vel_pub.publish(cmd_vel)
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

        # print(touchCord.y)

        # turn1 = Turn(Orientation(0, 0, 0), 0.5 / 10, duration=0.1)
        # pygame.draw.circle(screen, "red", turn1.turnCenter.toScreen().xy(), 10)
        # pygame.draw.circle(screen, "red", turn1.endPos.toScreen().xy(), 10)
        # if turn1.turnCenter:
        #     pygame.draw.circle(screen, "red", turn1.turnCenter.toScreen().xy(), turn1.turnRadius*100, width=3)

        # print(touchCord.xy(), turn1.free([touchCord]))

        # for x in range(-30, 30):
        #     for y in range(-30, 30):
        #         pnt = Point(x/20, y/20)
        #         if turn1.free([pnt]):
        #             pygame.draw.circle(screen, "green", pnt.toScreen().xy(), 2)
        #         else:
        #             pygame.draw.circle(screen, "red", pnt.toScreen().xy(), 2)

        target = touchCord.fromReferenceFrame(robotPos)
        print('Target:', target)
        turns = AStar(robotPos, target, points)
        turnStart = None

    if turns:
        for t in turns:
            # print(t.endPos, target.distance(t.endPos))
            pygame.draw.circle(screen, "green", t.endPos.toReferenceFrame(robotPos).toScreen().xy(), 3)
            if t.turnCenter:
                pygame.draw.circle(screen, "red", t.turnCenter.toReferenceFrame(robotPos).toScreen().xy(), t.turnRadius*100, width=1)

    # KJØRING
    if turns and not pygame.mouse.get_pressed()[0]:
        if not turnStart:
            turnStart = datetime.datetime.now()
        
        currTurn = None
        for turnIndex in range(len(turns)):
            if sum([t.duration for t in turns[0:turnIndex+1]]) > (datetime.datetime.now() - turnStart).total_seconds():
                currTurn = turns[turnIndex]
                print('Turn nr', turnIndex, currTurn.turnSpeed)
                break

        if currTurn:
            # # Kjør direkte
            # cmd_vel.linear.x = 0.1
            # cmd_vel.angular.z = currTurn.turnSpeed + (random.random() - 0.5) / 10

            # Utfør planen ved å følg et punkt (for å korriger)
            partway = currTurn.partway((datetime.datetime.now() - turnStart).total_seconds() % 3).toReferenceFrame(robotPos)
            pygame.draw.circle(screen, "blue", partway.toScreen().xy(), 3)

            print(partway.x, partway.y)

            # TODO: Dette funke greit for linear.x, men for vinkelen på hjula treng vi å gjør bedre. 
            # Vi må konverter fra vinkelen fra bilen te punktet, til rotasjonshastighet som gjør at hjulan peke på det punktet. 
            # Merk også her at hjulan e foran der bilen rotere rundt, altså for at hjulan ska pek på punktet må vi ta høyde for det.

            # Så: 
            # - Ta origoAngle av punktet i robotens referanseramme (men justert så sentrum kjem på forhjula)
            # - Ta den vinkelen 90 grader til høyre/venstre, for å finn sentrum av sirkelen vi må roter rundt
            # - Bruk så sentrum av sirkelen te å regn ut ka rotasjonshastighet vi må ha. 

            cmd_vel.linear.x = partway.x
            cmd_vel.angular.z = partway.y
        else:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0

        # Dette er at den bare sikte på punktet du trykke på, naivt. 
        # Burda skrevet en bedre versjon av dette med nøyaktige svinger. 
        # pygame.draw.circle(screen, "green", target.toReferenceFrame(robotPos).toScreen().xy(), 5)
        # if target.distance(robotPos.point()) < 0.2:
        #     target = None
        # else:
        #     targetForRobot = target.toReferenceFrame(robotPos)
        #     cmd_vel.linear.x = 0.1 if targetForRobot.x > 0 else -0.1
        #     cmd_vel.angular.z = targetForRobot.y * 4 * (1 if targetForRobot.x > 0 else -1)
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

    # if frameCount % 10 == 0:
    #     print(benchmark)

    pygame.display.flip() # Draw the screen

    clock.tick(10)  # 10 FPS

    frameCount += 1

# Stopp ROS om vi kryssa ut
rospy.signal_shutdown(0)
