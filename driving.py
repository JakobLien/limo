import datetime
import math
import random
from typing import List
import pygame

from consts import ROBOT_SPEED

from geometry import Point, angleFromPoints, circularWheelDriver, closestPointOnLine, unitCirclePoint

class Turn:
    def __init__(self, startPos, turnRadius, distance):
        '''
        Positiv turnRadius e te venstre, negativ te høyre. For rett fram bruk turnRadius = math.inf
        Positiv distance e framover, negativ e bakover. 
        '''
        if distance == 0:
            raise Exception('Must have nonzero distance')
        self.distance = distance

        if turnRadius == 0:
            raise Exception('Must have nonzero turn radius')
        self.turnRadius = math.copysign(max(abs(turnRadius), 0.4), turnRadius)

        self.startPos = startPos

        if math.isinf(self.turnRadius):
            self.turnCenter = None
        else:
            self.turnCenter = self.startPos.add(unitCirclePoint(self.startPos.angle + math.pi / 2).scale(self.turnRadius))

        self.endPos = self.partway(abs(distance), clamp=False)

    def partway(self, distance, clamp=True):
        if distance < 0:
            raise Exception('Can\'t have negative distance here')

        if clamp:
            distance = min(max(0, distance), abs(self.distance))

        if math.isinf(self.turnRadius):
            return self.startPos.add(unitCirclePoint(self.startPos.angle).scale(math.copysign(distance, self.distance)))
        else:
            return self.startPos.rotateAround(math.copysign(distance, self.distance) / self.turnRadius , self.turnCenter)

    def free(self, points, margin=0.25, forwardMargin=True):
        '''
        Om det e nån punkt i points som bilen kræsje med. Margin e kor my klaring vi ønske for bilen på hver side. 
        Returne en tuple med en boolean om vi har åpen bane, og evt hvilket punkt vi kræsja i. 
        '''
        # For dette trur e vi bare treng å filtrer ut dem punktan som e i retning den anndelen av sirkelen vi kjøre i, 
        # for så å sjekk at avstanden fra sentrum av sirkelen e minst margin differanse fra radius av sirkelen. 
        minRadius = abs(self.turnRadius) - margin
        maxRadius = abs(self.turnRadius) + margin

        if math.isinf(self.turnRadius):
            for point in points:
                closest = closestPointOnLine(self.startPos, self.endPos, point, limit=not forwardMargin)
                if closest and closest.distance(point) < margin:
                    return (False, point)
            return (True, None)

        for point in points:
            # Sjekk at avstand fra turn center stemme ish
            if not (minRadius < point.distance(self.turnCenter) < maxRadius):
                continue

            # Sjekk at punktet e på rett del av sirkelen
            if (self.turnRadius > 0) == (self.distance > 0):
                angle = angleFromPoints(self.startPos, self.turnCenter, point)
            else:
                angle = angleFromPoints(point, self.turnCenter, self.startPos)

            if 0 < angle < (abs(self.distance) + (margin if forwardMargin else 0)) / abs(self.turnRadius):
                return (False, point)

        return (True, None)

    def draw(self, screen, robotPos, width=0.25, steps=5, color='red'):
        'Heller enn å tegn sirkla korrekt rotert i pygame, tar vi bare å bruke partway på sidan av svingen, som en rullebane. Funke fjell:)'
        for i in range(steps):
            partway = self.partway(abs(self.distance) * i / (steps-1))
            if width == 0:
                pygame.draw.circle(screen, color, partway.toReferenceFrame(robotPos).toScreen().xy(), 1)
            else:
                pygame.draw.circle(screen, color, partway.add(unitCirclePoint(partway.angle + math.pi / 2).scale(width)).toReferenceFrame(robotPos).toScreen().xy(), 1)
                pygame.draw.circle(screen, color, partway.add(unitCirclePoint(partway.angle - math.pi / 2).scale(width)).toReferenceFrame(robotPos).toScreen().xy(), 1)


class TurnManager():
    'Håndtere konverteringen av en turn sequence til et punkt man kan følg etter.'
    def __init__(self, turns=[]):
        self.turns = turns

    @property
    def distance(self):
        return sum([abs(t.distance) for t in self.turns])

    def currTurnIndex(self, distance):
        if not self.turns:
            return None

        distanceRemaining = distance

        turn = None
        for i, t in enumerate(self.turns):
            turnIndex, turn = i, t
            if distanceRemaining - abs(t.distance) < 0 or t == self.turns[-1]:
                break
            distanceRemaining -= abs(t.distance)
        return turnIndex, turn

    def guidePoint(self, distance, ahead=0.3):
        'Call partway for den turnen som e på den distancen'
        turnIndex, turn = self.currTurnIndex(distance)
        nextTurn = self.turns[turnIndex+1] if turnIndex != None and turnIndex+1 < len(self.turns) else None
        if nextTurn and math.copysign(1, turn.distance) == math.copysign(1, nextTurn.distance):
            # Dersom dem kjøre samme vei, få punktet te å følg den neste svingen
            return nextTurn.partway(distance - sum([abs(t.distance) for t in self.turns[0:self.turns.index(turn)]]) + ahead - abs(turn.distance), clamp=False)
        if turn:
            # Ellers, beveg punktet videre bortover. 
            return turn.partway(distance - sum([abs(t.distance) for t in self.turns[0:self.turns.index(turn)]]) + ahead, clamp=False)


def rateTurns(goalPos, turns):
    return sum([abs(t.distance) for t in turns]) + goalPos.distance(turns[-1].endPos)


def AStar(startPos, goalPos: Point, obstacles: List[Point], backLimit=1, stepLength=0.3, stepAngles=5, minTurnRadius=0.45):
    '''
    Returne en liste av påfølgende Turns, som kan utføres for å nå en posisjon med Hybrid A*. 
    La stepAngles vær et oddetall så vi får med kjøring rett fram og bak:) 
    '''
    turnRadiusAndStep = []
    for i in range(-(stepAngles//2), stepAngles//2+1):
        # Konverter fra i=-n...n til turning radius slik at vi får en lineær endring i turning speed
        # Vi bruke en større minTurnRadius slik at det skal vær mulig for roboten å korriger på skarpe svinga
        turnRadiusAndStep.append((stepAngles//2 * minTurnRadius / i if i != 0 else math.inf, stepLength))
        turnRadiusAndStep.append((stepAngles//2 * minTurnRadius / i if i != 0 else math.inf, -stepLength))

    turnSequences = []
    turnSequencesRating = []

    # Vi shuffle randomly for å raskar kunna kræsj i ting. Jo fleire punkt som hadd forhindra svingen lønne
    # det seg å sjå på dem for å finn et av dem tidlig, heller enn å ta det i scan rekkefølge. 
    random.shuffle(obstacles)

    for _ in range(200):
        # Legg til dem 5 nye turn sekvensan, i lista, og fjern den vi jobbe fra
        if turnSequences:
            turnSequence = turnSequences.pop(0)
            turnSequencesRating.pop(0)
        else:
            turnSequence = []

        for turnRadius, step in turnRadiusAndStep:
            if step < 0 and (backLimit == 0 or (turnSequence and backLimit != None and all(t.distance < 0 for t in turnSequence[::-1][:backLimit]))):
                # Begrens påfølgende bakover kjøring
                continue

            if turnSequence:
                turn = Turn(turnSequence[-1].endPos, turnRadius, step)
            else:
                turn = Turn(startPos, turnRadius, step)

            if not turn.free(obstacles, margin=0.25 if turnSequence else 0.15)[0]:
                # Om denne svingen kjøre for nær/inni veggen, skip den. 
                continue

            turnSeq = turnSequence + [turn]

            turnRating = rateTurns(goalPos, turnSeq)

            # Sett inn på rett plass, for å slepp å sorter heile greia hver gong:)
            for seqIndex in range(len(turnSequences)+1):
                if seqIndex == len(turnSequences):
                    turnSequences.append(turnSeq)
                    turnSequencesRating.append(rateTurns(goalPos, turnSeq))
                    break

                if turnSequencesRating[seqIndex] > turnRating:
                    turnSequences.insert(seqIndex, turnSeq)
                    turnSequencesRating.insert(seqIndex, rateTurns(goalPos, turnSeq))
                    break

        if not turnSequences:
            return None
        elif goalPos.distance(turnSequences[0][-1].endPos) < stepLength * 2/3:
            break

    return turnSequences[0] if turnSequences else None


class Driver:
    def __init__(self):
        self.target, self.turns, self.turnStart = None, None, None
        self.backLimit = None

    def setTarget(self, target, robotPos, obstacles: List[Point]):
        self.target = target
        self.replan(robotPos, obstacles)

    def replan(self, robotPos, obstacles):
        turns = AStar(robotPos, self.target, obstacles, backLimit=self.backLimit)
        if not turns:
            print('Couldn\'t find turn sequence to that point')
            self.target = None
            return

        self.turnStart = datetime.datetime.now()
        self.turns = TurnManager(turns)

    def stop(self):
        self.target, self.turns, self.turnStart = None, None, None

    def currDist(self):
        'Dette e kjernen, vi skaffe nåværende distanse i turn sekvensen ved å sjå på når vi begynt den.'
        return (datetime.datetime.now() - self.turnStart).total_seconds() * ROBOT_SPEED

    def currTurnIndex(self):
        return self.turns.currTurnIndex(self.currDist())

    def directionDistance(self) -> int:
        '''
        Returne kor my distanse vi har igjen i denne kjøreretninga (fram/bak). 
        Tiltenkt bruk på collision avoidance, sånn at roboten tørr å kjør opp te veggen. 
        '''
        lastSameDirTurnIndex = self.currTurnIndex()[0]
        while len(self.turns.turns) > lastSameDirTurnIndex+1 and (self.turns.turns[lastSameDirTurnIndex].distance > 0) == (self.turns.turns[lastSameDirTurnIndex+1].distance > 0):
            lastSameDirTurnIndex += 1
        return sum([abs(t.distance) for t in self.turns.turns[:lastSameDirTurnIndex+1]]) - self.currDist()

    def motion(self, robotPos):
        '''
        Denne bruke TurnManager sin guidePoint og circularWheelDriver for å produser en bevegense. 
        Den justere også hastigheten med distansen fra guidePoint, så roboten korrigere alle retninger. 
        Returne linear.x og angular.z som en tuple, med 0, 0 dersom det ikkje e en bevegelse akk no.
        '''
        if not self.target:
            return 0, 0

        if (datetime.datetime.now() - self.turnStart).total_seconds() * ROBOT_SPEED >= self.turns.distance:
            self.stop()
            return 0, 0

        guidePoint = self.turns.guidePoint(self.currDist(), 0.3).toReferenceFrame(robotPos)
        turnRadius = circularWheelDriver(guidePoint)

        distanceSpeed = (guidePoint.distance(Point(0, 0)) - 0.20) * 10 * ROBOT_SPEED
        linearX = min(max(-ROBOT_SPEED-0.05, math.copysign(max(0, distanceSpeed), guidePoint.x)), ROBOT_SPEED+0.05)
        angularZ = linearX / turnRadius

        return linearX, angularZ

    def draw(self, screen, robotPos):
        'Tegn guidePoint og marker kor roboten kjem te å kjør'
        if not self.target:
            return

        currDist = (datetime.datetime.now() - self.turnStart).total_seconds() * ROBOT_SPEED
        guidePoint = self.turns.guidePoint(currDist, 0.3).toReferenceFrame(robotPos)
        pygame.draw.circle(screen, "blue", guidePoint.toScreen().xy(), 3)

        for t in self.turns.turns[self.currTurnIndex()[0]:]:
            t.draw(screen, robotPos, width=0, color='gray')
