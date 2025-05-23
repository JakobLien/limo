import math
from typing import List, Tuple
import pygame
import random
from benchmark import Benchmark
from geometry import ClosestGrid, Orientation, Point, closestPoint, getUnitPointFromAngle, splitAndMerge, tryTranslations, lineDistance, linesCrossing

from consts import LINE_POINTS_PER_METER, LINE_MIN_LENGTH, MAX_FRONTIER_WIDTH, MIN_FRONTIER_WIDTH, POINT_MERGE_DISTANCE, POINT_LINE_MERGE_DISTANCE, POINT_MERGE_MOVEMENT, SEE_THROUGH_LINE_BONUS, SEE_THROUGH_LINE_COUNT, LINE_POINT_MAX_JUMP, SIGHT_DISTANCE

# TODO: Kunna også prøvd å lagt inn en mekanisme for at det skal vær en viss penalty for å endre mappet. 
# Typ at endringer skal koste meir enn bekreftelser, så mappet bli mindre jumpy. 

class LidarScan:
    '''
    Representere en Lidar scan, med feature extraction og linja. 
    Er my greier å ha det slik, så slepp vi å tenk så my på hvilken data vi gir videre til hvilke funksjoner:)
    '''
    def __init__(self, ranges: List[int], robotPos, benchmark: Benchmark=None):
        # Når dette har kjørt har vi features og linja representert i scan objektet. 
        benchmark.start('Adding points')
        self.ranges = ranges
        self.points, self.pointIndexes = [], []
        for i, r in list(enumerate(ranges))[int(len(ranges)*8/36):int(len(ranges)*28/36)]:
            if r == None or r > SIGHT_DISTANCE:
                # Fjern målingen om den e ugyldig eller lenger unna enn vi akseptere
                continue
            p = getUnitPointFromAngle(i, len(ranges)).scale(r).add(Point(0.2, 0)).fromReferenceFrame(robotPos)
            self.points.append(p)
            self.pointIndexes.append(i)

        benchmark.start('Split and merge')
        self.featurePointIndexes, self.featurePoints = splitAndMerge(self.points)

        benchmark.start('Line assignment')
        # En liste av booleans som sie om punktet på denne indexen og det neste i featurePoints danne en linje eller ikkje. 
        # Linesbool bli følgelig alltid nøyaktig en kortar enn både featurePoints og featurePointIndexes. 
        self.linesBool = []
        for fpIndexIndex in range(len(self.featurePointIndexes)-1):
            p1Index, p2Index = self.featurePointIndexes[fpIndexIndex], self.featurePointIndexes[fpIndexIndex+1]
            lineLength = self.points[p1Index].distance(self.points[p2Index])

            isLine = False
            if (p2Index - p1Index) / lineLength > LINE_POINTS_PER_METER and lineLength > LINE_MIN_LENGTH:
                isLine = all(p1.distance(p2) < LINE_POINT_MAX_JUMP for p1, p2 in zip(self.points[p1Index:p2Index+1], self.points[p1Index+1:p2Index+1]))
            self.linesBool.append(isLine)

        # print(benchmark)

    def correct(self, offset):
        'Korrigere referanseramma te pointsa'
        self.points = [p.toReferenceFrame(offset) for p in self.points]
        self.featurePoints = [self.points[i] for i in self.featurePointIndexes]

    def draw(self, screen, robotPos):
        for p in self.featurePoints:
            pygame.draw.circle(screen, 'orange', p.toReferenceFrame(robotPos).toScreen().xy(), 3)

        for i, (p1, p2) in enumerate(zip(self.featurePoints, self.featurePoints[1:])):
            if self.linesBool[i]:
                # Potensielt vanlig vegg
                pygame.draw.line(screen, 'black', p1.toReferenceFrame(robotPos).toScreen().xy(), p2.toReferenceFrame(robotPos).toScreen().xy(), 2)
            else:
                # Potensielt frontier
                pygame.draw.line(screen, 'blue', p1.toReferenceFrame(robotPos).toScreen().xy(), p2.toReferenceFrame(robotPos).toScreen().xy(), 2)

class GlobalMap:
    'Et kart for features, for localization'
    def __init__(self):
        self.points: List[Point] = []
        self.lines: List[Tuple[int]] = []
        self.frontierLines: List[Tuple[int]] = []

        # Ved å splitt disse e håpet at vi kan få bedre performance ved å bestem sjølv når det oppdateres, og å ha en konsekvent state ellers. 
        self.robotPos = Orientation(0.0, 0.0, 0.0) # x, y og vinkel der x e fram, og radian vinkel (positiv venstre)
        self.odomRobotPos = Orientation(0.0, 0.0, 0.0) # Samme som robotPos, bare at denne oppdateres kontinuerlig. 

    def addPoint(self, p: Point, moveMapPoint=True):
        '''
        Legg til et punkt i mappet, og merge med eksisterende punkt om dem e nærme nok, returne indexen av punktet.
        Dette e litt meir komplisert enn man sku tru, i stor grad fordi når man har slått sammen en måling inn i kartet,
        så flytte den punktet på kartet, kanskje inn i radiusen til et anna punkt på kartet. 
        '''
        pointIndex, point = closestPoint(p, self.points, different=False)
        if point == None or p.distance(point) >= POINT_MERGE_DISTANCE:
            self.points.append(p)
            p.mapPointIndex = len(self.points) - 1
            return len(self.points) - 1

        # Flytt mot målingen. 
        self.points[pointIndex] = point = point.toward(p, POINT_MERGE_MOVEMENT if moveMapPoint else 0)

        while True:
            # Så flytt mot nærmste punkt på kartet om det kjem innen rekkevidde. 
            mapPointIndex, mapPoint = closestPoint(point, self.points)
            if mapPoint == None or point.distance(mapPoint) >= POINT_MERGE_DISTANCE:
                break
            self.points[pointIndex] = point = point.toward(mapPoint, 0.5)
            self.transferLines(mapPointIndex, pointIndex, deleteLine=True)
            self.removePoint(mapPointIndex)

        p.mapPointIndex = pointIndex
        return pointIndex

    def removePoint(self, pointIndex: int):
        'Fjerne et punkt, ved å erstatt det med None slik at indeksan i lines forblir rett.'
        for line in self.lines:
            if pointIndex in line:
                raise Exception('Can\'t remove point with line!')

        # Å slett frontiers når vi slette et punkt virke heilt fair
        for lineIndex, line in enumerate(self.frontierLines):
            if pointIndex in line:
                self.removeFrontierLine(lineIndex)

        self.points[pointIndex] = None

    def transferLines(self, p1Index: int, p2Index: int, deleteLine=False):
        'Flytte alle linja fra p2 til p1'
        if p1Index == p2Index:
            raise Exception(f'Can\'t transfer lines between the same index: {p1Index} {p2Index}')

        lineBetweenP1P2 = (min(p1Index, p2Index), max(p1Index, p2Index))

        if not deleteLine:
            if lineBetweenP1P2 in self.lines:
                raise Exception('This merge would create lines between the same point.')
            if lineBetweenP1P2 in self.frontierLines:
                raise Exception('This merge would create lines between the same point.')
        else:
            if lineBetweenP1P2 in self.lines:
                self.removeLine(self.lines.index(lineBetweenP1P2))
            if lineBetweenP1P2 in self.frontierLines:
                self.removeFrontierLine(self.frontierLines.index(lineBetweenP1P2))

        for i, (p1, p2) in enumerate(self.lines):
            if p1Index == p1:
                if (min(p2, p2Index), max(p2, p2Index)) in self.lines:
                    self.removeLine(i)
                else:
                    self.lines[i] = (min(p2, p2Index), max(p2, p2Index))
                continue
            if p1Index == p2:
                if (min(p1, p2Index), max(p1, p2Index)) in self.lines:
                    self.removeLine(i)
                else:
                    self.lines[i] = (min(p1, p2Index), max(p1, p2Index))
                continue
        
        for i, (p1, p2) in enumerate(self.frontierLines):
            if p1Index == p1:
                if (min(p2, p2Index), max(p2, p2Index)) in self.frontierLines:
                    self.removeFrontierLine(i)
                else:
                    self.frontierLines[i] = (min(p2, p2Index), max(p2, p2Index))
                continue
            if p1Index == p2:
                if (min(p1, p2Index), max(p1, p2Index)) in self.frontierLines:
                    self.removeFrontierLine(i)
                else:
                    self.frontierLines[i] = (min(p1, p2Index), max(p1, p2Index))
                continue

    def addLine(self, p1Index: int, p2Index: int):
        'Legg til linja om den ikkje allerede finnes og returne indexen.'
        p1Index, p2Index = min(p1Index, p2Index), max(p1Index, p2Index)
        if p1Index > len(self.points) or p2Index > len(self.points):
            raise Exception('Can\'t have line without point')
        if p1Index == p2Index:
            raise Exception('Line must have different indexes')
        if p1Index == None or p2Index == None:
            raise Exception('Can\'t add line with None point indexes')

        if (p1Index, p2Index) in self.lines:
            return self.lines.index((p1Index, p2Index))

        self.lines.append((p1Index, p2Index))
        return len(self.lines) - 1

    def removeLine(self, lineIndex: int):
        self.lines[lineIndex] = (None, None)

    def removeFrontierLine(self, lineIndex: int):
        self.frontierLines[lineIndex] = (None, None)

    def correctIndexes(self):
        '''
        For å støtt iterering mens vi endre antall underveis erstatte vi heller element vi fjerne med None (og linjer med (None, None)). 
        For å unngå at listan øke uten ende korrigere vi her for det og fjerne det. 
        '''
        # En mapping fra nåværende indexer til nye indexer
        indexMapping = []
        for p in self.points:
            if p != None:
                indexMapping.append((indexMapping[-1] if indexMapping else -1)+1)
                continue
            indexMapping.append((indexMapping[-1] if indexMapping else -1))

        self.points = [p for p in self.points if p != None]
        self.lines = [(indexMapping[p1], indexMapping[p2]) for (p1, p2) in self.lines if p1 != None]
        self.frontierLines = [(indexMapping[p1], indexMapping[p2]) for (p1, p2) in self.frontierLines if p1 != None]

        # self.validateData(allowNonePoint=False, allowNoneLine=False)

    def validateLine(self, pi1, pi2, allowNone=True):
        if pi1 == None:
            if not allowNone:
                raise Exception('None')
            return
        if pi1 == pi2:
            raise Exception('Line between same point')
        if pi1 > pi2:
            raise Exception('Invalid line ordering')
        if pi1 >= len(self.points) or pi2 >= len(self.points):
            raise Exception('Invalid point index')
        if self.points[pi1] == None or self.points[pi2] == None:
            raise Exception('Pointing at None point!')

    def validateData(self, allowNonePoint=True, allowNoneLine=True):
        'Debugging verktøy som validere at alle linjer og punkt er gyldige'
        if not allowNonePoint and None in self.points:
            raise Exception('NonePoint after removal')
        
        for pi1, pi2 in self.lines:
            self.validateLine(pi1, pi2, allowNone=allowNoneLine)

        for pi1, pi2 in self.frontierLines:
            self.validateLine(pi1, pi2, allowNone=allowNoneLine)

        # Sjekk for duplicates overalt
        for li, l1 in enumerate(self.lines):
            if l1[0] == None:
                continue
            for l2 in self.lines[li+1:]:
                if l1 == l2:
                    raise Exception('Duplicate lines')
        for li, l1 in enumerate(self.frontierLines):
            if l1[0] == None:
                continue
            for l2 in self.frontierLines[li+1:]:
                if l1 == l2:
                    raise Exception('Duplicate frontierLines')
        for pi, p1 in enumerate(self.points):
            if p1 == None:
                continue
            for p2 in self.points[pi+1:]:
                if p1 == p2:
                    raise Exception('Duplicate points')

    def addLinesAndLocalize(self, scan: LidarScan, localize=False, localizeBy=None, benchmark=None):
        # Korriger posisjon dersom vi e i bevegelse. 
        if len(scan.points) <= 5:
            return

        if not benchmark:
            benchmark = Benchmark()

        offset = Orientation(0.0, 0.0, 0.0)
        if localize:
            benchmark.start('Localize')
            # TODO: Er vel litt sløsing å bruk all denne tida på å generer linja, for så å bare fortsett å bruk
            # hjørnan til lokalisering? Test ut å bruk features og en par punkt for lokalisering
            self.robotPos = self.odomRobotPos.copy()
            # Ignorer første og siste featurepoint for lokalisering, for det e sansynligvis ikkje et ekte hjørne. 
            offset = tryTranslations(localizeBy or self.points, scan.featurePoints[1:-1])
            self.robotPos = self.robotPos.toReferenceFrame(offset)
            self.odomRobotPos = self.robotPos.copy()

            # Korriger punktan i scannen. 
            scan.correct(offset)

        # self.validateData()

        benchmark.start('Adding points')
        # Legg te alle punkt og linjer
        for i, (p1, p2) in enumerate(zip(scan.featurePoints, scan.featurePoints[1:])):
            isScanEnd = i == 0 or i == len(scan.featurePoints) - 2 # zip gjør lengden en kortar enn ellers
            p1Index = self.addPoint(p1, moveMapPoint=not isScanEnd)
            p2Index = self.addPoint(p2, moveMapPoint=not isScanEnd)

            if p1Index == p2Index or self.points[p1Index] == None or not scan.linesBool[i] or p1.distance(p2) < POINT_MERGE_DISTANCE:
                # Om det bli samme punkt pga nærhet eller ikkje e en linje, skip
                continue

            # Legg te linja te mappet. 
            self.addLine(p1Index, p2Index)

        # self.validateData()

        benchmark.start('Building grid')
        grid = ClosestGrid(self.points) # Points kjem ikkje te å flytt seg meir denne iterasjonen:)

        # self.validateData()
        
        benchmark.start('Merge points into lines')
        # For å unngå evig loop, vedlikehold en liste av linjan vi fjerna så dem ikkje bli lagt te igjen
        removedLines = []
        for lineIndex, line in enumerate(self.lines):
            if line[0] == None:
                continue

            p1, p2 = self.points[line[0]], self.points[line[1]]
            lineCenter = Point((p1.x+p2.x)/2, (p1.y+p2.y)/2)
            checkMargin = lineCenter.distance(p1) + POINT_LINE_MERGE_DISTANCE
            pointsWithinDistance = grid.within(lineCenter, checkMargin)

            # Å kunna skaff alle punkt innen en gitt radius raskt gjor denne metoden myy raskar. 
            for pointIndex, point in pointsWithinDistance:
                if pointIndex in line or not self.points[pointIndex]:
                    # Om det e denne linja, eller et punkt som e fjerna, skip
                    continue

                if lineDistance(p1, p2, point) < POINT_LINE_MERGE_DISTANCE:
                    removedLines.append(self.lines[lineIndex])
                    if (min(line[0], pointIndex), max(line[0], pointIndex)) in removedLines or (min(pointIndex, line[1]), max(pointIndex, line[1])) in removedLines:
                        # Om vi skal te å legg te en linje som vi fjerna tidligar, don't. 
                        continue

                    self.removeLine(lineIndex)
                    self.addLine(line[0], pointIndex)
                    self.addLine(pointIndex, line[1])
                    break # Når sammenslåing har skjedd finnes ikkje denne linja lenger, så vi må gå til neste

        # self.validateData()
        
        benchmark.start('Remove see through lines')
        for measurementPoint in [random.choice(scan.points) for _ in range(SEE_THROUGH_LINE_COUNT)]:
            for lineIndex, pointLineIndexes in enumerate(self.lines):
                if pointLineIndexes[0] != None and linesCrossing(self.robotPos, measurementPoint.toward(self.robotPos, 0, meters=SEE_THROUGH_LINE_BONUS), *[self.points[i] for i in pointLineIndexes]):
                    self.removeLine(lineIndex)

        # self.validateData()
        
        benchmark.start('Remove points from lines')
        for pointIndex, point in enumerate(self.points):
            connectedLines = [(i, l) for i, l in enumerate(self.lines) if pointIndex in l]
            connectedPointIndexes = [p for (_, l) in connectedLines for p in l if p != pointIndex]

            if len(connectedPointIndexes) != 2:
                if len(connectedPointIndexes) == 0:
                    self.removePoint(pointIndex)
                continue

            if lineDistance(self.points[connectedPointIndexes[0]], self.points[connectedPointIndexes[1]], point) < POINT_LINE_MERGE_DISTANCE:
                self.removeLine(connectedLines[0][0])
                self.removeLine(connectedLines[1][0])
                self.addLine(connectedPointIndexes[0], connectedPointIndexes[1])
                self.removePoint(pointIndex)

        # self.validateData()
        
        benchmark.start('Remove short lines')
        for lineIndex, line in enumerate(self.lines):
            if line[0] == None:
                continue
            if self.points[line[0]].distance(self.points[line[1]]) < LINE_MIN_LENGTH:
                self.removeLine(lineIndex)

        # self.validateData()
        
        benchmark.start('Add frontiers')
        for i, (p1, p2) in enumerate(zip(scan.featurePoints, scan.featurePoints[1:])):
            if scan.linesBool[i] or not (MIN_FRONTIER_WIDTH < p1.distance(p2) < MAX_FRONTIER_WIDTH):
                # Ikkje en frontier, skip
                continue

            i1, i2 = p1.mapPointIndex, p2.mapPointIndex #getattr(p1, 'mapPointIndex', None), getattr(p2, 'mapPointIndex', None)
            i1, i2 = min(i1, i2), max(i1, i2)
            if (i1, i2) in self.frontierLines:
                continue # Den finnes alt

            p1, p2 = self.points[i1], self.points[i2]

            if p1 == None or p2 == None or lineDistance(p1, p2, self.robotPos) < 1:
                # Punktet har blitt fjernet fra kartet, eller vi e for nærme linja
                continue

            if len([True for l in self.lines if i1 in l]) != 1 or \
               len([True for l in self.lines if i2 in l]) != 1:
                # En av punktan har ikke nøyaktig 1 ekte linje
                continue

            # Legg til frontier, fjern eksisterende på dem punktan. 
            self.frontierLines = [l for l in self.frontierLines if i1 not in l and i2 not in l]
            self.frontierLines.append((i1, i2))

        # self.validateData()
        
        benchmark.start('Remove frontiers')
        for i, line in enumerate(self.frontierLines):
            if line[0] == None:
                continue
            p1, p2 = self.points[line[0]], self.points[line[1]]

            if not (MIN_FRONTIER_WIDTH < p1.distance(p2) < MAX_FRONTIER_WIDTH):
                # Fjern frontiers som har en unyttig lengde
                self.frontierLines[i] = (None, None)
            elif lineDistance(p1, p2, self.robotPos) < 1:
                # Fjern forntiers som vi nærme oss
                self.frontierLines[i] = (None, None)

        # self.validateData()
        
        benchmark.start('Correct indexes')
        self.correctIndexes()

        # print(benchmark)

    def getLinePoints(self, maxGap):
        'Skaffe punkt jevnt fordelt på linjan på kartet med et maksimalt mellomrom på maxGap'
        points = set()
        for line in self.lines:
            if line[0] == None:
                continue
            p1, p2 = self.points[line[0]], self.points[line[1]]
            points.add(p1)
            points.add(p2)
            dist = p1.distance(p2)
            if dist < maxGap:
                continue
            numPoints = math.ceil(dist / maxGap)
            for i in range(1, numPoints+1):
                points.add(p1.toward(p2, i/(numPoints+1)))
        return list(points)

    def draw(self, screen):
        for p in self.points:
            pygame.draw.circle(screen, 'red', p.toReferenceFrame(self.robotPos).toScreen().xy(), 2)

        for i1, i2 in self.lines:
            pygame.draw.line(screen, 'black', self.points[i1].toReferenceFrame(self.robotPos).toScreen().xy(), self.points[i2].toReferenceFrame(self.robotPos).toScreen().xy(), 2)

        for i1, i2 in self.frontierLines:
            pygame.draw.line(screen, 'green', self.points[i1].toReferenceFrame(self.robotPos).toScreen().xy(), self.points[i2].toReferenceFrame(self.robotPos).toScreen().xy(), 2)
