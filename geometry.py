import math
import random

from benchmark import Benchmark
from consts import GRID_SLOTS_PER_METER, LOCALIZATION_POINT_MATCH_DISTANCE, MAP_SCALE, SPLIT_DISTANCE, SCREEN_SIZE

class Point:
    def __init__(self, x, y=None):
        if y == None:
            point = x
            self.x = point[0]
            self.y = point[1]
        else:
            self.x = x
            self.y = y

    def __str__(self):
        return f'Point({self.x}, {self.y})'

    def __eq__(self, other):
        if type(other) is not Point:
            return False
        return self.x == other.x and self.y == other.y

    def xy(self):
        return self.x, self.y

    def orientation(self, angle=0.0):
        return Orientation(self.x, self.y, angle=angle)

    def rotate(self, angle):
        x_new = self.x * math.cos(angle) - self.y * math.sin(angle)
        y_new = self.x * math.sin(angle) + self.y * math.cos(angle)
        return Point(x_new, y_new)

    def rotateAround(self, angle, point):
        # print(self, self.toReferenceFrame(point.orientation()), self.toReferenceFrame(point.orientation()).rotate(angle), self.toReferenceFrame(point.orientation()).rotate(angle).fromReferenceFrame(point.orientation()))
        return self.toReferenceFrame(point.orientation()).rotate(angle).fromReferenceFrame(point.orientation())

    def origoAngle(self):
        return math.atan2(self.y, self.x)

    def add(self, point):
        return Point(self.x + point.x, self.y + point.y)

    def scale(self, scale):
        return Point(self.x * scale, self.y * scale)

    def toward(self, point, percent=0.5, meters=0):
        '''
        Returne et nytt punkt mellom dette og et anna punkt, med prosentandel for å si kor. 
        Kan evt også flytt n meter i retning et anna punkt. 
        '''
        if meters:
            return self.add(point.add(-self).scale(percent)).add(unitCirclePoint(point.add(-self).origoAngle()).scale(meters))
        return self.add(point.add(-self).scale(percent))

    def __neg__(self):
        return Point(-self.x, -self.y)

    def distance(self, point):
        return math.sqrt((self.x-point.x)**2 + (self.y-point.y)**2)
    
    def angularDistance(self, point):
        return abs(self.origoAngle() - point.origoAngle())

    # REFERANSERAMME
    def toReferenceFrame(self, orientation):
        return self.add(-orientation.point()).rotate(-orientation.angle)

    def fromReferenceFrame(self, orientation):
        return self.rotate(orientation.angle).add(orientation.point())

        #return self.rotate(-frame[2]).add(Point(frame[:2]))

    def toScreen(self):
        return Point(toScreen(*self.xy()))

    def fromScreen(self):
        return Point(fromScreen(*self.xy()))


class Orientation(Point):
    'Punkt og retning'
    def __init__(self, x, y=None, angle=0.0):
        if y == None:
            orientation = x
            self.x = orientation[0]
            self.y = orientation[1]
            self.angle = orientation[2]
        else:
            self.x = x
            self.y = y
            self.angle = angle
    
    def __str__(self):
        return f'Orientation({self.x}, {self.y}, {self.angle})'
    
    def __eq__(self, other):
        if type(other) is not Orientation:
            return False
        return self.x == other.x and self.y == other.y and self.angle == other.angle

    def xya(self):
        return self.x, self.y, self.angle
    
    def point(self):
        return Point(self.x, self.y)
    
    def add(self, orientation):
        angle = 0.0
        if isinstance(orientation, Orientation):
            angle = orientation.angle
        return Orientation(self.x + orientation.x, self.y + orientation.y, self.angle + angle)

    def rotate(self, angle):
        return Orientation(*super().rotate(angle).xy(), self.angle + angle)
    
    def toReferenceFrame(self, orientation):
        return Orientation(*super().toReferenceFrame(orientation).xy(), angle=self.angle - orientation.angle)

    def fromReferenceFrame(self, orientation):
        return Orientation(*super().fromReferenceFrame(orientation).xy(), angle=self.angle + orientation.angle)


class Line:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
    
    def __str__(self):
        return f'Line({self.p1}, {self.p2})'
    
    def closestPointOnLine(self, p, bounded=True):
        return closestPointOnLine(self.p1, self.p2, self.p3, bounded=bounded)
    
    def lineDistance(self, p1, p2, p3, bounded=True):
        return lineDistance(p1, p2, p3, bounded=bounded)
    
    @property
    def points(self):
        return [self.p1, self.p2]


def unitCirclePoint(angle):
    return Point(1, 0).rotate(angle)


def getUnitPointFromAngle(index, rangesLength, correction=0):
    'Returns a (x, y) touple on the unit circle from an index in the Lidar list'
    return unitCirclePoint((2 * math.pi * index / rangesLength) - math.pi + correction) # Den første målingen e rett bakover


def closestPointOnLine(p1, p2, p3, bounded=True):
    'Det nærmeste punktet til p3 på linja mellom p1 og p2'
    t = ((p3.x - p1.x) * (p2.x - p1.x) + (p3.y - p1.y) * (p2.y - p1.y)) / ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

    if bounded:
        t = min(max(0, t), 1)

    return Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y))


def lineDistance(p1, p2, p3, bounded=True, checkEqual=True):
    'Avstanden fra p3 til linja mellom p1 og p2'
    if checkEqual and p1 == p2:
        # Om linjepunktan e samme sted, return avstanden direkte. 
        return p1.distance(p3)
    if bounded:
        return closestPointOnLine(p1, p2, p3, bounded=True).distance(p3)
    return abs((p2.y - p1.y) * p3.x - (p2.x - p1.x) * p3.y + p2.x * p1.y - p2.y * p1.x) \
        / p1.distance(p2)


def angleFromPoints(p1, p2, p3):
    'Finn vinkelen p1, p2, p3. (med p1 som høyre vinkelbein)'
    p1Angle = Point(p1.x-p2.x, p1.y-p2.y).origoAngle()
    p3Angle = Point(p3.x-p2.x, p3.y-p2.y).origoAngle()
    return (p3Angle - p1Angle) % (2 * math.pi)


def toScreen(x, y):
    'Konvertere fra robotens referanseramme der x e framover, til skjermen der negativ y e framover, der vi også sentrere på bilen'
    # TODO: Skriv om til å bruk toReferenceFrame
    return SCREEN_SIZE/2 - y * MAP_SCALE, SCREEN_SIZE/2 - x * MAP_SCALE


def fromScreen(x, y):
    # TODO: Skriv om til å bruk toReferenceFrame
    return (-y + SCREEN_SIZE/2) / MAP_SCALE, (-x + SCREEN_SIZE/2) / MAP_SCALE


# Her prøvd e meg på å implementer en raskar versjon av split and merge, men det va vanskelig. 
# Dette e et forsøk på å bruk distansen mellom nærliggende punkt, men det ble ikkje nyttig. 
# def featuresByAngle(points):
#     pointIndexes = [(i, p) for (i, p) in enumerate(points)]
#     deg1, deg2, deg3, deg4 = [False, False], [False, False], [False, False], [False, False]
#     for i in range(2, len(pointIndexes)-2):
#         # pm2, pm1, p, p1, p2 = pointIndexes[i-2][1], pointIndexes[i-1][1], pointIndexes[i][1], pointIndexes[i+1][1], pointIndexes[i+2][1]
#         deg1.append(abs(abs(pointIndexes[i-1][1].add(-pointIndexes[i][1]).origoAngle()) + abs(pointIndexes[i+1][1].add(-pointIndexes[i][1]).origoAngle()) - math.pi) > math.pi / 4)
#         deg2.append(abs(abs(pointIndexes[i-2][1].add(-pointIndexes[i][1]).origoAngle()) + abs(pointIndexes[i+2][1].add(-pointIndexes[i][1]).origoAngle()) - math.pi) > math.pi / 2)
#         deg3.append(points[i].scale(-2).add(points[i-1]).add(points[i+1]).distance(Point(0, 0)) / 2 / points[i-1].distance(points[i+1]) > 0.45)
#         deg4.append(points[i].scale(-2).add(points[i-2]).add(points[i+2]).distance(Point(0, 0)) / 2 / points[i-2].distance(points[i+2]) > 0.5)
#         # if None not in points[i-1:i+1] and points[i].scale(2).add(-points[i-1]).add(-points[i+1]).distance(Point(0, 0)) < 0.05:
#         #     deg3.append(None)
#     deg1.extend([False, False])
#     deg2.extend([False, False])
#     deg3.extend([False, False])
#     deg4.extend([False, False])
#     print(len([b for b in deg1 if b]), len([b for b in deg2 if b]), len([b for b in deg3 if b]), len([b for b in deg4 if b]))
#     # # splitForDistance = pointIndexes[i-1][1].distance(pointIndexes[i+1][1]) > 0.3 # En 30 cm jump telles som en feature.
#     # if splitForDegree or splitForDegree2:# or (iteration==0 and splitForDistance): # 20 grader
#     #     newPointIndexes.append(pointIndexes[i])
#     pointIndexes = [(i, p) for (i, p) in pointIndexes if deg4[i]]
#     return [i for (i, p) in pointIndexes], [p for (i, p) in pointIndexes]


def splitAndMerge(points, distanceParameter=None):
    '''
    Kjøre split and merge med grunnleggende distanceParameter. Funke, men ikkje forferdelig effektivt.
    Vi burda ha prøvd oss på en meir tilpassningdyktig distanseberegning, typ om mange punkt på rad e langt unna linja,
    ska det ha meir å si enn om bare ett punkt e det. 
    '''
    if len(points) < 3:
        return [], []

    benchmark = Benchmark()

    if distanceParameter == None:
        distanceParameter = SPLIT_DISTANCE

    points = list(points)

    # Indeksan te punktan
    splitIndexes = [0, len(points)-1]

    benchmark.start('Split')

    # Split
    # Ish det e vil gjør: Ta linja fra, finn det største avviket, legg inn det i indexes. 
    # Etter hver split, repeter for hver halvdel inntil vi har splitta det greit. 
    while True:
        newSplitIndexes = []
        for startPointIndex, endPointIndex in zip(splitIndexes, splitIndexes[1:]):
            # For alle påfølgende par med indeksa i punktan
            maxDistIndex, maxDist = 0, 0
            p1, p2 = points[startPointIndex], points[endPointIndex]
            # Regn ut linje parametran på utsida for å gjør det raskar
            p1p2Dist = p1.distance(p2)
            xDiff = p2.x - p1.x
            yDiff = p2.y - p1.y
            x2y1Minusy2x1 = p2.x * p1.y - p2.y * p1.x
            for pointIndex in range(startPointIndex+1, endPointIndex):
                # For hvert punkt mellom dem 2 punktan

                # Inlined utregning for å gjør det raskar.
                dist = abs(yDiff * points[pointIndex].x - xDiff * points[pointIndex].y + x2y1Minusy2x1) / p1p2Dist
                if dist > maxDist:
                    maxDistIndex, maxDist = pointIndex, dist

            # Om det e over en threshold dele vi. 
            if maxDist > distanceParameter:
                newSplitIndexes.append(maxDistIndex)

        if not newSplitIndexes:
            break
        splitIndexes.extend(newSplitIndexes)
        splitIndexes.sort()

    benchmark.start('Merge')

    # Merge
    # My det samme, bare at vi sjekke at hver av indeks punktan e en viss avstand fra linja mellom nabopunktan.
    # Om ikkje fjerne vi det punktet. 
    # TODO: En bug med dette e at rekkefølgen har noko å si, vi burda istedet begyn med dem mest avvikende. 
    i = 0
    while i < len(splitIndexes)-3:
        if lineDistance(points[splitIndexes[i]], points[splitIndexes[i+2]], points[splitIndexes[i+1]], bounded=False) < distanceParameter:
            splitIndexes.pop(i)
        else:
            i += 1

    benchmark.stop()
    # print(benchmark)

    return splitIndexes, [points[i] for i in splitIndexes]


def closestPoint(point, points, different=True):
    '''
    Skaffe det nærmeste punktet og index ved å gå gjennom alle punktan.
    different er om vi skal tell med like punkt (med lik x og y)
    '''
    bestDist = 100 # 100 meter altså
    bestPointIndex, bestPoint = None, None
    for i, p in enumerate(points):
        if p == None:
            continue
        if point.distance(p) < bestDist:
            if different and p == point:
                # Unngå å gi oss dette punktet om det e i points lista. 
                continue
            bestDist = point.distance(p)
            bestPointIndex, bestPoint = i, p
    return bestPointIndex, bestPoint


class ClosestGrid:
    'Indeksere points på et grid slik at vi kan gjør oppslag på nærmeste punkt'
    # Denne e ca 10 gong raskar enn naivt closestPoint om man tune den greit:)
    # Er en balanse mellom å ha lite å sjekk, og å ikkje måtta sjekk for mange felt når det e langt til nærmeste punkt. 
    def __init__(self, points, slotsPerMeter=None, *args, **kwargs):
        if slotsPerMeter:
            self.slotsPerMeter = slotsPerMeter # 2 slots per meter medføre 4 ruta i en kvadratmeter
        else:
            self.slotsPerMeter = GRID_SLOTS_PER_METER
        self.grid = {} # dict med liste av enumerate element. 
        for i, point in enumerate(points):
            if point == None:
                continue
            key = self.getKey(point)
            if key not in self.grid:
                self.grid[key] = [(i, point)]
            else:
                self.grid[key].append((i, point))

    def getKey(self, point: Point):
        return (round(point.x*self.slotsPerMeter), round(point.y*self.slotsPerMeter))

    def within(self, point: Point, distance):
        'Returne minst alle punkt innen distance fra punktet'
        key = self.getKey(point)
        gridDist = distance * self.slotsPerMeter
        xMin, yMin, xMax, yMax = key[0] - gridDist, key[1] - gridDist, key[0] + gridDist, key[1] + gridDist
        # Her må e faktisk bare skaff keys som vi har i griddet, om ikkje bli det veldig tregt for store radiusa
        keys = [key for key in self.grid.keys() if xMin <= key[0] <= xMax and yMin <= key[1] <= yMax]
        return self.getGridContents(keys)
    
    def keyWithinGridDist(self, key, gridDist):
        xMin, yMin, xMax, yMax = key[0] - gridDist, key[1] - gridDist, key[0] + gridDist, key[1] + gridDist
        # Her må e faktisk bare skaff keys som vi har i griddet, om ikkje bli det veldig tregt for store radiusa
        keys = [key for key in self.grid.keys() if xMin <= key[0] <= xMax and yMin <= key[1] <= yMax]
        return self.getGridContents(keys)

    def getGridContents(self, keys):
        return [i for key in keys for i in self.grid.get(key, [])]

    def closest(self, point: Point):
        key = self.getKey(point)
        closestIndex, closestPoint, closestDistance = None, None, 100
        if len(self.grid) < 3:
            return None, None
        for layer in range(1, 100):
            for i, p in self.keyWithinGridDist(key, layer**2):
                if p == point:
                    continue
                if point.distance(p) < closestDistance:
                    closestIndex, closestPoint, closestDistance = i, p, point.distance(p)

            if closestDistance * self.slotsPerMeter < layer:
                # Da e dette garantert nærmeste punkt, fordi vi no har sjekka alle punkt som kan vær innen radiusen
                # Kunna optimalisert det meir og sjekka færre felt ved å sjå på den nærmeste "kanten", men det bli bare knot
                break

        return closestIndex, closestPoint


def tryTranslations(oldPoints, newPoints):
    'Prøv deg fram til transformasjoner som gjør at points stemme bedre'

    totalTransform = Orientation(0.0, 0.0, 0.0)
    pointMatches = []
    for p in newPoints:
        _, closest = closestPoint(p, oldPoints)
        if not closest:
            continue
        elif p.distance(closest) < LOCALIZATION_POINT_MATCH_DISTANCE:
            pointMatches.append((p, closest))

    if len(pointMatches) < 3:
        print(f'GOT LOST!, {len(oldPoints)} {len(newPoints)}')
        # Om veldig få points matche, ikkje transformer shit!
        return totalTransform

    distances = [p1.toReferenceFrame(totalTransform).distance(p2) ** 2 for p1, p2 in pointMatches]
    # averageDist = math.sqrt(sum(distances)/len(distances))
    # Dette ^ e det vi egentlig sammenligne med, altså snittet av kvadratet av distansan. 
    # Men til seinar i loopen e antall distansa punktpar det samme, og kvadratrot er ikke nødvendig for sammenligningen. 
    # Derfor gjør vi istedet følgende, så vi kan hopp ut av indre loopen under, så fort som mulig:)
    dist = sum(distances)

    initialDist = dist

    for j in range(1, 20, 1):
        # Velg en tilfeldig transformasjon, men med stadig mindre bound på tilfeldigheten. Går gradvis 25 cm pr loop til 0.5 cm pr loop. 
        newTransform = Orientation((random.random() - 0.5) / 10 / j, (random.random() - 0.5) / 10 / j, (random.random() - 0.5) / 5 / j)
        newTotalTransform = totalTransform.add(newTransform)
        # newTotalTransform = [t1+t2 for t1, t2 in zip(totalTransform, newTransform)]

        newDist = 0
        for p1, p2 in pointMatches:
            newDist += p1.toReferenceFrame(newTotalTransform).distance(p2) ** 2
            if newDist > dist:
                break # Da kan vi ikkje få te nå bedre denne iterasjonen

        if newDist < dist:
            dist = newDist
            totalTransform = newTotalTransform

    # print(totalTransform, 'improved distance by', round(math.sqrt(initialDist)/len(pointMatches) - math.sqrt(dist)/len(pointMatches), 5))
    return totalTransform


def straightWheelDriver(point: Point):
    ''''
    For et punkt relativt til bilen gir dette turn radius som gjør at framhjulan peke på det punktet. 
    Speed / Turn radius = turn speed:)
    Turn radiusen e te venstre, så om man ska sving te høyre e turn radius negativ. 

    Punktet må vær i bilens referanseramme!
    '''

    if point.y == 0:
        return math.inf

    p = point
    if p.x < 0.2:
        p = Point(0.2, p.y)

    wheelAngle = p.add(Point(-0.2, 0)).origoAngle()
    # wheelAngle = p.origoAngle()

    # Vinkelen trekanten av sirkel sentrum, bakre hjul og midt mellom framre hjul. 
    innerWheelAngle = math.pi - (wheelAngle + math.copysign(math.pi / 2, wheelAngle))

    turnRadius = abs(math.tan(innerWheelAngle) * 0.2)

    if p.y > 0:
        return max(turnRadius, 0.4)
    else:
        return min(-turnRadius, -0.4)


def circularWheelDriver(point: Point):
    '''
    Samme som over, bare at denne ikkje skal pek forhjula på punktet. I stedet skal denne gi turning radius som 
    resultere i at vi når det punktet om vi held den turning radiusen. 
    '''
    if point.y == 0:
        return math.inf

    # Formelen for en sirkel tilsie at dette bli (x**2 + y**2) / 2y, og e bekrefte visuelt at det stemme. 

    if point.y > 0:
        return max((point.x**2 + point.y**2) / (2*point.y), 0.4)
    else:
        return min((point.x**2 + point.y**2) / (2*point.y), -0.4)


def pointIndexClosestToAngle(points, angle):
    # Anntar at punktan e sortert fra stor til liten vinkel, hvilket stemme for lidar sensoren. 
    bestPointIndexMin, bestPointIndexMax = 0, len(points) - 1

    while bestPointIndexMax - bestPointIndexMin > 1:
        bestPointIndex = int((bestPointIndexMax + bestPointIndexMin) / 2)
        # Merk at Lidaren gir oss punktan fra stor til lav vinkel (fra venstre mot høyre)
        if points[bestPointIndex].origoAngle() > angle:
            bestPointIndexMax = bestPointIndex
        else:
            bestPointIndexMin = bestPointIndex

    return bestPointIndexMin
    
    # measurementPoint = points[int((bestPointIndexMax + bestPointIndexMin) / 2)].toReferenceFrame(robotPos)


def ccw(p1, p2, p3):
    return (p3.y-p1.y) * (p2.x-p1.x) > (p2.y-p1.y) * (p3.x-p1.x)


def linesCrossing(p1, p2, p3, p4):
    'Returne bool om linja fra p1 til p2 og linja fra p3, til p4 krysse hverandre.'
    # Bygge på at om linjan krysse må det vær en firkant. 
    # Håndtere ikkje colinearity, men det treng ikkje vi pr no. 
    # Se https://stackoverflow.com/a/9997374
    return ccw(p1,p3,p4) != ccw(p2,p3,p4) and ccw(p1,p2,p3) != ccw(p1,p2,p4)
