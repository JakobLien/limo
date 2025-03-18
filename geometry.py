import math
import random


class Point:
    def __init__(self, x, y=None, data=None):
        if y == None:
            point = x
            self.x = point[0]
            self.y = point[1]
        else:
            self.x = x
            self.y = y
        self.data = {}

    def __str__(self):
        return f'Point({self.x}, {self.y})'

    def xy(self):
        return self.x, self.y
    
    def orientation(self, angle=0.0):
        return Orientation(self.x, self.y, angle=angle)

    def setData(self, key, value):
        p = Point(self.x, self.y)
        p.data = self.data.copy()
        p.data[key] = value
        return p

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

    def toward(self, point, percent=0.5):
        'Returne et nytt punkt mellom dette og et anna punkt, med prosentandel for å si kor'
        return self.add(point.add(-self).scale(percent))

    def __neg__(self):
        return Point(-self.x, -self.y)

    def distance(self, point):
        return math.sqrt((self.x-point.x)**2 + (self.y-point.y)**2)

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
    def __init__(self, x, y=None, angle=0.0, data=None):
        if y == None:
            orientation = x
            self.x = orientation[0]
            self.y = orientation[1]
            self.angle = orientation[2]
        else:
            self.x = x
            self.y = y
            self.angle = angle
        self.data = data
    
    def __str__(self):
        return f'Orientation({self.x}, {self.y}, {self.angle})'
    
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


def unitCirclePoint(angle):
    return Point(1, 0).rotate(angle)


def getUnitPointFromAngle(index):
    'Returns a (x, y) touple on the unit circle from an index in the Lidar list'
    return unitCirclePoint(2 * math.pi * (index + 215) / 430) # Den første målingen e rett bakover


def closestPointOnLine(p1, p2, p3, bounded=True):
    'Det nærmeste punktet til p3 på linja mellom p1 og p2'
    t = ((p3.x - p1.x) * (p2.x - p1.x) + (p3.y - p1.y) * (p2.y - p1.y)) / ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

    if bounded:
        t = min(max(0, t), 1)

    return Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y))


def lineDistance(p1, p2, p3, bounded=True):
    'Avstanden fra p3 til linja mellom p1 og p2'
    if bounded:
        # Vi kjøre pytagoras for å sjekk om det e nærmast linja eller endepunktan. 
        # For p1 som hjørne
        if p1.distance(p2)**2 + p1.distance(p3)**2 < p2.distance(p3)**2:
            return p1.distance(p3)

        # For p2 som hjørne
        if p2.distance(p1)**2 + p2.distance(p3)**2 < p1.distance(p3)**2:
            return p2.distance(p3)

    if p1.distance(p2) == 0:
        # Om linjepunktan e samme sted, return avstanden direkte. 
        return p1.distance(p3)
    return abs((p2.y - p1.y) * p3.x - (p2.x - p1.x) * p3.y + p2.x * p1.y - p2.y * p1.x) \
        / p1.distance(p2)


def angleFromPoints(p1, p2, p3):
    'Finn vinkelen p1, p2, p3. (med p1 som høyre vinkelbein)'
    p1Angle = Point(p1.x-p2.x, p1.y-p2.y).origoAngle()
    p3Angle = Point(p3.x-p2.x, p3.y-p2.y).origoAngle()
    return p3Angle - p1Angle


def toScreen(x, y):
    'Konvertere fra robotens referanseramme der x e framover, til skjermen der negativ y e framover, der vi også sentrere på bilen'
    # TODO: Skriv om til å bruk toReferenceFrame
    return 300 - y * 100, 300 - x * 100


def fromScreen(x, y):
    # TODO: Skriv om til å bruk toReferenceFrame
    return (-y + 300) / 100, (-x + 300) / 100


def splitAndMerge(points, distanceParameter=0.1, minPoints=3):
    '''
    Kjøre split and merge med grunnleggende distanceParameter. Funke, men ikkje forferdelig effektivt.
    Vi burda ha prøvd oss på en meir tilpassningdyktig distanseberegning, typ om mange punkt på rad e langt unna linja,
    ska det ha meir å si enn om bare ett punkt e det. 
    '''
    # Indeksan te punktan
    indexes = [0, len(points)-1]

    # Split
    # Ish det e vil gjør: Ta linja fra, finn det største avviket, legg inn det i indexes. 
    # Etter hver split, repeter for hver halvdel inntil vi har splitta det greit. 
    indexesLen = 0
    while indexesLen != len(indexes):
        indexesLen = len(indexes)
        for indexIndex in range(len(indexes)-1):
            # Gå over mellom alle indeksan (altså fra punkt til punkt)
            maxDistIndex, maxDist = 0, 0
            # For hvert punkt mellom dem 2 punktan
            for i in range(indexes[indexIndex]+1, indexes[indexIndex+1]):
                startLinePoint = points[indexes[indexIndex]]
                endLinePoint = points[indexes[indexIndex+1]]
                currPoint = points[i]

                dist = lineDistance(startLinePoint, endLinePoint, currPoint)
                if dist > maxDist:
                    maxDistIndex, maxDist = i, dist

            # Om det e over en threshold dele vi. 
            if maxDist > distanceParameter:
                indexes.insert(indexIndex+1, maxDistIndex)

    # Merge
    # My det samme, bare at vi sjekke at hver av indeks punktan e en viss avstand fra linja mellom nabopunktan.
    # Om ikkje fjerne vi det punktet. 
    # TODO: En bug med dette e at rekkefølgen har noko å si, vi burda istedet begyn med dem mest avvikende. 
    i = 0
    while i < len(indexes)-3:
        if lineDistance(points[indexes[i]], points[indexes[i+2]], points[indexes[i+1]]) < distanceParameter:
            indexes.pop(i)
        else:
            i += 1

    return [p for i, p in enumerate(points) if i in indexes]


def closestPointIndex(point, points):
    bestDist = 100 # 100 meter altså
    bestPointIndex = None
    for i, p in enumerate(points):
        if point.distance(p) < bestDist:
            bestDist = point.distance(p)
            bestPointIndex = i
    return bestPointIndex


def closestPoint(point, points):
    i = closestPointIndex(point, points)
    if i:
        return points[i]

def tryTranslations(oldPoints, newPoints):
    'Prøv deg fram til transformasjoner som gjør at points stemme bedre'

    totalTransform = Orientation(0.0, 0.0, 0.0)
    pointMatches = []
    for p in newPoints:
        closest = closestPoint(p, oldPoints)
        if not closest:
            continue
        elif p.distance(closest) < 0.1:
            pointMatches.append((p, closest))

    if len(pointMatches) < 3:
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
