import math


class Point:
    def __init__(self, x, y=None):
        if y == None:
            point = x
            self.x = point[0]
            self.y = point[1]
        else:
            self.x = x
            self.y = y

    def xy(self):
        return self.x, self.y

    def rotate(self, angle):
        x_new = self.x * math.cos(angle) + self.y * math.sin(angle)
        y_new = -self.x * math.sin(angle) + self.y * math.cos(angle)
        return Point(x_new, y_new)
    
    def add(self, point):
        return Point(self.x + point.x, self.y + point.y)

    def scale(self, scale):
        return Point(self.x * scale, self.y * scale)
    
    def distance(self, point):
        return math.sqrt((self.x-point.x)**2 + (self.y-point.y)**2)

    def toReferenceFrame(self, frame):
        return self.add(Point(frame[:2]).scale(-1)).rotate(frame[2])

    def fromReferenceFrame(self, frame):
        return self.rotate(-frame[2]).add(Point(frame[:2]))

    def toScreen(self):
        return Point(300 - self.y * 100, 300 - self.x * 100)


def getUnitPointFromAngle(i, theta=0):
    'Returns a (x, y) touple on the unit circle from an index in the Lidar list'
    return Point(1, 0).rotate(-2 * math.pi * (i + 215) / 430) # Den første målingen e rett bakover


def lineDistance(p1, p2, p3):
    'Avstanden fra p3 til linja som går mellom p1 og p2'
    if min(p1.distance(p3), p2.distance(p3)) > p1.distance(p2):
        # Om den e forbi linja, heller return avstanden til endepunktan. 
        return min(p1.distance(p3), p2.distance(p3))
    if p1.distance(p2) == 0:
        # Om linjepunktan e samme sted, return avstanden direkte. 
        return p1.distance(p3)
    return abs((p2.y - p1.y) * p3.x - (p2.x - p1.x) * p3.y + p2.x * p1.y - p2.y * p1.x) \
        / p1.distance(p2)


def toScreen(x, y):
    'Konvertere fra robotens referanseramme der x e framover, til skjermen der negativ y e framover, der vi også sentrere på bilen'
    # TODO: Skriv om til å bruk toReferenceFrame
    return 300 - y * 100, 300 - x * 100


def splitAndMerge(points, distanceParameter=0.2):
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

    return indexes
