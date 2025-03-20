import math

from geometry import angleFromPoints, lineDistance, unitCirclePoint

class Turn:
    def __init__(self, startPos, turnRadius, distance):
        '''
        Positiv turnRadius e te venstre. For rett fram bruk turnRadius = math.inf
        Positiv distance e framover. 
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
            distance = min(max(0, distance), self.distance) if self.distance > 0 else min(max(self.distance, distance), 0)

        if math.isinf(self.turnRadius):
            return self.startPos.add(unitCirclePoint(self.startPos.angle).scale(math.copysign(distance, self.distance)))
        else:
            return self.startPos.rotateAround(math.copysign(distance, self.distance) / self.turnRadius , self.turnCenter)

    def free(self, points, margin=0.25):
        'Om det e nån punkt i points som bilen kræsje med. Margin e kor my klaring vi ønske for bilen på hver side'
        # For dette trur e vi bare treng å filtrer ut dem punktan som e i retning den anndelen av sirkelen vi kjøre i, 
        # for så å sjekk at avstanden fra sentrum av sirkelen e minst margin differanse fra radius av sirkelen. 
        minRadius = abs(self.turnRadius) - margin
        maxRadius = abs(self.turnRadius) + margin

        if math.isinf(self.turnRadius):
            for point in points:
                if lineDistance(self.startPos, self.endPos, point) < margin:
                    return False
            return True

        for point in points:
            # Sjekk at avstand fra turn center stemme ish
            if not (minRadius < point.distance(self.turnCenter) < maxRadius):
                continue

            # Sjekk at punktet e på rett del av sirkelen
            if (self.turnRadius > 0) == (self.distance > 0):
                angle = angleFromPoints(self.startPos, self.turnCenter, point)
            else:
                angle = angleFromPoints(point, self.turnCenter, self.startPos)

            if 0 < angle < (abs(self.distance) + margin) / abs(self.turnRadius):
                return False

        return True


class TurnManager():
    'Håndtere konverteringen av en turn sequence til et punkt man kan følg etter.'
    def __init__(self, turns=[]):
        self.turns = turns

    @property
    def distance(self):
        return sum([abs(t.distance) for t in self.turns])

    def currTurn(self, distance):
        if not self.turns:
            return None

        distanceRemaining = distance

        turn = None
        for i, t in enumerate(self.turns):
            turn = t
            if distanceRemaining - abs(t.distance) < 0 or t == self.turns[-1]:
                break
            distanceRemaining -= abs(t.distance)
        return turn

    def partway(self, distance, ahead=0.3, clamp=True):
        'Call partway for den turnen som e på den distancen'
        turn = self.currTurn(distance)
        if turn:
            return turn.partway(distance - sum([abs(t.distance) for t in self.turns[0:self.turns.index(turn)]]) + ahead, clamp=clamp)


def rateTurns(goalPos, turns):
    return sum([abs(t.distance) for t in turns]) + goalPos.distance(turns[-1].endPos)


def AStar(startPos, goalPos, obstacles, stepLength=0.3, stepAngles=5):
    '''
    Returne en liste av påfølgende Turns, som kan utføres for å nå en posisjon med A*, uten driftkorrigering. 
    Om vi replanne ofte nok burda det ikkje bli et problem. 
    La stepAngles vær et oddetall så vi får med kjøring rett fram og bak:) 
    '''
    turnRadiusAndStep = []
    for i in range(-(stepAngles//2), stepAngles//2+1):
        # Konverter fra i=-n...n til turning radius slik at vi får en lineær endring i turning speed
        turnRadiusAndStep.append((stepAngles//2 * 0.4 / i if i != 0 else math.inf, stepLength))
        turnRadiusAndStep.append((stepAngles//2 * 0.4 / i if i != 0 else math.inf, -stepLength))

    turnSequences = []
    turnSequencesRating = []

    for _ in range(100):
        # Legg til dem 5 nye turn sekvensan, i lista, og fjern den vi jobbe fra
        if turnSequences:
            turnSequence = turnSequences.pop(0)
            turnSequencesRating.pop(0)
        else:
            turnSequence = []

        for turnRadius, step in turnRadiusAndStep:
            if turnSequence:
                turn = Turn(turnSequence[-1].endPos, turnRadius, step)
            else:
                turn = Turn(startPos, turnRadius, step)

            if not turn.free(obstacles):
                continue

            turnSeq = turnSequence + [turn]

            turnRating = rateTurns(goalPos, turnSeq)

            # Sett inn på rett plass, for å slepp å sorter heile greia hver gong:)
            for seqIndex in range(len(turnSequences)+1):
                if seqIndex == len(turnSequences):
                    turnSequences.append(turnSeq)
                    turnSequencesRating.append(rateTurns(goalPos, turnSeq))
                    break

                if turnSequencesRating[seqIndex] < turnRating:
                    continue

                turnSequences.insert(seqIndex, turnSeq)
                turnSequencesRating.insert(seqIndex, rateTurns(goalPos, turnSeq))
                break

        if goalPos.distance(turnSequences[0][-1].endPos) < stepLength * 2/3:
            break

    return turnSequences[0] if turnSequences else None
