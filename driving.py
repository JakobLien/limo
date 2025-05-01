import math
import random
from typing import List

from geometry import Point, angleFromPoints, lineDistance, unitCirclePoint

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
            distance = min(max(0, distance), self.distance) if self.distance > 0 else min(max(self.distance, distance), 0)

        if math.isinf(self.turnRadius):
            return self.startPos.add(unitCirclePoint(self.startPos.angle).scale(math.copysign(distance, self.distance)))
        else:
            return self.startPos.rotateAround(math.copysign(distance, self.distance) / self.turnRadius , self.turnCenter)

    def free(self, points, margin=0.25):
        '''
        Om det e nån punkt i points som bilen kræsje med. Margin e kor my klaring vi ønske for bilen på hver side. 
        Returne en touple med en boolean om vi har åpen bane, og evt hvilket punkt vi kræsja i. 
        '''
        # For dette trur e vi bare treng å filtrer ut dem punktan som e i retning den anndelen av sirkelen vi kjøre i, 
        # for så å sjekk at avstanden fra sentrum av sirkelen e minst margin differanse fra radius av sirkelen. 
        minRadius = abs(self.turnRadius) - margin
        maxRadius = abs(self.turnRadius) + margin

        if math.isinf(self.turnRadius):
            for point in points:
                if lineDistance(self.startPos, self.endPos, point) < margin:
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

            if 0 < angle < (abs(self.distance) + margin) / abs(self.turnRadius):
                return (False, point)

        return (True, None)

    # # Greier for å debug turn sin free metode. 
    # testTurn = Turn(Orientation(0, 0, 0), 1, -0.5)
    # pygame.draw.circle(screen, "red", testTurn.turnCenter.toScreen().xy(), 10)
    # pygame.draw.circle(screen, "blue", testTurn.endPos.toScreen().xy(), 10)
    # if testTurn.turnCenter:
    #     pygame.draw.circle(screen, "red", testTurn.turnCenter.toScreen().xy(), abs(testTurn.turnRadius)*100, width=1)

    # for x in range(-30, 30):
    #     for y in range(-30, 30):
    #         pnt = Point(x/20, y/20)
    #         if testTurn.free([pnt]):
    #             pygame.draw.circle(screen, "green", pnt.toScreen().xy(), 1)
    #         else:
    #             pygame.draw.circle(screen, "red", pnt.toScreen().xy(), 1)


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


def AStar(startPos, goalPos: Point, obstacles: List[Point], stepLength=0.3, stepAngles=5, minTurnRadius=0.45):
    '''
    Returne en liste av påfølgende Turns, som kan utføres for å nå en posisjon med A*, uten driftkorrigering. 
    Om vi replanne ofte nok burda det ikkje bli et problem. 
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

    # Vi shuffle randomly for å raskar kunna kræsj i ting. Dersom mange punkt som hadd forhindra svingen lønne
    # det seg å sjå på dem i tilfeldig rekkefølge heller enn i f.eks. scan rekkefølge. 
    random.shuffle(obstacles)

    for _ in range(200):
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

            if not turn.free(obstacles, margin=0.25)[0]:
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

        if goalPos.distance(turnSequences[0][-1].endPos) < stepLength * 2/3:
            break

    return turnSequences[0] if turnSequences else None
