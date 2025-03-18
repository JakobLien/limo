import math

from geometry import angleFromPoints, lineDistance, unitCirclePoint

class Turn:
    def __init__(self, startPos, turnSpeed, speed=0.1, duration=3):
        'Så leng vi held oss på speed=0.1 e max turnSpeed i teorien på 0.1/0.4=0.25'
        # Kunna også tatt duration som argument? Hmm
        if duration <= 0:
            raise Exception('Must have positive duration.')
        
        if speed <= 0:
            raise Exception('Must have positive speed')

        self.startPos = startPos
        self.turnSpeed = turnSpeed
        self.speed = speed
        self.duration = duration

        if self.turnSpeed == 0:
            self.turnRadius = math.inf
        else:
            self.turnRadius = abs(self.speed) / abs(self.turnSpeed)

        self.arcLength = self.duration * abs(self.speed)

        self.arcAngle = self.arcLength / self.turnRadius

        # Om vi ikkje svinge e det ingen sentrum av svingen
        if self.turnSpeed == 0:
            self.turnCenter =  None
        elif self.turnSpeed > 0:
            # Return punktet 90 grader direkte til venstre for roboten. 
            self.turnCenter = self.startPos.add(unitCirclePoint(self.startPos.angle + math.pi / 2).scale(self.turnRadius))
        else:
            # Return punktet 90 grader direkte til høyre for roboten. 
            self.turnCenter = self.startPos.add(unitCirclePoint(self.startPos.angle - math.pi / 2).scale(self.turnRadius))

        if self.turnSpeed == 0:
            self.endPos = self.startPos.add(unitCirclePoint(self.startPos.angle).scale(self.arcLength))
        elif self.turnSpeed > 0:
            self.endPos = self.startPos.rotateAround(self.arcAngle, self.turnCenter)
        else:
            self.endPos = self.startPos.rotateAround(-self.arcAngle, self.turnCenter)

    def partway(self, time):
        if time < 0:
            return self.startPos
        elif time > self.duration:
            return self.endPos

        ratio = time / self.duration

        if self.turnSpeed == 0:
            return self.startPos.add(unitCirclePoint(self.startPos.angle).scale(self.arcLength * ratio))
        elif self.turnSpeed > 0:
            return self.startPos.rotateAround(self.arcAngle * ratio, self.turnCenter)
        else:
            return self.startPos.rotateAround(-self.arcAngle * ratio, self.turnCenter)

    def free(self, points, margin=0.25):
        'Om det e nån punkt i points som bilen kræsje med. Margin e kor my klaring vi ønske for bilen på hver side'
        # For dette trur e vi bare treng å filtrer ut dem punktan som e i retning den anndelen av sirkelen vi kjøre i, 
        # for så å sjekk at avstanden fra sentrum av sirkelen e minst margin differanse fra radius av sirkelen. 
        turnCenter = self.turnCenter
        turnRadius = self.turnRadius

        minRadius = turnRadius - margin
        maxRadius = turnRadius + margin

        if self.turnSpeed == 0:
            for point in points:
                if lineDistance(self.startPos, self.endPos, point) < margin:
                    return False
            return True

        for point in points:
            # Sjekk at avstand fra turn center stemme ish
            if not (minRadius < point.distance(turnCenter) < maxRadius):
                # print('RadiusMiss')
                continue

            # Sjekk at punktet e på rett del av sirkelen
            if self.turnSpeed > 0:
                # I venstre sving ska vi gi punktet, også sirkelsentrum, også bilen
                angle = angleFromPoints(self.startPos, turnCenter, point)
            else:
                # I høyre sving ska vi gi bilen, også sirkelsentrum, også punktet
                angle = angleFromPoints(point, turnCenter, self.startPos)

            if 0 < angle < (self.arcLength + margin * 1.5) / self.turnRadius:
                return False

        return True


def rateTurns(goalPos, turns):
    return sum([t.arcLength for t in turns]) + goalPos.distance(turns[-1].endPos)


def AStar(startPos, goalPos, obstacles):
    ''''
    Returne en liste av påfølgende Turns, som kan utføres for å nå en posisjon med A*, uten driftkorrigering.
    Om vi replanne ofte nok burda det ikkje bli et problem.
    '''
    tries = 7
    triesHalf = tries // 2

    turnSequences = []
    turnSequencesRating = []

    for _ in range(500):
        # Legg til dem 5 nye turn sekvensan, i lista, og fjern den vi jobbe fra
        if turnSequences:
            turnSequence = turnSequences.pop(0)
            turnSequencesRating.pop(0)
        else:
            turnSequence = []

        for i in range(-triesHalf, triesHalf+1):
            if turnSequence:
                turn = Turn(turnSequence[-1].endPos, i * 0.25/triesHalf)
            else:
                turn = Turn(startPos, i * 0.25/triesHalf)

            if not turn.free(obstacles):
                # Skip, kan ikkje kjør der
                # print('Blocked by point')
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

        if goalPos.distance(turnSequences[0][-1].endPos) < 0.5:
            return turnSequences[0] if turnSequences else None

    return turnSequences[0] if turnSequences else None
