import math
import pygame
from geometry import Orientation, Point, closestPoint, pointIndexClosestToAngle, splitAndMerge, tryTranslations


class LocalizationMap:
    'Et kart for features, for localization'
    def __init__(self):
        self.points = []
        self.pointsConfidence = []

    def addPointsAndLocalize(self, robotPos, points, localize=True):
        'Returne en ny robotPos etter å ha justert med alle punktan.'

        # Skriv om så vi heller har en confidence på punktan våre, enda en liste altså
        # Heller enn å ha data på hvert eneste punkt, for det komplisert koden en god del føle e. 

        featurePoints = splitAndMerge(points)

        # Korriger posisjon dersom vi e i bevegelse. 
        offset = Orientation(0.0, 0.0, 0.0)
        if localize:
            offset = tryTranslations(self.points, featurePoints)

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

        # Vedlikehold kartet
        hitIndexes = []
        featurePoints = [p.toReferenceFrame(offset) for p in featurePoints]
        for i, featurePoint in enumerate(featurePoints):
            closest = closestPoint(featurePoint, self.points)
            if closest and featurePoint.distance(closest) < 0.1:
                # Eksisterende pukt
                closestIndex = self.points.index(closest)
                self.points[closestIndex] = closest.toward(featurePoint, 0.2)
                self.pointsConfidence[closestIndex] = max(self.pointsConfidence[closestIndex] - 1, 0)
                hitIndexes.append(closestIndex)
            else:
                # Nytt punkt
                # # TODO: Bare lag et nytt punkt dersom nabomåligen ikkje er rett atmed vinkelmessig, for å unngå
                # # "hjørnepunkt" som bare er en konsekvens av obscurement
                # prevPoint = featurePoints[i-1] if i-1 > 0 else None
                # nextPoint = featurePoints[i+1] if i+1 < len(featurePoints) else None

                # # Omskriv logikken: Hvis vi ikkje har nabo punktet e det lov, men hvis vi har det og det e langt foran,
                # # da ska vi ikkje legg inn som nytt punkt. 

                # if prevPoint.distance(Point(0, 0)) < 0.2 and nextPoint.distance(Point(0, 0)) < 0.2 and \
                #     featurePoint.angluarDistance(prevPoint) < 5 * math.pi / 180 and \
                #     featurePoint.angluarDistance(nextPoint) < 5 * math.pi / 180:

                self.points.append(featurePoint)
                self.pointsConfidence.append(7) # Denne tallverdien avgjør hvor frampå vi skal vær med å registrer hjørnepunkt

        pointsInRobotFrame = [p.toReferenceFrame(robotPos) for p in points]

        # For resten av punktan
        deleteIndexes = []
        for i, p in enumerate(self.points):
            if i in hitIndexes:
                continue

            # En potensiell miss, spørs på obscurement
            pInRobotFrame = p.toReferenceFrame(robotPos)
            pInRobotFrameAngle = pInRobotFrame.origoAngle()

            # # Binærsøk fram til rett måling

            measurementPoint = pointsInRobotFrame[pointIndexClosestToAngle(pointsInRobotFrame, pInRobotFrameAngle)]

            # bestPointIndexMin, bestPointIndexMax = 0, len(points) - 1
            # while bestPointIndexMax - bestPointIndexMin > 1:
            #     bestPointIndex = int((bestPointIndexMax + bestPointIndexMin) / 2)
            #     # Merk at Lidaren gir oss punktan fra stor til lav vinkel (venstre mot høyre)
            #     if points[bestPointIndex].toReferenceFrame(robotPos).origoAngle() > pInRobotFrameAngle:
            #         bestPointIndexMax = bestPointIndex
            #     else:
            #         bestPointIndexMin = bestPointIndex
            # measurementPoint = points[int((bestPointIndexMax + bestPointIndexMin) / 2)].toReferenceFrame(robotPos)

            if abs(measurementPoint.origoAngle() - pInRobotFrame.origoAngle()) < 5 * math.pi / 180 and \
                measurementPoint.distance(Point(0, 0)) + 0.1 > pInRobotFrame.distance(Point(0, 0)):
                # Om næmreste målingens vinkel e 5 grader unna, og punktet e foran målingen
                self.pointsConfidence[i] = self.pointsConfidence[i] + 2
            elif self.pointsConfidence[i] > 5:
                # Punktet e ikkje synlig herifra, synk sikkerheten dersom vi e usikker på punktet
                self.pointsConfidence[i] = self.pointsConfidence[i] + 1

            if self.pointsConfidence[i] >= 10:
                # Må ta høyde for at lista bli kortar når vi slette ting, lettast å gjør det her:)
                deleteIndexes.append(i - len(deleteIndexes))

        for i in deleteIndexes:
            self.points.pop(i)
            self.pointsConfidence.pop(i)

        return robotPos

    def draw(self, robotPos, screen):
        for i, point in enumerate(self.points):
            # Heller enn å tegn indexes kan vi tegn dem med mørkhet basert på sikkerheten vår i punktan. 
            # Virk bedre det syns e:)

            pygame.draw.circle(screen, pygame.Color(*[int(self.pointsConfidence[i] * 25.5) for _ in range(3)]), point.toReferenceFrame(robotPos).toScreen().xy(), 4)


# class NavigationMap:
#     'Et map som har fleir rekka med punkt, som altså markere hjørnan av veggan'
#     pass
