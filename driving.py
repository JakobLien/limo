import math

class Turn:
    def __init__(self, startPos, turnSpeed, speed=0.1):
        self.startPos = startPos
        self.turnSpeed = turnSpeed
        self.speed = speed

    @property
    def turnCenter(self):
        'Senrum av svingradiusen av bilen'
        # Om vi ikkje svinge e det ingen sentrum av svingen
        if self.turnSpeed == 0:
            return None

        # Turning radius e bare speed / turning speed
        turningRadius = abs(self.speed / self.turnSpeed)

        # Om turnSpeed e positiv e det te venstre for kor enn bilen peke akk no. 
        # Om den e negativ e det te høyre. 

        if self.turnSpeed > 0:
            # Return punktet 90 grader direkte til venstre for roboten. 
            pass
        else:
            # Return punktet 90 grader direkte til høyre for roboten. 
            pass

    @property
    def endPos(self):
        'Posisjon og orientering roboten vil ha til slutt'
        # Dette bli bare å flytt referansesystemet til midta av sirkelen, roter rundt den, og flytt tilbake. 
        # Burda kanskje ha en ting for punkt med retning, typ orientation elns, som håndtere det fint for me. 
        return self.startPos

    def free(self, points, margin=0.2):
        'Om det e nån punkt i points som bilen kræsje med. Margin e kor my klaring vi ønske for bilen på hver side'
        # For dette trur e vi bare treng å filtrer ut dem punktan som e i retning den anndelen av sirkelen vi kjøre i, 
        # for så å sjekk at avstanden fra sentrum av sirkelen e minst margin differanse fra radius av sirkelen. 
        pass