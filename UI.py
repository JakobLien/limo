import pygame

from geometry import Point

pygame.font.init()
f = pygame.font.Font(size=24)

buttonSizes = [10, 10, 110, 50]
buttonPadding = 5

class Button:
    def __init__(self, label):
        global buttonSizes
        self.label = label
        self.minX, self.minY, self.maxX, self.maxY = buttonSizes
        self.width, self.height = self.maxX - self.minX, self.maxY - self.minY
        self.clicked = False
        self.centerBonus = (self.height - f.get_height()) / 2

        # Fiks neste buttonSizes. 
        buttonSizes = [a+b for (a, b) in zip(buttonSizes, [0, 50, 0, 50])]

    def isClick(self, clickPoint: Point):
        self.clicked = self.minX-buttonPadding <= clickPoint.x <= self.maxX+buttonPadding and self.minY-buttonPadding <= clickPoint.y <= self.maxY+buttonPadding
        return self.clicked

    def draw(self, screen: pygame.SurfaceType):
        if not self.clicked:
            pygame.draw.rect(screen, 'black', pygame.Rect(self.minX, self.minY, self.width, self.height), width=3)
        screen.blit(f.render(self.label, True, "black", "white"), (self.minX+5, self.minY+self.centerBonus))

        self.clicked = False
