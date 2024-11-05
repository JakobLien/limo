import pygame

# pygame setup
pygame.init()
screen = pygame.display.set_mode((600, 600))
clock = pygame.time.Clock()
running = True

while True:
    # fill the screen with a color to wipe away anything from last frame
    screen.fill("white")

    # RENDER YOUR GAME HERE
    pygame.draw.circle(screen, "red", [300, 300], 10)

    # flip() the display to put your work on screen
    pygame.display.flip()    

    clock.tick(60)  # limits FPS to 60