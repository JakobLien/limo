# Code fetched from https://github.com/tdrmk/pygame_recorder/tree/master, altered slightly to use mp4 instead

import cv2
import pygame

class ScreenRecorder:
    def __init__(self, width, height, fps, filePath):
        four_cc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video = cv2.VideoWriter(filePath, four_cc, float(fps), (width, height))

    def capture_frame(self, surf):
        # Note: surface must have the dimensions specified in the constructor.
        # transform the pixels to the format used by open-cv
        pixels = cv2.rotate(pygame.surfarray.pixels3d(surf), cv2.ROTATE_90_CLOCKWISE)
        pixels = cv2.flip(pixels, 1)
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)

        # write the frame
        self.video.write(pixels)

    def end_recording(self):
        # stop recording
        self.video.release()
