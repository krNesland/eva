import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

class Obstacle:
    def __init__(self, cnt):
        self.cnt = cnt
        (x, y), radius = cv.minEnclosingCircle(cnt)
        self.center = (int(x), int(y))
        self.radius = int(radius)

    def draw(self, canvas):
        cv.circle(canvas, self.center, self.radius, 255, 1)


img = cv.imread('morph3.png', 0)

# Threshold.
ret, thresh = cv.threshold(img, 150, 255, cv.THRESH_BINARY)

# Close to make more connected.
se_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, se_close)

# Open to remove noise.
se_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
opened = cv.morphologyEx(closed, cv.MORPH_OPEN, se_open)

# Find contours.
im2, contours, hierarchy = cv.findContours(opened, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

obstacles = []
canvas = np.zeros((384, 384))

for cnt in contours:
    if cv.contourArea(cnt) < 5:
        continue

    ellipse = cv.fitEllipse(cnt)

    # Multiplying the length of the two main axes.
    if ellipse[1][0]*ellipse[1][1] < 4:
        continue

    obst = Obstacle(cnt)
    obstacles.append(obst)

# cv.drawContours(canvas, approved_contours, -1, 127, thickness=cv.FILLED)

for obst in obstacles:
    obst.draw(canvas)

plt.imshow(canvas, 'gray')
plt.show()