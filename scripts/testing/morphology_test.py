import numpy as np
import cv2 as cv
from sklearn.cluster import KMeans
from matplotlib import pyplot as plt
from itertools import cycle

img = cv.imread('morph3.png', 0)

# Threshold.
ret, thresh = cv.threshold(img, 150, 255, cv.THRESH_BINARY)

# Close.
se = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, se)

# Find labels.
ret, labels = cv.connectedComponents(closed, connectivity=8)

# Find contours.
im2, contours, hierarchy = cv.findContours(closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

approved_contours = []
mask = np.zeros((384, 384))

for cnt in contours:
    if cv.contourArea(cnt) < 5:
        continue

    ellipse = cv.fitEllipse(cnt)

    # Multiplying the length of the two main axes.
    if ellipse[1][0]*ellipse[1][1] < 4:
        continue

    approved_contours.append(cnt)

cv.drawContours(mask, approved_contours, -1, 255, thickness=cv.FILLED)

labels[mask == 0] = 0

plt.imshow(labels, 'gray')
plt.show()