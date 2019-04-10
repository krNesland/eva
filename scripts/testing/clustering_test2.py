import numpy as np
import cv2 as cv
from sklearn.cluster import KMeans
from matplotlib import pyplot as plt
from itertools import cycle

# Kan fungere.

img = cv.imread('morph3.png', 0)
ret, thresh = cv.threshold(img, 150, 255, cv.THRESH_BINARY)

x, y = np.nonzero(thresh)

points = np.stack((x, y), axis=1)

y_pred = KMeans(n_clusters=15).fit_predict(points)

plt.scatter(x, y, c=y_pred)
plt.show()