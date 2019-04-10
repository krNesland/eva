import numpy as np
import cv2 as cv
from sklearn.cluster import MeanShift, estimate_bandwidth
from matplotlib import pyplot as plt
from itertools import cycle

img = cv.imread('morph3.png', 0)
ret, thresh = cv.threshold(img, 150, 255, cv.THRESH_BINARY)

x, y = np.nonzero(thresh)

points = np.stack((x, y), axis=1)


bandwidth = estimate_bandwidth(points)
ms = MeanShift()
ms.fit(points)

labels = ms.labels_
cluster_centers = ms.cluster_centers_

labels_unique = np.unique(labels)
n_clusters_ = len(labels_unique)

print("number of estimated clusters : %d" % n_clusters_)

# #############################################################################
# Plot result

plt.figure(1)
plt.clf()

colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
for k, col in zip(range(n_clusters_), colors):
    my_members = labels == k
    cluster_center = cluster_centers[k]
    plt.plot(points[my_members, 0], points[my_members, 1], col + '.')
    plt.plot(cluster_center[0], cluster_center[1], 'o', markerfacecolor=col,
             markeredgecolor='k', markersize=14)
plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()