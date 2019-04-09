import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

img = cv.imread('img/morph2.png')

ret, thresh = cv.threshold(img, 129, 255, cv.THRESH_BINARY)

if not ret:
    print("Not able to open image.")

kernel = np.ones((3, 3))

closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)

titles = ['Original', 'Binary', 'Closed']
images = [img, thresh, closed]

for i in xrange(3):
    plt.subplot(1,3,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
plt.show()

hei = input('Press enter to exit: ')