#!/usr/bin/env python

# Subscribe: /eva/scan_mismatches

import roslib
import rospy
import numpy as np
import cv2 as cv
import math

from eva_a.msg import *

# ----- Start of DBSCAN -----

"""
This is a simple implementation of DBSCAN intended to explain the algorithm.

@author: Chris McCormick
"""

def MyDBSCAN(D, eps, MinPts):
    """
    Cluster the dataset `D` using the DBSCAN algorithm.
    
    MyDBSCAN takes a dataset `D` (a list of vectors), a threshold distance
    `eps`, and a required number of points `MinPts`.
    
    It will return a list of cluster labels. The label -1 means noise, and then
    the clusters are numbered starting from 1.
    """
 
    # This list will hold the final cluster assignment for each point in D.
    # There are two reserved values:
    #    -1 - Indicates a noise point
    #     0 - Means the point hasn't been considered yet.
    # Initially all labels are 0.    
    labels = [0]*len(D)

    # C is the ID of the current cluster.    
    C = 0
    
    # This outer loop is just responsible for picking new seed points--a point
    # from which to grow a new cluster.
    # Once a valid seed point is found, a new cluster is created, and the 
    # cluster growth is all handled by the 'expandCluster' routine.
    
    # For each point P in the Dataset D...
    # ('P' is the index of the datapoint, rather than the datapoint itself.)
    for P in range(0, len(D)):
    
        # Only points that have not already been claimed can be picked as new 
        # seed points.    
        # If the point's label is not 0, continue to the next point.
        if not (labels[P] == 0):
           continue
        
        # Find all of P's neighboring points.
        NeighborPts = regionQuery(D, P, eps)
        
        # If the number is below MinPts, this point is noise. 
        # This is the only condition under which a point is labeled 
        # NOISE--when it's not a valid seed point. A NOISE point may later 
        # be picked up by another cluster as a boundary point (this is the only
        # condition under which a cluster label can change--from NOISE to 
        # something else).
        if len(NeighborPts) < MinPts:
            labels[P] = -1
        # Otherwise, if there are at least MinPts nearby, use this point as the 
        # seed for a new cluster.    
        else: 
           C += 1
           growCluster(D, labels, P, NeighborPts, C, eps, MinPts)
    
    # All data has been clustered!
    return labels


def growCluster(D, labels, P, NeighborPts, C, eps, MinPts):
    """
    Grow a new cluster with label `C` from the seed point `P`.
    
    This function searches through the dataset to find all points that belong
    to this new cluster. When this function returns, cluster `C` is complete.
    
    Parameters:
      `D`      - The dataset (a list of vectors)
      `labels` - List storing the cluster labels for all dataset points
      `P`      - Index of the seed point for this new cluster
      `NeighborPts` - All of the neighbors of `P`
      `C`      - The label for this new cluster.  
      `eps`    - Threshold distance
      `MinPts` - Minimum required number of neighbors
    """

    # Assign the cluster label to the seed point.
    labels[P] = C
    
    # Look at each neighbor of P (neighbors are referred to as Pn). 
    # NeighborPts will be used as a FIFO queue of points to search--that is, it
    # will grow as we discover new branch points for the cluster. The FIFO
    # behavior is accomplished by using a while-loop rather than a for-loop.
    # In NeighborPts, the points are represented by their index in the original
    # dataset.
    i = 0
    while i < len(NeighborPts):    
        
        # Get the next point from the queue.        
        Pn = NeighborPts[i]
       
        # If Pn was labelled NOISE during the seed search, then we
        # know it's not a branch point (it doesn't have enough neighbors), so
        # make it a leaf point of cluster C and move on.
        if labels[Pn] == -1:
           labels[Pn] = C
        
        # Otherwise, if Pn isn't already claimed, claim it as part of C.
        elif labels[Pn] == 0:
            # Add Pn to cluster C (Assign cluster label C).
            labels[Pn] = C
            
            # Find all the neighbors of Pn
            PnNeighborPts = regionQuery(D, Pn, eps)
            
            # If Pn has at least MinPts neighbors, it's a branch point!
            # Add all of its neighbors to the FIFO queue to be searched. 
            if len(PnNeighborPts) >= MinPts:
                NeighborPts = NeighborPts + PnNeighborPts
            # If Pn *doesn't* have enough neighbors, then it's a leaf point.
            # Don't queue up it's neighbors as expansion points.
            #else:
                # Do nothing                
                #NeighborPts = NeighborPts               
        
        # Advance to the next point in the FIFO queue.
        i += 1        
    
    # We've finished growing cluster C!


def regionQuery(D, P, eps):
    """
    Find all points in dataset `D` within distance `eps` of point `P`.
    
    This function calculates the distance between a point P and every other 
    point in the dataset, and then returns only those points which are within a
    threshold distance `eps`.
    """
    neighbors = []
    
    # For each point in the dataset...
    for Pn in range(0, len(D)):
        
        # If the distance is below the threshold, add it to the neighbors list.
        if np.linalg.norm(D[P] - D[Pn]) < eps:
           neighbors.append(Pn)
            
    return neighbors

# ----- End of DBSCAN -----

obstacle_map = np.zeros((384, 384), dtype=np.uint8)

class Obstacle:
    def __init__(self, label):
        self.label = label
        self.points = []

    def add_point(self, x, y):
        self.points.append((x, y))

    def draw(self, canvas):
        cv.circle(canvas, self.center, self.radius, 255, 1)

    def get_lat(self, resolution):
        x_sum = 0

        for point in self.points:
            x_sum = x_sum + point[0] 

        # Average position.
        return (x_sum/len(self.points) - 199)*resolution

    def get_lng(self, resolution):
        y_sum = 0

        for point in self.points:
            y_sum = y_sum + point[1] 

        # Average position.
        return (y_sum/len(self.points) - 183)*resolution

    def get_radius(self, resolution):
        # Approximating as circle.
        return (np.sqrt(len(self.points))/3.14)*resolution

def find_obstacles():
    global obstacle_map

    ret, thresh = cv.threshold(obstacle_map, 150, 255, cv.THRESH_BINARY)

    # Getting an array of vectors where each vector contains the pixel coordinates of the possible occupied cells.
    points = np.transpose(np.nonzero(thresh))

    if len(points) == 0:
        return []

    # At least six points in a cluster.
    labels = np.array(MyDBSCAN(points, 3, 6))
    # Want the first cluster to be number 0.
    labels = labels - 1

    num_clusters = max(labels) + 1

    obstacles = []

    # If no clusters were found.
    if max(labels) < 0:
        return obstacles

    # Generating obstacle instances, one per unique label.
    for i in range(0, num_clusters):
        obst = Obstacle(i)
        obstacles.append(obst)

    # Adding points to their corresponding obstacle.
    for i, label in enumerate(labels):
        if label >= 0:
            obstacles[label].add_point(points[i][1], points[i][0])

    return obstacles


def talker():
    pub = rospy.Publisher('/eva/obstacles', Obstacles, queue_size=10)

    try:
        resolution = rospy.get_param('/eva/resolution')
    except:
        resolution = 0.05

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        msg = Obstacles()
        obstacle_array = find_obstacles()

        if len(obstacle_array) > 0:
            print("Num obstacles: " + str(len(obstacle_array)))
            msg.numObstacles = len(obstacle_array)
            msg.latCenters = []
            msg.lngCenters = []
            msg.radii = []

            for obstacle in obstacle_array:
                msg.latCenters.append(obstacle.get_lat(resolution))
                msg.lngCenters.append(obstacle.get_lng(resolution))
                msg.radii.append(obstacle.get_radius(resolution))

        else:
            msg.numObstacles = 0
            msg.latCenters = []
            msg.lngCenters = []
            msg.radii = []

        pub.publish(msg)
        rate.sleep()

def callback(data):
    global obstacle_map
    obstacle_data = np.array(data.data, dtype=np.uint8)
    obstacle_map = np.reshape(obstacle_data, (data.height, data.width))
    
def listener():
    rospy.init_node('obstacle_publisher', anonymous=True)

    rospy.Subscriber('/eva/scan_mismatches',
                     ScanMismatches, callback, queue_size=1)

    talker()

    rospy.spin()

if __name__ == '__main__':
    listener()