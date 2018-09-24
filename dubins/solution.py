#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Ajinkya Khoche}
# {student id}
# {khoche@kth.se}

from dubins import Car
import random
from scipy.spatial import distance
import numpy as np

'''
create a class called tree
-- member elements:
    - vertex:       list of coordinates
    - edge:         list of tuple of vertices (index of vertex list) signifying a connecting edge  

-- member functions:
    
'''
class tree(object):
    # constructor
    def __init__(self, car):
        self.car = car
        self.vertex = []
        self.heading = []
        self.edges = [] 
        self.nIter = 1000
        self.bias_freq = 10     # every 10th point is biased towards goal
        self.tl = []
        self.tl.append(0)
        self.phil = []

        self.vertex.append((car.x0, car.y0))
        self.heading.append(np.arctan2((car.yt - car.y0),(car.xt - car.x0)))

    def build_tree(self):
        for k in range(self.nIter):
            # find a random state 
            xrand = self.rand_state(k)
            # find point nearest in tree to xrand
            xnear, xnear_ind = self.nearest_neighbor(xrand)
            print(xnear_ind)
            # find, phi_desired using xnear and xrand and previous heading (theta_prev)
            phi_desired = self.find_phi_des(xrand, xnear, self.heading[-1])
            # execute control action, done for us by the car.step() function
            xn, yn, thetan = self.car.step(xnear[0], xnear[1], self.heading[-1], phi_desired )
            xnew = (xn, yn)
            # find if there's a collision between xnew and xrand
            collision = self.check_collision(xrand, xnew)

            if collision == 0:   
                self.add_vertex(xnew, thetan)
                self.add_edge(xnear_ind)
                self.phil.append(phi_desired)
                self.tl.append(self.tl[-1] + self.car.dt)

    def rand_state(self, k):
        # x = 100
        # y = 100
        if k % self.bias_freq == 0:
            xrand = (self.car.xt, self.car.yt)
        else:
            #while self.car.xlb <= x <= self.car.xub and self.car.ylb <= y <= self.car.yub:
            xtemp = random.uniform(self.car.xlb, self.car.xub)
            ytemp = random.uniform(self.car.ylb, self.car.yub)
            xrand = (xtemp, ytemp) 
        return xrand

    def nearest_neighbor(self, xrand):
        nearest = min(self.vertex, key=lambda c: (c[0]- xrand[0])**2 + (c[1]-xrand[1])**2)
        nearest_ind = min(range(len(self.vertex)), key=lambda c: (self.vertex[c][0]- xrand[0])**2 + (self.vertex[c][1]-xrand[1])**2)
        return nearest, nearest_ind

    def find_phi_des(self, xrand, xnear, theta_prev):
        theta_desired = np.arctan2((xrand[1] - xnear[1]),(xrand[0] - xnear[0]))
        phi_desired = np.arctan2((theta_desired - theta_prev),(self.car.dt))

        if phi_desired >= np.pi/4:
            phi_desired = np.pi/4
        elif phi_desired <= -np.pi/4:
            phi_desired = - np.pi/4
        return phi_desired

    def add_vertex(self, xnew, thetan):
        self.vertex.append(xnew)
        self.heading.append(thetan)

    def add_edge(self, xnear_ind):
        self.edges.append((xnear_ind, len(self.vertex)-1))

    def check_collision(self, xrand, xnew):
        slope = (xrand[1] - xnew[1])/(xrand[0] - xnew[0])
        #dist = ((xnew[0]- xrand[0])**2 + (xnew[1]-xrand[1])**2)**0.5

        collision = 0

        if self.safe(xnew[0], xnew[1]) == 0:
            collision = 1
        
        if self.safe(xrand[0], xrand[1]) == 0:
            collision = 1

        for i in range(abs(round((xnew[0]- xrand[0])/self.car.dt))):
            xtemp = xnew[0] + self.car.dt
            ytemp = xnew[1] + slope*(xtemp - xnew[0])
            
            collision_free = self.safe(xtemp, ytemp)

            if collision_free == 0:
                collision = 1
                break
        
        return collision

    def safe(self, x, y):
        ''' Tests whether a point is within the obstacle. '''
        ret = True

        for i in range(len(self.car.obs)):

            # relative position
            x -= self.car.obs[i][0]
            y -= self.car.obs[i][1]

            # distance to obstacle
            d = (x**2 + y**2)**0.5

            # if intersecting obstacle
            if d <= self.car.obs[i][2]:
                ret = False 

        return ret


def solution(car):
    #control, times = dummy_soln(car)

    ''' <<< write your code below >>> '''
    # controls=[0]
    # times=[0,1]

    ''' <<< write your code below >>> '''
    rrt = tree(car)
    rrt.build_tree()

    control = rrt.phil
    times = rrt.tl     

    return control, times

def dummy_soln(car):
    # initial conditions
    x, y = car.x0, car.y0
    theta = 0
    phi = 0.1
    t = 0

    # numerical integration
    xl, yl, thetal, phil, tl = [x], [y], [theta], [], [t]
    while tl[-1] < 15:
        xn, yn, thetan = car.step(xl[-1], yl[-1], thetal[-1], phi)
        xl.append(xn)
        yl.append(yn)
        thetal.append(theta)
        phil.append(phi)
        tl.append(tl[-1] + car.dt)
    
    return phil, tl
    

'''MAIN FUNCTION'''
def main():
    car = Car()
    print('car object initialized')

    solution(car)

if __name__ == '__main__':
    main()