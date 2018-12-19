#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Ajinkya Khoche}
# {student id}
# {khoche@kth.se}
DEBUG = 0
from dubins import Car
import random
# from scipy.spatial import distance
import numpy as np
if DEBUG:
    from matplotlib import pyplot as plt 

'''
class vertex
- member elements:
    - phi[i]: list of phi needed to get there from connecting point
    - t[i]: time steps

class tree
-- member elements:
    - vertex:       it's another class
    - edge:         list of tuple of vertices (index of vertex list) signifying a connecting edge  

-- member functions:

RRT Algo:
- select a random point xrand (every few iterations, xrand = xtarget)
- find nearest neighbor , xnear
- move move 100 (design parameter) steps from xnear to xrand. everytime you move
keep a record of phi[i] at every t[i]. 
- after every step check if you hit an obstacle. suppose you hit an obstacle at
67th step. then rescind around 20 steps, i.e. 47th step. this allows car to be in 
safe zone again. 
- after the motion is complete (bc of completing 100 steps or encountering obstacle),
the end point becomes a new vertex of rrt (xnew). add xnew to vertex list.
- update edge as (xnear, xnew).
- repeat    
'''
class vertex: 
    # constructor
    def __init__(self, x0, y0, theta0):
        self.phi_vertex = []
        #self.t_vertex = []
        self.xn_vertex = []
        self.yn_vertex = []
        self.thetan_vertex = []

        self.vertex_position = (x0,y0)
        self.vertex_heading = theta0
    
    def set_phi(self, phi_result):
        self.phi_vertex = phi_result

    def set_positions(self, xn_result, yn_result, thetan_result):
        self.xn_vertex = xn_result
        self.yn_vertex = yn_result
        self.thetan_vertex = thetan_result

class tree(object):
    # constructor
    def __init__(self, car):
        self.car = car
        self.vertex_list = []
        #self.heading = []
        self.edges = [] 
        self.nIter = 5000
        self.bias_freq = 5        # every 10th point is biased towards goal
        self.nSteps = 100           # number of steps car takes towards a xrand

        self.sufficient_steps = 50
        self.nsteps_back = 20
        
        self.feasible_path = []
        #final list of phi and t
        self.tl = []
        self.tl.append(0)
        self.phil = []

        #OPTIONAL: list of states
        self.xl = []
        self.yl = []
        self.thetal = []

        self.vertex_list.append(vertex(car.x0, car.y0, 0))
        #self.vertex_list[-1].set_positions([car.x0], [car.y0], [0])

    def build_tree(self):
        goal_reached = 0

        for k in range(self.nIter):
            if DEBUG:
                print(k)
    
            xrand = self.rand_state(k)
            # find point nearest in tree to xrand
            xnear, xnear_ind = self.nearest_neighbor(xrand)
        
            # once xrand and xnear is known, now we need to take self.nSteps number 
            # of steps starting from xnear to xrand. we store all intermediate
            # phi's and t's in list. at end append these phi's and t's to vertex class
            phi_temp = []
            xn_list = []
            yn_list = []
            thetan_list = []
            
            xn = 0
            yn = 0
            thetan = 0 
            for j in range(self.nSteps):
                if j == 0 :
                    xn = xnear[0]
                    yn = xnear[1]
                    thetan = self.vertex_list[xnear_ind].vertex_heading
            
                    xn_list.append(xn)
                    yn_list.append(yn)
                    thetan_list.append(thetan)
            
                              
                # find, phi_desired using xnear and xrand and previous heading (theta_prev)
                phi_desired = self.find_phi_des(xrand, (xn,yn), thetan)

                phi_temp.append(phi_desired)
                
                # execute control action, done for us by the car.step() function
                xn, yn, thetan = self.car.step(xn, yn, thetan, phi_desired )

                xn_list.append(xn)
                yn_list.append(yn)
                thetan_list.append(thetan)

                #if not(self.safe(xn, yn)):      # i.e. if car has hit an obstacle
                if not(self.car._environment.safe(xn,yn)):
                    if DEBUG:
                        print('NOT SAFE')
                    # if len(phi_temp) > self.sufficient_steps:   # if sufficient steps have been taken
                    #     #xnew = (xn,yn)
                    #     xnew = (xn_list[-self.nsteps_back], yn_list[-self.nsteps_back])
                    #     v = vertex(xnew[0], xnew[1], thetan_list[-self.nsteps_back])
                    #     v.set_phi(phi_temp[0:len(phi_temp)-self.nsteps_back])
                    #     #v.set_t(t_temp[0:len(t_temp)-self.nsteps_back])
                    #     v.set_positions(xn_list[0:len(phi_temp)-self.nsteps_back], yn_list[0:len(phi_temp)-self.nsteps_back], thetan_list[0:len(phi_temp)-self.nsteps_back])
                        
                    #     self.add_vertex(v)
                    #     self.add_edge(xnear_ind)
                    #     #new_vertex_added = 1
                    break


                if abs(xn - self.car.xt) <0.1 and abs(yn - self.car.yt) <0.1:
                    xnew = (xn,yn)
                    v = vertex(xnew[0], xnew[1], thetan)
                    v.set_phi(phi_temp)
                    #v.set_t(t_temp)
                    v.set_positions(xn_list, yn_list, thetan_list)

                    self.add_vertex(v)
                    self.add_edge(xnear_ind)
                    goal_reached = 1
                    break
            
            if j == self.nSteps -1:     # i.e. car travelled without facing any obstacles
                xnew = (xn,yn)
                v = vertex(xnew[0], xnew[1], thetan)
                v.set_phi(phi_temp)
                v.set_positions(xn_list, yn_list, thetan_list)
                
                self.add_vertex(v)
                self.add_edge(xnear_ind)
                      
            if goal_reached==1:
                if DEBUG:
                    print('>>>>>>>>>>>>>>>> FOUND a feasible path!! <<<<<<<<<<<<<<<<<<')
                self.extract_feasible_path()
                break
            
        if not(goal_reached):
            if DEBUG:
                print('>>>>>>>goal not reached, increase no. of iterations<<<<<<<<<<<<<<<<<')
            self.extract_feasible_path()
                        
            
    '''Pick a random state for iteration k'''
    def rand_state(self, k):
        if k % self.bias_freq == 0:
            xrand = (self.car.xt, self.car.yt)
        else:
            xtemp = random.uniform(self.car.xlb, self.car.xub)
            ytemp = random.uniform(self.car.ylb, self.car.yub)
            xrand = (xtemp, ytemp) 
        return xrand

    # def nearest_neighbor(self, xrand):
    #     nearest = min(self.vertex, key=lambda c: (c[0]- xrand[0])**2 + (c[1]-xrand[1])**2)
    #     nearest_ind = min(range(len(self.vertex)), key=lambda c: (self.vertex[c][0]- xrand[0])**2 + (self.vertex[c][1]-xrand[1])**2)
    #     return nearest, nearest_ind
    def nearest_neighbor(self, xrand):
        vertex_position_list = []
        for ver in self.vertex_list:
            vertex_position_list.append(ver.vertex_position)
        nearest = min(vertex_position_list, key=lambda c: (c[0]- xrand[0])**2 + (c[1]-xrand[1])**2)
        nearest_ind = min(range(len(vertex_position_list)), key=lambda c: (vertex_position_list[c][0]- xrand[0])**2 + (vertex_position_list[c][1]-xrand[1])**2)
        return nearest, nearest_ind

    def find_phi_des(self, xrand, xnear, theta_prev):
        theta_desired = np.arctan2((xrand[1] - xnear[1]),(xrand[0] - xnear[0])) - theta_prev

        if theta_desired > 0.005:
            phi_desired = np.pi/4
        elif theta_desired < -0.005:
            phi_desired = - np.pi/4
        else:
            phi_desired = 0
        return phi_desired

    def add_vertex(self, v):
        self.vertex_list.append(v)

    def add_edge(self, xnear_ind):
        self.edges.append((xnear_ind, len(self.vertex_list)-1))

    def extract_feasible_path(self):
        # we go back from last vertex to first vertex, through connections
        # established by edge list. very interesting!
        # self.feasible_path = []
        self.feasible_path.append(len(self.vertex_list) - 1)
        # starting point of back-iteration is last vertex, i.e. one which reached target
        temp_vertex = len(self.vertex_list) - 1     

        while temp_vertex != 0:     # while we don't reach 0th vertex
            temp_vertex = self.edges[temp_vertex - 1][0] 
            self.feasible_path.append(temp_vertex)

        self.extract_control_activations()           


    def extract_control_activations(self):
        # NOTE: feasible path is in reversed order!!!
        self.xl.append(self.car.x0)
        self.yl.append(self.car.y0)
        self.thetal.append(0)
        for ver in self.feasible_path[::-1]:
            self.phil = self.phil + self.vertex_list[ver].phi_vertex
            #OPTIONAL
            self.xl = self.xl + self.vertex_list[ver].xn_vertex[1:]
            self.yl = self.yl + self.vertex_list[ver].yn_vertex[1:]
            self.thetal = self.thetal + self.vertex_list[ver].thetan_vertex[1:]
            length_phi = len(self.vertex_list[ver].phi_vertex)

            for kk in range(length_phi):
                self.tl.append(self.tl[-1]+self.car.dt)



def solution(car):
    ''' <<< write your code below >>> '''
    rrt = tree(car)
    rrt.build_tree()
    
    control = rrt.phil
    times = rrt.tl   
    if DEBUG:  
        plt.plot(rrt.xl, rrt.yl)
        rrt.car.evaluate(control, times, verbose=True)
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