#!/usr/bin/env python3

"""
    # {student full name}
    # {student id}
    # {student email}
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
    
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        #print(resolution)

        """
        Fill in your solution here
        
        Objective: To add laser scan data to occupancy grid as 'occupied space'

        Observations: 
            - pose.pose.orientation.z works incorrectly. This is because the orientation is
            in quarternions. And we need to extract yaw (euler angle) from quarternion
        """
        # Step-1 Convert the laser scan ranges and bearings to coordinates in the laser frame
        scan_data = np.zeros((len(scan.ranges),2))
        scan_data[:,0] = np.arange(0, 2*np.pi + scan.angle_increment, scan.angle_increment)
        scan_data[:,1] = scan.ranges
        # print('-----scan_data------')
        # print(scan_data)
        
        for i in range(scan_data.shape[0]):
            if scan_data[i,1] <= scan.range_min or scan_data[i,1] >= scan.range_max :
                continue
            else:
                #x_robot = scan_data[i,1] * np.cos(scan_data[i,0])
                #y_robot = scan_data[i,1] * np.sin(scan_data[i,0])

                x_map = pose.pose.position.x + scan_data[i,1] * np.cos(scan_data[i,0] + robot_yaw)
                y_map = pose.pose.position.y + scan_data[i,1] * np.sin(scan_data[i,0] + robot_yaw)

                x_grid_map = int((x_map - origin.position.x)/resolution) 
                y_grid_map = int((y_map - origin.position.y)/resolution) 

                # [C part]: use ray trace to update free space
                rob_pos_grid_x = int((pose.pose.position.x - origin.position.x)/resolution)
                rob_pos_grid_y = int((pose.pose.position.y - origin.position.y)/resolution)

                traversed = self.raytrace((rob_pos_grid_x, rob_pos_grid_y), (x_grid_map,y_grid_map))
                #print(traversed)
                for cell in traversed:
                    if not(grid_map[cell[0], cell[1]] == self.occupied_space or grid_map[cell[0], cell[1]] == self.c_space):
                        self.add_to_map(grid_map, cell[0], cell[1], self.free_space)
                
                self.add_to_map(grid_map, x_grid_map, y_grid_map, self.occupied_space)

        # for i in range(scan_data.shape[0]):
        #     if scan_data[i,1] <= scan.range_min or scan_data[i,1] >= scan.range_max :
        #         continue
        #     else:
        #         #x_robot = scan_data[i,1] * np.cos(scan_data[i,0])
        #         #y_robot = scan_data[i,1] * np.sin(scan_data[i,0])

        #         x_map = pose.pose.position.x + scan_data[i,1] * np.cos(scan_data[i,0] + robot_yaw)
        #         y_map = pose.pose.position.y + scan_data[i,1] * np.sin(scan_data[i,0] + robot_yaw)

        #         x_grid_map = int((x_map - origin.position.x)/resolution) 
        #         y_grid_map = int((y_map - origin.position.y)/resolution) 

        #         self.add_to_map(grid_map, x_grid_map, y_grid_map, self.occupied_space)
        """
        For C only!
        Fill in the update correctly below.
        """         
        #print(grid_map[:,:])
        #print(self.unknown_space)	# -1
        #print(self.free_space)		# 0
        #print(self.c_space)		# -128
        #print(self.occupied_space)	# -2
        #print('----------------------------')

        h = grid_map.get_height()
        w = grid_map.get_width()

        grid_map_copy = np.zeros((h, w), dtype=int)
        for i in range(h):
            for j in range(w):
                grid_map_copy[i,j] = grid_map[i,j]

        known_idx_h, known_idx_w = np.where(grid_map_copy != -1)
     
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = np.amin(known_idx_w)
        # The minimum y index in 'grid_map' that has been updated
        update.y = np.amin(known_idx_h)
        # Maximum x index - minimum x index + 1
        update.width = np.amax(known_idx_w) - update.x + 1 
        # Maximum y index - minimum y index + 1
        update.height = np.amax(known_idx_h) - update.y + 1

        #print('update.x :' + str(update.x) + 'update.y :' + str(update.y) + 'update.width :' + str(update.width) + 'update.height :' + str(update.height))
        
        # The map data inside the rectangle, in row-major order.
        temp_update = grid_map_copy[update.y: update.y + update.height, update.x: update.x+update.width]
        #print('-------'+str(temp_update.shape))
        
        temp_update = (temp_update).flatten(order='C') 
        update.data = temp_update.tolist()
        # Return the updated map together with only the
        # part of the map that has been updated
        #self.inflate_map(grid_map)
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        h = grid_map.get_height()
        w = grid_map.get_width()
    
        for i in range(h):
            for j in range(w):
                # if grid_map[i,j] == self.free_space:
                #     count = 0
                #     for k in range(self.radius):
                #         for l in range(self.radius):
                #             if k == 0 or k == self.radius-1:
                #                 if l > 0 and l < self.radius - 1:
                #                     if grid_map[i - int(self.radius/2) + k, j - int(self.radius/2) + l] == self.occupied_space:
                #                         count = count + 1
                #                         if count > 0:



                if grid_map[i,j] == self.occupied_space:  # grid is occupied
                    for k in range(2*self.radius + 1):
                        for l in range(2*self.radius + 1):
                            if k == 0 or k == 2*self.radius:
                                if l >1 and l < 2*self.radius - 1:
                                    if not(grid_map[i - self.radius + k, j - self.radius + l] == self.occupied_space):
                                        self.add_to_map(grid_map, i - self.radius + k, j - self.radius + l, self.c_space)
                            elif k == 1 or k == 2*self.radius -1:
                                if l >0 and l < 2*self.radius:
                                    if not(grid_map[i - self.radius + k, j - self.radius + l] == self.occupied_space):
                                        self.add_to_map(grid_map, i - self.radius + k, j - self.radius + l, self.c_space)
                

        # initialize a numpy array of scores, of same size as grid_map
        # if a grid cell has same state as previous, it's score increases by +3, upto a max of +100
        # if a grid cell has changed its state, its score reduces by -1, upto a min of 0.
        # we only consider those cells as occupied, if their status is self.occupied and its grid score is >80
        # h = grid_map.get_height()
        # w = grid_map.get_width()

        # grid_map_copy = np.zeros((h, w), dtype=int)
        # grid_checked = np.zeros((h, w), dtype=int)      # 0 - unchecked, 1 = checked. 
        # for i in range(h):
        #     for j in range(w):
        #         grid_map_copy[i,j] = grid_map[i,j]
        
        # occupied_grid = np.where(grid_map_copy == -2)
        # #print(occupied_grid[0].shape)
        # for i in range(occupied_grid[0].shape[0]):
        #     if grid_checked[occupied_grid[0][i],occupied_grid[1][i]] != 1:
        #         grid_checked[occupied_grid[0][i],occupied_grid[1][i]] = 1   # check current grid
            
        #         temp_yy = occupied_grid[0][i] - self.radius
        #         temp_xx = occupied_grid[1][i] - self.radius

        #         for j in range(2*self.radius + 1): 
        #             for k in range(2*self.radius + 1):
        #                 if self.is_in_bounds(grid_map, temp_yy, temp_xx):
        #                     if (grid_map_copy[temp_yy, temp_xx] == -128):
        #                         grid_checked[temp_yy,temp_xx] = 1   # if this cell is already occupied  or marked as c_space, check it as 1
        #                     elif (grid_map_copy[temp_yy, temp_xx] == -1 or grid_map_copy[temp_yy, temp_xx] == 0):
        #                         # if cell is free or unknown, then mark it as self.c_space
        #                         self.add_to_map(grid_map, temp_yy, temp_xx, self.c_space)
        #                         grid_checked[temp_yy, temp_xx] == 1
        #                         temp_yy = temp_yy + 1
        #                         temp_xx = temp_xx + 1


            # if grid_checked[occupied_grid[0][i],occupied_grid[1][i]] != 1:
            #     grid_checked[occupied_grid[0][i],occupied_grid[1][i]] = 1   # check current grid
            #     neighbors = self.find_neighbors(occupied_grid[0][i],occupied_grid[1][i])
            #     # occupied_neighbors = np.where(grid_map_copy[yy,xx] == -2) 
            #     for j in range(neighbors[0].shape[0]):
            #         if grid_map_copy[neighbors[0][j][0], neighbors[1][j][0]] == -2:
            #             grid_checked[neighbors[0][j][0], neighbors[1][j][0]] = 1
        # Return the inflated map
        return grid_map
    
    def find_neighbors(self, y_grid,x_grid):
        temp_y = y_grid - self.radius
        temp_x = x_grid - self.radius
        # use radius of bot to find neighbors of a grid cell
        neighbourhood_cell = np.zeros((2*self.radius+1, 2*self.radius+1))
        neighbourhood_cell[0,0] = temp_y
        x = np.arange((2*self.radius+1))
        y = np.arange((2*self.radius+1))
        xx, yy = np.meshgrid(x,y)
        xx = xx + temp_x
        yy = yy + temp_y

        return yy, xx
        