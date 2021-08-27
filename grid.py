import numpy as np
import graph
import math
import sys
sys.setrecursionlimit(2**31-1)

"""
The coordinates of this grid are spaced every 5 units. However, you can use whichever units you want provided 
you change the other parameters appropriately.
The origin is from the very center of the top of the robot, or the center
of the arms. The range is from (-half the dimension, half the dimension) for each dimension.
The grid is uneven, meaning there are more coordinates surrounding obstacles implemented into the grid.
The width of the finer mesh and the distance of the mesh from each obstacle can be
altered with the step and the distanceFromObstacle parameters.
"""

"""
First, input your obstacles by calling the add_obstacle function.
Then, once all obstacles are included, call finalize() to complete the graph.
The finalize function returns the graph of nodes and edges.
"""
# TO DO: Implement large edge coords

class Grid():
    def __init__(self, length, width, height, distanceFromObstacle=3, small_step=0.5, large_step=1):
        self._length = length
        self._width = width
        self._height = height
        self.graph = graph.Graph()
        self._distanceFromObstacle = distanceFromObstacle
        self._small_step = small_step
        self._large_step = large_step
        self._coordinates = set()
        # coordinates included within the fine mesh and obstacles
        self.avoid_coords = set()
        # edge coordinates of the fine and large mesh to link larger grid to small grid
        # each element has two lists -> (large_edge_nodes, fine_edge_nodes)
        self.edge_coords = []
        # A dictionary of all the coordinates paired with either 0, 1, or 2
        # 0 means that there is no obstacle
        # 1 means that the coordinate is an obstacle/large grid point in small grid area
        self.grid_array = {}
        """
        Generates the base grid prior to adding obstacles.
        The default obstacle value for each coordinate is 0, meaning that
        there is no obstacle here.

        Each coordinate is spaced 1 centimeter apart.
        """ 
        x_range, y_range, z_range = self._length / 2, self._width / 2, self._height / 2

        # generating the base grid
        # to account for floats, coordinates to the exact float are added too and not rounded away 

        for x in np.arange(-math.trunc(x_range), math.trunc(x_range) + 1, self._large_step):
            for y in np.arange(-math.trunc(y_range), math.trunc(y_range) + 1, self._large_step):
                for z in np.arange(-math.trunc(z_range), math.trunc(z_range) + 1, self._large_step):
                    for i in (-x_range, x, x_range):
                        for j in (-y_range, y, y_range):
                            for k in (-z_range, z, z_range):
                                if self.check_inbounds((i,j,k)):
                                    self.add_coord((i,j,k))
        

    @property
    def size(self):
        return (self._length, self._width, self._height)
    
    @property
    def coordinates(self):
        return self._coordinates
    
    @property
    def distanceFromObstacle(self):
        return self._distanceFromObstacle

    @property
    def small_step(self):
        return self._small_step

    def check_inbounds(self, coord):
        """
        Checks to see if the coordinate is within the boundaries.
        """
        x, y, z = coord
        if x <= self._length/2 and x >= -self._length/2 and y <= self._width/2 and y >= -self._width/2 \
            and z <= self._height/2 and z >= -self._height/2:
            return True
        return False


    def add_coord(self, coord, val=0):
        """
        Adds coordinate to the grid.
        """
        x, y, z = coord
        if coord in self.coordinates:
            return
        # adds coord to set list + the grid itself
        self._coordinates.add(coord)
        if x not in self.grid_array:
            self.grid_array[x] = {y: {z: val}}
        
        if y not in self.grid_array[x]:
            self.grid_array[x][y] = {z: val}
        
        if z not in self.grid_array[x][y]:
            self.grid_array[x][y][z] = val
    
    def find_value(self, coord):
        """
        Finds the value (either 0 or 1) at that particular coordinate.
        
        """
        if coord in self.coordinates:
            return self.grid_array[coord[0]][coord[1]][coord[2]]
        else:
            return False


    def create_obstacle(self, coord1, coord2, coord3):
        """
        Generates a rectangular obstacle from the three coordinates given.

        Parameters:
        obstacle = Obstacle object
        distanceFromObstacle = the distance away from the obstacle. Used to generate fine mesh 
        coordinates of a certain width.
        steps = the dimension of the new cube

        """

        # Makes sure to add obstacle coordinates if not already in grid
        # so that any coordinates inside are not ignored
        if coord1 or coord2 or coord3 not in self.coordinates:
            self.add_coord(coord1, 0)
            self.add_coord(coord2, 0)
            self.add_coord(coord3, 0)
            self.avoid_coords.update({coord1, coord2, coord3})

        # Maximum and minimum coordinate value for each dimension for the obstacle + fine mesh
        maxes_mins = tuple(zip(map(min, zip(coord1, coord2, coord3)), map(max, zip(coord1, coord2, coord3))))
        extended_maxes_mins = [(i-self.distanceFromObstacle, j+self.distanceFromObstacle) for i, j in maxes_mins]
       
        # function for finding the available keys within the range of the max and min for each coordinate, exclusive of both end points
        # we don't want to create new coordinates within the obstacle
        find_avail_vals = lambda idx, dictionary: filter(lambda el: el>extended_maxes_mins[idx][0] and el<extended_maxes_mins[idx][1], dictionary.keys())
        
        # function to find the closest x/y/z values to the extended max and min val for an individual axis
        def find_nearest_large_val(min_max, dictionary):
            min_border = min_max[0]
            max_border = min_max[1]
            min_plus = float('inf')
            min_minus = float('inf')
            new_vals = [0,0]
            for x in dictionary.keys():
                if 0 <= min_border - x < min_minus:
                    min_minus = min_border - x
                    new_vals[0] = x
                if 0 <= x - max_border < min_plus:
                    min_plus = x - max_border
                    new_vals[1] = x
    
            return tuple(new_vals)

        x_vals = find_nearest_large_val(extended_maxes_mins[0], self.grid_array)
        y_vals = find_nearest_large_val(extended_maxes_mins[1], self.grid_array[x_vals[0]])
        z_vals = find_nearest_large_val(extended_maxes_mins[2], self.grid_array[x_vals[0]][y_vals[0]])

        large_nodes = set()

        edge_coords = lambda dictionary, vals: filter(lambda el: el>=vals[0] and el<=vals[1], dictionary.keys())

        # adding all edge coords to set list
        for i in x_vals:
            for j in edge_coords(self.grid_array[x_vals[0]], y_vals):
                for k in edge_coords(self.grid_array[x_vals[0]][y_vals[0]], z_vals):
                    if self.check_inbounds((i,j,k)) and self.find_value((i,j,k)) == 0:
                        large_nodes.add((i,j,k))

        for j in y_vals:
           for i in edge_coords(self.grid_array, x_vals):
                for k in edge_coords(self.grid_array[x_vals[0]][y_vals[0]], z_vals):
                    if self.check_inbounds((i,j,k)) and self.find_value((i,j,k)) == 0:
                        large_nodes.add((i,j,k))


        for k in z_vals:
            for i in edge_coords(self.grid_array, x_vals):
                for j in edge_coords(self.grid_array[x_vals[0]], y_vals):
                    if self.check_inbounds((i,j,k)) and self.find_value((i,j,k)) == 0:
                        large_nodes.add((i,j,k))
        

        # changing all of the vals to 1 to indicate obstacle
        for x in find_avail_vals(0, self.grid_array):
            for y in find_avail_vals(1, self.grid_array[x]):
                for z in find_avail_vals(2, self.grid_array[x][y]):
                    self.add_coord((x,y,z), 1)
                    self.avoid_coords.add((x,y,z))

        
        def add_finer_mesh():
            """
            Once obstacles are added, new coordinates are added to the graph for a finer mesh around these points.
            Edges are then generated between all of the new points, and then edges between the greater grid and the
            fine mesh are generated.
            Parameters distanceFromObstacle and step are used here.
            
            """
            fine_mesh_coords = []
            edge_coords = []

            # function that creates the range for the surrounding area of the obstacle
            num1 = int(self.distanceFromObstacle/self.small_step)
            new_range = lambda range: np.concatenate((\
            np.linspace(range[0] - self.distanceFromObstacle + self.small_step, range[0], num1, endpoint=True), \
            np.linspace(range[1], range[1] + self.distanceFromObstacle, num1)))

            total_range = lambda idx: np.linspace(extended_maxes_mins[idx][0], extended_maxes_mins[idx][1], num=int((abs(extended_maxes_mins[idx][0] - extended_maxes_mins[idx][1]))/self.small_step + 1), endpoint=True)

            new_x_range = new_range(maxes_mins[0])
            new_y_range = new_range(maxes_mins[1])
            new_z_range = new_range(maxes_mins[2])

            # adding all edge coords to set list
            for i in new_x_range:
                for j in total_range(1):
                    for k in total_range(2):
                        if self.check_inbounds((i,j,k)):
                            self.add_coord((i,j,k))
                            fine_mesh_coords.append((i,j,k))
                        if i in (new_z_range[0], new_x_range[-1]):
                            edge_coords.append((i,j,k))

            for j in new_y_range:
                for i in total_range(0):
                        for k in total_range(2):
                            if self.check_inbounds((i,j,k)):
                                self.add_coord((i,j,k))
                                fine_mesh_coords.append((i,j,k))
                            if j in (new_y_range[0], new_y_range[-1]):
                                edge_coords.append((i,j,k))


            for k in new_z_range:
                for i in total_range(0):
                    for j in total_range(1):
                        if self.check_inbounds((i,j,k)):
                            self.add_coord((i,j,k))
                            fine_mesh_coords.append((i,j,k))
                        if k in (new_z_range[0], new_z_range[-1]):
                            edge_coords.append((i,j,k))
            
            self.avoid_coords.update(set(fine_mesh_coords))
            # creates all the edges for the fine mesh
            self.create_new_edges(list(set(fine_mesh_coords)), self.small_step)

            return list(set(edge_coords))


        fine_edge_nodes = add_finer_mesh()
        self.edge_coords.append((large_nodes, fine_edge_nodes))       


    def create_new_edges(self, coords_list, step):
        """
        Given a coordinate and the set of coordinates it's included in, generates edges between the coordinate and its neighboring nodes.
        
        Parameters:
        coords_list = a list of all of the coordinates to be used in generating the nodes and edges.
        step = step size to be used. Included because function is called both when creating an obstacle and generating the graph.
        """

        if len(coords_list) <= 1:
            return
        
        coord = coords_list[0]
        x, y, z = coord
        potential_coords = set(coords_list[1:])

        pot_neighbors = []
        new_x, new_y, new_z = math.trunc(float(x)), math.trunc(float(y)), math.trunc(float(z))

        for i in tuple(set((x-step, x, new_x, x+step))):
            for j in tuple(set((y-step, y, new_y, y+step))):
                for k in tuple(set((z-step, z, new_z, z+step))):
                    if (i,j,k) in potential_coords:
                        pot_neighbors.append((i,j,k))
        
        
        if len(pot_neighbors) == 0:
            self.graph.add_node(coord)

        
        for pot_x, pot_y, pot_z in pot_neighbors:
            # checks to see if it is an obstacle, if so then skips
            if self.grid_array[pot_x][pot_y][pot_z] == 1:
                continue
            self.graph.add_edge(coord, (pot_x, pot_y, pot_z))
            
        self.create_new_edges(coords_list[1:], step)

    def finalize(self):
        """
        Only call this function once done inputting the rest of the obstacles.
        Builds the edges and nodes in the areas not dominated by fine mesh and
        connects the fine mesh areas to the regular grid.
        """
    

        remaining_large_coords = self.coordinates - self.avoid_coords

        self.create_new_edges(list(remaining_large_coords), self._large_step)

        if self.edge_coords != []:
            for large, small in self.edge_coords:
                for large_x, large_y, large_z in large:
                    near_large_coords = filter(lambda coord: abs(coord[0]-large_x) <= self._large_step/2 and abs(coord[1]-large_y) <= self._large_step/2 and abs(coord[2]-large_z) <= self._large_step, small)
                    for coord in near_large_coords:
                        self.graph.add_edge((large_x, large_y, large_z), coord)

        return self.graph
