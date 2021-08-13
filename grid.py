import numpy as np

"""
The coordinates of this grid are in centimeters units. However, you can use whichever units you want provided 
you change the other parameters appropriately.
The origin is from the very center of the top of the robot, or the center
of the arms. The range is from (-half the dimension, half the dimension) for each dimension.
The grid is uneven, meaning there are more coordinates surrounding obstacles implemented into the grid.
The width of the finer mesh and the distance of the mesh from each obstacle can be
altered with the step and the distanceFromObject parameters included in the
add_obstacle function.
"""

class Grid():
    def __init__(self, length, width, height):
        self._length = length
        self._width = width
        self._height = height
        self._coordinates = set()
        # A dictionary of all the coordinates paired with either 0 or 1
        # 0 means that there is no obstacle
        # 1 means that the coordinate is an obstacle
        self.grid_array = {}
        self.grid_array = self.generate_array()

    @property
    def size(self):
        return (self._length, self._width, self._height)
    
    @property
    def coordinates(self):
        return self._coordinates

    def add_coord(self, coord, val=0):
        """
        Adds coordinate to the grid.
        """
        x, y, z = coord
        # makes sure coordinate in bounds, otherwise will not add
        if x <= self._length/2 and x >= -self._length/2 and y <= self._width/2 and y >= -self._width/2 and z <= self._height/2 and z >= -self._height/2: 
            # adds coord to set list + the grid itself
            self._coordinates.add(coord)
            if x not in self.grid_array:
                self.grid_array[x] = {y: {z: val}}
            
            if y not in self.grid_array[x]:
                self.grid_array[x][y] = {z: val}
            
            if z not in self.grid_array[x][y]:
                self.grid_array[x][y][z] = val
    
    def find_value(self, coord):
        if coord in self.coordinates:
            return self.grid_array[coord[0]][coord[1]][coord[2]]
        else:
            raise ValueError("Coordinate is not in the grid.")
    
    def generate_array(self):
        """
        Generates the base grid prior to adding obstacles.
        The default obstacle value for each coordinate is 0, meaning that
        there is no obstacle here.

        Each coordinate is spaced 1 centimeter apart.
        """ 

        grid_template = {}

        # generating the base grid
        # to account for floats, coordinates to the exact float are added too and not rounded away 
    
        x_range, y_range, z_range = self._length / 2, self._width / 2, self._height / 2

        for x in np.arange(-np.floor(x_range), np.floor(x_range)):
            for y in np.arange(-np.floor(y_range), np.floor(y_range)):
                for z in np.arange(-np.floor(z_range), np.floor(z_range)):
                    self.add_coord((x,y,z))
                    self.add_coord((-x_range, y, z))
                    self.add_coord((x_range, y, z))
                    self.add_coord((x, -y_range, z))
                    self.add_coord((x, y_range, z))
                    self.add_coord((x, y, -z_range))
                    self.add_coord((x, y, -z_range))

        # all 8 corner vertices
        coord_list = [(i,j,k) for i in (-x_range, x_range) for j in (-y_range, y_range) for k in (-z_range, z_range)]
        for coord in coord_list: 
            self.add_coord(coord)


        return grid_template


    def create_obstacle(self, coord1, coord2, coord3, distanceFromObstacle=5, step=0.1):
        """
        Generates a rectangular obstacle from the three coordinates given.

        Parameters:
        coord1, coord2, coord3 = three coordinates of the obstacle from any vertex
        distanceFromObstacle = the distance away from the obstacle. Used to generate fine mesh 
        coordinates of a certain width.
        steps = the dimension of the new cube

        """

        # Makes sure to add obstacle coordinates if not already in grid
        # so that any coordinates inside are not ignored
        if coord1 or coord2 or coord3 not in self.coordinates:
            self.add_coord(coord1, 1)
            self.add_coord(coord2, 1)
            self.add_coord(coord3, 1)
            

        # Maximum and minimum coordinate value for each dimension for the obstacle
        maxes_mins = tuple(zip(map(min, zip(coord1, coord2, coord3)), map(max, zip(coord1, coord2, coord3))))
       
        # function for finding the available keys within the range of the max and min for each coordinate
        # we don't want to create new coordinates within the obstacle
        find_avail_vals = lambda idx, dictionary: filter(lambda el: el>=maxes_mins[idx][0] and el<=maxes_mins[idx][1], dictionary.keys())

      
        # changing all of the vals to 1 to indicate obstacle
        for x in find_avail_vals(0, self.grid_array):
            for y in find_avail_vals(1, self.grid_array[x]):
                for z in find_avail_vals(2, self.grid_array[x][y]):
                    self.add_coord((x,y,z), 1)

        
        def add_finer_mesh():
            """
            Once obstacles are added, new coordinates are added to the graph for a finer mesh around these points.
            Parameters distanceFromObstacle and step are used here.
            
            """
            # function that creates the range for the surrounding area of the obstacle

            num = int(distanceFromObstacle/step)
            new_range = lambda range: np.concatenate((\
            np.linspace(range[0] - distanceFromObstacle, range[0], num, endpoint=False), \
            np.linspace(range[1] + step, range[1] + distanceFromObstacle, num)))

            new_x_range = new_range(maxes_mins[0])
            new_y_range = new_range(maxes_mins[1])
            new_z_range = new_range(maxes_mins[2])
            
            
            for i in new_x_range:
                for j in new_y_range:
                    for k in new_z_range:
                        self.add_coord((i,j,k))
                        
        add_finer_mesh()
        