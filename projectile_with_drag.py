import numpy as np
from matplotlib import pyplot as plt


class Object:
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area, height):
        self.mass = mass
        self.initial_velocity = initial_velocity
        self.final_velocity = 0
        self.angle_with_horizontal = angle_with_horizontal
        self.area = area
        self.distance = 0
        self.height = height
        self.max_height = 0
        self.prev_height = self.height
        self.time = 0
        self.time_flight = 0
        self.time_up = 0
        self.time_down = 0
        self.air_density = 1.225
        self.dt = 0.0001

    def calculate_initial_velocities(self):
        self.velocity_y_axis = self.initial_velocity * \
            np.sin(np.pi*self.angle_with_horizontal/180)
        self.velocity_x_axis = self.initial_velocity * \
            np.cos(np.pi*self.angle_with_horizontal/180)
        self.prev_velocity_y_axis = self.velocity_y_axis
        return self.velocity_x_axis, self.velocity_y_axis, self.prev_velocity_y_axis

    def calculate_acceleration(self, v1y, v1x):
        b = 0.5/self.mass*self.air_density*self.drag_coefficient*self.area
        acceleration_y = -9.8-b*np.sqrt(v1x**2+v1y**2)*v1y
        acceleration_x = -b*np.sqrt(v1x**2+v1y**2)*v1x
        return acceleration_y, acceleration_x

    def calculate_velocities_and_range_and_height(self):
        v1y = self.velocity_y_axis
        v1x = self.velocity_x_axis
        ka1y, ka1x = self.calculate_acceleration(v1y, v1x)
        v2y = self.velocity_y_axis + self.dt*ka1y/2
        v2x = self.velocity_x_axis + self.dt*ka1x/2
        ka2y, ka2x = self.calculate_acceleration(v2y, v2x)
        v3y = self.velocity_y_axis + self.dt*ka2y/2
        v3x = self.velocity_x_axis + self.dt*ka2x/2
        ka3y, ka3x = self.calculate_acceleration(v3y, v3x)
        v4y = self.velocity_y_axis + self.dt*ka3y
        v4x = self.velocity_x_axis + self.dt*ka3x
        ka4y, ka4x = self.calculate_acceleration(v4y, v4x)
        self.prev_height = self.height
        self.height = self.height + self.dt*(v1y+2*v2y+2*v3y+v4y)/6
        self.prev_velocity_y_axis = self.velocity_y_axis
        self.velocity_y_axis = self.velocity_y_axis + \
            self.dt*(ka1y+2*ka2y+2*ka3y+ka4y)/6
        self.distance = self.distance + self.dt*(v1x+2*v2x+2*v3x+v4x)/6
        self.velocity_x_axis = self.velocity_x_axis + \
            self.dt*(ka1x+2*ka2x+2*ka3x+ka4x)/6
        return self.velocity_y_axis, self.height, self.velocity_x_axis, self.distance

    def calculate_time_flight(self):
        if self.prev_velocity_y_axis > 0 and self.velocity_y_axis < 0:
            self.time_up = self.time + self.dt*self.prev_velocity_y_axis / \
                (self.prev_velocity_y_axis-self.velocity_y_axis)
        if self.prev_height > 0 and self.height < 0:
            self.time_flight = self.time + self.dt * \
                self.prev_height/(self.prev_height-self.height)

        self.time = self.time + self.dt
        if self.time_flight != 0 and self.time_up != 0:
            self.time_down = self.time_flight - self.time_up
        return self.time, self.time_flight, self.time_up, self.time_down

    def simulate(self):
        self.height_values = np.zeros(shape=(100000000))
        self.velocity_values = np.zeros(shape=(100000000))
        self.distance_values = np.zeros(shape=(100000000))
        self.time_values = np.zeros(shape=(1000000000))
        self.time_count = 0
        self.terminal_velocity_time = 0
        self.velocity_x_axis, self.velocity_y_axis, self.prev_velocity_y_axis = self.calculate_initial_velocities()
        self.velocity = self.initial_velocity
        while self.height >= 0:
            self.height_values[self.time_count] = self.height
            self.distance_values[self.time_count] = self.distance
            self.velocity_values[self.time_count] = self.velocity
            self.time_values[self.time_count] = self.time
            acceleration_y_axis, acceleration_x_axis = self.calculate_acceleration(self.velocity_y_axis, self.velocity_x_axis)
            if self.terminal_velocity_time == 0:
                if abs(acceleration_y_axis) < 0.05 and abs(acceleration_x_axis) < 0.05:   # tolerans
                    self.terminal_velocity_time = self.time
            self.time_count += 1
            self.prev_height = self.height
            self.prev_velocity_y_axis = self.velocity_y_axis
            self.velocity_y_axis, self.height, self.velocity_x_axis, self.distance = self.calculate_velocities_and_range_and_height()
            self.time, self.time_flight, self.time_up, self.time_down = self.calculate_time_flight()
            if self.prev_velocity_y_axis > 0 and self.velocity_y_axis < 0:
                self.max_height = self.prev_height + (self.height - self.prev_height) * (
                    self.prev_velocity_y_axis / (self.prev_velocity_y_axis - self.velocity_y_axis))
            self.prev_velocity = self.velocity
            self.velocity = np.sqrt(self.velocity_x_axis**2+self.velocity_y_axis**2)
            if  0.1<self.prev_velocity and 0.00001<self.dt<0.0005:
                self.criterion = np.abs(self.velocity-self.prev_velocity) / np.abs(self.prev_velocity)
                if self.criterion >= 0.03:
                    self.dt = self.dt * 0.5           
                elif self.criterion <= 0.005:
                    self.dt = self.dt * 1.2
                else:
                    pass
        if self.terminal_velocity_time == 0:
            self.terminal_velocity_time = "Terminal velocity not reached"
        self.final_velocity = np.sqrt(
            self.velocity_x_axis**2+self.velocity_y_axis**2)
        self.distance_values = self.distance_values[:self.time_count]
        self.height_values = self.height_values[:self.time_count]
        self.velocity_values = self.velocity_values[:self.time_count]
        self.time_values = self.time_values[:self.time_count]
        return {"name":self.name,
                "mass":self.mass,
                "initial velocity":self.initial_velocity,
                "area":self.area,
                "angle":self.angle_with_horizontal,
                "flight time":self.time_flight,
                "time up":self.time_up,
                "time down":self.time_down,
                "terminal_velocity_time": self.terminal_velocity_time,
                "velocity y axis":self.velocity_y_axis,
                "velocity x axis":self.velocity_x_axis,
                "height":self.max_height,
                "distance":self.distance,
                "final velocity":self.final_velocity,
                "distance_values": self.distance_values,
                "height_values": self.height_values,
                "velocity_values": self.velocity_values,
                "time_values": self.time_values}

    def print_results(self):
        print(f"{self.name}")
        print(f"    mass: {self.mass}")
        print(f"    initial velocity: {self.initial_velocity}")
        print(f"    area: {self.area}")
        print(f"    angle: {self.angle_with_horizontal}")
        print(f"    flight_time: {self.time_flight}")
        print(f"    time_up: {self.time_up}")
        print(f"    time_down: {self.time_down}")
        print(f"    terminal_velocity_time: {self.terminal_velocity_time}")
        print(f"    velocity_y_axis: {self.velocity_y_axis}")
        print(f"    velocity_x_axis: {self.velocity_x_axis}")
        print(f"    height: {self.max_height}")
        print(f"    range: {self.distance}")
        print(f"    final velocity: {self.final_velocity}")
    
    def show_graph(self, x_axis, y_axis):
        x = getattr(self, x_axis)
        y = getattr(self, y_axis)
        plt.plot(x, y)
        plt.xlabel(x_axis)
        plt.ylabel(y_axis)
        plt.grid()
        plt.show()


class Sphere(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area, height):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area, height)
        self.name = "Sphere"
        self.drag_coefficient = 0.47

class FlatPlate(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area, height):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area, height)
        self.name = "Flat Plate"
        self.drag_coefficient = 1.28

class Cube(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area, height):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area, height)
        self.name = "Cube"
        self.drag_coefficient = 1.05

class Cylinder(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area, height):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area, height)
        self.name = "Cylinder"
        self.drag_coefficient = 0.82

class Cone(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area, height):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area, height)
        self.name = "Cone"
        self.drag_coefficient = 0.50

class Vacuum(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area, height):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area, height)
        self.name = "Vacuum"
        self.drag_coefficient = 0.00
   
sphere = Sphere(0.43, 100, 36, 0.38, 10)
sphere.simulate()
sphere.print_results()
sphere.show_graph("distance_values", "height_values")
