import numpy as np


class Object:
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area):
        self.mass = mass
        self.initial_velocity = initial_velocity
        self.final_velocity = 0
        self.angle_with_horizontal = angle_with_horizontal
        self.area = area
        self.distance = 0
        self.height = 0
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
        self.velocity_x_axis, self.velocity_y_axis, self.prev_velocity_y_axis = self.calculate_initial_velocities()
        while self.height >= 0:
            self.prev_height = self.height
            self.prev_velocity_y_axis = self.velocity_y_axis
            self.velocity_y_axis, self.height, self.velocity_x_axis, self.distance = self.calculate_velocities_and_range_and_height()
            self.time, self.time_flight, self.time_up, self.time_down = self.calculate_time_flight()
            if self.prev_velocity_y_axis > 0 and self.velocity_y_axis < 0:
                self.max_height = self.prev_height + (self.height - self.prev_height) * (
                    self.prev_velocity_y_axis / (self.prev_velocity_y_axis - self.velocity_y_axis))
        self.final_velocity = np.sqrt(
            self.velocity_x_axis**2+self.velocity_y_axis**2)

        return [self.name,
                self.mass,
                self.initial_velocity,
                self.area,
                self.angle_with_horizontal,
                self.time_flight,
                self.time_up,
                self.time_down,
                self.velocity_y_axis,
                self.velocity_x_axis,
                self.max_height,
                self.distance,
                self.final_velocity]

    def print_results(self):
        print(f"{self.name}")
        print(f"    Air Density: {self.air_density}")
        print(f"    Mass: {self.mass}")
        print(f"    Initial Velocity: {self.initial_velocity}")
        print(f"    Area: {self.area}")
        print(f"    Angle: {self.angle_with_horizontal}")
        print(f"    Flight Time: {self.time_flight}")
        print(f"    Time_up: {self.time_up}")
        print(f"    Time_down: {self.time_down}")
        print(f"    Velocity y axis: {self.velocity_y_axis}")
        print(f"    Velocity x axis: {self.velocity_x_axis}")
        print(f"    Height: {self.max_height}")
        print(f"    Range: {self.distance}")
        print(f"    Final Velocity: {self.final_velocity}")


class Sphere(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area)
        self.name = "Sphere"
        self.drag_coefficient = 0.47


class FlatPlate(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area)
        self.name = "Flat Plate"
        self.drag_coefficient = 1.28


class Cube(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area)
        self.name = "Cube"
        self.drag_coefficient = 1.05


class Cylinder(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area)
        self.name = "Cylinder"
        self.drag_coefficient = 0.82


class Cone(Object):
    def __init__(self, mass, initial_velocity, angle_with_horizontal, area):
        super().__init__(mass, initial_velocity, angle_with_horizontal, area)
        self.name = "Cone"
        self.drag_coefficient = 0.50


sphere = Sphere(0.43, 35, 20, 0.038)
results_sphere = sphere.simulate()
sphere.print_results()

cylinder = Cylinder(0.43, 35, 20, 0.038)
results_cylinder = cylinder.simulate()
cylinder.print_results()

cube = Cube(0.43, 35, 20, 0.038)
results_cube = cube.simulate()
cube.print_results()

flat_plate = FlatPlate(0.43, 35, 20, 0.038)
results_flat_plate = flat_plate.simulate()
flat_plate.print_results()

cone = Cone(0.43, 35, 20, 0.038)
results_cone = cone.simulate()
cone.print_results()
