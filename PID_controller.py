'''
PID Controller Code
'''

from calendar import c
import numpy as np

class PID():

    def __init__(self, Kp, Ki, Kd, dt, tf, mass, posCmd):
        '''
        Initializing function for controller class

        Arguments: 
            - Kp: a float representing the proportional gain
            - Ki: a float representing the integral gain
            - Kd: a float representing the derivative gain 
            - dt: a float representing the time step size
            - tf: a float representing the final time
            - mass: a float representing the mass of the object
            - posCmd: a float representing the desired position of the object
        '''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.tf = tf
        self.mass = mass
        self.posCmd = posCmd
    
    def controller(self):

        # print("def controller")

        # print("initializing")
        num_steps = self.tf/self.dt
        times = np.linspace(0, self.tf, num_steps)

        acceleration = [0] * len(times) # m/s^2
        velocity = [0] * len(times) # m/s
        position = [0] * len(times) # m

        integerations = [0] * len(times)
        error = [0] * len(times)
        

        counter = 2
        # print("finished initializing")


        while counter < len(times):
            # print(f"inside while loop: counter = {counter}")

            e = self.posCmd - position[counter-1]
            integerations[counter-1] = (error[counter] + error[counter - 1]) \
                *self.dt/2
            ctrl = self.Kp * e + self.Ki * sum(integerations) + \
                self.Kd*((error[counter] - error[counter-1])/self.dt)
            # print(f"ctrl = {ctrl}")

            force_tot = ctrl - self.Kp*position[counter-1] - \
                1*velocity[counter-1]
            acceleration[counter] = force_tot/self.mass
            velocity[counter] = velocity[counter - 1] + acceleration[counter]\
                *self.dt
            position[counter] = position[counter - 1] + velocity[counter]*\
                self.dt
            # print("finished force, accel, vel, pos, calcs")
            counter += 1
        return (position, velocity, acceleration)



