'''
PID Controller Code
'''


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

        self.num_steps = tf/dt
        self.times = np.linspace(0, tf, self.num_steps)

        self.integrations = [0] * len(self.times)
        self.error = [0] * len(self.times)

    def counter(f):
        global calls
        calls = 2
        def g(*args, **kwargs):
            global calls
            calls += 1
            return f(*args, **kwargs)
        return g

    @counter
    def get_new_position(self, curr_pos, curr_vel, curr_accel):
        e = self.posCmd - curr_pos
        self.integrations[calls - 1] = (self.error[calls] + self.error[calls - 1])*self.dt/2
        ctrl = self.Kp*e + self.Ki * sum(self.integrations) + self.Kd*((self.error[calls] - self.error[calls-1])/self.dt)
        force_tot = ctrl # need to factor in the other forces...rip me who sucks at physics
        new_accel = force_tot/self.mass
        new_vel = curr_vel + new_accel*self.dt
        new_pos = curr_pos + new_vel*self.dt

        return new_pos

    def __call__(self):

        # print("def controller")

        # print("initializing")
        num_steps = self.tf/self.dt
        times = np.linspace(0, self.tf, num_steps)

        acceleration = [0] * len(times) # m/s^2
        velocity = [0] * len(times) # m/s
        position = [0] * len(times) # m

        integrations = [0] * len(times)
        error = [0] * len(times)
        

        counter = 2
        # print("finished initializing")
    

        while counter < len(times):
            # print(f"inside while loop: counter = {counter}")

            e = self.posCmd - position[counter-1]
            integrations[counter-1] = (error[counter] + error[counter - 1]) \
                *self.dt/2
            ctrl = self.Kp * e + self.Ki * sum(integrations) + \
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



