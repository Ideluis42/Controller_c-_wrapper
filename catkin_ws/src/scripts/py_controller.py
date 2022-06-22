import numpy as np


class Logger():
    def __init__(self):
        self.log = {} # Log of all data
        self.logentry = {} # Data from this time step
        self.length = 0

    def __len__(self):
        return self.length

    def __call__(self, field : str, value):
        ''' Write value with name field in temporary memory '''
        self.logentry[field] = value

    def record(self):
        ''' Save all the values stored in the temporary memory as a new step in the log '''
        for field in self.logentry:
            try:
                # Note that we could use log.get(field, <default>), but that would evaluate <default> every time and be slow
                # self.log[field] = self.log[field].append(self.logentry[field]) #np.append(self.log[field], [self.logentry[field]], axis=0)
                self.log[field].append(self.logentry[field]) #np.append(self.log[field], [self.logentry[field]], axis=0)
            except KeyError:
                # First, create the array we are going to append to; back fills the previous entries with nan's
                self.log[field] = []
                for i in range(self.length): # backfilling default
                    self.log[field].append(np.nan * np.ones(shape=self.logentry[field].shape))
                # self.log[field] = np.nan * np.ones((len(self),) + self.logentry[field].shape) # slower way of doing this

                # Write the las field with the recorded value
                self.log[field].append(self.logentry[field]) 
                # self.log[field] = np.append(self.log[field], [self.logentry[field]], axis=0) # slower way of doing this
        self.length = self.length + 1

    def get_log(self):
        for key, value in self.log.items():
            self.log[key] = np.stack(value, axis=0)
        return self.log

class DummyLogger():
    def __init__(self):
        pass

    def __len__(self):
        return 0

    def __call__(self, *arg, **kwargs):
        ''' Write value with name field in temporary memory '''
        pass

    def record(self):
        ''' Save all the values stored in the temporary memory '''
        pass

    def get_log(self):
        return None

class Vehicle():
    P : np.ndarray
    V : np.ndarray


    def __init__(self, log = DummyLogger()):
        self.log = log
        pass

    def reset(self):
        ''' Return the control to its initial state '''
        raise NotImplementedError

    def dynamics(self, U, t) -> np.ndarray:
        ''' Computes and returns the acceleration vector, V_dot '''
        raise NotImplementedError

    def measure(self, t) -> np.ndarray:
        ''' Computes the system output, nominally the full state '''
        return np.concatenate(self.P, self.V)

    def normalize(self, t) -> np.ndarray:
        ''' For overdefined systems, reinforce constraints on the state '''
        pass

class Controller():
    ''' Abstract base class for controller implementations. '''
    def __init__(self, log = DummyLogger()):
        self.log = log
        pass

    def reset(self):
        raise NotImplementedError

    def __call__(self, Y, R, t, **kwargs):
        raise NotImplementedError

    def start_adaptation(self):
        pass

    def stop_adaptation(self):
        pass

    # added by me to test the getter--works though
    def test(self, x, y, z, **kwargs):
        print("within test")
        return 1

class Planner():
    def __init__(self) -> None:
        pass

    def __call__(self, t, **kwargs) -> np.ndarray:
        raise NotImplementedError

DEFAULT_START_TIME = 0.
DEFAULT_END_TIME = 60.
DEFAULT_INTEGRATION_STEP = 1e-3

def run(vehicle : Vehicle, controller : Controller, planner : Planner, log : Logger = None, sim_options = {}):
    ''' Run simulation with the options passed in sim_options
        Passing in log will allow the controller, vehicle, and planner to all record their data to the same log 
    '''
    # Reset all of the objects - maybe this should be handled outside the fun function?
    vehicle.reset()
    controller.reset()
    planner.reset()

    # Initialize the sim and logger
    t = sim_options.setdefault('t_start', DEFAULT_START_TIME)
    sim_options.setdefault('t_stop', DEFAULT_END_TIME)
    dt = sim_options.setdefault('integration_step', DEFAULT_INTEGRATION_STEP)
    if log is None:
        log = Logger()

    controller.start_adaptation()

    while t < sim_options['t_stop']:
        Y = vehicle.measure(t)
        R = planner(t)
        U = controller(Y, R, t)

        # step controller
        A = vehicle.dynamics(U, t)
        vehicle.V = vehicle.V + dt * A
        vehicle.P = vehicle.P + dt * vehicle.V

        vehicle.normalize(t)

        log('Y', Y)
        log('R', R)
        log('U', U)
        log('A', A)
        log('V', vehicle.V)
        log('P', vehicle.P)

        log.record()

    return log.get_log()