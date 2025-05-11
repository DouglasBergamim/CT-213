import random
import math
from constants import *


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.time = 0.0

    def check_transition(self, agent, fsm):
        if agent.get_bumper_state():
            fsm.change_state(GoBackState())
        if self.time > MOVE_FORWARD_TIME:
            self.time = 0.0
            fsm.change_state(MoveInSpiralState())
        self.time += SAMPLE_TIME

    def execute(self, agent):
        agent.set_velocity(FORWARD_SPEED, 0.0)

class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.time = 0.0

    def check_transition(self, agent, fsm):
        if agent.get_bumper_state():
            self.time = 0.0
            fsm.change_state(GoBackState())
        if self.time > MOVE_IN_SPIRAL_TIME:
            self.time = 0.0
            fsm.change_state(MoveForwardState())
        self.time += SAMPLE_TIME

    def execute(self, agent):
        r = INITIAL_RADIUS_SPIRAL + SPIRAL_FACTOR * self.time
        w = FORWARD_SPEED/r
        agent.set_velocity(FORWARD_SPEED, w)

class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.time = 0.0

    def check_transition(self, agent, fsm):
        if self.time > GO_BACK_TIME:
            fsm.change_state(RotateState())
        self.time += SAMPLE_TIME

    def execute(self, agent):
        agent.set_velocity(BACKWARD_SPEED, 0.0)

class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.time = 0.0
        self.angular_rotation = random.uniform(-math.pi, math.pi)

    def check_transition(self, agent, fsm):
        if self.time > abs(self.angular_rotation)/ANGULAR_SPEED:
            fsm.change_state(MoveForwardState())
            self.angular_rotation = random.uniform(-math.pi, math.pi)
        self.time += SAMPLE_TIME

    def execute(self, agent):        
        if self.angular_rotation > 0:
            agent.set_velocity(0.0, ANGULAR_SPEED)
        else:
            agent.set_velocity(0.0, -ANGULAR_SPEED)