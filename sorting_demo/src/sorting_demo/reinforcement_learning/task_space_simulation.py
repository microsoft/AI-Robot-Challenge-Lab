import gym
from gym import utils
from gym.envs.toy_text import discrete
import numpy as np

class PositionEnv(discrete.DiscreteEnv):
    """
    Grid 5x5 world: 25 slots 4 posibles values (B,G,R,-)
    25^4 = 390625 states (390K)
    
    Actions: 25 (pick or place)
    
    The action policy is going to be: 390K 
    
    """

    def __init__(self, nS, nA, P, isd):
        super(PositionEnv, self).__init__(nS, nA, P, isd)

        self.desc = np.array()
        self.rows = 5
        self.cols = 5
        self.values = 3 + 1

        self.states = (self.rows*self.cols)^(self.values)
        self.actions = self.rows* self.cols


        # transition model
        # {action: [(probability, nextstate, reward, done)]}
        self.P = {s: {a: [] for a in range(self.actions)} for s in range(self.states)}


    def is_holand_flag(self, ):
        pass

    def encode(self):
        pass


    def step(self,action):
        pass


    def render(self):
        pass


def encode_state_aux(i, values, slots):
    if slots == 1:
        return [i]
    else:
        current = i % values
        return encode_state_aux(i / values, values, slots - 1) + [current]

slots = 16
values = 4

count = values**slots

for i in xrange(count):
    print str(i)+ ":" + str(encode_state_aux(i, values, slots))