import numpy as np
from daepy import BVP
import matplotlib.pyplot as plt


class DAE():

    def __init__(self, alpha):
        self.N = 1
        self.dindex = [0]
        self.aindex = []
        self.alpha = alpha

    def fun(self, x, y):
        yx = y(x)
        y_prime = y.scaled_derivate(x)
        sigma = y.scaled_delay(x, 0)
        s = y.transformed_coordinate(x)

        r = np.zeros((self.N, x.shape[0]))
        r[0] = y[0](sigma) + (3 + self.alpha)*(s**(2 + self.alpha)) - s**()