#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as linalg
import random

def fitLine(x, y):
    # Form Matrix A
    A = np.zeros((len(x), 3), dtype=np.float32)
    for i in range(len(x)):
        A[i][0] = x[i]
        A[i][1] = y[i]
        A[i][2] = 1
        
    # Take SVD of A
    U, Sigma, V_transpose = linalg.svd(A)
    
    # Line equation is of the form ax + by + c = 0
    # lets convert it as y = slope*x + intercept
    slope = -1*V_transpose[-1][0]/V_transpose[-1][1]
    intercept = -1*V_transpose[-1][2]/V_transpose[-1][1]
    return slope, intercept

if __name__ == '__main__':
    x = []
    y = []
    scatter = 2.5
    offset = 10.0
    for i in range(-20, 20):
        x.append(i+ scatter*random.uniform(-1.0, 1.0))
        y.append(offset + i+ scatter*random.uniform(-1.0, 1.0))
    
    plt.plot(x, y, 'ro')
    
    slope, intercept = fitLine(x, y)
    # Create a list of values in the best fit line
    x = []
    y = []
    for i in range(-20, 20):
        x.append(i)
        y.append(slope * i+intercept)
    
    plt.plot(x, y, 'b')
    plt.legend(['noisy data', 'approximated line'], loc='upper left')
    plt.show()