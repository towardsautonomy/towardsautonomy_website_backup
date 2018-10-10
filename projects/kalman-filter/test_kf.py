#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import math
from KalmanFilter import KalmanFilter2D as kf

if __name__ == '__main__':
    # Generate Test Data
    numTestData = 500;
    positionX = 0;
    positionY = 0;
    velocityX = 50;
    velocityY = 50;
    accelX = 0;
    accelY = 0;
    deltaT = 1;
    idealStateX = np.matrix(np.empty([numTestData, 4]));
    testStateX = np.matrix(np.empty([numTestData, 4]));
    for i in range(0, numTestData):
        ## Generate X Position and Velocity data
        velocityX = velocityX + deltaT*accelX;
        positionX = positionX + deltaT*velocityX + (1/2)*accelX*math.pow(deltaT, 2);
        
        ## Generate Y Position and Velocity data
        velocityY = velocityY + deltaT*accelY;
        positionY = positionY + deltaT*velocityY + (1/2)*accelY*math.pow(deltaT, 2);
        
        ## Create Ideal State Matrix table for comparision
        idealStateX[i, 0] = positionX;
        idealStateX[i, 1] = positionY;
        idealStateX[i, 2] = velocityX;
        idealStateX[i, 3] = velocityY;
        
        ## Add random noise to the Ideal State Matrix to get input data for Kalman Filter
        # Position dataset
        testStateX[i, 0] = idealStateX[i, 0] + np.random.normal(0, 10, 1);   
        testStateX[i, 1] = idealStateX[i, 1] + np.random.normal(0, 10, 1);  
        # Velocity dataset                  
        testStateX[i, 2] = idealStateX[i, 2] + np.random.normal(0, 1, 1);    
        testStateX[i, 3] = idealStateX[i, 3] + np.random.normal(0, 1, 1);    

    # Initial State Matrix
    initStateX = testStateX[0, :].T;	# Convert to a column matrix
    #initStateX = np.matrix([[0],[0],[0],[0]]);
    prevMesStateMat = np.empty([4, 1]);
    kalmanFilter = kf(initStateX, deltaT);
    stateXMat = np.empty([(testStateX.shape[0] - 1), testStateX.shape[1]]);
    predictedStateXMat = np.empty([(testStateX.shape[0]), testStateX.shape[1]]);
    
    # Initial prediction of next state
    predictedStateXMat[0] = kalmanFilter.stateXp.T;
    for i in range(1, testStateX.shape[0]):
        print "\n#################################";
        print "#\t Iteration %d \t\t#" %i;
        print "#################################\n";
        mesStateMat = testStateX[i, :].T;
        [currKalmanState, nextPredictedState] = kalmanFilter.getKalmanState(mesStateMat);
        prevMesStateMat = mesStateMat;
        predictedStateXMat[i] = nextPredictedState.T;
        stateXMat[i - 1] = currKalmanState.T;

    stateXMat = np.matrix(stateXMat);
    xPositionTrackingErr = np.empty([stateXMat.shape[0], 1]);
    yPositionTrackingErr = np.empty([stateXMat.shape[0], 1]);
    xVelocityTrackingErr = np.empty([stateXMat.shape[0], 1]);
    yVelocityTrackingErr = np.empty([stateXMat.shape[0], 1]);
    for i in range (0, stateXMat.shape[0]):
        xPositionTrackingErr[i] = np.subtract(idealStateX[i+1, 0], stateXMat[i, 0]);
        yPositionTrackingErr[i] = np.subtract(idealStateX[i+1, 1], stateXMat[i, 1]);
        xVelocityTrackingErr[i] = np.subtract(idealStateX[i+1, 2], stateXMat[i, 2]);
        yVelocityTrackingErr[i] = np.subtract(idealStateX[i+1, 3], stateXMat[i, 3]);
    
    ## Plot of X Position, Velocity, and Errors
    plt.figure(1);
    plt.subplot(3,2,1);
    plt.plot(range(0, len(xPositionTrackingErr)), xPositionTrackingErr, 'r-', linewidth=2);
    plt.xlabel("Index");
    plt.ylabel("X Position Error (meters)");
    plt.title("X Position Tracking Error");
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,2,2);
    plt.plot(range(0, len(xVelocityTrackingErr)), xVelocityTrackingErr, 'r-', linewidth=2);
    plt.xlabel("Index");
    plt.ylabel("X Velocity Error (meters/sec)");
    plt.title("X Velocity Tracking Error");
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,1,2);
    plt.plot(range(0, testStateX.shape[0]), testStateX[:, 0], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(range(1, predictedStateXMat.shape[0] + 1), predictedStateXMat[:, 0], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(range(1, stateXMat.shape[0] + 1), stateXMat[:, 0], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("Index");
    plt.ylabel("X Position");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Position)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,1,3);
    plt.plot(range(0, testStateX.shape[0]), testStateX[:, 2], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(range(1, predictedStateXMat.shape[0] + 1), predictedStateXMat[:, 2], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(range(1, stateXMat.shape[0] + 1), stateXMat[:, 2], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("Index");
    plt.ylabel("X Velocity");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Velocity)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);

    ## Plot of Y Position, Velocity, and Errors
    plt.figure(2);
    plt.subplot(3,2,1);
    plt.plot(range(0, len(yPositionTrackingErr)), yPositionTrackingErr, 'r-', linewidth=2);
    plt.xlabel("Index");
    plt.ylabel("Y Position Error (meters/sec)");
    plt.title("Y Position Tracking Error");
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,2,2);
    plt.plot(range(0, len(yVelocityTrackingErr)), yVelocityTrackingErr, 'r-', linewidth=2);
    plt.xlabel("Index");
    plt.ylabel("Y Velocity Error (meters)");
    plt.title("Y Velocity Tracking Error");
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,1,2);
    plt.plot(range(0, testStateX.shape[0]), testStateX[:, 1], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(range(1, predictedStateXMat.shape[0] + 1), predictedStateXMat[:, 1], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(range(1, stateXMat.shape[0] + 1), stateXMat[:, 1], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("Index");
    plt.ylabel("Y Position");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Position)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,1,3);
    plt.plot(range(0, testStateX.shape[0]), testStateX[:, 3], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(range(1, predictedStateXMat.shape[0] + 1), predictedStateXMat[:, 3], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(range(1, stateXMat.shape[0] + 1), stateXMat[:, 3], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("Index");
    plt.ylabel("Y Velocity");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Velocity)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    ## Plot of X and Y Position and Velocity
    plt.figure(3);
    plt.subplot(2,1,1);
    plt.plot(testStateX[:, 0], testStateX[:, 1], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(predictedStateXMat[:, 0], predictedStateXMat[:, 1], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(stateXMat[:, 0], stateXMat[:, 1], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("X Position");
    plt.ylabel("Y Position");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Position)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(2,1,2);
    plt.plot(testStateX[:, 2], testStateX[:, 3], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(predictedStateXMat[:, 2], predictedStateXMat[:, 3], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(stateXMat[:, 2], stateXMat[:, 3], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("X Velocity");
    plt.ylabel("Y Velocity");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Velocity)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.show();