#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import math

# State Matrix
stateX = [];
					
class KalmanFilter:

    def __init__(self, initStateX, delT):			
        # Initial Conditions
        self.deltaT = delT;		# Delta T; Unit: Seconds
        self.x_2dot = 0;			# Acceleration in X direction; Unit: meters/sec
        
        # Observation Errors/ Possible errors with measurement
        self.deltaX = 2;		# Unit: meters
        self.deltaXdot = 1;		# Unit: meters/sec
        
        # Initial Process Errors in Process covariance matrix
        self.deltaPx = 2;		# Unit: meters
        self.deltaPxdot = 1;	# Unit: meters/sec
        
        #            _     _
        #           | x     |
        # State X = | x_dot |
        #           |_     _|
        #
        
        ## Initial State
        self.stateX = initStateX;# State Matrix
        self.obsState = 0;      # Observed State
        self.u = self.x_2dot;		# Control Variable Matrix
        self.w = 0;				# Predicted State Noise Matrix
        self.noiseFactor = 0.1; # Noise Factor for Process Noise Covariance Matrix
                                # Vary Noise Factor to tune the algorithm to follow either
                                # Prediction of Measurement closely; Higher Value->Follow Measurement closely
                                #                                    Lower Value->Follow Prediction closely
        self.alpha = 0.3;       # Forgetting Factor to adaptively estimate R & Q; Range 0 to 1
                                # Higher Value->Small update caused by past states
                                # Lower Value->Higher update caused by past states
        self.Z = 0;             # Measurement Noise
        self.stateXp = 0;		# Predicted State
        self.Pkp = 0;			# Predicted Process Covariance Matrix
        
        # Matrix A; State Transition Matrix
        self.matrixA = np.matrix([
                                  [1, self.deltaT],
                                  [0, 1]]); 	
                            
        # Matrix B
        self.matrixB = np.matrix([ (0.5*math.pow(self.deltaT, 2)), self.deltaT ]).T; 
        
        # Matrix C
        self.matrixC = np.matrix([[1, 0], [0, 1]]);
        
        # Matrix H
        self.matrixH = np.matrix([[1, 0], [0, 1]]);
        
        # Identity Matrix
        self.matrixI = np.matrix([[1, 0], [0, 1]]);
        
        # Get Sensor Noise Covariance Matrix
        self.R = self.getSensorNoiseCovarMat(self.deltaX, self.deltaXdot);
        
        # Get Initial Process Covariance Matrix
        self.Pk = self.getInitProcCovarMat(self.deltaPx, self.deltaPxdot);
        
        # Get Process Noise Covariance Matrix
        #self.Q = self.getProcNoiseCovarMat(self.noiseFactor);
        self.Q = self.matrixI
        
        ### Initial Prediction ###
        ## Prediction ##
        self.stateXp = self.getPredictedState(self.stateX);
        self.Pkp = self.predictProcCovarMat(self.Pk);
        ## Compute Kalman Gain ##
        self.kg = self.getKalmanGain(self.Pkp);
        
    def getProcNoiseCovarMat(self, noiseFactor):
        # Assuming continuous-time white noise
        procNoiseCovarMat = noiseFactor*(np.matrix([[ ((1/3)*math.pow(self.deltaT, 3)), ((1/2)*math.pow(self.deltaT, 2)) ],
                                        [ ((1/2)*math.pow(self.deltaT, 2)), self.deltaT ]]));
        return procNoiseCovarMat;
		
    def getInitProcCovarMat(self, deltaPx, deltaPxdot):
		procCovarMat = np.matrix([
								  [math.pow(deltaPx, 2), 0], 
								  [0, math.pow(deltaPxdot, 2)]
								 ]);
		return procCovarMat;
        
    def getSensorNoiseCovarMat(self, deltaX, deltaXdot):
        R = np.matrix([
                    [math.pow(deltaX, 2), 0], 
                    [0, math.pow(deltaXdot, 2)]
                    ]);
        return R;
    
    def getInnovation(self, mesStateX, predictedStateX):
        innovation = mesStateX - self.matrixH*predictedStateX;
        return innovation;
        
    def updateEstimationQ(self, mesStateX, predictedStateX, kalmanGain):
        innovation = self.getInnovation(mesStateX, predictedStateX);
        self.Q = (self.alpha*self.Q) + ((1 - self.alpha)*(kalmanGain*innovation*np.transpose(innovation)*np.transpose(kalmanGain)));
        
    def getResidual(self, mesStateX, kalmanStateX):
        residual = mesStateX - self.matrixH*kalmanStateX;
        return residual;
        
    def updateEstimationR(self, mesStateX, kalmanStateX, procCovarMat):
        residual = self.getResidual(mesStateX, kalmanStateX);
        self.R = (self.alpha*self.R) + (1 - self.alpha)*((residual*np.transpose(residual) + self.matrixH*procCovarMat*np.transpose(self.matrixH)));
 
    def getPredictedState(self, prevStateX):
        stateXp = (self.matrixA*prevStateX) + (self.matrixB*self.u) + self.w;
        return stateXp;
        
    def predictProcCovarMat(self, prevCovarMat):
        procCovarMat = self.matrixA*prevCovarMat*np.transpose(self.matrixA) + self.Q;
        
        # Set Non-Diagonal elements to zero
        for i in range(0, procCovarMat.shape[0]):
            for j in range(0, procCovarMat.shape[1]):
                if i != j:
                    procCovarMat[i, j] = 0;
                    
        return procCovarMat;
        
    def getKalmanGain(self, procCovarMat):
        num = procCovarMat*np.transpose(self.matrixH);
        den = (self.matrixH*procCovarMat*np.transpose(self.matrixH)) + self.R;
        kg = np.true_divide(num, den, where=(num!=0) & (den!=0));
        return kg;
        
    def getObservedState(self, measuredState):
        obsState = (self.matrixC*measuredState) + self.Z;
        return obsState;
        
    def getCorrectedState(self, predictedState, kg, obsState):
        newState = predictedState + kg*(obsState - self.matrixH*predictedState);
        return newState;
        
    def updateProcCovarMat(self, kg, predictedProcCovarMat):
        procCovarMat = (self.matrixI - kg*self.matrixH)*self.Pkp;
        return procCovarMat;
        
    def getKalmanState(self, measuredStateMat):
        '''
        Kalman Filter originally suggests to Predict the state first,
        and then correct it based on measured/observed state. However,
        here I am predicting the next state in previous cycle and 
        doing correction after getting the actual measurement in next
        cycle.
        
        For classic Kalman Filter the following line will be here instead
        of being at the end:
        
        ## Prediction ##
        self.stateXp = self.getPredictedState(self.stateX);
        self.Pkp = self.predictProcCovarMat(self.Pk);
        '''
        
        ## Get observed state from the Measurement System ##
        self.obsState = self.getObservedState(measuredStateMat);
        ## Compute Kalman Gain ##
        self.kg = self.getKalmanGain(self.Pkp);
        ## Correction / Update ##
        self.stateX = self.getCorrectedState(self.stateXp, self.kg, self.obsState);
        self.Pk = self.updateProcCovarMat(self.kg, self.Pkp);
        ## Innovation Based Adaptive estimation of Q ##
        # If the velocity mean is almost constant, then the best Q estimation will
        # be obtained by getProcNoiseCovarMat(); whereas if the velocity mean keeps
        # increasing/decreasing with time, then the best Q estimation will be 
        # obtained by updateEstimationQ(). For vehicle tracking it will be best to 
        # use a constant Q as obtained by getProcNoiseCovarMat();
        self.updateEstimationQ(measuredStateMat, self.stateX, self.kg);
        ## Residual Based Adaptive Estimation of R ##
        self.updateEstimationR(measuredStateMat, self.stateX, self.Pkp);
        
        ## Prediction of the next state ##
        self.stateXp = self.getPredictedState(self.stateX);
        self.Pkp = self.predictProcCovarMat(self.Pk);
        
        print "Predicted State :\n", self.stateXp;
        print "\nPredicted Process Covariance Matrix :\n", self.Pkp;
        print "\nKalman Gain :\n", self.kg;
        print "\nObserved State :\n", self.obsState;
        print "\nKalman Corrected State :\n", self.stateX;
        print "\nProcess Covariance Matrix :\n", self.Pk;
        
        return [self.stateX, self.stateXp];

if __name__ == '__main__':
    # Generate Test Data
    numTestData = 1000;
    positionX = 0;
    velocityX = 20;
    accelX = 0.1;
    deltaT = 1;
    idealStateX = np.matrix(np.empty([numTestData, 2]));
    testStateX = np.matrix(np.empty([numTestData, 2]));
    for i in range(0, numTestData):
        velocityX = velocityX + deltaT*accelX;
        positionX = positionX + deltaT*velocityX + (1/2)*accelX*math.pow(deltaT, 2);
        idealStateX[i, 1] = velocityX;
        idealStateX[i, 0] = positionX;
        # Position dataset
        testStateX[i, 0] = idealStateX[i, 0] + np.random.normal(0, 50, 1);     #Adding Random Noise
        # Velocity dataset
        testStateX[i, 1] = idealStateX[i, 1] + np.random.normal(0, 3, 1);    #Adding Random Noise

    # Initial State Matrix
    initStateX = testStateX[0, :].T;	# Convert to a column matrix
    prevMesStateMat = np.empty([2, 1]);
    kf = KalmanFilter(initStateX, deltaT);
    stateXMat = np.empty([(testStateX.shape[0] - 1), testStateX.shape[1]]);
    predictedStateXMat = np.empty([(testStateX.shape[0]), testStateX.shape[1]]);
    
    # Initial prediction of next state
    predictedStateXMat[0] = kf.stateXp.T;
    for i in range(1, testStateX.shape[0]):
        print "\n#################################";
        print "#\t Iteration %d \t\t#" %i;
        print "#################################\n";
        mesStateMat = testStateX[i, :].T;
        [currKalmanState, nextPredictedState] = kf.getKalmanState(mesStateMat);
        prevMesStateMat = mesStateMat;
        predictedStateXMat[i] = nextPredictedState.T;
        stateXMat[i - 1] = currKalmanState.T;

    stateXMat = np.matrix(stateXMat);
    positionTrackingErr = np.empty([stateXMat.shape[0], 1]);
    velocityTrackingErr = np.empty([stateXMat.shape[0], 1]);
    for i in range (0, stateXMat.shape[0]):
        positionTrackingErr[i] = np.subtract(idealStateX[i+1, 0], stateXMat[i, 0]);
        velocityTrackingErr[i] = np.subtract(idealStateX[i+1, 1], stateXMat[i, 1]);
    
    plt.subplot(3,2,1);
    plt.plot(range(0, len(positionTrackingErr)), positionTrackingErr, 'r-', linewidth=2);
    plt.xlabel("Index");
    plt.ylabel("Position Error (meters)");
    plt.title("Position Tracking Error");
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,2,2);
    plt.plot(range(0, len(velocityTrackingErr)), velocityTrackingErr, 'r-', linewidth=2);
    plt.xlabel("Index");
    plt.ylabel("Velocity Error (meters)");
    plt.title("Velocity Tracking Error");
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,1,2);
    plt.plot(range(0, testStateX.shape[0]), testStateX[:, 0], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(range(1, predictedStateXMat.shape[0] + 1), predictedStateXMat[:, 0], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(range(1, stateXMat.shape[0] + 1), stateXMat[:, 0], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("Index");
    plt.ylabel("Position(X direction)");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Position)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    
    plt.subplot(3,1,3);
    plt.plot(range(0, testStateX.shape[0]), testStateX[:, 1], 'r-', linewidth=2, label="Observed State");
    plt.hold(True);
    plt.plot(range(1, predictedStateXMat.shape[0] + 1), predictedStateXMat[:, 1], 'g--', linewidth=2, label="Predicted State");
    plt.hold(True);
    plt.plot(range(1, stateXMat.shape[0] + 1), stateXMat[:, 1], 'b-', linewidth=2, label="Kalman Filtered State");
    plt.xlabel("Index");
    plt.ylabel("Velocity(X direction)");
    plt.title("Plots of Observed, Predicted, and Kalman filtered State (Velocity)");
    plt.legend();
    plt.grid(color='r', linestyle='--', linewidth=0.5);
    plt.show();