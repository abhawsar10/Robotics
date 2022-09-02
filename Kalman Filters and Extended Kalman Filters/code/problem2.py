import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv
import math

class KalmanFilter():
    """
    Implementation of a Kalman Filter.
    """
    def __init__(self, mu, sigma, A, C, R=0., Q=0.):
        """
        :param mu: prior mean
        :param sigma: prior covariance
        :param A: process model
        :param C: measurement model
        :param R: process noise
        :param Q: measurement noise
        """
        # prior
        self.mu = mu
        self.sigma = sigma
        self.mu_init = mu
        self.sigma_init = sigma
        # process model
        self.A = A
        self.R = R
        # measurement model
        self.C = C
        self.Q = Q

    def reset(self):
        """
        Reset belief state to initial value.
        """
        self.mu = self.mu_init
        self.sigma = self.sigma_init
        #print(self.mu)
        #print(self.sigma.shape)

    def run(self, sensor_data):
        """
        Run the Kalman Filter using the given sensor updates.

        :param sensor_data: array of T sensor updates as a TxS array.

        :returns: A tuple of predicted means (as a TxD array) and predicted
                  covariances (as a TxDxD array) representing the KF's belief
                  state AFTER each predict/update cycle, over T timesteps.
        """
        # FILL in your code here
        means = []
        covs = []

        
        for i  in range(len(sensor_data)):
            
            self._predict()
            self._update(sensor_data[i])
            #print(self.mu)
            means.append(self.mu.transpose()[0])
            covs.append(self.sigma)

        #print(means)
        return np.array(means),np.array(covs)



    def _predict(self):
        
        # FILL in your code here

        self.mu_bar = self.A @ self.mu

        self.sigma_bar = self.A @ self.sigma @ self.A.transpose() + self.R



    def _update(self, z):
        # FILL in your code here

        #print(self.C @self.sigma @ self.C.transpose() +self.Q)
        K_t = self.sigma_bar @ self.C.transpose() @ np.linalg.inv(self.C @self.sigma_bar @ self.C.transpose() +self.Q) 

        self.mu = self.mu_bar + K_t @ ( z - self.C @ self.mu_bar) 

        self.sigma = (np.identity(4) - K_t @ self.C ) @ self.sigma_bar
        
def plot_prediction(t, ground_truth, measurement, predict_mean, predict_cov):
    """
    Plot ground truth vs. predicted value.

    :param t: 1-dimensional array representing timesteps, in seconds.
    :param ground_truth: Tx1 array of ground truth values
    :param measurement: Tx1 array of sensor values
    :param predict_mean: TxD array of mean vectors
    :param predict_cov: TxDxD array of covariance matrices
    """
    predict_pos_mean = predict_mean[:, 0]
    predict_pos_std = predict_cov[:, 0, 0]

    plt.figure()
    plt.plot(t, ground_truth, color='k')
    plt.plot(t, measurement, color='r')
    plt.plot(t, predict_pos_mean, color='g')
    
    plt.fill_between(
        t,
        predict_pos_mean-predict_pos_std,
        predict_pos_mean+predict_pos_std,
        color='g',
        alpha=0.5)
    plt.legend(("ground truth", "measurements", "predictions"))
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.title("Predicted Values")
    plt.savefig('Predicted Values.png')
    plt.show()


def plot_mse(t, ground_truth, predict_means):
    """
    Plot MSE of your KF over many trials.

    :param t: 1-dimensional array representing timesteps, in seconds.
    :param ground_truth: Tx1 array of ground truth values
    :param predict_means: NxTxD array of T mean vectors over N trials
    """
    predict_pos_means = predict_means[:, :, 0]
    errors = ground_truth.squeeze() - predict_pos_means
    mse = np.mean(errors, axis=0) ** 2

    plt.figure()
    plt.plot(t, mse)
    plt.xlabel("time (s)")
    plt.ylabel("position MSE (m^2)")
    plt.title("Prediction Mean-Squared Error")
    plt.savefig('Prediction Mean-Squared Error.png')
    plt.show()


def problem2a():

    mu_0 = np.array([[5],[1],[0],[0]])
    
    sigma_0 = np.array([
        [10,0,0,0],
        [0,10,0,0],
        [0,0,10,0],
        [0,0,0,10]])

    A = np.array([
        [1,0.1,0,0],
        [0,1,0.1,0],
        [0,0,1,0.1],
        [0,0,0,1]])

    C = np.array([[1.0,0.0,0.0,0.0]])

    p=[]
    p_dash=[]
    t=0
    t_arr = []
    T =100
    for t in range(T):
        t_arr.append(t)
        p.append(np.sin(0.1*t))

        p_dash.append(np.sin(0.1*t) + np.random.normal(0.0, 1.0))



    KF = KalmanFilter(mu_0,sigma_0,A,C,Q=1.0)
    KF.reset()
    means,covs = KF.run(p_dash)

    plot_prediction(np.array(t_arr), 
        np.array(p), 
        np.array(p_dash), 
        means, 
        covs)
    
#----------------------------------------------------------------------
    
    KF.reset()
    N=10000
    N_means = []

    for i in range(N):

        p_dash=[]
        T =100
        for t in range(T):
            p_dash.append(np.sin(0.1*t) + np.random.normal(0.0, 1.0))
                  

        means,_ = KF.run(p_dash)
        N_means.append(means)
        KF.reset()

    plot_mse(np.array(t_arr), np.array(p), np.array(N_means))
    

def problem2b():
    
    # FILL in your code here



    mu_0 = np.array([[5],[1],[0],[0]])
    
    sigma_0 = np.array([
        [10,0,0,0],
        [0,10,0,0],
        [0,0,10,0],
        [0,0,0,10]])

    A = np.array([
        [1,0.1,0,0],
        [0,1,0.1,0],
        [0,0,1,0.1],
        [0,0,0,1]])

    R = np.array([
        [0.1,0,0,0],
        [0,0.1,0,0],
        [0,0,0.1,0],
        [0,0,0,0.1]])

    C = np.array([[1.0,0.0,0.0,0.0]])

    p=[]
    p_dash=[]
  
    t_arr = []
    T =100
    for t in range(T):
        t_arr.append(t)
        p.append(math.sin(0.1*t))

        p_dash.append(math.sin(0.1*t) + np.random.normal(0, 1.0))


    KF = KalmanFilter(mu_0,sigma_0,A,C,R=R,Q=1.0)
    means,covs = KF.run(p_dash)


    plot_prediction(t_arr, p, p_dash, means, covs)
    
#----------------------------------------------------------------------

    
    KF.reset()
    N=10000
    N_means = []

    for i in range(N):

        p_dash=[]
      
        T =100
        for t in range(T):
            p_dash.append(np.sin(0.1*t) + np.random.normal(0.0, 1.0))
                      

        means,_ = KF.run(p_dash)
        N_means.append(means)
        KF.reset()

    plot_mse(np.array(t_arr), np.array(p), np.array(N_means))


if __name__ == '__main__':
    problem2a()
    problem2b()
