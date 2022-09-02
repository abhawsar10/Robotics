import matplotlib.pyplot as plt
import numpy as np


class ExtendedKalmanFilter():
    """
    Implementation of an Extended Kalman Filter.
    """
    def __init__(self, mu, sigma, g, g_jac, h, h_jac, R=0., Q=0.):
        """
        :param mu: prior mean
        :param sigma: prior covariance
        :param g: process function
        :param g_jac: process function's jacobian
        :param h: measurement function
        :param h_jac: measurement function's jacobian
        :param R: process noise
        :param Q: measurement noise
        """
        # prior
        self.mu = mu
        self.sigma = sigma
        self.mu_init = mu
        self.sigma_init = sigma
        # process model
        self.g = g
        self.g_jac = g_jac
        self.R = R
        # measurement model
        self.h = h
        self.h_jac = h_jac
        self.Q = Q

    def reset(self):
        """
        Reset belief state to initial value.
        """
        self.mu = self.mu_init
        self.sigma = self.sigma_init

    def run(self, sensor_data):
        """
        Run the Kalman Filter using the given sensor updates.

        :param sensor_data: array of T sensor updates as a TxS array.

        :returns: A tuple of predicted means (as a TxD array) and predicted
                  covariances (as a TxDxD array) representing the KF's belief
                  state AFTER each predict/update cycle, over T timesteps.
        """
        # FILL in your code here

        means=[]
        covs=[]
        self.mu_bar = np.array([[0],[0]])


        for i  in range(len(sensor_data)):
            
            
            self.g = np.array( [ self.mu[1][0] * self.mu[0][0], [self.mu[1][0]] ] )

            self.g_jac = np.array([ [self.mu[1][0] , self.mu[0][0] ], [ 0 , 1] ])

            self.h = ((self.mu[0][0])**2+1)**(1/2)
            self.h_jac = np.array( [[self.mu[0][0]/(self.h) , 0]] )


            self._predict()
            self._update(sensor_data[i])
            #print(self.mu)
            means.append(self.mu.transpose()[0])
            covs.append(self.sigma)


        #print(np.array(means))
        #print(np.array(covs).shape)
        return np.array(means),np.array(covs)




        return

    def _predict(self):
        # FILL in your code here

        self.mu_bar[0] = self.mu[1][0] * self.mu[0][0]
        self.mu_bar[1] = self.mu[1][0]

        self.sigma_bar = self.g_jac @ self.sigma @ self.g_jac.T + self.R *np.identity(2)

        
        
        return

    def _update(self, z):
        # FILL in your code here



        K = self.sigma_bar @ self.h_jac.T @ np.linalg.inv((self.h_jac @ self.sigma_bar @ self.h_jac.T + self.Q))

        self.mu = self.mu_bar + K* (z - (self.mu_bar[0][0]**2 + 1)**(1/2) )
        self.sigma = (np.identity(2) - K @ self.h_jac) @ self.sigma_bar

        return


def plot_prediction(t, ground_truth, predict_mean, predict_cov):
    """
    Plot ground truth vs. predicted value.

    :param t: 1-dimensional array representing timesteps, in seconds.
    :param ground_truth: Tx1 array of ground truth values
    :param predict_mean: TxD array of mean vectors
    :param predict_cov: TxDxD array of covariance matrices
    """
    gt_x, gt_a = ground_truth[:, 0], ground_truth[:, 1]
    pred_x, pred_a = predict_mean[:, 0], predict_mean[:, 1]
    pred_x_std = np.sqrt(predict_cov[:, 0, 0])
    pred_a_std = np.sqrt(predict_cov[:, 1, 1])

    #print(pred_x_std)


    plt.figure(figsize=(7, 10))
    plt.subplot(211)
    plt.plot(t, gt_x, color='k')
    plt.plot(t, pred_x, color='g')
    
    plt.fill_between(
        t,
        pred_x-pred_x_std,
        pred_x+pred_x_std,
        color='g',
        alpha=0.5)
    
    plt.legend(("ground_truth", "prediction"))
    plt.xlabel("time (s)")
    plt.ylabel(r"$x$")
    plt.title(r"EKF estimation: $x$")

    plt.subplot(212)
    plt.plot(t, gt_a, color='k')
    plt.plot(t, pred_a, color='g')
    plt.fill_between(
        t,
        pred_a-pred_a_std,
        pred_a+pred_a_std,
        color='g',
        alpha=0.5)
    plt.legend(("ground_truth", "prediction"))
    plt.xlabel("time (s)")
    plt.ylabel(r"$\alpha$")
    plt.title(r"EKF estimation: $\alpha$")

    plt.show()


def problem3():
    # FILL in your code here


    #------------------ GROUND TRUTH VALUES---------------------------

    p=[]
    p_dash=[]
    t=0
    T =20
    t_arr = []

    t_arr.append(t)
    p.append(2)
    t+=0.1

    for i in range(1,T):

        t_arr.append( t )

        p_dash.append( np.sqrt( p[-1]**2 + 1 ) + np.random.normal(0,1.0))
        p.append( 0.1* p[-1])
        t+=0.1

    p_dash.append( np.sqrt( p[-1]**2 + 1 ))

    new_col = np.array([[0.1] for x in range(20)]) 
    #print("newcol",new_col)


    t = np.array([t_arr]).T
    p = np.array([p]).T
    pd = np.array([p_dash]).T

    p = np.append(p,new_col,axis=1)    
   
   # print("t",t)
   # print("p",p )
   # print("pd", pd)

    #-----------------------------------------------------


    mu_0 = np.array([[1],[2]])
    
    sigma_0 = np.array([[2,0],[0,2]])

    g= np.array([[2],[2]])
    g_jac = np.array([[2,1],[0,1]])

    h = (2)**(1/2)
    h_jac = np.array([[1/(2**(1/2)) , 0 ]])

    EKF = ExtendedKalmanFilter(mu_0, sigma_0, g, g_jac, h, h_jac, Q=1.0, R=0.5)

    means,covs = EKF.run(p_dash)
    #def __init__(self, mu, sigma, g, g_jac, h, h_jac, R=0., Q=0.):

    #print(t.shape,p.shape)
    plot_prediction(np.array(t_arr),p, means, covs)


"""
    Plot ground truth vs. predicted value.

    :param t: 1-dimensional array representing timesteps, in seconds.
    :param ground_truth: Tx1 array of ground truth values
    :param predict_mean: TxD array of mean vectors
    :param predict_cov: TxDxD array of covariance matrices
    """


if __name__ == '__main__':
    problem3()
