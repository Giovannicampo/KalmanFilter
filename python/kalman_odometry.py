from matplotlib import pylab
import numpy as np
import math


class KalmanOdometry:

    def __init__(self, delta_t, wheelbase):
        self.order = 3
        self.wheelbase = wheelbase
        # state vector x y th, initial state to 0
        self.x = np.matrix([0, 0, 0]).transpose()

        self.x_r = 0
        self.y_r = 0
        self.th_r = 0

        # process covariance
        self.Q = np.eye(self.order, self.order) * 0.005

        # measure covariance (initially high)
        self.R = np.eye(self.order, self.order) * 1000

        self.H = np.matrix([ [0, 0, 0],
                             [0, 1, 0],
                             [0, 0, 0] ])
        # error covariance matrix
        self.P = np.matrix([ [0, 0, 0],
                             [0, 0, 0],
                             [0, 0, 0] ])
        # Prediction
        self.K = None


    def prediction(self, delta_left, delta_right):
        delta_l = (delta_left + delta_right) / 2.0
        delta_th = (delta_right - delta_left) / self.wheelbase

        delta_x = delta_l * math.cos(self.th_r + delta_th / 2.0)
        delta_y = delta_l * math.sin(self.th_r + delta_th / 2.0)

        self.x_r = self.x_r + delta_x
        self.y_r = self.y_r + delta_y
        self.th_r = self.th_r + delta_th

        self.A = np.matrix( [[1,  0, - delta_y],
                             [0,  1, delta_x],
                             [0,   0,  1]])

        self.x = np.matrix([self.x_r, self.y_r, self.th_r]).transpose()

        self.P = self.A * self.P * self.A.transpose() + self.Q
        S = self.H * self.P * self.H.transpose() + self.R
        self.K = (self.P * self.H.transpose()) * S.I


    def measure(self, measures):
        measures = np.matrix(measures).transpose()
        self.x = self.x + self.K * (measures - self.H * self.x)
        self.x_r = self.x.A[0][0]
        self.y_r = self.x.A[1][0]
        self.th_r = self.x.A[2][0]


    def update(self):
        self.P = (np.eye(self.order, self.order) - self.K * self.H) * self.P



delta_t = 2.5e-3  # 2.5 ms

t = 0.0

trajectory_x = []
trajectory_y = []

measure_x = []
measure_y = []

x = 0
y = 0

process_error_std = 0.005
measure_error_std = 5

predicted_trajectory_x = []
predicted_trajectory_y = []

measures = [ ]

times = [ ]

f = KalmanOdometry(delta_t, 270.0)
f.y_r = 100.0
i = 0

while t < 15:
    delta_l = 79.6
    delta_r = 79.6005

    if t > 5:
        delta_l = 0
        delta_r = 0

    f.prediction(delta_l, delta_r)

    measure =  100.0 + np.random.normal(0.0, measure_error_std)
    measures.append(measure)
    times.append(t)

    f.measure([0, measure, 0])
    f.update()

    predicted_trajectory_x.append(f.x_r)
    predicted_trajectory_y.append(f.y_r)

    t = t + delta_t
    i = i + 1

pylab.figure(1)
pylab.plot(times, measures, 'b-+', label='masures')
pylab.xlabel('t')
pylab.ylabel('d')
pylab.legend()

pylab.figure(2)
#pylab.plot(trajectory_x, trajectory_y, 'r-+', label='real trajectory')
pylab.plot(predicted_trajectory_x, predicted_trajectory_y, 'g-+', label='fitered')
pylab.xlabel('x')
pylab.ylabel('y')
pylab.legend()

pylab.show()
