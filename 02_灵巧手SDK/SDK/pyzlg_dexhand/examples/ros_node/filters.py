import numpy as np
from filterpy.kalman import KalmanFilter

class DampedVelocityKalmanFilter:
    def __init__(self, dt, process_noise_var, measurement_noise_var, damping):
        """
        Initialize the Kalman Filter for a damped velocity model.

        Parameters:
        - dt (float): Time step between measurements.
        - process_noise_var (float): Variance of the process noise.
        - measurement_noise_var (float): Variance of the measurement noise.
        - damping (float): Damping factor for the velocity model.
        """
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.dt = dt
        self.kf.F = np.array([[1, dt], [0, damping]])  # State transition matrix
        self.kf.H = np.array([[1, 0]])  # Measurement matrix
        self.kf.x = np.zeros((2, 1))  # Initial state (position, velocity)
        self.kf.P *= 1000.  # Initial large covariance
        self.kf.R = np.array([[measurement_noise_var]])  # Measurement noise
        self.kf.Q = self._calculate_process_noise(dt, process_noise_var)  # Process noise

    def _calculate_process_noise(self, dt, var):
        """ Calculate the process noise matrix Q based on the time step and variance. """
        q = np.array([[0.25*dt**4, 0.5*dt**3],
                      [0.5*dt**3, dt**2]]) * var
        return q

    def predict(self):
        """ Perform a prediction step. """
        self.kf.predict()
        return self.kf.x_prior[0, 0]

    def update(self, measurement):
        """ Update the filter with a new measurement. """
        self.kf.update(measurement)
        return self.kf.x_post[0, 0]

    def get_current_state(self):
        """ Return the current state estimate. """
        return self.kf.x_post.copy()

    def step(self, measurement):
        """ Perform a prediction step and update the filter with a new measurement. """
        self.predict()
        self.update(measurement)
        return self.kf.x_post[0, 0]

if __name__ == "__main__":
    cv_kf = DampedVelocityKalmanFilter(dt=1, process_noise_var=0.01, measurement_noise_var=5, damping=0.9)
    measurements = [1, 2, 3, 4, 5]  # Example position measurements
    for measurement in measurements:
        cv_kf.predict()
        cv_kf.update(measurement)
        print(f"Updated State: Position = {cv_kf.get_current_state()[0][0]}, Velocity = {cv_kf.get_current_state()[1][0]}")
