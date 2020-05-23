import rospy

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # For steering pid
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        # For throttle pid
        kp = 3
        ki = 0.1
        kd = 0.0
        mn = 0.0  # Minimum throttle value
        mx = 0.2  # Maximum throttle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Filtering out all of the high-frequency noise in the velocity
        tau = 0.5  # 1/(2pi*tau) = Cutoff frequency
        ts = 0.02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        """
        Control gets called in dbw.py
        :param current_vel:
        :param dbw_enabled:
        :param linear_vel:
        :param angular_vel:
        :return:
        """
        # Check dbw enable or not
        # As we're sitting, waiting for traffic light --> stop --> turn dbw off
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            # Carla has an automatic transmission, which means the car will roll forward if no brake and no throttle is
            # applied. To prevent Carla from moving requires about 700 Nm of torque.
            brake = 700  # N*m --> to hold the car in place if we are stopped at a light
        elif throttle < 0.1 and vel_error < 0.0:
            throttle = 0.0
            decel = max(vel_error, self.accel_limit)
            brake = abs(decel) * self.wheel_radius * self.vehicle_mass  # Torque N*m

        # Return throttle, brake, steer
        return throttle, brake, steering
