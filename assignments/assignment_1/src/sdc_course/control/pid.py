from collections import deque
import math
import numpy as np
from sdc_course.utils.utility import *


class PIDLongitudinalController:
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """

        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_velocity_ms, debug=False):
        """
        Execute one step of longitudinal control to reach a given target velocity.

        :param target_velocity_ms: target velocity in m/s
        :param debug: boolean for debugging
        :return: throttle control
        """
        current_velocity_ms = get_velocity_ms(self._vehicle)

        if debug:
            print("Current velocity = {}".format(current_velocity_ms))

        return self._pid_control(target_velocity_ms, current_velocity_ms)

    def _pid_control(self, target_velocity_ms, current_velocity_ms):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

        :param target_velocity_ms:  target velocity in m/s
        :param current_velocity_ms: current velocity of the vehicle in m/s
        :return: throttle/brake control
        """
        error = target_velocity_ms - current_velocity_ms
        self._error_buffer.append(error)

        p = error
        i = sum(self._error_buffer) * self._dt
        d = (p - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >1 else 0

        acceleration = self._k_p * p + self._k_i * i + self._k_d * d

        return acceleration

class PIDLateralController:
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._error_buffer = deque(maxlen=10)

    def run_step(self, waypoints):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control
        """
        return self._pid_control(waypoints, self._vehicle.get_transform())

    def _get_steering_direction(self, v1, v2):
        """
        Note that Carla uses a left hand coordinate system, this is why a positive
        cross product requires a negative steering direction.
        :param v1: vector between vehicle and waypoint
        :param v2: vector in direction of vehicle
        :return: steering direction
        """
        cross_prod = v1[0] * v2[1] - v1[1] * v2[0]
        if cross_prod >= 0:
            return -1
        return 1

    def _pid_control(self, waypoints, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoints: local waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0

        vehicle_loc = vehicle_transform.location
        # vehicle_vec = vehicle_transform.get_forward_vector()
        # vehicle_vec = np.array([vehicle_vec.x, vehicle_vec.y, 0.0])
        print(f"vehicle location: x : {vehicle_loc.x}, y : {vehicle_loc.y}")

        #getting the nearest waypoint
        near_wp, _ = get_nearest_waypoint(self._vehicle, waypoints)
        print(f"nearest waypoint: x : {near_wp[0]}, y : {near_wp[1]}")

        # vehicle to waypoint vector
        waypoint_vehicle_vec = np.array([near_wp[0] - vehicle_loc.x, near_wp[1] - vehicle_loc.y])

        # direction vector of the vehicle
        direction_vec = np.array([np.cos(np.radians(vehicle_transform.rotation.yaw)), np.sin(np.radians(vehicle_transform.rotation.yaw))])
        
        steering_direction = self._get_steering_direction(waypoint_vehicle_vec, direction_vec)
        
        error = np.linalg.norm(waypoint_vehicle_vec) * np.linalg.norm(direction_vec) * steering_direction
        
        self._error_buffer.append(error)
        p = error
        i = sum(self._error_buffer) * self._dt if len(self._error_buffer) >1 else 0.0
        d = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >1 else 0.0

        steering = self._k_p * p + self._k_i * i + self._k_d * d
        # print("kp", self._k_p)
        print("steering", steering)

        return steering
        

