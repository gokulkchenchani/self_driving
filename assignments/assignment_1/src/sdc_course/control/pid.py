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
        acceleration = 0.0
        
        error = target_velocity_ms - current_velocity_ms
        self._error_buffer.append(error)
        integral = sum(self._error_buffer) * self._dt
        derivative = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt if len(self._error_buffer) >=2 else 0
        acceleration = np.clip(self._k_p * error + self._k_i * integral + self._k_d * derivative, 1.0, -1.0)

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

    def _calculate_lateral_error(self, vehicle_transform, waypoints):
        vehicle_position = np.array([vehicle_transform.location.x, vehicle_transform.location.y])

        closest_point, closest_index = self._find_closest_point_on_path(vehicle_position, waypoints)


        delta_x = np.max(waypoints[:, 0]) - np.min(waypoints[:, 0])
        delta_y = np.max(waypoints[:, 1]) - np.min(waypoints[:, 1])

        print(f"delta_x: {delta_x}, delta_y: {delta_y}")
        print(f"actual x : {waypoints[closest_index, 0]}, actual y : {waypoints[closest_index, 1]}")
        print(f"current x : {vehicle_position[0]}, current y : {vehicle_position[1]}")

        if delta_x > delta_y:
            lateral_error = abs(waypoints[closest_index, 1]) - abs(vehicle_position[1])
        else:
            lateral_error = abs(waypoints[closest_index, 0]) - abs(vehicle_position[0])

        return lateral_error

    def _find_closest_point_on_path(self, point, path):
        path = np.array(path)
        vectors = path[:, :2] - point[:2]
        distances = np.linalg.norm(vectors, axis=1)
        closest_index = np.argmin(distances)
        closest_point = path[closest_index, :2]

        return closest_point, closest_index
    def _pid_control(self, waypoints, vehicle_transform):
            path = np.array(waypoints)[:, :2]

            lateral_error = self._calculate_lateral_error(vehicle_transform, path)
            print(lateral_error)

            self._error_buffer.append(lateral_error)

            P = lateral_error
            I = sum(self._error_buffer) * self._dt
            D = (lateral_error - self._error_buffer[-2]) / self._dt if len(self._error_buffer) > 1 else 0.0

            steering = self._k_p * P + self._k_i * I + self._k_d * D

            print("steering", steering)

            steering = max(min(steering, 1.0), -1.0)

            ######################################################################
            ################## TODO: IMPLEMENT LATERAL PID CONTROL HERE ###########
            #######################################################################
            return steering
