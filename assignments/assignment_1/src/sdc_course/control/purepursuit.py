import math
import numpy as np
from sdc_course.utils.utility import *


class PurePursuitLateralController:
    """
    PurePursuitLateralController implements lateral control using the pure pursuit controller.
    """

    def __init__(self, vehicle, L, ld, K_pp):
        self._vehicle = vehicle
        self._L = L
        self._ld = ld
        self._k_pp = K_pp

    def run_step(self, waypoints):
        return self._pure_pursuit_control(waypoints, self._vehicle.get_transform())

    def _get_goal_waypoint_index(self, vehicle, waypoints, lookahead_dist):
        for i in range(len(waypoints)):
            dist = compute_distance_to_waypoint(vehicle, waypoints[i])
            if dist >= lookahead_dist:
                return max(0, i)
        return len(waypoints) - 1

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

    def _pure_pursuit_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        vehicle_loc = vehicle_transform.location
        # vehicle_vec = vehicle_transform.get_forward_vector()
        # vehicle_vec = np.array([vehicle_vec.x, vehicle_vec.y, 0.0])
        print(f"vehicle location: x : {vehicle_loc.x}, y : {vehicle_loc.y}")
    
        waypoint = waypoints[self._get_goal_waypoint_index(self._vehicle, waypoints, self._ld)]
        print(f"waypoint: x : {waypoint[0]}, y : {waypoint[1]}")

        # vehicle to waypoint vector
        waypoint_vehicle_vec = np.array([waypoint[0] - vehicle_loc.x, waypoint[1] - vehicle_loc.y])
        
        alpha = np.arctan2(waypoint_vehicle_vec[1], waypoint_vehicle_vec[0]) - np.radians(vehicle_transform.rotation.yaw)
        
        cte = np.sin(alpha) * self._ld
        k = 2 * cte / (self._ld ** 2)

        steering = np.arctan2(k * self._L, 1)
        print("steering", steering)

        return steering
