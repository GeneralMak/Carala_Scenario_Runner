#--**coding:utf-8**--
# Author : Mark
# time   : 2021/7/14 21:56
# File   : slow_traffic.py

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

class SlowTraffic(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=240):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()

        # Parameters
        self._step_distance = [11, 13]  # Distance between vehicles slow traffic(m)
        self._dist_interval = [5, 7]  # distance leading vehicle TM parameter(m)
        self._drive_distance = 150  # Distance before ending the scenario (m)
        self._slow_velocity = 30  # Velocity of the slow traffic (Km/h)
        random.seed(2006)

        super(SlowTraffic, self).__init__("SlowTraffic", ego_vehicles, config,
                                          world, debug_mode,
                                          criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        spawn_transforms = []
        i = 0

        # Gets the initial and final waypoints
        init_location = config.other_actors[0].transform.location
        end_location = config.other_actors[1].transform.location

        spawn_waypoint = self._map.get_waypoint(init_location)
        sink_location = self._map.get_waypoint(end_location).transform.location

        # Add vehicles every certain distance
        while True:

            # Add it to the list
            spawn_transforms.append(spawn_waypoint)

            # Get the next
            spawn_dist = random.randrange(self._step_distance[0], self._step_distance[1])
            spawn_waypoint = spawn_waypoint.next(spawn_dist)[0]

            # Stop if close to the sink
            distance = sink_location.distance(spawn_waypoint.transform.location)
            if distance < self._step_distance[1]:
                break

        new_actors = CarlaActorPool.request_new_batch_actors("vehicle.*",
                                                             len(spawn_transforms),
                                                             spawn_transforms)
        for actor in new_actors:
            self.other_actors.append(actor)

    def _create_behavior(self):
        """
        Creates the behavior tree
        """

        sequence = py_trees.composites.Sequence("EnterHighwayWithSlowTraffic")

        parameters = {"auto_lane_change": False,
                      "max_speed": self._slow_velocity,
                      "distance_between_vehicles": 0}

        for other_actor in self.other_actors:
            distance_between_vehicles = random.randrange(
                self._dist_interval[0], self._dist_interval[1])
            parameters["distance_between_vehicles"] = distance_between_vehicles

            sequence.add_child(ChangeAutoPilot(other_actor, True, parameters=parameters))

        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._drive_distance))

        for other_actor in self.other_actors:
            sequence.add_child(ChangeAutoPilot(other_actor, False))

        for other_actor in self.other_actors:
            sequence.add_child(ActorDestroy(other_actor))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria used.
        """
        criteria = []

        collision_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criteria)

        return criteria

