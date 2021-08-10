#--**coding:utf-8**--
# Author : Mark
# time   : 2021/7/13 16:02
# File   : follow_nosignaljuntioncrossing_vehicle.py

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      SyncArrival,
                                                                      WaypointFollower)

from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill,
                                                                               InTriggerRegion)
from srunner.scenariomanager.timer import TimeOut
from srunner.tools.scenario_helper import get_waypoint_in_distance
from srunner.scenarios.basic_scenario import BasicScenario


class Follow_NoSignalJunctionCrossing_Vehicle(BasicScenario):
    """
    依然使用跟车场景
    """
    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    timeout = 120

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=1000):

        """
        设置所有相关参数并创建场景
        """
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_transform = None
        self.timeout = timeout
        self._first_vehicle_speed = 15
        self._first_vehicle_location = 25
        # self._second_vehicle_velocity = 15
        # self._other_actor_target_velocity = 15
        self._other_actor_stop_in_front_intersection = 0
        self._other_actor_max_brake = 1.0


        self._other_actor_target_velocity = 15  # npc车辆的速度

        super(Follow_NoSignalJunctionCrossing_Vehicle, self).__init__("Follow_NoSignalJunctionCrossing_Vehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)  #如果设定了随机值，则随机一个初始距离

            # Example code how to randomize start location 示例：如何随机起始位置
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        '''
        自定义初始化
        '''
        # 先跟车  test1

        # first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        # self._other_actor_transform = carla.Transform(
        #     carla.Location(first_vehicle_waypoint.transform.location.x,
        #                    first_vehicle_waypoint.transform.location.y,
        #                    first_vehicle_waypoint.transform.location.z + 1),
        #     first_vehicle_waypoint.transform.rotation)
        #
        # first_vehicle_transform = carla.Transform(
        #     carla.Location(self._other_actor_transform.location.x,
        #                    self._other_actor_transform.location.y,
        #                    self._other_actor_transform.location.z - 500),
        #     self._other_actor_transform.rotation)
        # first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        # first_vehicle.set_simulate_physics(enabled=False)
        # self.other_actors.append(first_vehicle)

        '''
        下面设置的第一种场景的位置出现错误
        '''
        #先跟车  test2
        # self._other_actor_transform = config.other_actors[0].transform  # actor变换的位置和方向
        # first_vehicle_transform = carla.Transform(
        #     carla.Location(config.other_actors[0].transform.location.x,
        #                    config.other_actors[0].transform.location.y,
        #                    config.other_actors[0].transform.location.z - 500),
        #     config.other_actors[0].transform.rotation)
        # first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        # first_vehicle.set_simulate_physics(enabled=False)
        # self.other_actors.append(first_vehicle)

        # 后十字路口 test1  test2
        # self._other_actor_transform = config.other_actors[1].transform  # actor变换的位置和方向
        # second_vehicle_transform = carla.Transform(
        #     carla.Location(config.other_actors[1].transform.location.x,
        #                    config.other_actors[1].transform.location.y,
        #                    config.other_actors[1].transform.location.z - 500),
        #     config.other_actors[1].transform.rotation)
        # second_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[1].model, second_vehicle_transform)
        # second_vehicle.set_simulate_physics(enabled=False)
        # self.other_actors.append(second_vehicle)

        # # add actors from xml file test 3
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        # following vehicle, tesla  test3.1
        # first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        # self._other_actor_transform = carla.Transform(
        #     carla.Location(first_vehicle_waypoint.transform.location.x,
        #                    first_vehicle_waypoint.transform.location.y,
        #                    first_vehicle_waypoint.transform.location.z + 1),
        #     first_vehicle_waypoint.transform.rotation)
        #
        # first_vehicle_transform = carla.Transform(
        #     carla.Location(self._other_actor_transform.location.x,
        #                    self._other_actor_transform.location.y,
        #                    self._other_actor_transform.location.z - 500),
        #     self._other_actor_transform.rotation)
        # first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        # first_vehicle.set_simulate_physics(enabled=False)
        # self.other_actors.append(first_vehicle)

        #  following vehicle, tesla test3.2
        self._other_actor_transform1 = config.other_actors[0].transform  # actor变换的位置和方向
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        # intersection vehicle test3
        self._other_actor_transform2 = config.other_actors[1].transform  # actor变换的位置和方向
        second_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[1].transform.location.x,
                           config.other_actors[1].transform.location.y,
                           config.other_actors[1].transform.location.z - 500),
            config.other_actors[1].transform.rotation)
        second_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[1].model, second_vehicle_transform)
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)


    def _create_behavior(self):
        """
        后面定义的场景是"跟车场景"。调用这个场景后，它会等待用户控制的车辆进入起始区域，然后让另一个actor驶向障碍物。一旦障碍物清除道路，让另一个演员开车前往下一个路口。
        最后，用户控制的车辆必须足够接近其他参与者才能结束场景。如果这在 60 秒内没有发生，超时将停止场景
        """
        # stage1 跟车
        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform1)

        driving_to_next_intersection = py_trees.composites.Parallel(  # 前往路口
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))

        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))

        # stop vehicle 调用atomic_behaviors中的StopVehicle类
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")

        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # stage2 十字路口
        start_other_trigger = InTriggerRegion(  # ego触发区域
            self.ego_vehicles[0],
            -80, -70,
            -75, -60)

        sync_arrival = SyncArrival(  # 同时到达交叉点
            self.other_actors[1], self.ego_vehicles[0],
            carla.Location(x=-74.63, y=-136.34))

        pass_through_trigger = InTriggerRegion(  # ego通过触发区域
            self.ego_vehicles[0],
            -90, -70,
            -124, -119)
        keep_velocity_other = KeepVelocity(  # ncp 车辆的速度为恒定
            self.other_actors[1],
            self._other_actor_target_velocity)

        stop_other_trigger = InTriggerRegion(  # ncp 车辆的触发区域
            self.other_actors[1],
            -45, -35,
            -140, -130)

        stop_other = StopVehicle(  # npc 车辆停车
            self.other_actors[1],
            self._other_actor_max_brake)

        end_condition = InTriggerRegion(  # ego 车辆终止区域
            self.ego_vehicles[0],
            -90, -70,
            -170, -156
        )

        endcondition_part3 = end_condition
        endcondition.add_child(endcondition_part3)

        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # Build behavior tree 建立行为树
        root = py_trees.composites.Parallel("All Behaviors Trees",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sequence1 = py_trees.composites.Sequence("Sequence1 Behavior")
        sequence1.add_child(start_transform)
        sequence1.add_child(driving_to_next_intersection)
        sequence1.add_child(stop)

        sequence1.add_child(ActorDestroy(self.other_actors[0]))

        sequence2 = py_trees.composites.Sequence("Sequence2 Behavior")
        sequence2.add_child(ActorTransformSetter(self.other_actors[1], self._other_actor_transform2))
        sequence2.add_child(start_other_trigger)
        sequence2.add_child(sync_arrival_parallel)
        sequence2.add_child(keep_velocity_other_parallel)
        sequence2.add_child(stop_other)

        sequence2.add_child(ActorDestroy(self.other_actors[1]))

        sync_arrival_parallel.add_child(sync_arrival)  # 同时到达设置为 Parallel节点
        sync_arrival_parallel.add_child(pass_through_trigger)  # 通过触发区域设置为Parallel节点
        keep_velocity_other_parallel.add_child(keep_velocity_other)  # npc车速设置为恒速为Parallel节点
        keep_velocity_other_parallel.add_child(stop_other_trigger)  # npc车辆到达触发区域为 Parallel节点

        root.add_child(sequence1)
        root.add_child(sequence2)
        root.add_child(endcondition)
        return root


    def _create_test_criteria(self):
        """
        创建所有的测试标准的列表，稍后在进行行为树中使用
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        删除所有的actor
        """
        self.remove_all_actors()


