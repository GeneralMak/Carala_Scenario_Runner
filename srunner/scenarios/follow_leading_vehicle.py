#!/usr/bin/env python
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Follow leading vehicle scenario:跟车场景

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle follows a leading car driving down
a given road. At some point the leading car has to slow down and
finally stop. The ego vehicle has to react accordingly to avoid
a collision. The scenario ends either via a timeout, or if the ego
vehicle stopped close enough to the leading vehicle

该场景实现了一种常见的驾驶行为，其中用户控制的自我车辆跟随前车行驶给定的道路。
在某些时候，领先的汽车必须减速并终于停下了。 自我车辆必须相应地做出反应以避免
碰撞。 该场景通过超时结束，或者如果自我车辆停在离前车足够近的地方
"""

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


class FollowLeadingVehicle(BasicScenario):

    """
    此类包含简单的"跟车"所需的一切涉及两辆车的场景
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario 这是一个只有一辆测试车的场景
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=120):
        '''

        :param world:         代表carla中的世界
        :param ego_vehicles: 场景中ego车辆的列表
        :param config:       场景配置(ScenarioConfiguration)
        :param randomize:
        :param debug_mode:
        :param criteria_enable:
        :param timeout:
        '''
        """
        设置所有相关参数并创建场景
        Setup all relevant parameters and create scenario
        如果randomize = True,则场景参数随机化
        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        # print(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicle, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location 示例：如何随机起始位置
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization 自定义初始化
        初始化方法旨在设置场景和所有车辆所需的参数，包括选择车辆，车辆生成的位置，
        """
        '''
        get_waypoint_in_distance:从当前actor的位置获取给定距离内的航路点
        注意：搜索在第一个路口停止
        返回航路点和行驶距离。
        '''
        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)

        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        后面定义的场景是“跟随领先车辆”的场景。然后调用这个场景后，它会等待用户控制的车辆进入起始区域，
        然后让另一个actor驾驶直到到达下一个路口，最后，用户控制的车辆必须足够接近其他参与者(actor)才能结束场景。如果这在 60 秒内没有发生，超时将停止场景
        行为树使用atomic_scenario_behavior.py中定义的行为
        """
        '''
        ActorTransformSetter:该类用于设置actor的行为转换
            重要参数：
                actor:执行行为的actor
                transform:actor变换的目标(位置+方向)
                physics[optional]:如果physics为真，将重新设置actor的physics
                当actor设置为新的actor 变换(接近1m)时，行为终止
            注意：
                防止出现运行时错误，因此确保actor生成到新变换的位置非常重要
                带有LocalPlanner 的WaypointFollower，如果在actor 真正定位到新变换之前将new_status 设置为success，则可能会失败。 
                因此：calculate_distance(actor, transform) < 1 米 
        '''
        # to avoid the other actor blocking traffic, it was spawed elsewhere 为避免与其它actor阻塞交通，让其姿势重置为所需的姿势
        # reset its pose to the required one

        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)  # 生成actor的位置

        # let the other actor drive until next intersection  让另一个演员开车到下一个路口
        # 我们应该添加一些反馈机制来响应 ego_vehicle 行为
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        driving_to_next_intersection = py_trees.composites.Parallel(   # 前往路口
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        '''
        WaypointFollower:该类是保持给定速度的同时，跟随航路点的行为类。如果未规划路径，则actor将一直跟随航路点前进
                        否则，将在到达路径规格的尽头以success结束，如果没有目标速度，则actor将以当前速度前进。
            重要参数：
                actor:执行行为的actor
                target_speed:actor的期望速度
                plan(carla.Location或carla.Waypoint, carla.agent.navigation.local_planner可选):actor应遵循的航路点规划
                blackboard_queue_name(str,可选)：即时创建额外的actor
                avoid_collision(bool,可选):Enable/Disable(默认)车辆/自行车的碰撞避免，默认为假
                name:(str,可选)：行为的名称，默认为："FollowWaypoints"   
        '''
        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        '''
        InTriggerDistanceToNextIntersection:该类为场景到下一个交叉点的距离触发器
        重要参数：
            actor:执行行为的actor
            name:触发条件的名称
            ditance:actor和下一个交叉路口的触发距离
        当actor到达下一个路口的目标距离时，条件以success终止
        '''
        # 将该行为树注释后，可以出现npc车辆闯红灯
        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))

        # stop vehicle 调用atomic_behaviors中的StopVehicle类
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        # end condition
        '''
        InTriggerDistanceToVehicle:该类为场景参与者之间的触发距离条件
        重要参数：
            actor :执行行为的actor
            reference_actor:参照actor
            distance:触发actor的距离
            name:触发条件名字
            dx, dy, dz:到reference_location的距离(reference_actor的位置)
        '''
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")

        '''
        StandStill:atomic_trigger_conditions中的场景停顿
        重要参数：
            actor :执行行为的actor
            name:触发条件名字
            duration:以秒为单位的行为持续时间
        '''
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree 建立行为树
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        创建所有测试标准的列表，标准基于atomic_scenario_criteria.py中定义的标准
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        删除所有的actors
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowLeadingVehicleWithObstacle(BasicScenario):

    """
    这个类包含一个类似于 FollowLeadingVehicle 的场景,但在领先车辆的前方有障碍物。
    This class holds a scenario similar to FollowLeadingVehicle

    but there is an obstacle in front of the leading vehicle

    This is a single ego vehicle scenario  这是一个单主车场景
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        """
        设置所有相关的参数并创建场景
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._first_actor_location = 25
        self._second_actor_location = self._first_actor_location + 41
        self._first_actor_speed = 10
        self._second_actor_speed = 1.5
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._first_actor_transform = None
        self._second_actor_transform = None

        super(FollowLeadingVehicleWithObstacle, self).__init__("FollowLeadingVehicleWithObstacle",
                                                               ego_vehicles,
                                                               config,
                                                               world,
                                                               debug_mode,
                                                               criteria_enable=criteria_enable)
        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        自定义初始化
        Custom initialization
        """

        first_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        second_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)
        first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z - 500),
            first_actor_waypoint.transform.rotation)
        self._first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z + 1),
            first_actor_waypoint.transform.rotation)
        yaw_1 = second_actor_waypoint.transform.rotation.yaw + 90
        second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z - 500),
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
                           second_actor_waypoint.transform.rotation.roll))
        self._second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z + 1),
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
                           second_actor_waypoint.transform.rotation.roll))

        first_actor = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', first_actor_transform)
        second_actor = CarlaDataProvider.request_new_actor(
            'vehicle.diamondback.century', second_actor_transform)

        first_actor.set_simulate_physics(enabled=False)
        second_actor.set_simulate_physics(enabled=False)
        self.other_actors.append(first_actor)
        self.other_actors.append(second_actor)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive towards obstacle.
        Once obstacle clears the road, make the other actor to drive towards the
        next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario

        后面定义的场景是“跟随领先车辆”场景。调用这个场景后，它会等待用户控制的车辆进入起始区域，
        然后让另一个actor驶向障碍物。一旦障碍物清除道路，让另一个演员开车前往下一个路口。
        最后，用户控制的车辆必须足够接近其他参与者才能结束场景。
        如果这在 60 秒内没有发生，超时将停止场景
        """

        # let the other actor drive until next intersection 让另一个演员开车到下一个路口
        driving_to_next_intersection = py_trees.composites.Parallel(
            "Driving towards Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        obstacle_clear_road = py_trees.composites.Parallel("Obstalce clearing road",
                                                           policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_clear_road.add_child(DriveDistance(self.other_actors[1], 4))
        obstacle_clear_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))

        stop_near_intersection = py_trees.composites.Parallel(
            "Waiting for end position near Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_near_intersection.add_child(WaypointFollower(self.other_actors[0], 10))
        stop_near_intersection.add_child(InTriggerDistanceToNextIntersection(self.other_actors[0], 20))

        '''
        WaypointFollower:该类是保持给定速度的同时，跟随航路点的行为类。如果未规划路径，则actor将一直跟随航路点前进
                        否则，将在到达路径规格的尽头以success结束，如果没有目标速度，则actor将以当前速度前进。
            重要参数：
                actor:执行行为的actor
                target_speed:actor的期望速度
                plan(carla.Location或carla.Waypoint, carla.agent.navigation.local_planner可选):actor应遵循的航路点规划
                blackboard_queue_name(str,可选)：即时创建额外的actor
                avoid_collision(bool,可选):Enable/Disable(默认)车辆/自行车的碰撞避免，默认为假
                name:(str,可选)：行为的名称，默认为："FollowWaypoints"   
        '''
        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToVehicle(self.other_actors[1],
                                                                          self.other_actors[0], 15))

        # end condition  结束条件
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree 创建行为树
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._first_actor_transform))
        sequence.add_child(ActorTransformSetter(self.other_actors[1], self._second_actor_transform))
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        sequence.add_child(TimeOut(3))
        sequence.add_child(obstacle_clear_road)
        sequence.add_child(stop_near_intersection)
        sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        创建所有测试标准的列表，稍后在并行行为树中使用
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
