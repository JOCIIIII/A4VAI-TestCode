# Librarys

# Library for common
import numpy as np
import os
import math
import time

# ROS libraries
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
# Custom libraries
from ..lib.common_fuctions import set_initial_variables, state_logger, publish_to_plotter, set_wp
from ..lib.timer import HeartbeatTimer, MainTimer, CommandPubTimer
from ..lib.subscriber import PX4Subscriber, FlagSubscriber, CmdSubscriber, HeartbeatSubscriber, EtcSubscriber
from ..lib.publisher import PX4Publisher, HeartbeatPublisher, ModulePublisher, PlotterPublisher
from ..lib.publisher import PubFuncHeartbeat, PubFuncPX4, PubFuncModule, PubFuncPlotter
from ..lib.data_class import *

# custom message
from custom_msgs.msg import StateFlag
from px4_msgs.msg import FusionWeight
# ----------------------------------------------------------------------------------------#

class CAPFIntegrationTest(Node):
    def __init__(self):
        super().__init__("ca_pf_integ_test")
        self.get_logger().info("CAPFIntegrationTest node initialized")

        self.weight = FusionWeight()
        self.current_target = 0.0  # 0 (ca) or 1 (pf)
        self.prev_target = 0.0
        self.transition_start_time = None
        self.transition_duration_to_pf = 2  # seconds
        self.transition_duration_to_ca = 1  # seconds
        self.initial_setting = False
        # ----------------------------------------------------------------------------------------#
        # region INITIALIZE
        dir = os.path.dirname(os.path.abspath(__file__))
        sim_name = "ca_pf_integ_test"
        set_initial_variables(self, dir, sim_name)
        
        self.offboard_mode.attitude = True
        # endregion
        # -----------------------------------------------------------------------------------------#
        # region PUBLISHERS
        # PX4 publisher
        self.pub_px4 = PX4Publisher(self)
        self.pub_px4.declareVehicleCommandPublisher()
        self.pub_px4.declareOffboardControlModePublisher()
        self.pub_px4.declareAttitudeCommandPublisher()                  # Declare PX4 Attitude Command Publisher
        # self.pub_px4.declareFusionWeightPublisher() 
        self.pub_px4.declareTrajectorySetpointPublisher()
        self.weight_publisher = self.create_publisher(FusionWeight, '/fmu/in/fusion_weight', self.qos_profile_px4)

        # module data publisher
        self.pub_module = ModulePublisher(self)
        self.pub_module.declareLocalWaypointPublisherToPF()
        self.pub_module.declareModeFlagPublisherToCC()
        self.pub_module.declareVehicleModePublisher()

        self.sub_obstacle_flag = self.create_subscription(Bool, '/obstacle_flag', self.obstacle_flag_callback, 1)
        # heartbeat publisher
        self.pub_heartbeat = HeartbeatPublisher(self)
        self.pub_heartbeat.declareControllerHeartbeatPublisher()
        self.pub_heartbeat.declarePathPlanningHeartbeatPublisher()
        self.pub_heartbeat.declareCollisionAvoidanceHeartbeatPublisher()

        # Todo: plotter will be replaced with fox glove
        # # plotter publisher
        # self.pub_plotter = PlotterPublisher(self)
        # self.pub_plotter.declareGlobalWaypointPublisherToPlotter()
        # self.pub_plotter.declareLocalWaypointPublisherToPlotter()
        # self.pub_plotter.declareHeadingPublisherToPlotter()
        # self.pub_plotter.declareStatePublisherToPlotter()
        # self.pub_plotter.declareMinDistancePublisherToPlotter()
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region PUB FUNC
        self.pub_func_heartbeat = PubFuncHeartbeat(self)
        self.pub_func_px4       = PubFuncPX4(self)
        self.pub_func_module  = PubFuncModule(self)
        # self.pub_func_plotter   = PubFuncPlotter(self)
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region SUBSCRIBERS
        self.sub_px4 = PX4Subscriber(self)
        self.sub_px4.declareVehicleLocalPositionSubscriber()
        self.sub_px4.declareVehicleAttitudeSubscriber()

        self.sub_cmd = CmdSubscriber(self)
        self.sub_cmd.declarePFAttitudeSetpointSubscriber()
        self.sub_cmd.declareCAVelocitySetpointSubscriber()

        self.sub_flag = FlagSubscriber(self)
        self.sub_flag.declareConveyLocalWaypointCompleteSubscriber()
        self.sub_flag.declarePFCompleteSubscriber()

        self.rand_point_sub = self.create_subscription(
            Bool,
            "/ca_rand_point_flag",
            self.rand_point_callback,
            qos_profile_sensor_data,  # best-effort sensor QoS
        )

        self.sub_etc = EtcSubscriber(self)
        self.sub_etc.declareHeadingWPIdxSubscriber()

        self.sub_hearbeat = HeartbeatSubscriber(self)
        # Todo : Collision Avoidance Hearbeat need to be added
        # self.sub_hearbeat.declareCollisionAvoidanceHeartbeatSubscriber()
        self.sub_hearbeat.declarePathFollowingHeartbeatSubscriber()
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region TIMER
        self.timer_offboard_control = MainTimer(self)
        self.timer_offboard_control.declareOffboardControlTimer(self.offboard_control_main)

        self.timer_cmd = CommandPubTimer(self)
        self.timer_cmd.declareAttitudeCommandTimer()
        # self.timer_cmd.declareFusionWeightTimer()
        self.timer_cmd.declareOffboardVelocityControlTimer()

        self.timer_heartbeat = HeartbeatTimer(self)
        self.timer_heartbeat.declareControllerHeartbeatTimer()
        self.timer_heartbeat.declarePathPlanningHeartbeatTimer()
        self.timer_heartbeat.declareCollisionAvoidanceHeartbeatTimer()


        self.timer_weight = self.create_timer(0.01, self.weight_callback)
        # endregion
    # --------------------------------------------------------------------------------------------#
    # region MAIN CODE
    def offboard_control_main(self):
        # Todo : Offboard Control start after all heartbeat is received
        # if self.offboard_var.ca_heartbeat == True and self.offboard_var.pf_heartbeat == True:
            
        # send offboard mode and arm mode command to px4
        if self.mode_status.DISARM == True:
            self.get_logger().info('Mode Status : TAKEOFF')
            self.mode_status.TAKEOFF = True
            self.mode_status.DISARM = False
        
        if self.offboard_var.counter == self.offboard_var.flight_start_time and self.mode_status.TAKEOFF == True:
            # send arm cmd to px4
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_arm_mode)
            # send takeoff mode cmd to px4
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_takeoff_mode)

        # increment counter until the flight start time
        elif self.offboard_var.counter <= self.offboard_var.flight_start_time:
            self.offboard_var.counter += 1

        # check if the vehicle is ready to initial position
        if self.mode_status.TAKEOFF == True and self.state_var.z > self.guid_var.init_pos[2]:
            self.mode_status.TAKEOFF = False
            self.flags.path_planning = True
            self.get_logger().info('Vehicle is reached to initial position')
            set_wp(self)

        if self.flags.path_planning == True:
            self.pub_func_module.local_waypoint_publish(True)
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_position_mode)
            
            # for debugging
            # self.get_logger().info(f"pf_get_local_waypoint: {self.flags.pf_get_local_waypoint}")

        if self.flags.pf_get_local_waypoint == True and self.mode_status.OFFBOARD == False:
            self.flags.path_planning = False
            self.mode_status.OFFBOARD = True
            self.mode_status.PATH_FOLLOWING = True
            self.get_logger().info('Mode Status : OFFBOARD/Path Following')

        # check if path following is recieved the local waypoint
        if self.mode_status.OFFBOARD == True and self.flags.pf_done == False:
            if self.initial_setting == False:
                self.initial_setting = True
                self.offboard_mode.attitude = False
                self.offboard_mode.velocity = True
                self.current_target = 1.0  # 0 (ca) or 1 (pf)
                self.prev_target = 1.0
                self.weight.fusion_weight = float(1.0)
            self.pub_func_module.publish_flags()

            if self.mode_status.PATH_FOLLOWING == True:
                self.current_target = 1.0
            
            if self.mode_status.COLLISION_AVOIDANCE == True:
                self.current_target = 0.0

            self.pub_func_px4.publish_offboard_control_mode(self.offboard_mode)
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_offboard_mode)
            
        if self.flags.pf_done == True and self.mode_status.LANDING == False:
            self.get_logger().info('Mode Status : LANDING')
            self.mode_status.OFFBOARD = False
            self.mode_status.PATH_FOLLOWING = False
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_land_mode)

            # check if the vehicle is landed
            if np.abs(self.state_var.vz_n) < 0.05 and np.abs(self.state_var.z < 0.05):
                self.mode_status.LANDING = True
                self.get_logger().info('Vehicle is landed')

        # if the vehicle is landed, disarm the vehicle
        if self.mode_status.LANDING == True and self.mode_status.is_disarmed == False:
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_disarm_mode)    
            self.mode_status.is_disarmed = True
            self.get_logger().info('Vehicle is disarmed')  

        # state_logger(self)
    # endregion

    def obstacle_flag_callback(self, msg):
        # self.get_logger().info(f"Flag received: PATH_FOLLOWING: {msg.PATH_FOLLOWING}, COLLISION_AVOIDANCE: {msg.COLLISION_AVOIDANCE}") 씨발 누가 쏘는거야
        self.flags.obstacle_flag = msg.data
        
        # update mode status
        if self.mode_status.OFFBOARD == True:
            
            if self.flags.obstacle_flag == False and self.mode_status.PATH_FOLLOWING == False:
                # self.weight.fusion_weight = float(1.0)
                self.mode_status.PATH_FOLLOWING = True
                self.mode_status.COLLISION_AVOIDANCE = False
                self.get_logger().info("Mode Status : PATH_FOLLOWING")
                self.pub_func_module.publish_vehicle_mode()
            elif self.flags.obstacle_flag == True and self.mode_status.COLLISION_AVOIDANCE == False:
                # self.weight.fusion_weight = float(0.0)
                self.mode_status.PATH_FOLLOWING = False
                self.mode_status.COLLISION_AVOIDANCE = True
                self.get_logger().info("Mode Status : COLLISION_AVOIDANCE")
                self.pub_func_module.publish_vehicle_mode()

        





            # if self.mode_status.PATH_FOLLOWING == True:
            #     z = self.guid_var.waypoint_z[self.guid_var.cur_wp]
            #     self.guid_var.waypoint_x = self.guid_var.waypoint_x[self.guid_var.cur_wp:]
            #     self.guid_var.waypoint_y = self.guid_var.waypoint_y[self.guid_var.cur_wp:]
            #     self.guid_var.waypoint_z = self.guid_var.waypoint_z[self.guid_var.cur_wp:]

            #     # self.guid_var.waypoint_x = list(np.insert(self.guid_var.waypoint_x, 0, msg.x))
            #     # self.guid_var.waypoint_y = list(np.insert(self.guid_var.waypoint_y, 0, msg.y))
            #     # self.guid_var.waypoint_z = list(np.insert(self.guid_var.waypoint_z, 0, z))

            #     self.guid_var.waypoint_x = list(np.insert(self.guid_var.waypoint_x, 0, self.state_var.x))
            #     self.guid_var.waypoint_x = list(np.insert(self.guid_var.waypoint_x, 0, self.state_var.x))
            #     self.guid_var.waypoint_y = list(np.insert(self.guid_var.waypoint_y, 0, self.state_var.y))
            #     self.guid_var.waypoint_y = list(np.insert(self.guid_var.waypoint_y, 0, self.state_var.y))
            #     self.guid_var.waypoint_z = list(np.insert(self.guid_var.waypoint_z, 0, self.state_var.z))
            #     self.guid_var.waypoint_z = list(np.insert(self.guid_var.waypoint_z, 0, self.state_var.z))

            #     self.guid_var.real_wp_x = self.guid_var.waypoint_x
            #     self.guid_var.real_wp_y = self.guid_var.waypoint_y
            #     self.guid_var.real_wp_z = self.guid_var.waypoint_z

            #     self.pub_func_module.local_waypoint_publish(False)

    def rand_point_callback(self, msg: Bool):
        self.flags.rand_point_flag = msg.data
        # for debugging
        # self.get_logger().info(f"Rand point callback: {msg.data}")

        # if self.mode_status.OFFBOARD == True:
        #     if msg.data == True and self.flags.obstacle_flag == False and self.mode_status.PATH_FOLLOWING == False:
        #         self.weight.fusion_weight = float(1.0)
        #         self.mode_status.PATH_FOLLOWING = True
        #         self.mode_status.COLLISION_AVOIDANCE = False
        #         self.get_logger().info("Mode Status : PATH_FOLLOWING")
        #         self.pub_func_module.publish_vehicle_mode()

        #         # z = self.guid_var.waypoint_z[self.guid_var.cur_wp]
        #         # self.guid_var.waypoint_x = self.guid_var.waypoint_x[self.guid_var.cur_wp:]
        #         # self.guid_var.waypoint_y = self.guid_var.waypoint_y[self.guid_var.cur_wp:]
        #         # self.guid_var.waypoint_z = self.guid_var.waypoint_z[self.guid_var.cur_wp:]

        #         # # self.guid_var.waypoint_x = list(np.insert(self.guid_var.waypoint_x, 0, msg.x))
        #         # # self.guid_var.waypoint_y = list(np.insert(self.guid_var.waypoint_y, 0, msg.y))
        #         # # self.guid_var.waypoint_z = list(np.insert(self.guid_var.waypoint_z, 0, z))

        #         # self.guid_var.waypoint_x = list(np.insert(self.guid_var.waypoint_x, 0, self.state_var.x))
        #         # self.guid_var.waypoint_x = list(np.insert(self.guid_var.waypoint_x, 0, self.state_var.x))
        #         # self.guid_var.waypoint_y = list(np.insert(self.guid_var.waypoint_y, 0, self.state_var.y))
        #         # self.guid_var.waypoint_y = list(np.insert(self.guid_var.waypoint_y, 0, self.state_var.y))
        #         # self.guid_var.waypoint_z = list(np.insert(self.guid_var.waypoint_z, 0, self.state_var.z))
        #         # self.guid_var.waypoint_z = list(np.insert(self.guid_var.waypoint_z, 0, self.state_var.z))

        #         # self.guid_var.real_wp_x = self.guid_var.waypoint_x
        #         # self.guid_var.real_wp_y = self.guid_var.waypoint_y
        #         # self.guid_var.real_wp_z = self.guid_var.waypoint_z

        #         # self.pub_func_module.local_waypoint_publish(False)


        #     if msg.data == False and self.flags.obstacle_flag == True and self.mode_status.PATH_FOLLOWING == True:
        #         self.weight.fusion_weight = float(0.0)
        #         self.mode_status.PATH_FOLLOWING = False
        #         self.mode_status.COLLISION_AVOIDANCE = True
        #         self.get_logger().info("Mode Status : COLLISION_AVOIDANCE")
        #         self.pub_func_module.publish_vehicle_mode()


    def weight_callback(self):
        self.weight.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        if not self.mode_status.OFFBOARD:
            self.weight.fusion_weight = float(0.0)
            self.weight_publisher.publish(self.weight)
            return
        

        target_weight = 1.0 if self.mode_status.PATH_FOLLOWING else 0.0

        if target_weight != self.prev_target:
            self.transition_start_time = time.time()
            self.transition_direction = target_weight  # 0→1 or 1→0
            self.prev_target = target_weight

        if self.transition_start_time is None:
            self.weight.fusion_weight = float(target_weight)
            self.weight_publisher.publish(self.weight)
            return

        elapsed = time.time() - self.transition_start_time


        if self.transition_direction == 1.0:  # CA → PF (0 → 1)
            duration = self.transition_duration_to_pf
            if elapsed < duration:
                alpha = elapsed / duration
                self.weight.fusion_weight = float(alpha)  # 0에서 1로
            else:
                self.weight.fusion_weight = 1.0
                self.transition_start_time = None

        else:  # PF → CA (1 → 0)
            duration = self.transition_duration_to_ca
            if elapsed < duration:
                alpha = elapsed / duration
                self.weight.fusion_weight = float(1.0 - alpha)  # 1에서 0으로
            else:
                self.weight.fusion_weight = 0.0
                self.transition_start_time = None


        #     if self.mode_status.PATH_FOLLOWING == True:
        #         self.weight.fusion_weight = float(1.0)
        #         self.current_target = 1.0
        #     if self.mode_status.COLLISION_AVOIDANCE == True:
        #         self.weight.fusion_weight = float(0.0)

 

            # if self.mode_status.PATH_FOLLOWING == True:
            #     self.current_target = 1.0
            # if self.mode_status.COLLISION_AVOIDANCE == True:
            #     self.current_target = 0.0
            # # self.weight.fusion_weight = float(self.current_target)
            # if self.current_target != self.prev_target:
            #     self.transition_start_time = time.time()
            #     self.prev_target = self.current_target


            # 시그모이드 기반 alpha 업데이트
            # if self.transition_start_time is not None:
            #     elapsed = time.time() - self.transition_start_time
            #     if elapsed < self.transition_duration_to_pf and self.current_target == 1.0:
            #         alpha = self.sigmoid(elapsed)
            #         if self.current_target == 0.0:  # pf → ca (1 → 0)
            #             alpha = 1.0 - alpha
            #         self.weight.fusion_weight = float(alpha)
            #     elif elapsed < self.transition_duration_to_ca and self.current_target == 0.0:
            #         alpha = self.sigmoid(elapsed)
            #         self.weight.fusion_weight = float(alpha)
            #     else:
            #         self.weight.fusion_weight = float(self.current_target)
            #         self.transition_start_time = None
            # else:
            #     self.weight.fusion_weight = float(self.current_target)

        self.weight_publisher.publish(self.weight)
        self.pub_func_px4.publish_att_command(self.veh_att_set)

    def sigmoid(self,t, k=10, t0=1):
        alpha = 1 / (1 + math.exp(-k * (t - t0)))
        if alpha > 1.0:
            alpha = 1.0
        elif alpha < 0.0:
            alpha = 0.0

        return alpha

def main(args=None):
    rclpy.init(args=args)
    ca_pf_integration_test = CAPFIntegrationTest()
    rclpy.spin(ca_pf_integration_test)
    ca_pf_integration_test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
