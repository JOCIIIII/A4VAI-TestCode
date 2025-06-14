# Librarys

# Library for common
import numpy as np
import os

# ROS libraries
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from ..lib.common_fuctions import set_initial_variables, state_logger, publish_to_plotter, set_wp
from ..lib.timer import HeartbeatTimer, MainTimer, CommandPubTimer
from ..lib.subscriber import PX4Subscriber, FlagSubscriber, CmdSubscriber, EtcSubscriber
from ..lib.publisher import PX4Publisher, HeartbeatPublisher, ModulePublisher, PlotterPublisher
from ..lib.publisher import PubFuncHeartbeat, PubFuncPX4, PubFuncModule, PubFuncPlotter

class PathFollowingTest(Node):
    def __init__(self):
        super().__init__("path_following_test")
        # ----------------------------------------------------------------------------------------#
        # region INITIALIZE
        dir = os.path.dirname(os.path.abspath(__file__))
        sim_name = "pf_unit_test"
        set_initial_variables(self, dir, sim_name)
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region PUBLISHERS
        self.pub_px4        = PX4Publisher(self)
        self.pub_px4.declareVehicleCommandPublisher()                   # Declare PX4 Vehicle Command Publisher
        self.pub_px4.declareOffboardControlModePublisher()              # Declare PX4 Offboard Control Mode Publisher
        self.pub_px4.declareAttitudeCommandPublisher()                  # Declare PX4 Attitude Command Publisher
        self.pub_px4.declareFusionWeightPublisher()                     # Declare PX4 Fusion Weight Publisher

        self.pub_module   = ModulePublisher(self)
        self.pub_module.declareLocalWaypointPublisherToPF()        # Declare Local Waypoint Publisher to Path Following

        self.pub_heartbeat  = HeartbeatPublisher(self)
        self.pub_heartbeat.declareControllerHeartbeatPublisher()
        self.pub_heartbeat.declareCollisionAvoidanceHeartbeatPublisher()
        self.pub_heartbeat.declarePathPlanningHeartbeatPublisher()

        self.pub_plotter    = PlotterPublisher(self)
        self.pub_plotter.declareGlobalWaypointPublisherToPlotter()
        self.pub_plotter.declareLocalWaypointPublisherToPlotter()
        self.pub_plotter.declareHeadingPublisherToPlotter()
        self.pub_plotter.declareStatePublisherToPlotter()
        self.pub_plotter.declareMinDistancePublisherToPlotter()
        # end region
        # ----------------------------------------------------------------------------------------#
        # region SUBSCRIBERS
        self.sub_px4 = PX4Subscriber(self)
        self.sub_px4.declareVehicleLocalPositionSubscriber(self.state_var)          # Declare Vehicle Local Position Subscriber

        self.sub_cmd = CmdSubscriber(self)
        self.sub_cmd.declarePFAttitudeSetpointSubscriber(self.veh_att_set)          # Declare Path Following Attitude Setpoint Subscriber

        self.sub_flag = FlagSubscriber(self)
        self.sub_flag.declareConveyLocalWaypointCompleteSubscriber(self.mode_flag)  # Declare Convey Local Waypoint Complete Subscriber
        self.sub_flag.declarePFCompleteSubscriber(self.mode_flag)                   # Declare Path Following Complete Subscriber

        self.sub_etc = EtcSubscriber(self)
        self.sub_etc.declareHeadingWPIdxSubscriber(self.guid_var)                   # Declare Heading WP Index Subscriber
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region PUB FUNC
        self.pub_func_heartbeat = PubFuncHeartbeat(self)
        self.pub_func_px4       = PubFuncPX4(self)
        self.pub_func_module    = PubFuncModule(self)
        self.pub_func_plotter   = PubFuncPlotter(self)
        # endregion
        # ----------------------------------------------------------------------------------------#
        # region TIMER
        self.timer_offboard_control = MainTimer(self, self.offboard_var)
        self.timer_offboard_control.declareOffboardControlTimer(self.offboard_control_main)

        self.timer_cmd = CommandPubTimer(self, self.offboard_var)
        self.timer_cmd.declareAttitudeCommandTimer(self.mode_flag, self.veh_att_set, self.pub_func_px4)
        self.timer_cmd.declareFusionWeightTimer(self.weight, self.pub_func_px4)

        # heartbeat timer
        self.timer_heartbeat = HeartbeatTimer(self, self.offboard_var, self.pub_func_heartbeat)
        self.timer_heartbeat.declareControllerHeartbeatTimer()
        self.timer_heartbeat.declarePathPlanningHeartbeatTimer()
        self.timer_heartbeat.declareCollisionAvoidanceHeartbeatTimer()
        # endregion
    # ----------------------------------------------------------------------------------------#
    # region MAIN CODE
    def offboard_control_main(self):
        self.weight_callback()
        # send offboard mode and arm mode command to px4
        if self.mode_flag.is_standby == True:
            self.mode_flag.is_takeoff = True
            self.mode_flag.is_standby = False

        # send offboard mode and arm mode command to px4
        if self.offboard_var.counter == self.offboard_var.flight_start_time and self.mode_flag.is_takeoff == True:
            # arm cmd to px4
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_arm_mode)
            # offboard mode cmd to px4
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_takeoff_mode)

        # takeoff after a certain period of time
        elif self.offboard_var.counter <= self.offboard_var.flight_start_time:
            self.offboard_var.counter += 1

        # check if the vehicle is ready to initial position
        if self.mode_flag.is_takeoff == True and self.state_var.z > self.guid_var.init_pos[2]:
            self.mode_flag.is_takeoff = False
            self.mode_flag.pf_recieved_lw = True
            
            set_wp(self)
            self.pub_func_module.local_waypoint_publish(True)
            publish_to_plotter(self)
            self.get_logger().info('Vehicle is reached to initial position')

        if self.mode_flag.pf_recieved_lw == True and self.mode_flag.is_offboard == False:
            self.mode_flag.is_offboard = True
            self.mode_flag.is_pf = True
            self.get_logger().info('Vehicle is in offboard mode')



        # check if path following is recieved the local waypoint
        if self.mode_flag.is_offboard == True and self.mode_flag.pf_done == False:
            # offboard mode
            self.pub_func_px4.publish_offboard_control_mode(self.offboard_mode)
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_offboard_mode)
        
        if self.mode_flag.pf_done == True and self.mode_flag.is_landed == False:
            self.mode_flag.is_offboard = False
            self.mode_flag.is_pf = False
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_land_mode)

            # check if the vehicle is landed
            if np.abs(self.state_var.vz_n) < 0.05 and np.abs(self.state_var.z < 0.05):
                self.mode_flag.is_landed = True
                self.get_logger().info('Vehicle is landed')

        # if the vehicle is landed, disarm the vehicle
        if self.mode_flag.is_landed == True and self.mode_flag.is_disarmed == False:
            self.pub_func_px4.publish_vehicle_command(self.modes.prm_disarm_mode)    
            self.mode_flag.is_disarmed = True
            self.get_logger().info('Vehicle is disarmed')        

        state_logger(self)

    def weight_callback(self):
        self.weight.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        if self.mode_flag.is_offboard == False:
            self.weight.fusion_weight = 0.0
        else:
            self.weight.fusion_weight = 1.0
        # self.pub_func_px4.publish_fusion_weight(self.weight)
    # endregion
    # ----------------------------------------------------------------------------------------#

def main(args=None):
    rclpy.init(args=args)
    path_following_test = PathFollowingTest()
    rclpy.spin(path_following_test)
    path_following_test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()