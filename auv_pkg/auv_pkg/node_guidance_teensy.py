#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int16
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference

class SubGuidance(Node):

    def __init__(self):
        super().__init__('guidance_teensy')

        self.is_start = False
        self.start_time_flag = 0  # Mengganti nama self.start untuk menghindari konflik
        self.start_delay = False
        self.delay = 5
        self.boot_time = 0
        self.start_time = 0
        self.current_time = 0
        self.elapsed_time = 0
        self.status = "stop"
        self.boost = 0.0
        self.object_class = ""

        # Tambahkan flag untuk melacak publikasi status
        self.has_published_dpr_ssy = False
        self.has_published_forward = False
        self.has_published_sway = False
        self.has_published_back = False
        self.has_published_stop = False

        self.set_point = SetPoint()
        self.multi_pid_msg = MultiPID()
        self.set_point.yaw =  270.0
        self.set_point.pitch = 0.0
        self.set_point.roll = 0.0
        self.set_point.depth = -0.68

        self.param_delay = 3
        self.param_duration = 0

        # Publisher
        self.pub_multi_pid = self.create_publisher(MultiPID, "pid", 10)
        self.pub_set_point = self.create_publisher(SetPoint, "set_point", 10)
        self.pub_status = self.create_publisher(String, "status", 10)
        self.pub_boost = self.create_publisher(Float32, "boost", 10)
        self.pub_flag = self.create_publisher(Int16, "Flag", 10)

        # subscriber
        self.sub_guidance_objdif = self.create_subscription(
            ObjectDifference,
            'object_difference',
            self.object_difference_callback,
            10
        )

        # Create multiple PID messages
        self.pid_yaw = PID()
        self.pid_yaw.kp = 4.5 #3.0   #15 # Proportional constant for yaw
        self.pid_yaw.ki = 0.0   # Integral constant for yaw
        self.pid_yaw.kd = 0.3   # Derivative constant for yaw

        self.pid_pitch = PID()
        self.pid_pitch.kp = 10.0 #700.0  #4500  # Proportional constant for pitch 
        self.pid_pitch.ki = 0.0          # Integral constant for pitch
        self.pid_pitch.kd = 1.1    # Derivative constant for pitch

        self.pid_roll = PID()
        self.pid_roll.kp = 2.5 #300.0  #700 # Proportional constant for roll
        self.pid_roll.ki = 0.0     # Integral constant for roll
        self.pid_roll.kd = 0.3     # Derivative constant for roll

        self.pid_depth = PID()
        self.pid_depth.kp = 1350.0  #3000  # Proportional constant for depth
        self.pid_depth.ki = 0.0     # Integral constant for depth
        self.pid_depth.kd = 215.0 #0.0     # Derivative constant for depth

        self.pid_camera = PID()
        self.pid_camera.kp = 0.5    #1.0   # Proportional constant for camera
        self.pid_camera.ki = 0.0     # Integral constant for camera
        self.pid_camera.kd = 0.0     # Derivative constant for camera

        # Create MultiPID message and add multiple PID sets
        self.multi_pid_msg.pid_yaw = self.pid_yaw
        self.multi_pid_msg.pid_pitch = self.pid_pitch
        self.multi_pid_msg.pid_roll = self.pid_roll
        self.multi_pid_msg.pid_depth = self.pid_depth
        self.multi_pid_msg.pid_camera = self.pid_camera

        self.pub_multi_pid.publish(self.multi_pid_msg)  # Publishing MultiPID
        self.pub_set_point.publish(self.set_point)

        status_msg = String()
        status_msg.data = self.status
        self.pub_status.publish(status_msg)
        
        boost_msg = Float32()
        boost_msg.data = self.boost
        self.pub_boost.publish(boost_msg)

    def object_difference_callback(self, msg):
        self.object_class = msg.object_type

    def delay_time(self):
        self.current_time = self.get_clock().now().nanoseconds / 1e9
        self.elapsed_time = self.current_time - self.start_time_flag

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or \
               (start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay)

    def start(self):
        if not self.is_start:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.is_start = True

        # Generate boot time
        self.boot_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        # Tunggu beberapa detik sebelum start
        if self.boot_time < self.param_delay:
            self.get_logger().info('STARTING...')
            return

        if self.param_duration <= 0 or self.boot_time < self.param_duration:
            self.start_auv()

    def start_auv(self):
        
        if self.is_in_range(0, 11.5):
            self.get_logger().info("maju!!!")
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.pub_set_point.publish(self.set_point)
                
            status_msg = String()
            status_msg.data = "all"
            self.pub_status.publish(status_msg)

        # elif self.is_in_range(10, 18): # cari objek n do something
        #     self.get_logger().info("sway right!!!")

        #     status_msg = String()
        #     status_msg.data = "sway_right"
        #     self.pub_status.publish(status_msg)

        elif self.is_in_range(11.5, 15):
            self.get_logger().info("go back!!!")
                
            if self.set_point.yaw != 90.0:
                self.set_point.yaw = 90.0
                self.pub_set_point.publish(self.set_point)

            status_msg = String()
            status_msg.data = "all"
            self.pub_status.publish(status_msg)

        elif self.is_in_range(15, 17): # cari objek n do something
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.get_logger().info("surfacing!!!")
            if self.set_point.depth != -0.54 and self.set_point.yaw != 270.0:
                self.set_point.depth = -0.54
                self.set_point.yaw = 270.0
                self.pub_set_point.publish(self.set_point)

            status_msg = String()
            status_msg.data = "dpr_ssy"
            self.pub_status.publish(status_msg)
        
        elif self.is_in_range(21, None): # cari objek n do something
            if not self.has_published_stop:
                self.pub_multi_pid.publish(self.multi_pid_msg)
                self.get_logger().info("stop!!!")

                status_msg = String()
                status_msg.data = "stop"
                self.pub_status.publish(status_msg)
                self.has_published_stop = True


            # if (self.object_class == "Orange_Flare"):
            #     if not self.has_published_sway:
            #         # self.set_point.yaw = -80.0
            #         # self.pub_set_point.publish(self.set_point)
            #         self.get_logger().info("sway kanan!!!")     # sway maju kanan!!!
                    
            #         status_msg = String()
            #         status_msg.data = "sway_right_forward"              # sway_right_forward
            #         self.pub_status.publish(status_msg)

            #         self.has_published_sway = True  # Set flag agar tidak dipublish lagi

                # if self.is_in_range(19, 22):
                #     if not self.has_published_stop:
                #         self.get_logger().info("stopppp")
                #         self.set_point.depth = -0.75
                        
                #         status_msg = String()
                #         status_msg.data = "dpr_ssy"
                #         self.pub_status.publish(status_msg)
                #         self.has_published_stop = True  # Set flag agar tidak dipublish lagi

        

        # if self.is_in_range(0, 10):
        #     self.get_logger().info("Go Depth")
        #     self.pub_multi_pid.publish(self.multi_pid_msg)
        #     self.pub_set_point.publish(self.set_point)
            
        #     status_msg = String()
        #     status_msg.data = "dpr_ssy"
        #     self.pub_status.publish(status_msg)
            
        #     boost_msg = Float32()
        #     boost_msself.pub_set_point.publish(self.set_pointself.pub_set_point.publish(self.set_pointg.data = self.boost
        #     self.pub_boost.publish(boost_msg)
            
        # elif self.is_in_range(10, 15):
        #     self.pid_yaw.kp = 10.0
        #     self.pub_multi_pid.publish(self.multi_pid_msg)
        #     self.get_logger().info("maju!!!")
            
        #     status_msg = String()
        #     status_msg.data = "all"
        #     self.pub_status.publish(status_msg)
        
        # elif self.is_in_range(15, 40):
        #     self.set_point.yaw = -80.0
        #     self.pub_set_point.publish(self.set_point)
        #     self.get_logger().info("sway maju kanan!!!")
            
        #     status_msg = String()
        #     status_msg.data = "sway_right_forward"
        #     self.pub_status.publish(status_msg)

        # elif self.is_in_range(40, 41):
        #     self.get_logger().info("stopppp")
        #     self.set_point.depth = -0.75
            
        #     status_msg = String()
        #     status_msg.data = "dpr_ssy"
        #     self.pub_status.publish(status_msg)
        #     self.has_published_stop = True  # Set flag agar tidak dipublish lagi

def main(args=None):
    rclpy.init(args=args)
    
    node = SubGuidance()

    # Create a timer for the main loop
    timer_period = 0.1  # 10 Hz
    timer = node.create_timer(timer_period, node.start)
    
    rclpy.spin(node)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

