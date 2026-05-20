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

        status_msg = String()

        # Tambahkan flag untuk melacak publikasi status
        self.has_published_dpr_ssy = False
        self.has_published_forward = False
        self.has_published_sway = False
        self.has_published_back = False
        self.has_published_stop = False

        #locking Obstacle
        self.orange_flare_locking = False
        self.orange_flare_lock_start = 0
        self.orange_flare_elapsed = 0

        self.gate_locking = False
        self.gate_lock_start = 0
        self.gate_elapsed = 0

        #SAUVC Obstacle
        self.has_dodged_flare = False
        self.has_entered_gate = False
        self.has_dropped_ball = False

        self.set_point = SetPoint()
        self.multi_pid_msg = MultiPID()
        self.set_point.roll = 0.0
        self.set_point.pitch = 0.0 
        self.set_point.yaw =  85.0 #84 #82
        self.set_point.depth = -0.52

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
        self.pid_yaw.kp = 15.0    # Proportional constant for yaw
        self.pid_yaw.ki = 0.0   # Integral constant for yaw
        self.pid_yaw.kd = 0.0   # Derivative constant for yaw

        self.pid_pitch = PID()
        self.pid_pitch.kp = 4500.0    # Proportional constant for pitch 
        self.pid_pitch.ki = 0.0          # Integral constant for pitch
        self.pid_pitch.kd = 0.0    # Derivative constant for pitch

        self.pid_roll = PID()
        self.pid_roll.kp = 700.0   # Proportional constant for roll
        self.pid_roll.ki = 0.0     # Integral constant for roll
        self.pid_roll.kd = 0.0     # Derivative constant for roll

        self.pid_depth = PID()
        self.pid_depth.kp = 3000.0    # Proportional constant for depth
        self.pid_depth.ki = 0.0     # Integral constant for depth
        self.pid_depth.kd = 0.0     # Derivative constant for depth

        self.pid_camera = PID()
        self.pid_camera.kp = 1.0       # Proportional constant for camera
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
    



# ------------------------------- ---------- ------------------------------- #
# ------------------------------- OBJECTIONS ------------------------------- #

    def dodge_flare(self):
        status_msg.data = "camera"
        self.pub_status.publish(status_msg)

        if (self.object_class == "Orange_Flare"):
            status_msg.data = "camera_yaw"
            self.pub_status.publish(status_msg)

            orange_flare_timer = self.get_clock().now().nanoseconds / 1e9   # timer berjalan
            if not self.orange_flare_locking:
                self.orange_flare_lock_start = orange_flare_timer           # simpan waktu awal objek terdeteksi
                self.orange_flare_locking = True
            self.orange_flare_elapsed = orange_flare_timer - self.orange_flare_lock_start  # berapa lama objek terdeteksi
            
            if self.orange_flare_elapsed < 3.0:
                self.get_logger().info("locking target ORANGE FLARE: locking...")
                return
            
            if not self.has_published_sway:
                status_msg.data = "sway_right_forward"
                self.pub_status.publish(status_msg)
                self.has_published_sway = True  # Set flag agar tidak dipublish lagi

            if self.orange_flare_elapsed < 5.0:
                self.get_logger().info("DODGING FLARE")
                return
            
            self.has_dodged_flare = True
        else:
            self.orange_flare_locking = False

    def enter_gate(self):
        status_msg.data = "camera_yaw"
        self.pub_status.publish(status_msg)
        if (self.object_class == "Gate"):
            status_msg.data = "camera_yaw"
            self.pub_status.publish(status_msg)

            gate_timer = self.get_clock().now().nanoseconds / 1e9
            if not self.gate_locking:
                self.gate_lock_start = gate_timer
                self.gate_locking = True
            self.gate_elapsed = gate_timer - self.gate_lock_start

            if self.gate_elapsed < 3.0:
                self.get_logger().info("locking target GATE: locking...")
                return
            
            if not self.has_published_gate:
                status_msg.data = "all"
                self.pub_status.publish(status_msg)
                self.has_published_gate = True  # Set flag agar tidak dipublish lagi

            if self.gate_elapsed < 8.0:
                self.get_logger().info("ENTER GATE")
                return
            
            self.has_entered_gate = True
        else:
            self.gate_locking = False

    def drop_ball(self):
        status_msg.data = "camera_yaw"
        self.pub_status.publish(status_msg)

        if (self.object_class == "Bucket"):
                    
            self.get_logger().info("LOCKING TARGET BUCKET")
            # nanti isi code stabilize yaw berdasarkan arah bucket
            # isi code maju ke bucket

            self.get_logger().info("DROP BALL")
            status_msg.data = "drop_ball"
            self.pub_status.publish(status_msg)

            self.has_dropped_ball = True

    def surface(self):
        if not self.has_published_stop:
            self.get_logger().info("!!! STOPPING AUV --- SURFACING NOW !!!")
            self.set_point.depth = -0.75
                        
            status_msg = String()
            status_msg.data = "dpr_ssy"
            self.pub_status.publish(status_msg)
            self.has_published_stop = True  # Set flag agar tidak dipublish lagi

# ------------------------------- OBJECTIONS ------------------------------- #
# ------------------------------- ---------- ------------------------------- #




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
        else:
            self.get_logger().info('MISSION COMPLETE')




# ------------------------------- ---------------- ------------------------------- #
# ------------------------------- EXCECUTE ACTIONS ------------------------------- #

    def start_auv(self):
        
        if self.is_in_range(0, 10):
            if not self.has_published_dpr_ssy:
                self.get_logger().info("Go Depth")
                self.pub_multi_pid.publish(self.multi_pid_msg)
                self.pub_set_point.publish(self.set_point)
                
                status_msg.data = "dpr_ssy"
                self.pub_status.publish(status_msg)
                
                boost_msg = Float32()
                boost_msg.data = self.boost
                self.pub_boost.publish(boost_msg)

                self.has_published_dpr_ssy = True  # Set flag agar tidak dipublish lagi
###
        elif self.is_in_range(10, 50): # cari objek n do something

            if not self.has_dodged_flare:
                self.dodge_flare()

            elif not self.has_entered_gate:
                self.enter_gate()

            # elif not self.has_dropped_ball:
            #     self.drop_ball()

        elif self.is_in_range(50, 52):
                self.surface()

        # mungkin cari cara lain selain pake timer(?) mungkin buat kondisi surfacing kalau "semua task udah kelar" atau "kalau waktu udh lewat batas berapa detik dan ga ngedetect object selama beberapa detik"

# ------------------------------- EXCECUTE ACTIONS ------------------------------- #
# ------------------------------- ---------------- ------------------------------- #



# ----- MAIN FUNCTION ----- #

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

# ----- MAIN FUNCTION ----- #














# ignore (older codes archive)

# if not self.has_published_forward:
            #     self.pid_yaw.kp = 10.0
            #     self.pub_multi_pid.publish(self.multi_pid_msg)
            #     self.get_logger().info("maju!!!")
                
            #     status_msg = String()
            #     # mungkin bisa inisialisasi di awal self.status_msg = String()
            #     # (biar ga usah init berulang kali)

            #     status_msg.data = "all"
            #     self.pub_status.publish(status_msg)

            #     self.has_published_forward = True  # Set flag agar tidak dipublish lagi

            # if (self.object_class == "Orange_Flare") and (self.object_class != ""):
            #     if not self.has_published_sway:
            #         # self.set_point.yaw = -80.0
            #         # self.pub_set_point.publish(self.set_point)
            #         self.get_logger().info("sway kanan!!!")     # sway maju kanan!!!
                    
            #         status_msg = String()
            #         status_msg.data = "sway_right"              # sway_right_forward
            #         self.pub_status.publish(status_msg)

            #         self.has_published_sway = True  # Set flag agar tidak dipublish lagi

            #     if self.is_in_range(25, 27):
            #         if not self.has_published_stop:
            #             self.get_logger().info("stopppp")
            #             self.set_point.depth = -0.75
                        
            #             status_msg = String()
            #             status_msg.data = "dpr_ssy"
            #             self.pub_status.publish(status_msg)
            #             self.has_published_stop = True  # Set flag agar tidak dipublish lagi

        

        # if self.is_in_range(0, 10):
        #     self.get_logger().info("Go Depth")
        #     self.pub_multi_pid.publish(self.multi_pid_msg)
        #     self.pub_set_point.publish(self.set_point)
            
        #     status_msg = String()
        #     status_msg.data = "dpr_ssy"
        #     self.pub_status.publish(status_msg)
            
        #     boost_msg = Float32()
        #     boost_msg.data = self.boost
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