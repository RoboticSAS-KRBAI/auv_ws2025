#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String, Float32,Int8, Int16, Bool
from auv_interfaces.msg import SetPoint, MultiPID, PID, ObjectDifference,Sensor, Logging
from geometry_msgs.msg import Pose
from collections import deque

class Subscriber(Node):
    def __init__(self):
        super().__init__('node_guidance')
        self.is_start = False
        self.start_time_flag = 0  # Mengganti nama self.start untuk menghindari konflik
        self.boot_time = 0
        self.start_time = 0
        self.current_time = 0
        self.elapsed_time = 0
        self.status = "stop"
        self.boost = 350
        self.object = "None"
        self.x_difference = 0
        self.flag = 0 # ganti 0
        self.log = Logging()

        # TIMER GATE
        self.received_gate = False
        self.start_gate = None
        self.elapsed_gate = 0

        # TIMER FLARE
        self.received_flare = False
        self.start_flare = None
        self.elapsed_flare = 0
        
        # TIMER Blue_FLARE
        self.received_blue_flare = False
        self.start_blue_flare = None
        self.elapsed_blue_flare = 0
        
        # TIMER Yellow_FLARE
        self.received_yellow_flare = False
        self.start_yellow_flare = None
        self.elapsed_yellow_flare = 0
        
        # TIMER Red_FLARE
        self.received_red_flare = False
        self.start_red_flare = None
        self.elapsed_red_flare = 0
        
        # TIMER Orange_FLARE
        self.received_orange_flare = False
        self.start_orange_flare = None
        self.elapsed_orange_flare = 0

         # Timer delay bucket
        self.start_delay_bucket = None
        self.elapsed_bucket = 0

        # TIMER KETIKA FLAG 3 TAU KANAN
        self.start_rotation_time = None 
        self.elapsed_rotation = 0

        self.has_published_value = False

        self.lost_gate_time = None
        self.lost_red_flare_time = None
        self.lost_blue_flare_time = None
        self.lost_yellow_flare_time = None
        

        self.set_point = SetPoint()
        self.multi_pid_msg = MultiPID()
        self.set_point.roll = 0 # ganti
        self.set_point.pitch = 0 # ganti
        self.set_point_yaw_front = 100
        self.set_point.yaw = self.set_point_yaw_front
        self.set_point_yaw_right = self.set_point.yaw - 90
        self.set_point_yaw_left = self.set_point.yaw + 92
        self.set_point_yaw_back = self.set_point.yaw + 180
        self.set_point.depth = 1
        self.depth_surface = 0.07

        self.yaw_asli = 0

        self.param_delay = 5
        self.param_duration = 0

        
        self.zone = 0
        self.target_zone = 0
        self.target_x = 0
        self.target_y = 0
        self.robot_x = 0
        self.robot_y = 0
        self.stabilized = False
        # self.scenario = [1, 2, 3, 4, "Bucket"]
        self.scenario_zone = [2, 1, 1, 1]
        self.current_zone = 0
        self.flare = ['Y', 'R', 'B']     # R,B,Y
        self.target_flare = 0
        self.current_flare = 0
        self.done_flare = 0
        
        self.mode = 1
        self.hit_flare = 0 # ganti 0
        self.no_detect = True
        self.scenario_finish = False
        self.target_object = ""
        
        self.done_backward = False

        self.detected_start_time = None
        self.done_object = False # ganti false
        self.center = False

        self.done_drop = False
        self.bucket_detected = "False"

        self.start_check_object_time = None
        self.elapsed_check_object = 0
        self.check_object = True
        self.search_zone = 0
        self.status_depth = False
        
        # Buffer untuk menyimpan data selama 3 detik (~60 frame)
        self.detection_history = deque(maxlen=60)  
        
        # Threshold untuk mengambil keputusan
        self.detection_threshold = 0.5  # Jika lebih dari 60% frame mendeteksi objek, ambil keputusan
    
        self.do_action = True
        
        self.see_object = False
        self.start_see_object = None
        self.elapsed_see_object = 0

        
        self.start_bucket_lost = None
        self.elapsed_bucket_lost = 0
        self.status_drop = False
        
        self.start_to_back = None
        self.elapsed_to_back = 0
        
        # Publisher
        self.pub_multi_pid = self.create_publisher(MultiPID, "PID", 10)
        self.pub_set_point = self.create_publisher(SetPoint, "SetPoint", 10)
        self.pub_status = self.create_publisher(String, "Status", 10)
        self.pub_boost = self.create_publisher(Float32, "Boost", 10)
        self.pub_flag = self.create_publisher(Int16, "Flag", 10)
        self.pub_target_zone = self.create_publisher(Int16, "Target_zone", 10)
        self.pub_target_flare = self.create_publisher(String, "Target_flare", 10)
        self.pub_logging = self.create_publisher(Logging, "logging", 10)

        self.create_subscription(ObjectDifference,"object_difference",
            self.callback_object_difference,
            10
        )
        self.create_subscription(Sensor,"sensor_msg",
            self.callback_sensor,
            10
        )
        self.create_subscription(Pose,"/robot_pose",
            self.callback_pose,
            10
        )
        self.create_subscription(Int8,"Zone",
            self.callback_zone,
            10
        )
        self.create_subscription(String,"bucket_detected_python",
            self.callback_bucket,
            10
        )

        # Create multiple PID messages
        self.pid_yaw = PID()
        self.pid_yaw.Kp = 10    # Proportional constant for yaw
        self.pid_yaw.Ki = 0.0   # Integral constant for yaw
        self.pid_yaw.Kd = 1.0   # Derivative constant for yaw

        self.pid_pitch = PID()
        self.pid_pitch.Kp = 7000   # Proportional constant for pitch
        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch

        self.pid_roll = PID()
        self.pid_roll.Kp = 700 #500.0   # Proportional constant for roll
        self.pid_roll.Ki = 0.0     # Integral constant for roll
        self.pid_roll.Kd = 0.0     # Derivative constant for roll

        self.pid_depth = PID()
        self.pid_depth.Kp = 3000    # Proportional constant for depth
        self.pid_depth.Ki = 0.0     # Integral constant for depth
        self.pid_depth.Kd = 0.0     # Derivative constant for depth

        self.pid_camera = PID()
        self.pid_camera.Kp = 1       # Proportional constant for camera
        self.pid_camera.Ki = 0.0     # Integral constant for camera
        self.pid_camera.Kd = 0.0     # Derivative constant for camera

        # Create MultiPID message and add multiple PID sets
        self.multi_pid_msg.pid_yaw = self.pid_yaw
        self.multi_pid_msg.pid_pitch = self.pid_pitch
        self.multi_pid_msg.pid_roll = self.pid_roll
        self.multi_pid_msg.pid_depth = self.pid_depth
        self.multi_pid_msg.pid_camera = self.pid_camera

        self.pub_multi_pid.publish(self.multi_pid_msg)  # Publishing MultiPID
        self.pub_set_point.publish(self.set_point)
        self.pub_status.publish(self.status)
        self.pub_boost.publish(self.boost)

        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

    def loop(self):
        self.start()
        self.delay_flare()
        self.delay_gate()
        self.delay_blue_flare()
        self.delay_yellow_flare()
        self.delay_red_flare()
        self.delay_orange_flare()

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9
    
        
    def normalize_angle(self, angle):
        """ Normalize angle to range -180 to 180 degrees """
        return (angle + 180) % 360 - 180

    def callback_pose(self, data:Pose):
        self.robot_x = data.position.x
        self.robot_y = data.position.y

    def callback_bucket(self, data: Bool):
        self.bucket_detected = data.data
         
    def callback_zone(self, data:Int8):
        self.zone = data.data
        # self.get_logger().info(f"Robot berada di Zona: {self.zone}")

    def callback_sensor(self, data:Sensor):
        self.yaw_asli = data.yaw

    def callback_object_difference(self, data:ObjectDifference):
        self.object = data.object_type
        self.x_difference = data.x_difference
        
        flare = {
            'R': "Red_Flare",
            'Y': "Yellow_Flare",
            'B': "Blue_Flare",
            'O': "Orange_Flare"
        }
        
        self.target_flare = self.flare[self.current_flare]
        self.target_object = flare[self.target_flare]
        self.pub_target_flare.publish(self.target_flare)
        
        if self.object == self.target_object and self.flag == 3:
            
            self.check_object = False
            if self.start_check_object_time is None:
                self.start_check_object_time = self.now()
                
            self.elapsed_check_object = self.now() - self.start_check_object_time
            
            if self.elapsed_check_object <= 1:
                print("Check Object")
                self.status = "camera_sway"
                self.pub_status.publish("camera_sway")
            else:
                self.check_object = True
                self.stabilized = True
                self.start_rotation_time = self.now()
        else:
            self.check_object = True
            
        # Jika objek terdeteksi, tambahkan 1, jika tidak, tambahkan 0
        detected = 1 if data.is_target else 0
        self.detection_history.append(detected)
        
         # Jika buffer penuh (sudah berjalan selama 3 detik)
        if len(self.detection_history) == self.detection_history.maxlen:
            detection_rate = sum(self.detection_history) / len(self.detection_history)
            
            # Konversi buffer ke string "0101011..."
            detection_string = "".join(map(str, self.detection_history))
            # self.get_logger().info(f"Detection History: {detection_string}")
            self.get_logger().info(f"Detection Rate: {detection_rate:.2f}")

            if detection_rate > self.detection_threshold:
                # self.get_logger().info("Object detected consistently, taking action!")
                self.do_action = True
            else:
                # print("objek belum pasti")
                self.do_action = False

        # mode 1 (prioritas orange Flare)
        if self.mode == 1:
            if self.object == "Orange_Flare" and not self.received_flare and self.flag == 0 and self.status_depth and self.do_action:
                self.flag = 1
                self.pub_flag.publish(self.flag)
                self.received_flare = True
                self.start_flare = self.now()
            elif self.object == "Gate" and not self.received_gate and self.flag == 1 and self.do_action:
                self.flag = 2
                self.pub_flag.publish(self.flag)
                self.received_gate = True
                self.start_gate = self.now()

        # mode 2 (prioritas Gate)
        elif self.mode == 2:
            if self.object == "Gate" and not self.received_gate and self.flag == 0:
                self.flag = 1
                self.pub_flag.publish(self.flag)
                self.received_gate = True
                self.start_gate = self.now()

    def go_to_bucket(self):
        # Pergi ke bucket biru di zona 2 dan drop ball 
        self.get_logger().warn("go to bucket")
        # print("self.bucket_detected = " , self.bucket_detected)

        self.elapsed_rotation = self.now() - self.start_rotation_time
        
        if self.object == "Blue_Bucket" and self.flag == 4 :
            self.get_logger().warn("blue_bucket terlihat")
            if self.detected_start_time is None:  # Pertama kali objek terdeteksi
                self.detected_start_time = self.now()
            detection_duration = self.now() - self.detected_start_time

            if detection_duration <= 3 :
                self.pid_pitch.Kp = 2000   # Proportional constant for pitch
                self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                self.pub_multi_pid.publish(self.multi_pid_msg)
                self.get_logger().info(f"Deteksi bucket biru berlangsung {detection_duration:.2f} sampai 3 detik...")
                self.get_logger().warn(f"Lurusin bucket biru")
                self.status = "camera_sway"
                self.pub_status.publish("camera_sway")
                self.boost = 0
                self.pub_boost.publish(self.boost)
                self.see_object = True
            else:
                self.pid_pitch.Kp = 5000   # Proportional constant for pitch
                self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                self.pub_multi_pid.publish(self.multi_pid_msg)
                self.done_object = True 
                self.get_logger().info("Menuju bucket biru")
                self.status = "camera"
                self.pub_status.publish("camera")
                self.boost = 0
                self.pub_boost.publish(self.boost)
                self.see_object = False
                self.start_see_object = None
                
        elif self.see_object:
            self.pid_pitch.Kp = 2000   # Proportional constant for pitch
            self.pid_pitch.Ki = 0.0    # Integral constant for pitch
            self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.get_logger().warn("Memastikan")
            if self.start_see_object is None:
                self.start_see_object = self.now()
                
            self.elapsed_see_object = self.now() - self.start_see_object
            
            if self.elapsed_see_object <= 3:
                print("objek terlihat sekali check dulu")
                self.status = "dpr"
                self.pub_status.publish("dpr")
            else:
                print("Ga ada apa apa")
                self.see_object = False
                self.start_see_object = None
                
        elif self.elapsed_rotation <= 60 and not self.done_object:
            self.pid_pitch.Kp = 2000   # Proportional constant for pitch
            self.pid_pitch.Ki = 0.0    # Integral constant for pitch
            self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.detected_start_time = None
            self.get_logger().info(f"Robot berputar selama {self.elapsed_rotation:.2f} detik, target 30 detik...")
            self.status = "yaw_right"
            self.pub_status.publish("yaw_right")
            return
        
        # elif self.elapsed_rotation <= 31 and not self.done_object:
        #     self.search_zone += 1
        #     self.stabilized = False
        #     self.center = True
        #     return
        
        elif not self.done_object :
            self.get_logger().warn("Robot tidak melihat apa-apa dan melanjutkan misi")
            if self.hit_flare == 3:
                self.done_drop = False
                self.get_logger().info("Misi selesai! Berhenti dengan flag 5.")
                self.flag = 5  # Berhenti
                self.pub_flag.publish(self.flag)
            else:
                self.done_object = False
                self.center = False
                self.stabilized = False
                self.start_rotation_time = None
                self.elapsed_check_object = None
                self.start_bucket_lost = None
                self.done_drop = False
                self.get_logger().info("Belum selesai menabrak 3 flare, kembali ke flag 3.")
                self.flag = 3  # Kembali ke flag 3 untuk melanjutkan misi flare
                self.pub_flag.publish(self.flag)
            
        elif self.done_object and self.bucket_detected == "False":
            self.pid_pitch.Kp = 5000   # Proportional constant for pitch
            self.pid_pitch.Ki = 0.0    # Integral constant for pitch
            self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.get_logger().warn("Robot Melihat object tapi bucket_detected = false")
            if self.start_bucket_lost is None:
                self.start_bucket_lost = self.now()
                
            self.elapsed_bucket_lost = self.now() - self.start_bucket_lost
            
            if self.elapsed_bucket_lost <= 15:
                self.status = "camera"
                self.pub_status.publish("camera")
                self.boost = 0
                self.pub_boost.publish(self.boost)
            else:
                if self.hit_flare == 3:
                    self.done_drop = True
                    self.get_logger().info("Misi selesai! Berhenti dengan flag 5.")
                    self.flag = 5  # Berhenti
                    self.pub_flag.publish(self.flag)
                else:
                    self.done_object = False
                    self.center = False
                    self.stabilized = False
                    self.start_rotation_time = None
                    self.elapsed_check_object = None
                    self.start_bucket_lost = None
                    self.done_drop = True
                    self.get_logger().info("Belum selesai menabrak 3 flare, kembali ke flag 3.")
                    self.flag = 3  # Kembali ke flag 3 untuk melanjutkan misi flare
                    self.pub_flag.publish(self.flag)
            
        elif self.bucket_detected == "True" and not self.status_drop:
            self.get_logger().warn("Robot Melihat object tapi bucket_detected = true")
            if self.start_delay_bucket is None:
                self.start_delay_bucket = self.now()  # Inisialisasi jika belum diset
    
            self.status_drop = True

        elif self.status_drop:
            self.get_logger().warn("Robot sudah drop ball dan melanjutkan misi")
            self.elapsed_bucket = self.now() - self.start_delay_bucket
            if self.elapsed_bucket <= 3:
                self.pid_pitch.Kp = 2000   # Proportional constant for pitch
                self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                self.pub_multi_pid.publish(self.multi_pid_msg)
                self.get_logger().info("dropping")
                self.status = "dpr"
                self.pub_status.publish("dpr")   
            else:
                # Cek apakah flare sudah ditabrak 3 kali
                if self.hit_flare == 3:
                    self.done_drop = True
                    self.get_logger().info("Misi selesai! Berhenti dengan flag 5.")
                    self.flag = 5  # Berhenti
                    self.pub_flag.publish(self.flag)
                else:
                    self.done_object = False
                    self.center = False
                    self.stabilized = False
                    self.start_rotation_time = None
                    self.elapsed_check_object = None
                    self.done_drop = True
                    self.get_logger().info("Belum selesai menabrak 3 flare, kembali ke flag 3.")
                    self.flag = 3  # Kembali ke flag 3 untuk melanjutkan misi flare
                    self.pub_flag.publish(self.flag)
                    
        # elif self.bucket_detected == "Maju" and self.status_drop == False:
        #     self.get_logger().info("Bucket berada di depan")
        #     self.pub_status.publish("last_slow")
                    
        # elif self.bucket_detected == "Kanan" and self.status_drop == False:
        #     self.get_logger().info("Bucket berada di kanan")
        #     self.pub_status.publish("yaw_right")
            
        # elif self.bucket_detected == "Kiri" and self.status_drop == False:
        #     self.get_logger().info("Bucket berada di kiri")
        #     self.pub_status.publish("yaw_left")
                    
                
    def go_hit_flare(self):       
        elapsed_var = f"elapsed_{self.target_object.lower()}"  # e.g. "elapsed_blue_flare"
        lost_var = f"lost_{self.target_object.lower()}_time"  # e.g. "lost_blue_flare_time"
        
        elapsed_value = getattr(self, elapsed_var, 0)
        lost_time = getattr(self, lost_var, None)
        
        if self.object == self.target_object and self.hit_flare == self.done_flare and not self.done_backward:
            if self.detected_start_time is None:  # Pertama kali objek terdeteksi
                self.detected_start_time = self.now()

            detection_duration = self.now() - self.detected_start_time
            
            if detection_duration <= 3 and not self.done_object:  # Pastikan objek terdeteksi selama 5 detik
                self.pid_pitch.Kp = 2000   # Proportional constant for pitch
                self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                self.pub_multi_pid.publish(self.multi_pid_msg)
                self.get_logger().info(f"Deteksi {self.target_object.lower()} berlangsung {detection_duration:.2f} sampai 5 detik...")
                self.get_logger().warn(f"Lurusin {self.target_object.lower()}")
                self.status = "camera_sway"
                self.pub_status.publish("camera_sway")
                self.see_object = True
            else:
                self.pid_pitch.Kp = 5000   # Proportional constant for pitch
                self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                self.pub_multi_pid.publish(self.multi_pid_msg)
                self.boost = 0
                self.pub_boost.publish(self.boost)
                self.get_logger().info(f"Menuju {self.target_object.lower()}")
                self.status = "camera"
                self.pub_status.publish("camera")
                setattr(self, lost_var, self.now())  # Simpan waktu kehilangan flare
                self.no_detect = False
                self.done_object = True
                self.see_object = False
                self.start_see_object = None
                
        elif self.see_object:
            if self.start_see_object is None:
                self.start_see_object = self.now()
                
            self.elapsed_see_object = self.now() - self.start_see_object
            
            if self.elapsed_see_object <= 3:
                self.pid_pitch.Kp = 2000   # Proportional constant for pitch
                self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                self.pub_multi_pid.publish(self.multi_pid_msg)
                print("objek terlihat sekali check dulu")
                self.status = "dpr"
                self.pub_status.publish("dpr")
            else:
                print("Ga ada apa apa")
                self.see_object = False
                self.start_see_object = None
                
        elif self.elapsed_rotation <= 60 and self.no_detect and not self.done_object:
            self.pid_pitch.Kp = 2000   # Proportional constant for pitch
            self.pid_pitch.Ki = 0.0    # Integral constant for pitch
            self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.detected_start_time = None
            self.get_logger().info(f"Robot berputar selama {self.elapsed_rotation:.2f} detik, target 60 detik...")
            self.status = "yaw_right"
            self.pub_status.publish("yaw_right")
            return

        elif self.hit_flare == self.done_flare and not self.no_detect and self.done_object:
            self.pid_pitch.Kp = 5000   # Proportional constant for pitch
            self.pid_pitch.Ki = 0.0    # Integral constant for pitch
            self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
            self.pub_multi_pid.publish(self.multi_pid_msg)
            self.detected_start_time = None

            if lost_time is None:
                self.get_logger().info("MASUK WAKTU BERHENTI")
                setattr(self, lost_var, self.now())
            lost_duration = self.now() - getattr(self, lost_var)
            print("Lost Duration", lost_duration)
            if lost_duration >= 6: # Mundur selama 2 detik, 6 - 4 = 2
                self.hit_flare += 1
                self.scenario_finish = True
                self.get_logger().warn(f"{self.target_object.lower()} tidak terdeteksi selama 4 detik! Mengubah hit flare menjadi {self.hit_flare}.")
            elif lost_duration >= 4: # Maju selama 4 detik
                self.get_logger().warn("Mundur")
                self.set_point.yaw = self.set_point_yaw_front
                self.pub_set_point.publish(self.set_point)
                self.status = "backward"
                self.pub_status.publish("backward")
                self.done_backward = True
            else:
                self.get_logger().warn(f"{self.target_object.lower()} hilang selama {lost_duration:.2f} detik, menunggu 4 detik...")
                self.get_logger().info(f"camera_{self.target_object.lower()}")
                self.status = "camera"
                self.pub_status.publish("camera")
                self.boost = 350
                self.pub_boost.publish(self.boost)
                
        elif self.no_detect:
            self.get_logger().info("tidak detect apa-apa selama 60 detik")
            self.scenario_finish = True
            # self.flag = 5
        
    def stabilize_and_advance(self):
        self.get_logger().warn("Stabilize and advance")
 
        self.elapsed_rotation = self.now() - self.start_rotation_time
        
        if not self.scenario_finish:
            self.go_hit_flare()
        # elif self.scenario_finish and:
        #     if self.current_flare < len(self.flare) - 1:
        #         print(f"Pindah ke flare selanjutnya")
        #         self.current_flare += 1
        #         self.done_flare += 1
        #     else:
        #         print("Done all flare!")
  
        #     if self.current_zone < len(self.scenario_zone) - 2: # -1 jika indeks sepenuhnya, -2 jika indeks kurang 1, dan begitu seterusnya
        #         self.current_zone += 1
        #         # while (self.current_zone < len(self.scenario_zone) - 1 and self.scenario_zone[self.current_zone] == self.scenario_zone[self.current_zone - 1]):
        #         #     print(f"Skipping duplicate zone {self.scenario_zone[self.current_zone]}")
        #         #     self.current_zone += 1  # Lewati zona yang sama agar tidak stuck
        #     else :
        #         if self.done_drop :
        #             self.get_logger().warn("Sudah nabrak 3 flare dan drop ball.")
        #             self.flag = 5
        #             self.pub_flag.publish(self.flag)
        #         else :
        #             self.get_logger().warn("MASUK KE FLAG 4")
        #             self.flag = 4
        #             self.pub_flag.publish(self.flag)

        #     # Reset state untuk digunakan di zona berikutnya
        #     self.start_rotation_time = None
        #     self.detected_start_time = None
        #     self.stabilized = False
        #     self.detect = False
        #     self.scenario_finish = False
        #     self.no_detect = True
        #     self.done_backward = False
        #     self.done_object = False
        #     self.center = False
            # self.elapsed_check_object = None
        
        elif self.scenario_finish and self.no_detect:
            if self.done_drop :
                self.get_logger().warn("Sudah nabrak 3 flare dan drop ball.")
                self.flag = 5
                self.pub_flag.publish(self.flag)
            else :
                self.get_logger().warn("MASUK KE FLAG 4")
                self.flag = 4
                self.pub_flag.publish(self.flag)
                
            # Reset state untuk digunakan di zona berikutnya
            self.start_rotation_time = None
            self.detected_start_time = None
            self.stabilized = False
            self.detect = False
            self.scenario_finish = False
            self.no_detect = True
            self.done_backward = False
            self.done_object = False
            self.center = False
            self.elapsed_check_object = None
                
        elif self.scenario_finish and not self.no_detect:
            if self.current_flare < len(self.flare) - 1:
                print(f"Pindah ke flare selanjutnya")
                self.current_flare += 1
                self.done_flare += 1
            else:
                print("Done all flare!")
  
            if self.current_zone < len(self.scenario_zone) - 1: # -1 jika indeks sepenuhnya, -2 jika indeks kurang 1, dan begitu seterusnya
                self.current_zone += 1
                # while (self.current_zone < len(self.scenario_zone) - 1 and self.scenario_zone[self.current_zone] == self.scenario_zone[self.current_zone - 1]):
                #     print(f"Skipping duplicate zone {self.scenario_zone[self.current_zone]}")
                #     self.current_zone += 1  # Lewati zona yang sama agar tidak stuck
            else :
                if self.done_drop :
                    self.get_logger().warn("Sudah nabrak 3 flare dan drop ball.")
                    self.flag = 5
                    self.pub_flag.publish(self.flag)
                else :
                    self.get_logger().warn("MASUK KE FLAG 4")
                    self.flag = 4
                    self.pub_flag.publish(self.flag)

            # Reset state untuk digunakan di zona berikutnya
            self.start_rotation_time = None
            self.detected_start_time = None
            self.stabilized = False
            self.detect = False
            self.scenario_finish = False
            self.no_detect = True
            self.done_backward = False
            self.done_object = False
            self.center = False
            self.elapsed_check_object = None
            
        
        
    def correct_position(self):
        self.get_logger().warn("Correct position")
        self.pid_pitch.Kp = 7000   # Proportional constant for pitch
        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
        self.pub_multi_pid.publish(self.multi_pid_msg)
        # print("apakah sudah drop bucket = ", self.done_drop)
        self.boost = 350
        self.pub_boost.publish(self.boost)

        if self.zone == 2 and not self.done_drop:
            self.flag = 4
            self.pub_flag.publish(self.flag)

        # Toleransi posisi agar robot tidak terus menerus mengoreksi
        tolerance = 0.5
        self.boost = 350
        self.pub_boost.publish(self.boost)
        # # Koreksi sumbu Y
        if abs(self.robot_y - self.target_y) > tolerance:
            # print("Koreksi sumbu y")
            # self.boost = 350
            # self.pub_boost.publish(self.boost)
            if self.robot_y < self.target_y:
                self.set_point.yaw = self.set_point_yaw_front
                # self.get_logger().warn("Robot di belazone_coordinateskang target, bergerak maju")
                self.pub_set_point.publish(self.set_point)
                self.status = "all"
                self.pub_status.publish("all")
            else:
                self.set_point.yaw = self.normalize_angle(self.set_point_yaw_back)
                # self.get_logger().warn("Robot di depan target, bergerak mundur")
                self.status = "all"
                self.pub_status.publish("all")
                self.pub_set_point.publish(self.set_point)
            self.start_rotation_time = None  # Reset rotasi
            return
        # self.get_logger().info("Posisi Y sudah sesuai dengan target")
    
        # Koreksi sumbu X
        if abs(self.robot_x - self.target_x) > tolerance:
            # print("Koreksi sumbu x")
            # self.boost = 0
            # self.pub_boost.publish(self.boost)
            if self.robot_x < self.target_x:
                # self.get_logger().warn("Robot di kiri target, bergerak belok kanan")
                self.set_point.yaw = self.normalize_angle(self.set_point_yaw_right)
                self.pub_set_point.publish(self.set_point)
                self.status = "all"
                self.pub_status.publish("all")
                # self.get_logger().warn("Robot di kiri target, bergerak sway kanan")
                # self.pub_status.publish("sway_right")
            else:
                # self.get_logger().warn("Robot di kanan target, bergerak belok kiri")
                self.set_point.yaw = self.normalize_angle(self.set_point_yaw_left)
                self.pub_set_point.publish(self.set_point)
                self.status = "all"
                self.pub_status.publish("all")
                # self.get_logger().warn("Robot di kanan target, bergerak sway kiri")
                # self.pub_status.publish("sway_left")
            self.start_rotation_time = None  # Reset rotasi
            return
        
        # self.get_logger().info("Posisi X sudah sesuai dengan target")
   
        self.stabilized = True
        self.center = True
        self.start_rotation_time = self.now()

    def delay_flare(self):
        if self.received_flare : 
            self.elapsed_flare = self.now() - self.start_flare

    def delay_gate(self):
        if self.received_gate and self.start_gate is not None: 
            self.elapsed_gate = self.now() - self.start_gate
        else:
            self.elapsed_gate = 0  # atau tetap None jika tidak ingin mengubah tipe data
            
    def delay_blue_flare(self):
        if self.received_blue_flare and self.start_blue_flare is not None: 
            self.elapsed_blue_flare = self.now() - self.start_blue_flare
        else:
            self.elapsed_blue_flare = 0  # atau tetap None jika tidak ingin mengubah tipe data
            
    def delay_orange_flare(self):
        if self.received_orange_flare and self.start_orange_flare is not None: 
            self.elapsed_orange_flare = self.now() - self.start_orange_flare
        else:
            self.elapsed_orange_flare = 0  # atau tetap None jika tidak ingin mengubah tipe data
            
    def delay_yellow_flare(self):
        if self.received_yellow_flare and self.start_yellow_flare is not None: 
            self.elapsed_yellow_flare = self.now() - self.start_yellow_flare
        else:
            self.elapsed_yellow_flare = 0  # atau tetap None jika tidak ingin mengubah tipe data
            
    def delay_red_flare(self):
        if self.received_red_flare and self.start_red_flare is not None: 
            self.elapsed_red_flare = self.now() - self.start_red_flare
        else:
            self.elapsed_red_flare = 0  # atau tetap None jika tidak ingin mengubah tipe data

    def delay_time(self):
        self.current_time = self.now()
        self.elapsed_time = self.current_time - self.start_time_flag

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or (start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay)

    def start(self):
        if not self.is_start:
            self.start_time = self.now()
            self.is_start = True

        # Generate boot time
        self.boot_time = self.now() - self.start_time
            
        # Wait for a secs to tell other nodes (accumulator & control) to calibrate
        if self.boot_time < self.param_delay:
            self.get_logger().info('STARTING...')
            return

        # Timer condition
        if self.param_duration <= 0 or self.boot_time < self.param_duration:
            self.start_auv()
 
    def start_auv(self):
        if not self.has_published_value :
            self.get_logger().info("Publish Nilai")
            self.pub_multi_pid.publish(self.multi_pid_msg)  # Publishing MultiPID
            self.pub_set_point.publish(self.set_point)
            self.pub_boost.publish(self.boost)
            self.pub_flag.publish(self.flag)
            self.has_published_value = True  # Set flag agar tidak dipublish lagi
            
        # print('Flag', self.flag)
        # print("Status Drop", self.status_drop)
        self.log.pesan_flag1 = (f'Flag = {self.flag}, status = {self.status}, detection_history = {self.detection_history}')
        self.log.pesan_flag2 = (f'Flag = {self.flag}, status = {self.status}')
        self.log.pesan_flag3 = (f'Flag = {self.flag}, stabilized = {self.stabilized}, target zona = {self.target_zone}, zona = {self.zone}, target flare = {self.target_flare}, done_drop = {self.done_drop}, status = {self.status}')
        self.log.pesan_flag4 = (f'Flag = {self.flag}, done_object = {self.done_object}, bucket_detected = {self.bucket_detected}, status_drop = {self.status_drop}, hit flare = {self.hit_flare}, center = {self.center}, status = {self.status}')
        self.log.pesan_flag5 = (f'Flag = {self.flag}, depth_surface = {self.depth_surface}, status = {self.status}')
        self.pub_logging.publish(self.log)
        

        if self.is_in_range(0,320) and self.flag != 5:
            if self.is_in_range(0, 4):
                # depth selama 3 detik
                self.get_logger().info("Go Depth")
                self.status = "dpr_ssy"
                self.pub_status.publish("dpr_ssy")
            elif self.is_in_range(4, 6):
                self.get_logger().info("Mencari flare atau gate")
                self.status = "last_slow"
                self.pub_status.publish("last_slow")
                self.status_depth = True
            elif self.is_in_range(6, 7):
                self.set_point.yaw = self.yaw_asli
                self.pub_set_point.publish(self.set_point)

                # MODE 1 -----------------------------------------------------------------
            elif self.mode == 1 : # ketika gate di depan starting zone prioritas flare
                # self.pid_yaw.Kp = 32
                # self.pub_multi_pid.publish(self.multi_pid_msg)

                if self.flag == 0: 
                    self.detection_threshold = 0.5
                    self.pub_boost.publish(self.boost)
                    self.get_logger().info("Mencari flare atau gate")
                    self.status = "all"
                    self.pub_status.publish("all")
                
                
                # if self.flag == 0:
                #     self.boost = 0
                #     self.pub_boost.publish(self.boost)
                #     self.get_logger().info("Sway kirii")
                #     self.pub_status.publish("sway_left")
            
                elif self.object == "Orange_Flare" and self.flag == 1:
                    self.detection_threshold = 0.3
                    if self.elapsed_flare <= 5:
                        self.pid_pitch.Kp = 2000   # Proportional constant for pitch
                        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                        self.pub_multi_pid.publish(self.multi_pid_msg) 
                        self.get_logger().info("Lurusin Flare_Orange")
                        self.status = "camera_sway"
                        self.pub_status.publish("camera_sway")
                    elif self.elapsed_flare <= 8:
                        self.pid_pitch.Kp = 5000   # Proportional constant for pitch
                        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                        self.pub_multi_pid.publish(self.multi_pid_msg)
                        self.get_logger().info("Camera FLare !!!!")
                        self.status = "camera"
                        self.pub_status.publish("camera")
                    elif self.elapsed_flare <= 14: 
                        # self.get_logger().info("SWAY KANAN MAJU !!!!")
                        # self.pub_status.publish("sway_right_forward")
                        self.get_logger().info("SWAY Kirii MAJU !!!!")
                        self.status = "sway_left_forward"
                        self.pub_status.publish("sway_left_forward")
                    else:
                        self.pid_pitch.Kp = 2000   # Proportional constant for pitch
                        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                        self.pub_multi_pid.publish(self.multi_pid_msg)
                        self.get_logger().info("Tauu kanan")
                        self.status = "yaw_right"
                        self.pub_status.publish("yaw_right")

                elif self.object == "Gate" and self.flag == 2: 
                    # ketika detect gate sway selama 3 detik
                    if self.elapsed_gate <= 3 :
                        self.pid_pitch.Kp = 5000   # Proportional constant for pitch
                        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                        self.pub_multi_pid.publish(self.multi_pid_msg)
                        self.get_logger().info("Camera Gate !!!!")
                        self.status = "camera"
                        self.pub_status.publish("camera")
                    elif self.elapsed_gate <= 7 :
                        self.pid_pitch.Kp = 2000   # Proportional constant for pitch
                        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                        self.pub_multi_pid.publish(self.multi_pid_msg)
                        self.get_logger().info("Lurusin Gate !!!!")
                        self.status = "camera_sway"
                        self.pub_status.publish("camera_sway")
                    elif self.elapsed_gate <= 8 :
                        self.set_point.yaw = self.yaw_asli
                        self.pub_set_point.publish(self.set_point)
                        self.get_logger().info("PUBLISH NILAI SET_POINT BARU !!!!")
                        self.status = "dpr_ssy"
                        self.pub_status.publish("dpr_ssy")
                    else:
                        self.pid_pitch.Kp = 5000   # Proportional constant for pitch
                        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                        self.pub_multi_pid.publish(self.multi_pid_msg)
                        self.boost = 0
                        self.pub_boost.publish(self.boost)
                        self.get_logger().info("all")
                        self.status = "all"
                        self.pub_status.publish("all")
                        self.lost_gate_time = self.now()

                elif self.flag == 2:
                    # Jika pertama kali kehilangan gate, simpan waktu saat ini
                    if self.lost_gate_time is None:
                        self.get_logger().info("MASUK WAKTU BERHENTI")
                        self.lost_gate_time = self.now()

                    lost_duration = self.now() - self.lost_gate_time

                    if lost_duration >= 10:
                        self.get_logger().warn("Gate tidak terdeteksi selama 10 detik! Mengubah flag menjadi 3.")
                        self.flag = 3 # ganti
                        self.pub_flag.publish(self.flag)
                    else:
                        self.pid_pitch.Kp = 4000   # Proportional constant for pitch
                        self.pid_pitch.Ki = 0.0    # Integral constant for pitch
                        self.pid_pitch.Kd = 800.0    # Derivative constant for pitch
                        self.pub_multi_pid.publish(self.multi_pid_msg)
                        self.get_logger().warn(f"Gate hilang selama {lost_duration:.2f} detik, menunggu 10 detik...")
                        self.get_logger().info("last_slow_gate")
                        self.boost = 350
                        self.pub_boost.publish(self.boost)
                        self.status = "last_slow"
                        self.pub_status.publish("last_slow") # ganti
                        
                elif self.flag == 3:
                    self.target_zone = self.scenario_zone[self.current_zone]
                    self.pub_target_zone.publish(self.target_zone)
                    # print("target zona = ",self.target_zone)
                
                    zone_coordinates = {
                        4: (7, 14.5), # 7, 14.5
                        3: (-7, 14.5), # -7, 14.5
                        2: (-7, 19), # -7, 19
                        1: (7, 18.5) # 7, 20.5
                    }
                    
                    # if self.search_zone == 0:
                    #     self.target_x, self.target_y = zone_coordinates[self.target_zone]
                    # elif self.search_zone == 1:
                    #     self.target_x = zone_coordinates[self.target_zone][0] - 3
                    #     self.target_y = zone_coordinates[self.target_zone][1]
                    # elif self.search_zone == 2:
                    #     self.target_x = zone_coordinates[self.target_zone][0] 
                    #     self.target_y = zone_coordinates[self.target_zone][1] + 3

                    self.target_x, self.target_y = zone_coordinates[self.target_zone]
                    # print(f"Robot sedang menuju zone {self.target_zone} dengan X: {self.target_x}, Y: {self.target_y}")
                    # print(f"Robot berada di koordinat: x = {self.robot_x:.2f}, y = {self.robot_y:.2f}")
                    
                    print(f"Stablize: {self.stabilized}")
                    print(f"Center: {self.center}")
                    # print(f"Hit Flare: {self.hit_flare}")
                    
                    if self.check_object:
                        if self.target_zone == 4:
                            if not self.center:
                                self.correct_position()
                            elif self.stabilized and self.center: 
                                self.stabilize_and_advance()

                        elif self.target_zone == 3:
                            if not self.center:
                                self.correct_position()
                            elif self.stabilized and self.center: 
                                self.stabilize_and_advance()

                        elif self.target_zone == 2:
                            if not self.center:
                                self.correct_position()
                            elif self.stabilized and self.center: 
                                self.stabilize_and_advance()
                                
                        elif self.target_zone == 1:
                            if not self.center:
                                self.correct_position()
                            elif self.stabilized and self.center: 
                                self.stabilize_and_advance()
                        
                elif self.flag == 4:
                    # self.set_point.depth = -0.1
                    # self.pub_set_point.publish(self.set_point)
                    target_zone = 2
                    self.pub_target_zone.publish(target_zone)
                    zone_coordinates = {
                        4: (7, 14.5), # 7, 15.5
                        3: (-7, 14.5), # -7, 15.5
                        2: (-7, 19), # -7, 20
                        1: (7, 20.5) # 7, 21.5
                    }
                    self.target_x, self.target_y = zone_coordinates[target_zone]

                    print("flare yang sudah ditabrak = ",self.hit_flare)
                    # print("robot sudah di tengah = ", self.center)
                    # print("robot berada di zona = ", self.zone, "dan target_zona = ",self.target_zone)
                    # print("Target x = ", self.target_x , "Target y = ",self.target_y)

                    # self.center = True # ganti
                    if not self.center:
                        print("correct position bucket")
                        self.correct_position()
                    elif self.center:  
                        self.go_to_bucket()

                elif self.flag == 5:
                    self.get_logger().info("ROBOT BERHENTII")
                    self.set_point.depth = self.depth_surface
                    self.pub_set_point.publish(self.set_point)
                    self.status = "dpr"
                    self.pub_status.publish("dpr")
                    
        else :
            if self.center:
                self.get_logger().info("ROBOT BERHENTII")
                self.set_point.depth = self.depth_surface
                self.pub_set_point.publish(self.set_point)
                self.status = "dpr"
                self.pub_status.publish("dpr")
            else :
                if self.start_to_back is None:
                    self.start_to_back = self.now()
                self.elapsed_to_back = self.now() - self.start_to_back
                if self.elapsed_to_back <= 10 :
                    self.center = False
                    self.get_logger().info("ROBOT BERHENTII")
                    self.set_point.depth = self.depth_surface
                    self.pub_set_point.publish(self.set_point)
                    self.status = "dpr"
                    self.pub_status.publish("dpr")
                else :
                    self.get_logger().warn("robot menuju zona awal")
                    self.set_point.depth = self.depth_surface + 0.2
                    self.pub_set_point.publish(self.set_point)
                    self.target_x = 1
                    self.target_y = 1
                    self.correct_position()
            
def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass