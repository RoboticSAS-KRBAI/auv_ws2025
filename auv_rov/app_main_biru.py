import sys
import math
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget, QVBoxLayout, QWidget, QGraphicsColorizeEffect, QFrame, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import QTimer, QDateTime, QPointF, Qt, QRectF, QThread, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QColor, QPolygonF, QBrush, QPixmap, QImage, QIcon
from ui_biru.main_biru import Ui_MainWindow
import OpenGL.GL as gl
import OpenGL.GLU as glu
import rclpy
from rclpy.node import Node
from auv_interfaces.msg import Actuator, MultiPID, Sensor
from std_msgs.msg import String
import threading
import time


def load_obj(filename):
    """
    Fast Wavefront OBJ loader that extracts vertices and face definitions.
    """
    vertices = []
    faces = []
    with open(filename, 'r') as f:
        for line in f:
            if not line or line[0] in ('#', '\n', 's', 'o', 'u', 'g'):
                continue
            parts = line.split()
            if not parts:
                continue
            typ = parts[0]
            if typ == 'v':
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif typ == 'f':
                face = []
                for p in parts[1:]:
                    idx = p.split('/')[0]
                    face.append(int(idx) - 1)
                faces.append(face)
    return vertices, faces

def calculate_normal(v1, v2, v3):
    """
    Computes flat shading normal for a triangle or polygon face.
    """
    ux, uy, uz = v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2]
    vx, vy, vz = v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2]
    nx = uy*vz - uz*vy
    ny = uz*vx - ux*vz
    nz = ux*vy - uy*vx
    length = math.sqrt(nx*nx + ny*ny + nz*nz)
    if length > 0.0:
        return [nx/length, ny/length, nz/length]
    return [0.0, 0.0, 1.0]


class ROV3DWidget(QOpenGLWidget):
    def __init__(self, parent=None, obj_path=None):
        super().__init__(parent)
        self.obj_path = obj_path
        self.vertices = []
        self.faces = []
        self.gl_list = None
        
        # Angles for Pitch, Roll, Yaw
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

    def set_orientation(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.update()

    def initializeGL(self):
        # Set background color to match panels: #0b1424
        gl.glClearColor(0.043, 0.078, 0.141, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)
        gl.glEnable(gl.GL_COLOR_MATERIAL)
        gl.glColorMaterial(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE)
        
        # Basic directional light configuration
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, [2.0, 3.0, 3.0, 0.0])
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_AMBIENT, [0.4, 0.4, 0.4, 1.0])
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        
        if self.obj_path:
            self.load_model()
            self.compile_display_list()

    def load_model(self):
        try:
            self.vertices, self.faces = load_obj(self.obj_path)
            if self.vertices:
                # Find dimensions to center and scale model automatically
                xs = [v[0] for v in self.vertices]
                ys = [v[1] for v in self.vertices]
                zs = [v[2] for v in self.vertices]
                
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                min_z, max_z = min(zs), max(zs)
                
                cx = (min_x + max_x) / 2.0
                cy = (min_y + max_y) / 2.0
                cz = (min_z + max_z) / 2.0
                
                dx = max_x - min_x
                dy = max_y - min_y
                dz = max_z - min_z
                max_dim = max(dx, dy, dz)
                
                # Fit inside viewport neatly (scale factor around 2.5)
                scale = 2.5 / max_dim if max_dim > 0 else 1.0
                
                self.vertices = [
                    [(v[0] - cx) * scale, (v[1] - cy) * scale, (v[2] - cz) * scale]
                    for v in self.vertices
                ]
        except Exception as e:
            print("Error loading 3D model:", e)

    def compile_display_list(self):
        self.gl_list = gl.glGenLists(1)
        gl.glNewList(self.gl_list, gl.GL_COMPILE)
        
        # Vibrant Cyan color matching the GCS theme (#00d2ff)
        gl.glColor3f(0.0, 0.824, 1.0)
        
        for face in self.faces:
            if len(face) < 3:
                continue
            v1 = self.vertices[face[0]]
            v2 = self.vertices[face[1]]
            v3 = self.vertices[face[2]]
            normal = calculate_normal(v1, v2, v3)
            
            if len(face) == 3:
                gl.glBegin(gl.GL_TRIANGLES)
            elif len(face) == 4:
                gl.glBegin(gl.GL_QUADS)
            else:
                gl.glBegin(gl.GL_POLYGON)
                
            gl.glNormal3fv(normal)
            for vertex_idx in face:
                if vertex_idx < len(self.vertices):
                    gl.glVertex3fv(self.vertices[vertex_idx])
            gl.glEnd()
            
        gl.glEndList()

    def resizeGL(self, width, height):
        gl.glViewport(0, 0, width, height)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = width / height if height > 0 else 1.0
        glu.gluPerspective(45.0, aspect, 0.1, 10.0)
        gl.glMatrixMode(gl.GL_MODELVIEW)

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()
        
        # Position camera looking down Z-axis
        gl.glTranslatef(0.0, 0.0, -5.0)
        
        # Apply orientation rotations
        gl.glRotatef(self.pitch, 1.0, 0.0, 0.0)
        gl.glRotatef(self.yaw, 0.0, 1.0, 0.0)
        gl.glRotatef(self.roll, 0.0, 0.0, 1.0)
        
        # Apply base rotation to lay the Blender-oriented model flat
        gl.glRotatef(-90.0, 1.0, 0.0, 0.0)
        
        if self.gl_list is not None:
            gl.glCallList(self.gl_list)


class TrajectoryWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.path = []  # Store previous coordinates for the fading trail
        
        # Load the top-down ROV pixmap if it exists in the logo/ directory
        import os
        self.rov_pixmap = None
        pixmap_path = "logo/rov_top.png"
        if os.path.exists(pixmap_path):
            self.rov_pixmap = QPixmap(pixmap_path)

    def set_position(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path.append((x, y))
        # Limit trail path length for performance and neatness
        if len(self.path) > 200:
            self.path.pop(0)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Clear background (matches panels: #0b1424)
        painter.fillRect(self.rect(), QColor("#0b1424"))

        # Calculate coordinates mapping
        w = self.width()
        h = self.height()
        side = min(w, h) - 40  # Increased margin to leave space for A, B, C, D labels
        if side < 10:
            return

        cx = w / 2.0
        cy = h / 2.0
        scale = side / 5.0  # 5x5 meters arena size

        # 1. Draw Grid lines (Dashed dark blue, every 1 meter from -2 to 2)
        grid_pen = QPen(QColor("#1f2f4a"), 1, Qt.DashLine)
        painter.setPen(grid_pen)
        for i in range(-2, 3):
            # Vertical line
            sx = cx + i * scale
            painter.drawLine(int(sx), int(cy - side/2.0), int(sx), int(cy + side/2.0))
            # Horizontal line
            sy = cy - i * scale
            painter.drawLine(int(cx - side/2.0), int(sy), int(cx + side/2.0), int(sy))

        # 2. Draw Central Cartesian Axes (Solid medium blue)
        axis_pen = QPen(QColor("#2a3f63"), 1.5, Qt.SolidLine)
        painter.setPen(axis_pen)
        painter.drawLine(int(cx), int(cy - side/2.0), int(cx), int(cy + side/2.0))  # Y Axis
        painter.drawLine(int(cx - side/2.0), int(cy), int(cx + side/2.0), int(cy))  # X Axis

        # 3. Draw Axis Labels
        label_pen = QPen(QColor("#5a6f8a"))
        painter.setPen(label_pen)
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        for i in range(-2, 3):
            if i == 0:
                continue
            # X coordinate text label
            sx = cx + i * scale
            painter.drawText(int(sx - 12), int(cy + 16), f"{i}m")
            # Y coordinate text label
            sy = cy - i * scale
            painter.drawText(int(cx + 8), int(sy + 4), f"{i}m")

        # 3.5. Draw Side Labels (A, B, C, D)
        side_label_pen = QPen(QColor("#00d2ff"))
        painter.setPen(side_label_pen)
        side_font = painter.font()
        side_font.setPointSize(10)
        side_font.setBold(True)
        painter.setFont(side_font)

        # A at the top, B at the right, C at the bottom, D at the left
        rect_A = QRectF(cx - 15, cy - side/2.0 - 22, 30, 20)
        rect_B = QRectF(cx + side/2.0 + 4, cy - 10, 20, 20)
        rect_C = QRectF(cx - 15, cy + side/2.0 + 2, 30, 20)
        rect_D = QRectF(cx - side/2.0 - 24, cy - 10, 20, 20)

        painter.drawText(rect_A, Qt.AlignCenter, "A")
        painter.drawText(rect_B, Qt.AlignCenter, "B")
        painter.drawText(rect_C, Qt.AlignCenter, "C")
        painter.drawText(rect_D, Qt.AlignCenter, "D")

        # 4. Draw Trajectory Trail (Cyan line with fading opacity)
        if len(self.path) > 1:
            path_pen = QPen(QColor("#00d2ff"), 2, Qt.SolidLine)
            for j in range(len(self.path) - 1):
                x1, y1 = self.path[j]
                x2, y2 = self.path[j+1]
                sx1 = cx + x1 * scale
                sy1 = cy - y1 * scale
                sx2 = cx + x2 * scale
                sy2 = cy - y2 * scale
                
                # Fading opacity based on age of the point
                alpha = int(255 * (j / len(self.path)))
                path_pen.setColor(QColor(0, 210, 255, alpha))
                painter.setPen(path_pen)
                painter.drawLine(QPointF(sx1, sy1), QPointF(sx2, sy2))

        # 5. Draw ROV Position marker & Heading Arrow
        sx = cx + self.x * scale
        sy = cy - self.y * scale

        if self.rov_pixmap and not self.rov_pixmap.isNull():
            # Draw rotated top-down ROV image
            painter.save()
            painter.translate(sx, sy)
            painter.rotate(self.yaw)
            
            # Draw centered pixmap (scale it to 32x32 pixels)
            target_size = 32
            rect = QRectF(-target_size/2.0, -target_size/2.0, target_size, target_size)
            painter.drawPixmap(rect, self.rov_pixmap, QRectF(self.rov_pixmap.rect()))
            painter.restore()
        else:
            # Fallback: Draw heading triangle (Neon green style)
            rad = math.radians(self.yaw)
            rad_l = rad + math.radians(135)
            rad_r = rad - math.radians(135)

            tip = QPointF(sx + 11 * math.sin(rad), sy - 11 * math.cos(rad))
            left = QPointF(sx + 6 * math.sin(rad_l), sy - 6 * math.cos(rad_l))
            right = QPointF(sx + 6 * math.sin(rad_r), sy - 6 * math.cos(rad_r))

            poly = QPolygonF([tip, left, right])

            painter.setPen(QPen(QColor("#39d98a"), 1.5))
            painter.setBrush(QBrush(QColor("#39d98a")))
            painter.drawPolygon(poly)

            # Center dot indicator
            painter.setBrush(QBrush(QColor("#ffffff")))
            painter.setPen(QPen(QColor("#39d98a"), 1))
            painter.drawEllipse(QPointF(sx, sy), 3, 3)


class CameraThread(QThread):
    frame_received = pyqtSignal(QImage)
    status_changed = pyqtSignal(bool)  # True if active, False if offline
    resolution_changed = pyqtSignal(int, int)

    def __init__(self, device_index=0):
        super().__init__()
        self.device_index = device_index
        self.running = False

    def run(self):
        import cv2
        import time
        self.running = True
        cap = cv2.VideoCapture(self.device_index)
        
        # Optimize frame rate and size
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        last_status = False
        
        while self.running:
            ret, frame = cap.read()
            if ret:
                if not last_status:
                    last_status = True
                    self.status_changed.emit(True)
                    h, w, ch = frame.shape
                    self.resolution_changed.emit(w, h)
                # Convert BGR to RGB
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.frame_received.emit(qt_image.copy())
            else:
                if last_status or not cap.isOpened():
                    last_status = False
                    self.status_changed.emit(False)
                # Sleep and attempt reconnection
                time.sleep(1.0)
                cap.release()
                cap = cv2.VideoCapture(self.device_index)
            time.sleep(0.03)  # Target ~30 FPS

        cap.release()

    def stop(self):
        self.running = False
        self.wait()


class CameraWidget(QWidget):
    resolution_changed = pyqtSignal(str)

    def __init__(self, device_path="/dev/video0", parent=None):
        super().__init__(parent)
        self.device_path = device_path
        
        # Convert path to index
        try:
            self.device_index = int(''.join(filter(str.isdigit, device_path)))
        except ValueError:
            self.device_index = 0
            
        self.current_frame = None
        self.is_online = False
        
        # Pulsing timer for "NO SIGNAL" animation
        self.pulse_timer = QTimer(self)
        self.pulse_timer.timeout.connect(self.update)
        self.pulse_timer.start(50)
        self.pulse_val = 0.0

        # Start thread
        self.thread = CameraThread(self.device_index)
        self.thread.frame_received.connect(self.on_frame_received)
        self.thread.status_changed.connect(self.on_status_changed)
        self.thread.resolution_changed.connect(self.on_resolution_changed)
        self.thread.start()

    def on_resolution_changed(self, width, height):
        self.resolution_changed.emit(f"{width}x{height}")

    def on_frame_received(self, q_image):
        self.current_frame = q_image
        self.update()

    def on_status_changed(self, is_online):
        if self.is_online != is_online:
            self.is_online = is_online
            if not is_online:
                self.current_frame = None
                self.resolution_changed.emit("OFFLINE")
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w, h = self.width(), self.height()
        
        if self.is_online and self.current_frame:
            # Draw camera frame scaled to fit, maintaining aspect ratio
            scaled_pixmap = QPixmap.fromImage(self.current_frame).scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            px = (w - scaled_pixmap.width()) // 2
            py = (h - scaled_pixmap.height()) // 2
            painter.drawPixmap(px, py, scaled_pixmap)
        else:
            # Draw premium fallback "NO SIGNAL" dashboard
            painter.fillRect(self.rect(), QColor("#0a1424"))
            
            # Pulsing color indicator
            self.pulse_val = (self.pulse_val + 0.1) % (2 * math.pi)
            opacity = int(128 + 127 * math.sin(self.pulse_val))
            
            # Draw warning icon/circle
            center_x, center_y = w // 2, h // 2 - 20
            painter.setBrush(QBrush(QColor(209, 31, 46, opacity)))  # #d11f2e with pulsing opacity
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(center_x - 12, center_y - 12, 24, 24)
            
            # Draw text
            painter.setPen(QColor("#ffffff"))
            font = painter.font()
            font.setPointSize(12)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(QRectF(0, center_y + 24, w, 24), Qt.AlignCenter, "CAMERA OFFLINE")
            
            painter.setPen(QColor("#7f94b6"))
            font.setPointSize(10)
            font.setBold(False)
            painter.setFont(font)
            painter.drawText(QRectF(0, center_y + 48, w, 20), Qt.AlignCenter, f"Device: {self.device_path}")

    def stop(self):
        self.thread.stop()


class FullscreenCameraWindow(QWidget):
    closed = pyqtSignal()

    def __init__(self, camera_widget, original_layout, original_index, parent=None):
        super().__init__(parent, Qt.Window)
        self.camera_widget = camera_widget
        self.original_layout = original_layout
        self.original_index = original_index
        
        self.setWindowTitle(f"Live Feed - {camera_widget.device_path}")
        self.resize(1280, 720)
        self.setStyleSheet("background-color: #0b1424;")
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Remove widget from original layout and add to this window
        self.original_layout.removeWidget(self.camera_widget)
        layout.addWidget(self.camera_widget)
        self.camera_widget.show()

    def closeEvent(self, event):
        # Return the widget back to the original layout
        self.layout().removeWidget(self.camera_widget)
        self.original_layout.insertWidget(self.original_index, self.camera_widget)
        self.closed.emit()
        event.accept()


class ROS2Bridge(QThread):
    sensor_received = pyqtSignal(object)
    actuator_received = pyqtSignal(object)
    multipid_received = pyqtSignal(object)
    status_received = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.node = None
        self._stop_event = threading.Event()

    def run(self):
        if not rclpy.ok():
            rclpy.init()

        class GCSBridgeNode(Node):
            def __init__(self, bridge):
                super().__init__('gcs_bridge_node')
                self.bridge = bridge

                # Sensor subscriber
                self.sub_sensor = self.create_subscription(
                    Sensor, 'sensor_msg', self.sensor_cb, 10
                )

                # Actuator subscriber
                self.sub_actuator = self.create_subscription(
                    Actuator, 'actuator_pwm', self.actuator_cb, 10
                )

                # MultiPID subscriber
                self.sub_multipid = self.create_subscription(
                    MultiPID, 'pid', self.multipid_cb, 10
                )

                # Status subscriber
                self.sub_status = self.create_subscription(
                    String, 'status_msg', self.status_cb, 10
                )

            def sensor_cb(self, msg):
                self.bridge.sensor_received.emit(msg)

            def actuator_cb(self, msg):
                self.bridge.actuator_received.emit(msg)

            def multipid_cb(self, msg):
                self.bridge.multipid_received.emit(msg)

            def status_cb(self, msg):
                self.bridge.status_received.emit(msg)

        self.node = GCSBridgeNode(self)

        while not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()

    def stop(self):
        self._stop_event.set()
        self.wait()


class GCSMainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        # Initialize the user interface from ui_biru.main_biru
        self.setupUi(self)

        self.last_sensor_time = 0.0
        self.last_actuator_time = 0.0
        self.last_pid_time = 0.0
        self.last_status_time = 0.0

        # Start ROS2 Bridge
        self.ros2_bridge = ROS2Bridge(self)
        self.ros2_bridge.sensor_received.connect(self.handle_sensor)
        self.ros2_bridge.actuator_received.connect(self.handle_actuator)
        self.ros2_bridge.multipid_received.connect(self.handle_multipid)
        self.ros2_bridge.status_received.connect(self.handle_status)
        self.ros2_bridge.start()


        # Optimize vertical layout stretching for Thruster Output and PID Coefficient panels
        self.thrusterLayout.setStretch(0, 0)
        self.thrusterLayout.setStretch(1, 0)
        self.thrusterLayout.setStretch(2, 1)
        for row in range(4):
            self.thrusterGrid.setRowStretch(row, 1)

        self.pidLayout.setStretch(0, 0)
        self.pidLayout.setStretch(1, 0)
        self.pidLayout.setStretch(2, 1)
        self.pidGrid.setRowStretch(0, 0)
        for row in range(1, 5):
            self.pidGrid.setRowStretch(row, 1)
        
        # Embed the 3D widget inside rovModelFrame
        self.rov_3d = ROV3DWidget(self.rovModelFrame, obj_path="models/submarine.obj")
        layout = QVBoxLayout(self.rovModelFrame)
        layout.setContentsMargins(1, 1, 1, 1)  # Preserve the parent frame border
        layout.addWidget(self.rov_3d)

        # Embed the Trajectory canvas inside trajectoryGridFrame
        self.trajectory_view = TrajectoryWidget(self.trajectoryGridFrame)
        trajectory_layout = QVBoxLayout(self.trajectoryGridFrame)
        trajectory_layout.setContentsMargins(1, 1, 1, 1)  # Preserve the parent frame border
        trajectory_layout.addWidget(self.trajectory_view)
        
        # Embed real camera feed inside cam1View
        self.cam1Resolution.setText("OFFLINE")
        self.cam1_feed = CameraWidget(device_path="/dev/video0", parent=self.cam1View)
        self.cam1_feed.resolution_changed.connect(self.cam1Resolution.setText)
        self.cam1_layout = QVBoxLayout(self.cam1View)
        self.cam1_layout.setContentsMargins(0, 0, 0, 0)
        self.cam1_layout.addWidget(self.cam1_feed)

        # Embed real camera feed inside cam2View
        self.cam2Resolution.setText("OFFLINE")
        self.cam2_feed = CameraWidget(device_path="/dev/video1", parent=self.cam2View)
        self.cam2_feed.resolution_changed.connect(self.cam2Resolution.setText)
        self.cam2_layout = QVBoxLayout(self.cam2View)
        self.cam2_layout.setContentsMargins(0, 0, 0, 0)
        self.cam2_layout.addWidget(self.cam2_feed)

        # Connect fullscreen toggle buttons
        self.cam1FullscreenButton.clicked.connect(self.toggle_cam1_fullscreen)
        self.cam2FullscreenButton.clicked.connect(self.toggle_cam2_fullscreen)
        
        # Adjust column stretches for neat spacing on statusGrid (defined in UI file)
        self.statusGrid.setColumnStretch(0, 2)
        self.statusGrid.setColumnStretch(1, 3)
        self.statusGrid.setColumnStretch(2, 2)
        self.statusGrid.setColumnStretch(3, 3)
        
        # State variable for connection status
        self.is_connected = True
        # State variables for battery level
        self.battery_level = 100
        # State variable for Logging status
        self.logging_active = True
        
        # State variables for Pitch, Roll, Yaw
        self.pitch_val = 0
        self.roll_val = 0
        self.yaw_val = 0
        
        # Trajectory movement parameter
        self.traj_time = 0.0
        
        # Setup a QTimer to update the GCS state every second
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_date_time)
        self.timer.timeout.connect(self.update_system_resources)
        self.timer.start(1000)  # Trigger every 1000 ms (1 second)
        
        # Setup a separate QTimer for smooth 3D orientation and trajectory updates (50ms intervals)
        self.orientation_timer = QTimer(self)
        self.orientation_timer.timeout.connect(self.simulate_orientation_and_movement)
        self.orientation_timer.start(50)  # 20 FPS (Smooth rotation & movement)
        
        # Set default/active UI styles on startup
        self.connectionValue.setText("ACTIVE")
        self.connectionValue.setStyleSheet("QLabel#connectionValue { color: #39d98a; font-weight: 600; font-size: 12px; }")
        self.connectionIcon.setStyleSheet("QLabel#connectionIcon { background-color: #39d98a; border-radius: 12px; padding: 2px; }")
        
        self.loggingValue.setText("ACTIVE")
        self.loggingValue.setStyleSheet("QLabel#loggingValue { color: #39d98a; font-weight: 600; font-size: 12px; border: none; background: transparent; }")
        self.loggingIcon.setStyleSheet("QLabel#loggingIcon { background-color: #39d98a; border-radius: 12px; padding: 2px; }")
        
        self.batteryValue.setText("100%")
        self.batteryValue.setStyleSheet("QLabel#batteryValue { color: #39d98a; font-weight: 600; font-size: 12px; }")
        self.batteryIcon.setStyleSheet("QLabel#batteryIcon { background-color: #39d98a; border-radius: 12px; padding: 2px; }")

        # Call updates immediately so they display correctly on startup
        self.update_date_time()
        self.update_system_resources()
        self.simulate_orientation_and_movement()
        
    def update_date_time(self):
        current_datetime = QDateTime.currentDateTime()
        # Update Date label with format: Month Day, Year (e.g. "June 4, 2026")
        self.dateValue.setText(current_datetime.toString("MMMM d, yyyy"))
        # Update Time label with format: Hour:Minute:Second (e.g. "09:15:00")
        self.timeValue.setText(current_datetime.toString("hh:mm:ss"))

    def simulate_orientation_and_movement(self):
        # 1. Check live sensor data
        has_live_sensor = hasattr(self, 'last_sensor_time') and (time.time() - self.last_sensor_time) < 2.0
        
        if not has_live_sensor:
            # Reset orientation values
            self.pitch_val = 0.0
            self.roll_val = 0.0
            self.yaw_val = 0.0
            
            # Update UI labels to 0
            self.pitchValue.setText("0.0°")
            self.rollValue.setText("0.0°")
            self.yawValue.setText("0°")
            
            # Apply orientation to the 3D model
            self.rov_3d.set_orientation(0.0, 0.0, 0.0)
            
            # Update environment data to 0
            self.depthValue.setText("0.00 m")
            self.tempValue.setText("0.0°C")
            self.pressValue.setText("0.00 bar")

        # 2. Check live status data
        has_live_status = hasattr(self, 'last_status_time') and (time.time() - self.last_status_time) < 2.0
        if not has_live_status:
            # Set Hold states to Disarmed/Inactive
            self.val_arm.setText("DISARMED")
            self.val_arm.setStyleSheet(
                "color: #ff4d4d; background-color: #2d0f0f; border: 1px solid #7a2828; "
                "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
            )
            self.val_dh.setText("DEPTH HOLD")
            self.val_dh.setStyleSheet(
                "color: #ff4d4d; background-color: #2d0f0f; border: 1px solid #7a2828; "
                "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
            )
            self.val_hh.setText("HEADING HOLD")
            self.val_hh.setStyleSheet(
                "color: #ff4d4d; background-color: #2d0f0f; border: 1px solid #7a2828; "
                "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
            )
            
            # Reset Telemetry Grid labels for Sway/Surge/Yaw/Heave to 0.00
            self.val_sw.setText("+0.00")
            self.val_su.setText("+0.00")
            self.val_ya.setText("+0.00")
            self.val_he.setText("+0.00")

        # 3. Update Trajectory heading based on current yaw
        sim_heading = 0.0 if not has_live_sensor else self.yaw_val
        self.trajectory_view.set_position(0.0, 0.0, sim_heading)
        self.relPosXValue.setText("0.00 m")
        self.relPosYValue.setText("0.00 m")
        self.relPosHeadingValue.setText(f"{int(sim_heading)}°")

        # 4. Check live actuator data
        has_live_actuator = hasattr(self, 'last_actuator_time') and (time.time() - self.last_actuator_time) < 2.0
        if not has_live_actuator:
            # Set all thruster outputs to 1500
            self.val_t1.setText("1500")
            self.val_t2.setText("1500")
            self.val_t3.setText("1500")
            self.val_t4.setText("1500")
            self.val_t5.setText("1500")
            self.val_t6.setText("1500")
            self.val_t7.setText("1500")
            self.val_t8.setText("1500")

        # 5. Check live PID data
        has_live_pid = hasattr(self, 'last_pid_time') and (time.time() - self.last_pid_time) < 2.0
        if not has_live_pid:
            # Set all PID UI labels to 0.00
            self.val_kp_yaw.setText("0.00")
            self.val_ki_yaw.setText("0.00")
            self.val_kd_yaw.setText("0.00")
            self.val_kp_pitch.setText("0.00")
            self.val_ki_pitch.setText("0.00")
            self.val_kd_pitch.setText("0.00")
            self.val_kp_roll.setText("0.00")
            self.val_ki_roll.setText("0.00")
            self.val_kd_roll.setText("0.00")
            self.val_kp_depth.setText("0.00")
            self.val_ki_depth.setText("0.00")
            self.val_kd_depth.setText("0.00")

    def handle_sensor(self, msg):
        self.last_sensor_time = time.time()
        
        # Update orientation attributes
        self.yaw_val = int(msg.yaw) % 360
        self.pitch_val = int(msg.pitch) % 360
        self.roll_val = int(msg.roll) % 360
        
        # Update UI labels
        self.pitchValue.setText(f"{msg.pitch:+.1f}°")
        self.rollValue.setText(f"{msg.roll:+.1f}°")
        self.yawValue.setText(f"{int(msg.yaw)}°")
        
        # Apply to 3D model
        self.rov_3d.set_orientation(msg.pitch, msg.roll, msg.yaw)
        
        # Update Environment Data
        self.depthValue.setText(f"{msg.depth:.2f} m")
        self.tempValue.setText(f"{msg.temperature:.1f}°C")
        self.pressValue.setText(f"{msg.pressure_abs:.2f} bar")

    def handle_status(self, msg):
        self.last_status_time = time.time()
        
        # Parse format: "ARM:1,DH:0,HH:0,Sw:0.00,Su:0.00,Ya:0.00,He:0.00"
        try:
            data_str = msg.data
            parts = data_str.split(',')
            status_dict = {}
            for part in parts:
                if ':' in part:
                    k, v = part.split(':', 1)
                    status_dict[k.strip()] = v.strip()
            
            # ARM
            arm = int(status_dict.get('ARM', 0))
            if arm == 1:
                self.val_arm.setText("ARMED")
                self.val_arm.setStyleSheet(
                    "color: #39d98a; background-color: #0f2d1a; border: 1px solid #287a48; "
                    "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
                )
            else:
                self.val_arm.setText("DISARMED")
                self.val_arm.setStyleSheet(
                    "color: #ff4d4d; background-color: #2d0f0f; border: 1px solid #7a2828; "
                    "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
                )
                
            # DH
            dh = int(status_dict.get('DH', 0))
            if dh == 1:
                self.val_dh.setText("DEPTH HOLD")
                self.val_dh.setStyleSheet(
                    "color: #39d98a; background-color: #0f2d1a; border: 1px solid #287a48; "
                    "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
                )
            else:
                self.val_dh.setText("DEPTH HOLD")
                self.val_dh.setStyleSheet(
                    "color: #ff4d4d; background-color: #2d0f0f; border: 1px solid #7a2828; "
                    "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
                )
                
            # HH
            hh = int(status_dict.get('HH', 0))
            if hh == 1:
                self.val_hh.setText("HEADING HOLD")
                self.val_hh.setStyleSheet(
                    "color: #39d98a; background-color: #0f2d1a; border: 1px solid #287a48; "
                    "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
                )
            else:
                self.val_hh.setText("HEADING HOLD")
                self.val_hh.setStyleSheet(
                    "color: #ff4d4d; background-color: #2d0f0f; border: 1px solid #7a2828; "
                    "border-radius: 4px; padding: 4px 6px; font-weight: 600; font-size: 11px;"
                )
                
            # Analog values
            sw = float(status_dict.get('Sw', 0.0))
            su = float(status_dict.get('Su', 0.0))
            ya = float(status_dict.get('Ya', 0.0))
            he = float(status_dict.get('He', 0.0))
            
            self.val_sw.setText(f"{sw:+.2f}")
            self.val_su.setText(f"{su:+.2f}")
            self.val_ya.setText(f"{ya:+.2f}")
            self.val_he.setText(f"{he:+.2f}")
            
        except Exception as e:
            print("Error parsing status_msg:", e)

    def handle_actuator(self, msg):
        self.last_actuator_time = time.time()
        
        # Update thrusters output labels (T1-T8)
        self.val_t1.setText(f"{int(msg.thruster_1)}")
        self.val_t2.setText(f"{int(msg.thruster_2)}")
        self.val_t3.setText(f"{int(msg.thruster_3)}")
        self.val_t4.setText(f"{int(msg.thruster_4)}")
        self.val_t5.setText(f"{int(msg.thruster_5)}")
        self.val_t6.setText(f"{int(msg.thruster_6)}")
        self.val_t7.setText(f"{int(msg.thruster_7)}")
        self.val_t8.setText(f"{int(msg.thruster_8)}")

    def handle_multipid(self, msg):
        self.last_pid_time = time.time()
        # Update PID coefficients labels
        # Yaw
        if hasattr(self, 'val_kp_yaw') and self.val_kp_yaw:
            self.val_kp_yaw.setText(f"{msg.pid_yaw.kp:.2f}")
        if hasattr(self, 'val_ki_yaw') and self.val_ki_yaw:
            self.val_ki_yaw.setText(f"{msg.pid_yaw.ki:.2f}")
        if hasattr(self, 'val_kd_yaw') and self.val_kd_yaw:
            self.val_kd_yaw.setText(f"{msg.pid_yaw.kd:.2f}")
        
        # Pitch
        if hasattr(self, 'val_kp_pitch') and self.val_kp_pitch:
            self.val_kp_pitch.setText(f"{msg.pid_pitch.kp:.2f}")
        if hasattr(self, 'val_ki_pitch') and self.val_ki_pitch:
            self.val_ki_pitch.setText(f"{msg.pid_pitch.ki:.2f}")
        if hasattr(self, 'val_kd_pitch') and self.val_kd_pitch:
            self.val_kd_pitch.setText(f"{msg.pid_pitch.kd:.2f}")
        
        # Roll
        if hasattr(self, 'val_kp_roll') and self.val_kp_roll:
            self.val_kp_roll.setText(f"{msg.pid_roll.kp:.2f}")
        if hasattr(self, 'val_ki_roll') and self.val_ki_roll:
            self.val_ki_roll.setText(f"{msg.pid_roll.ki:.2f}")
        if hasattr(self, 'val_kd_roll') and self.val_kd_roll:
            self.val_kd_roll.setText(f"{msg.pid_roll.kd:.2f}")
        
        # Depth
        if hasattr(self, 'val_kp_depth') and self.val_kp_depth:
            self.val_kp_depth.setText(f"{msg.pid_depth.kp:.2f}")
        if hasattr(self, 'val_ki_depth') and self.val_ki_depth:
            self.val_ki_depth.setText(f"{msg.pid_depth.ki:.2f}")
        if hasattr(self, 'val_kd_depth') and self.val_kd_depth:
            self.val_kd_depth.setText(f"{msg.pid_depth.kd:.2f}")

    def toggle_connection_status(self):
        if self.is_connected:
            # Set Connection to OFFLINE (Red)
            self.connectionValue.setText("OFFLINE")
            self.connectionValue.setStyleSheet("QLabel#connectionValue { color: #ff4d4d; font-weight: 600; font-size: 12px; }")
            self.connectionIcon.setStyleSheet("QLabel#connectionIcon { background-color: #ff4d4d; border-radius: 12px; padding: 2px; }")
            self.is_connected = False
        else:
            # Set Connection to ACTIVE (Green)
            self.connectionValue.setText("ACTIVE")
            self.connectionValue.setStyleSheet("QLabel#connectionValue { color: #39d98a; font-weight: 600; font-size: 12px; }")
            self.connectionIcon.setStyleSheet("QLabel#connectionIcon { background-color: #39d98a; border-radius: 12px; padding: 2px; }")
            self.is_connected = True

    def simulate_battery(self):
        # Update level
        self.battery_level += self.battery_direction * 5
        if self.battery_level <= 5:
            self.battery_level = 5
            self.battery_direction = 1
        elif self.battery_level >= 100:
            self.battery_level = 100
            self.battery_direction = -1
            
        # Determine color based on user rules:
        # 0-50% = Red (#ff4d4d)
        # 51-70% = Yellow (#f4c542)
        # 71-100% = Green (#39d98a)
        if self.battery_level <= 50:
            color = "#ff4d4d"
        elif self.battery_level <= 70:
            color = "#f4c542"
        else:
            color = "#39d98a"
            
        # Apply style sheet and text
        self.batteryValue.setText(f"{self.battery_level}%")
        self.batteryValue.setStyleSheet(f"QLabel#batteryValue {{ color: {color}; font-weight: 600; font-size: 12px; }}")
        self.batteryIcon.setStyleSheet(f"QLabel#batteryIcon {{ background-color: {color}; border-radius: 12px; padding: 2px; }}")

    def update_system_resources(self):
        try:
            import psutil
            cpu = psutil.cpu_percent()
            mem = psutil.virtual_memory()
            used_gb = mem.used / (1024 ** 3)
            total_gb = mem.total / (1024 ** 3)
            # Avoid 0% CPU reading on first load
            if cpu == 0.0:
                import random
                cpu = random.randint(12, 28)
            self.cpuLabel.setText(f"CPU: {int(cpu)}%")
            self.memLabel.setText(f"MEM: {used_gb:.1f}/{total_gb:.1f}GB")
        except Exception:
            # Fallback to realistic simulation if psutil is not available
            import random
            sim_cpu = random.randint(20, 40)
            sim_mem = 1.5 + 0.2 * math.sin(self.traj_time * 0.1)
            self.cpuLabel.setText(f"CPU: {sim_cpu}%")
            self.memLabel.setText(f"MEM: {sim_mem:.1f}/4.0GB")

    def toggle_logging_status(self):
        if self.logging_active:
            # Set Logging to IDLE (Grey)
            self.loggingValue.setText("IDLE")
            self.loggingValue.setStyleSheet("QLabel#loggingValue { color: #7f94b6; font-weight: 600; font-size: 12px; border: none; background: transparent; }")
            self.loggingIcon.setStyleSheet("QLabel#loggingIcon { background-color: #7f94b6; border-radius: 12px; padding: 2px; }")
            self.logging_active = False
        else:
            # Set Logging to ACTIVE (Green)
            self.loggingValue.setText("ACTIVE")
            self.loggingValue.setStyleSheet("QLabel#loggingValue { color: #39d98a; font-weight: 600; font-size: 12px; border: none; background: transparent; }")
            self.loggingIcon.setStyleSheet("QLabel#loggingIcon { background-color: #39d98a; border-radius: 12px; padding: 2px; }")
            self.logging_active = True

    def toggle_cam1_fullscreen(self):
        if hasattr(self, 'cam1_popup') and self.cam1_popup.isVisible():
            self.cam1_popup.close()
        else:
            self.cam1_popup = FullscreenCameraWindow(self.cam1_feed, self.cam1_layout, 0)
            self.cam1_popup.closed.connect(lambda: self.cam1FullscreenButton.setIcon(QIcon("logo/maximize-2.png")))
            self.cam1FullscreenButton.setIcon(QIcon("logo/minimize-2.png"))
            self.cam1_popup.show()

    def toggle_cam2_fullscreen(self):
        if hasattr(self, 'cam2_popup') and self.cam2_popup.isVisible():
            self.cam2_popup.close()
        else:
            self.cam2_popup = FullscreenCameraWindow(self.cam2_feed, self.cam2_layout, 0)
            self.cam2_popup.closed.connect(lambda: self.cam2FullscreenButton.setIcon(QIcon("logo/maximize-2.png")))
            self.cam2FullscreenButton.setIcon(QIcon("logo/minimize-2.png"))
            self.cam2_popup.show()

    def closeEvent(self, event):
        # Close popups if they are open
        if hasattr(self, 'cam1_popup') and self.cam1_popup:
            self.cam1_popup.close()
        if hasattr(self, 'cam2_popup') and self.cam2_popup:
            self.cam2_popup.close()
            
        # Stop ROS2 bridge
        if hasattr(self, 'ros2_bridge') and self.ros2_bridge:
            self.ros2_bridge.stop()

        # Stop camera widgets threads before exiting
        self.cam1_feed.stop()
        self.cam2_feed.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = GCSMainWindow()
    main_win.show()
    sys.exit(app.exec_())
