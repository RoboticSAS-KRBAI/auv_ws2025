#!/usr/bin/env python3
"""
ROV Control GUI with 3D Visualization
Modern glassmorphism design with real-time 3D orientation display
"""

import sys
import math
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                             QTabWidget, QFrame, QGridLayout, QGraphicsDropShadowEffect)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor
from PyQt5.QtOpenGL import QGLWidget

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    OPENGL_AVAILABLE = True
except ImportError:
    OPENGL_AVAILABLE = False
    print("Warning: PyOpenGL not installed. 3D visualization will be disabled.")
    print("Install with: pip install PyOpenGL PyOpenGL_accelerate")


class GLWidget3D(QGLWidget):
    """Widget OpenGL untuk visualisasi 3D orientasi robot"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.zoom = -8.0
        
        # Mouse control
        self.last_pos = None
        self.rotation_x = 20.0
        self.rotation_y = 45.0
        
    def initializeGL(self):
        """Inisialisasi OpenGL"""
        glClearColor(0.12, 0.12, 0.12, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Setup lighting
        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        
    def resizeGL(self, w, h):
        """Resize viewport"""
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(w) / float(h) if h > 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
    def paintGL(self):
        """Render 3D scene"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Camera position
        glTranslatef(0.0, 0.0, self.zoom)
        glRotatef(self.rotation_x, 1.0, 0.0, 0.0)
        glRotatef(self.rotation_y, 0.0, 1.0, 0.0)
        
        # Apply robot orientation
        glRotatef(self.yaw, 0.0, 0.0, 1.0)      # Yaw (Z-axis)
        glRotatef(self.pitch, 1.0, 0.0, 0.0)    # Pitch (X-axis)
        glRotatef(self.roll, 0.0, 1.0, 0.0)     # Roll (Y-axis)
        
        # Draw coordinate axes
        self.draw_axes()
        
        # Draw ROV body
        self.draw_rov()
        
    def draw_axes(self):
        """Draw XYZ coordinate axes"""
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)
        
        glBegin(GL_LINES)
        # X-axis (Red)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(2.5, 0.0, 0.0)
        
        # Y-axis (Green)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 2.5, 0.0)
        
        # Z-axis (Blue)
        glColor3f(0.0, 0.5, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 2.5)
        glEnd()
        
        glEnable(GL_LIGHTING)
        
    def draw_rov(self):
        """Draw ROV body (simplified submarine shape)"""
        # Main body (cylinder)
        glPushMatrix()
        glColor3f(0.3, 0.4, 0.5)
        glRotatef(90, 0.0, 1.0, 0.0)
        quad = gluNewQuadric()
        gluCylinder(quad, 0.5, 0.5, 2.0, 32, 32)
        
        # Front cap
        glPushMatrix()
        glTranslatef(0.0, 0.0, 2.0)
        gluDisk(quad, 0.0, 0.5, 32, 32)
        glPopMatrix()
        
        # Back cap
        gluDisk(quad, 0.0, 0.5, 32, 32)
        glPopMatrix()
        
        # Top fin/sail
        glPushMatrix()
        glColor3f(0.4, 0.5, 0.6)
        glTranslatef(0.0, 0.5, 0.0)
        glScalef(0.3, 0.8, 0.8)
        self.draw_cube()
        glPopMatrix()
        
        # Front thrusters (left and right)
        glColor3f(0.8, 0.3, 0.3)
        # Left thruster
        glPushMatrix()
        glTranslatef(0.8, 0.0, 0.5)
        glRotatef(90, 0.0, 1.0, 0.0)
        gluCylinder(quad, 0.15, 0.15, 0.5, 16, 16)
        glPopMatrix()
        
        # Right thruster
        glPushMatrix()
        glTranslatef(-0.8, 0.0, 0.5)
        glRotatef(90, 0.0, 1.0, 0.0)
        gluCylinder(quad, 0.15, 0.15, 0.5, 16, 16)
        glPopMatrix()
        
    def draw_cube(self):
        """Draw a cube"""
        glBegin(GL_QUADS)
        # Front face
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        
        # Back face
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        
        # Top face
        glVertex3f(-0.5, 0.5, -0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, 0.5, -0.5)
        
        # Bottom face
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, -0.5, 0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        
        # Right face
        glVertex3f(0.5, -0.5, -0.5)
        glVertex3f(0.5, 0.5, -0.5)
        glVertex3f(0.5, 0.5, 0.5)
        glVertex3f(0.5, -0.5, 0.5)
        
        # Left face
        glVertex3f(-0.5, -0.5, -0.5)
        glVertex3f(-0.5, -0.5, 0.5)
        glVertex3f(-0.5, 0.5, 0.5)
        glVertex3f(-0.5, 0.5, -0.5)
        glEnd()
        
    def update_orientation(self, yaw, pitch, roll):
        """Update orientasi robot"""
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.updateGL()
        
    def mousePressEvent(self, event):
        """Handle mouse press"""
        self.last_pos = event.pos()
        
    def mouseMoveEvent(self, event):
        """Handle mouse drag untuk rotate camera"""
        if self.last_pos is not None:
            dx = event.x() - self.last_pos.x()
            dy = event.y() - self.last_pos.y()
            
            self.rotation_y += dx * 0.5
            self.rotation_x += dy * 0.5
            
            self.last_pos = event.pos()
            self.updateGL()
            
    def wheelEvent(self, event):
        """Handle mouse wheel untuk zoom"""
        delta = event.angleDelta().y()
        self.zoom += delta / 120.0
        self.zoom = max(-15.0, min(-3.0, self.zoom))
        self.updateGL()


class SensorCard(QFrame):
    """Widget sensor dengan glassmorphism design"""
    def __init__(self, label_text, icon="📊", unit="°", parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QFrame {
                background-color: rgba(255, 255, 255, 80);
                border: 1px solid rgba(255, 255, 255, 51);
                border-radius: 15px;
                padding: 15px;
            }
        """)
        
        # Shadow effect
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        shadow.setColor(QColor(31, 38, 135, 95))
        shadow.setOffset(0, 5)
        self.setGraphicsEffect(shadow)
        
        layout = QVBoxLayout()
        layout.setSpacing(5)
        layout.setAlignment(Qt.AlignCenter)
        
        # Icon
        self.icon_label = QLabel(icon)
        self.icon_label.setStyleSheet("""
            QLabel {
                font-size: 28px;
                background: transparent;
                border: none;
            }
        """)
        self.icon_label.setAlignment(Qt.AlignCenter)
        
        # Label nama sensor
        self.label = QLabel(label_text)
        self.label.setStyleSheet("""
            QLabel {
                color: rgba(255, 255, 255, 230);
                font-size: 11px;
                font-weight: bold;
                letter-spacing: 1.5px;
                background: transparent;
                border: none;
            }
        """)
        self.label.setAlignment(Qt.AlignCenter)
        
        # Display nilai
        self.value_label = QLabel("0.00")
        self.value_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 32px;
                font-weight: bold;
                background: transparent;
                border: none;
            }
        """)
        self.value_label.setAlignment(Qt.AlignCenter)
        
        # Unit label
        self.unit_label = QLabel(unit)
        self.unit_label.setStyleSheet("""
            QLabel {
                color: rgba(255, 255, 255, 180);
                font-size: 10px;
                font-weight: 500;
                background: transparent;
                border: none;
            }
        """)
        self.unit_label.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(self.icon_label)
        layout.addWidget(self.label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.unit_label)
        
        self.setLayout(layout)
        
    def update_value(self, value):
        """Update nilai sensor yang ditampilkan"""
        self.value_label.setText(f"{value:.2f}")


class ControlPanel(QFrame):
    """Panel kontrol dengan tab untuk setpoint"""
    activated = pyqtSignal(str, float)  # Signal: (control_name, setpoint)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QFrame {
                background-color: rgba(255, 255, 255, 80);
                border: 1px solid rgba(255, 255, 255, 51);
                border-radius: 15px;
                padding: 20px;
            }
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(15)
        
        # Tab widget
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: none;
                background: transparent;
            }
            QTabBar::tab {
                background-color: rgba(255, 255, 255, 80);
                color: rgba(255, 255, 255, 200);
                padding: 10px 20px;
                margin-right: 5px;
                border-top-left-radius: 10px;
                border-top-right-radius: 10px;
                font-size: 11px;
                font-weight: bold;
                letter-spacing: 1px;
                border: 1px solid rgba(255, 255, 255, 51);
            }
            QTabBar::tab:selected {
                background-color: rgba(255, 255, 255, 120);
                color: white;
                border: 1px solid rgba(255, 255, 255, 102);
            }
        """)
        
        # Create tabs for each control
        self.yaw_tab = self.create_control_tab("Yaw", "°")
        self.depth_tab = self.create_control_tab("Depth", "m")
        self.roll_tab = self.create_control_tab("Roll", "°")
        self.pitch_tab = self.create_control_tab("Pitch", "°")
        
        self.tab_widget.addTab(self.yaw_tab['widget'], "YAW")
        self.tab_widget.addTab(self.depth_tab['widget'], "DEPTH")
        self.tab_widget.addTab(self.roll_tab['widget'], "ROLL")
        self.tab_widget.addTab(self.pitch_tab['widget'], "PITCH")
        
        layout.addWidget(self.tab_widget)
        self.setLayout(layout)
        
    def create_control_tab(self, name, unit):
        """Create control tab layout"""
        widget = QWidget()
        widget.setStyleSheet("background: transparent;")
        layout = QVBoxLayout()
        layout.setSpacing(10)
        
        # Current value
        current_frame = QFrame()
        current_frame.setStyleSheet("""
            QFrame {
                background-color: rgba(255, 255, 255, 50);
                border: 1px solid rgba(255, 255, 255, 38);
                border-radius: 10px;
                padding: 10px;
            }
        """)
        current_layout = QVBoxLayout()
        
        current_label = QLabel("CURRENT")
        current_label.setStyleSheet("""
            QLabel {
                color: rgba(255, 255, 255, 180);
                font-size: 10px;
                font-weight: 600;
                letter-spacing: 1px;
                background: transparent;
                border: none;
            }
        """)
        current_label.setAlignment(Qt.AlignCenter)
        
        current_value = QLabel("0.00")
        current_value.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 24px;
                font-weight: bold;
                background: transparent;
                border: none;
            }
        """)
        current_value.setAlignment(Qt.AlignCenter)
        
        current_layout.addWidget(current_label)
        current_layout.addWidget(current_value)
        current_frame.setLayout(current_layout)
        
        # Setpoint input
        setpoint_input = QLineEdit()
        setpoint_input.setPlaceholderText(f"Enter {name.lower()} setpoint...")
        setpoint_input.setStyleSheet("""
            QLineEdit {
                background-color: rgba(255, 255, 255, 80);
                color: white;
                border: 1px solid rgba(255, 255, 255, 76);
                border-radius: 8px;
                padding: 10px;
                font-size: 14px;
                font-weight: 500;
            }
            QLineEdit:focus {
                border: 1px solid rgba(255, 255, 255, 153);
                background-color: rgba(255, 255, 255, 100);
            }
            QLineEdit::placeholder {
                color: rgba(255, 255, 255, 100);
            }
        """)
        
        # Activate button
        activate_btn = QPushButton(f"ACTIVATE {name.upper()}")
        activate_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(102, 126, 234, 255),
                    stop:1 rgba(118, 75, 162, 255));
                color: white;
                border: none;
                border-radius: 10px;
                padding: 12px;
                font-size: 12px;
                font-weight: bold;
                letter-spacing: 1px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(118, 142, 250, 255),
                    stop:1 rgba(134, 91, 178, 255));
            }
        """)
        activate_btn.setCursor(Qt.PointingHandCursor)
        activate_btn.clicked.connect(lambda: self.on_activate(name, setpoint_input))
        setpoint_input.returnPressed.connect(lambda: self.on_activate(name, setpoint_input))
        
        # Status label
        status_label = QLabel("⚪ INACTIVE")
        status_label.setStyleSheet("""
            QLabel {
                color: #ff6b9d;
                font-size: 11px;
                font-weight: bold;
                letter-spacing: 1px;
                background: transparent;
                border: none;
            }
        """)
        status_label.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(current_frame)
        layout.addWidget(setpoint_input)
        layout.addWidget(activate_btn)
        layout.addWidget(status_label)
        
        widget.setLayout(layout)
        
        return {
            'widget': widget,
            'current': current_value,
            'input': setpoint_input,
            'status': status_label,
            'button': activate_btn
        }
    
    def on_activate(self, control_name, input_widget):
        """Handle activation"""
        try:
            setpoint = float(input_widget.text())
            self.activated.emit(control_name, setpoint)
            
            # Update status based on control name
            if control_name == "Yaw":
                tab = self.yaw_tab
            elif control_name == "Depth":
                tab = self.depth_tab
            elif control_name == "Roll":
                tab = self.roll_tab
            else:
                tab = self.pitch_tab
                
            tab['status'].setText(f"✅ ACTIVE: {setpoint:.2f}")
            tab['status'].setStyleSheet("""
                QLabel {
                    color: #4ecca3;
                    font-size: 11px;
                    font-weight: bold;
                    letter-spacing: 1px;
                    background: transparent;
                    border: none;
                }
            """)
        except ValueError:
            pass
    
    def update_current_value(self, control_name, value):
        """Update current value display"""
        if control_name == "Yaw":
            self.yaw_tab['current'].setText(f"{value:.2f}")
        elif control_name == "Depth":
            self.depth_tab['current'].setText(f"{value:.2f}")
        elif control_name == "Roll":
            self.roll_tab['current'].setText(f"{value:.2f}")
        elif control_name == "Pitch":
            self.pitch_tab['current'].setText(f"{value:.2f}")


class ROVControlGUI(QMainWindow):
    """Main window untuk ROV Control GUI dengan 3D visualization"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV Control System - 3D Visualization")
        self.setGeometry(50, 50, 1600, 900)
        
        # Set gradient background
        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(102, 126, 234, 255),
                    stop:0.5 rgba(118, 75, 162, 255),
                    stop:1 rgba(240, 147, 251, 255));
            }
        """)
        
        # Central widget
        central_widget = QWidget()
        central_widget.setStyleSheet("background: transparent;")
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Header
        header = QFrame()
        header.setStyleSheet("""
            QFrame {
                background-color: rgba(255, 255, 255, 80);
                border: 1px solid rgba(255, 255, 255, 51);
                border-radius: 15px;
                padding: 15px;
            }
        """)
        header_layout = QHBoxLayout()
        header_label = QLabel("🤖 ROV Control System - 3D Visualization")
        header_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 24px;
                font-weight: bold;
                letter-spacing: 1px;
                background: transparent;
                border: none;
            }
        """)
        header_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(header_label)
        header.setLayout(header_layout)
        main_layout.addWidget(header)
        
        # Main content layout (horizontal split)
        content_layout = QHBoxLayout()
        content_layout.setSpacing(20)
        
        # Left side - 3D View and sensor data
        left_layout = QVBoxLayout()
        left_layout.setSpacing(15)
        
        # 3D Visualization
        if OPENGL_AVAILABLE:
            self.gl_widget = GLWidget3D()
            self.gl_widget.setMinimumSize(700, 400)
            self.gl_widget.setStyleSheet("""
                QWidget {
                    background-color: rgba(30, 30, 30, 200);
                    border: 2px solid rgba(255, 255, 255, 51);
                    border-radius: 15px;
                }
            """)
            left_layout.addWidget(self.gl_widget)
        else:
            # Fallback jika OpenGL tidak tersedia
            no_gl_label = QLabel("⚠️ OpenGL not available\nInstall PyOpenGL to enable 3D view")
            no_gl_label.setMinimumSize(700, 400)
            no_gl_label.setAlignment(Qt.AlignCenter)
            no_gl_label.setStyleSheet("""
                QLabel {
                    background-color: rgba(30, 30, 30, 200);
                    border: 2px solid rgba(255, 255, 255, 51);
                    border-radius: 15px;
                    color: white;
                    font-size: 16px;
                }
            """)
            left_layout.addWidget(no_gl_label)
            self.gl_widget = None
        
        # Sensor cards grid
        sensor_grid = QGridLayout()
        sensor_grid.setSpacing(15)
        
        self.yaw_card = SensorCard("YAW", "🧭", "degrees")
        self.pitch_card = SensorCard("PITCH", "⬆️", "degrees")
        self.roll_card = SensorCard("ROLL", "🔄", "degrees")
        self.depth_card = SensorCard("DEPTH", "🌊", "meters")
        
        sensor_grid.addWidget(self.yaw_card, 0, 0)
        sensor_grid.addWidget(self.pitch_card, 0, 1)
        sensor_grid.addWidget(self.roll_card, 1, 0)
        sensor_grid.addWidget(self.depth_card, 1, 1)
        
        left_layout.addLayout(sensor_grid)
        
        # Right side - Control panel
        self.control_panel = ControlPanel()
        self.control_panel.setMinimumWidth(400)
        self.control_panel.activated.connect(self.on_control_activated)
        
        content_layout.addLayout(left_layout, 2)
        content_layout.addWidget(self.control_panel, 1)
        
        main_layout.addLayout(content_layout)
        central_widget.setLayout(main_layout)
        
        # Timer untuk simulasi data (ganti dengan ROS2 subscriber)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sensor_data)
        self.timer.start(50)  # Update setiap 50ms
        
        # Simulasi data counter
        self.counter = 0
        
    def update_sensor_data(self):
        """Update data sensor (simulasi - ganti dengan ROS2 subscriber)"""
        self.counter += 0.05
        
        # Simulasi data sensor
        yaw = 0 #45 + 30 * math.sin(self.counter)
        pitch = 0 #15 * math.cos(self.counter * 1.5)
        roll = 0 #10 * math.sin(self.counter * 0.8)
        depth = 0 #5 + 2 * math.cos(self.counter * 0.5)
        
        # Update sensor cards
        self.yaw_card.update_value(yaw)
        self.pitch_card.update_value(pitch)
        self.roll_card.update_value(roll)
        self.depth_card.update_value(depth)
        
        # Update 3D visualization
        if self.gl_widget:
            self.gl_widget.update_orientation(yaw, pitch, roll)
        
        # Update control panel current values
        self.control_panel.update_current_value("Yaw", yaw)
        self.control_panel.update_current_value("Pitch", pitch)
        self.control_panel.update_current_value("Roll", roll)
        self.control_panel.update_current_value("Depth", depth)
    
    def on_control_activated(self, control_name, setpoint):
        """Handle control activation"""
        print(f"{control_name} activated with setpoint: {setpoint}")
        # TODO: Publish ke ROS2 topic


def main():
    app = QApplication(sys.argv)
    
    # Set application font
    font = QFont("Segoe UI", 10)
    app.setFont(font)
    
    window = ROVControlGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()