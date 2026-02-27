from PyQt5.QtWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
import pywavefront
import numpy as np



class ROV3DWidget(QOpenGLWidget):
    """QOpenGLWidget dengan kontrol kamera (rotate + zoom) dan OBJ loader"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Robot orientation (from sensor)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # Camera control
        self.rotation_x = 20.0
        self.rotation_y = 130.0
        self.zoom = -8.0

        self.last_pos = None

        # Model
        self.model = None
        self.scale = 1.0


    # ---------------------------------------------------------pitch
    # OpenGL INIT
    # ---------------------------------------------------------
    
    def initializeGL(self):
        glClearColor(0.01, 0.01, 0.01, 1.0)
        glEnable(GL_DEPTH_TEST)

        # Lighting
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_COLOR_MATERIAL)

        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])

        # Load OBJ
        model_path = "/home/reynard/auv_ws/src/robot_controller/robot_controller/gui/podlodka3.obj"
        self.model = pywavefront.Wavefront(model_path, create_materials=True, collect_faces=True)


        # Autoscale
        vertices = np.array(self.model.vertices)
        max_range = np.max(np.linalg.norm(vertices, axis=1))
        if max_range > 0:
            self.scale = 13.0 / max_range
        
        center = np.mean(vertices, axis=0)
        print(f"Model center: {center}")

        # Jika center tidak di (0,0,0), center-kan model:
        if np.linalg.norm(center) > 0.1:
            self.model.vertices = [
                [v[0]-center[0], v[1]-center[1], v[2]-center[2]] 
                for v in self.model.vertices
            ]


    # ---------------------------------------------------------
    # VIEWPORT RESIZE
    # ---------------------------------------------------------
    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h if h > 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)


    # ---------------------------------------------------------
    # MAIN DRAW
    # ---------------------------------------------------------
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Camera transform
        glTranslatef(0, 0, self.zoom)
        glRotatef(self.rotation_x, 1, 0, 0)
        glRotatef(self.rotation_y, 0, 1, 0)

        # Apply robot orientation from sensors
        glRotatef(self.yaw,   0, 1, 0)
        glRotatef(self.roll, 8, 0, 0)
        glRotatef(self.pitch,  0, 0, 8)

        # Axes
        # self.draw_axes()

        # Draw OBJ
        glScalef(self.scale, self.scale, self.scale)
        self.draw_obj()

        # self.update()


    # ---------------------------------------------------------
    # DRAW OBJ
    # ---------------------------------------------------------
    def draw_obj(self):
        if self.model is None:
            return

        glColor3f(0.6, 0.6, 0.7)

        for mesh in self.model.mesh_list:
            glBegin(GL_TRIANGLES)
            for face in mesh.faces:
                for vertex_index in face:
                    glVertex3fv(self.model.vertices[vertex_index])
            glEnd()


    # ---------------------------------------------------------
    # DRAW AXES
    # ---------------------------------------------------------
    def draw_axes(self):
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)

        glBegin(GL_LINES)

        # X - Merah
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(2, 0, 0)

        # Y - Hijau
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 2, 0)

        # Z - Biru
        glColor3f(0, 0.5, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 2)

        glEnd()

        glEnable(GL_LIGHTING)


    # ---------------------------------------------------------
    # UPDATE ORIENTATION FROM SENSOR
    # ---------------------------------------------------------

    def update_orientation(self, yaw, pitch, roll):
        """Update orientasi robot"""
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.update()
        
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
            self.update()
            
    def wheelEvent(self, event):
        """Handle mouse wheel untuk zoom"""
        delta = event.angleDelta().y()
        self.zoom += delta / 120.0
        self.zoom = max(-15.0, min(-3.0, self.zoom))
        self.update()