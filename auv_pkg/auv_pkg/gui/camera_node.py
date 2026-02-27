import cv2
from PyQt5 import QtCore, QtGui

class CameraThread(QtCore.QThread):
    frame_ready = QtCore.pyqtSignal(QtGui.QImage)

    def run(self):
        cap = cv2.VideoCapture(0)  # ganti index / rtsp jika perlu

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w

            qimg = QtGui.QImage(
                frame.data,
                w, h,
                bytes_per_line,
                QtGui.QImage.Format_RGB888
            )

            self.frame_ready.emit(qimg)

            self.msleep(30)  # ~30 FPS

        cap.release()
