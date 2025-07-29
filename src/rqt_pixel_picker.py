#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout
from python_qt_binding.QtCore import Qt
from rqt_gui_py.plugin import Plugin

class PixelPickerWidget(QWidget):
    def __init__(self):
        super(PixelPickerWidget, self).__init__()
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.mousePressEvent = self.mousePressEvent
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        self.setLayout(layout)
        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        image_type = rospy.get_param('~image_type', 'sensor_msgs/Image')
        if image_type == 'sensor_msgs/CompressedImage':
            from sensor_msgs.msg import CompressedImage
            self.sub = rospy.Subscriber(image_topic, CompressedImage, self.compressed_image_callback)
        else:
            self.sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.pub = rospy.Publisher('/selected_pixel', Point, queue_size=1)
        self.cv_img = None

    def image_callback(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channel = self.cv_img.shape
        bytes_per_line = 3 * width
        qt_img = QImage(self.cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.image_label.setPixmap(QPixmap.fromImage(qt_img))

    def compressed_image_callback(self, msg):
        import numpy as np
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if self.cv_img is not None:
            height, width, channel = self.cv_img.shape
            bytes_per_line = 3 * width
            qt_img = QImage(self.cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.image_label.setPixmap(QPixmap.fromImage(qt_img))

    def mousePressEvent(self, event):
        if self.cv_img is None:
            return
        x = event.pos().x()
        y = event.pos().y()
        # 映射到原始图像坐标
        label_size = self.image_label.size()
        img_h, img_w = self.cv_img.shape[:2]
        scale_w = float(img_w) / label_size.width()
        scale_h = float(img_h) / label_size.height()
        px = int(x * scale_w)
        py = int(y * scale_h)
        pt_msg = Point()
        pt_msg.x = px
        pt_msg.y = py
        pt_msg.z = 0
        self.pub.publish(pt_msg)

class PixelPickerPlugin(Plugin):
    def __init__(self, context):
        super(PixelPickerPlugin, self).__init__(context)
        self.setObjectName('PixelPickerPlugin')
        self._widget = PixelPickerWidget()
        context.add_widget(self._widget)

# 入口
if __name__ == '__main__':
    import sys
    import rospy
    from python_qt_binding.QtWidgets import QApplication
    rospy.init_node('rqt_pixel_picker')
    app = QApplication(sys.argv)
    widget = PixelPickerWidget()
    widget.show()
    sys.exit(app.exec_())
