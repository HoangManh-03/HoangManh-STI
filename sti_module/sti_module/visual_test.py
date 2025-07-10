import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QComboBox, QCheckBox, QDoubleSpinBox,QHBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QSlider
from message_pkg.msg import LineControllerRequest  # Import message type

class LineControllerNode(Node):
    def __init__(self):
        super().__init__('line_controller_node')
        self.publisher_ = self.create_publisher(LineControllerRequest, 'line_controller_request', 10)

    def publish_request(self, enable, direction, velocity, safety, kp, ki, kd):
        msg = LineControllerRequest()
        msg.enable = enable
        msg.direction_move = direction
        msg.velocity = velocity
        msg.safety = safety
        msg.kp = kp
        msg.ki = ki
        msg.kd = kd
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: enable={enable}, safety={safety}, direction={direction}, velocity={velocity}, kp={kp}, ki={ki}, kd={kd}')

class LineControllerGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('Line Controller Parameter')
        self.setGeometry(100, 100, 400, 300)

        layout = QVBoxLayout()

        self.enable_layout = QHBoxLayout()

        # Enable checkbox
        self.enable_checkbox = QCheckBox('Enable')
        self.enable_checkbox.stateChanged.connect(self.update_params)
        # layout.addWidget(self.enable_checkbox)

        # Checkbox cạnh Enable
        self.enable_checkbox_safety = QCheckBox('Safety')
        self.enable_checkbox_safety.stateChanged.connect(self.update_params)

        # Checkbox cạnh Enable
        self.enable_checkbox_2 = QCheckBox('By Hand')
        self.enable_checkbox_2.stateChanged.connect(self.update_params)

        # Thêm vào layout ngang
        self.enable_layout.addWidget(self.enable_checkbox)
        self.enable_layout.addWidget(self.enable_checkbox_safety)
        self.enable_layout.addWidget(self.enable_checkbox_2)
        layout.addLayout(self.enable_layout)

        # Direction move
        self.direction_label = QLabel('Direction Move:')
        self.direction_combo = QComboBox()
        self.direction_combo.addItems(['1', '2'])
        self.direction_combo.currentIndexChanged.connect(self.update_params)
        layout.addWidget(self.direction_label)
        layout.addWidget(self.direction_combo)

        # Velocity
        # self.velocity_label = QLabel('Velocity:')
        # self.velocity_input = QDoubleSpinBox()
        # self.velocity_input.setRange(0.0, 10.0)
        # self.velocity_input.setSingleStep(0.05)
        # self.velocity_input.valueChanged.connect(self.update_params)
        # layout.addWidget(self.velocity_label)
        # layout.addWidget(self.velocity_input)

        # Velocity Label
        self.velocity_label = QLabel('Velocity: 0.2')
        layout.addWidget(self.velocity_label)

        # Velocity Slider
        self.velocity_slider = QSlider(Qt.Horizontal)
        self.velocity_slider.setMinimum(0)  # 0.00
        self.velocity_slider.setMaximum(12)  # 0.7 * 20 (vì bước 0.05 nên nhân 20)
        self.velocity_slider.setTickInterval(1)  # Mỗi bước 0.05
        self.velocity_slider.setSingleStep(1)
        self.velocity_slider.setValue(4)
        self.velocity_slider.valueChanged.connect(self.update_velocity)
        layout.addWidget(self.velocity_slider)

        # Lưu giá trị mặc định
        self.velocity_value = 0.2

        # Kp
        self.kp_label = QLabel('Kp:')
        self.kp_input = QDoubleSpinBox()
        self.kp_input.setRange(0.0, 10.0)
        # self.kp_input.setSingleStep(0.001)
        self.kp_input.setDecimals(5)
        self.kp_input.valueChanged.connect(self.update_params)
        layout.addWidget(self.kp_label)
        layout.addWidget(self.kp_input)

        # Ki
        self.ki_label = QLabel('Ki:')
        self.ki_input = QDoubleSpinBox()
        self.ki_input.setRange(0.0, 10.0)
        # self.ki_input.setSingleStep(0.001)
        self.ki_input.setDecimals(7)
        self.ki_input.valueChanged.connect(self.update_params)
        layout.addWidget(self.ki_label)
        layout.addWidget(self.ki_input)

        # Kd
        self.kd_label = QLabel('Kd:')
        self.kd_input = QDoubleSpinBox()
        self.kd_input.setRange(0.0, 10.0)
        # self.kd_input.setSingleStep(0.001)
        self.kd_input.setDecimals(7)
        self.kd_input.valueChanged.connect(self.update_params)
        layout.addWidget(self.kd_label)
        layout.addWidget(self.kd_input)

        self.setLayout(layout)

    def update_velocity(self):
        velocity = self.velocity_slider.value() / 20.0  # Chuyển về float 0.00 - 0.70
        self.velocity_label.setText(f'Velocity: {velocity:.2f}')  # Cập nhật hiển thị
        self.velocity_value = velocity  # Lưu giá trị
        self.update_params()

    def update_params(self):
        if self.enable_checkbox.isChecked():
            enable = 2
        else:
            if self.enable_checkbox_2.isChecked():
                enable = 1
            else:
                enable = 0
        # enable = 1 if self.enable_checkbox.isChecked() else 0

        safety = True if self.enable_checkbox_safety.isChecked() else False
        direction = int(self.direction_combo.currentText())
        velocity = self.velocity_value  # Lấy từ slider
        kp = self.kp_input.value()
        ki = self.ki_input.value()
        kd = self.kd_input.value()

        self.ros_node.publish_request(enable, direction, velocity, safety, kp, ki, kd)

def main():
    rclpy.init()
    node = LineControllerNode()

    app = QApplication(sys.argv)
    gui = LineControllerGUI(node)
    gui.show()

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
