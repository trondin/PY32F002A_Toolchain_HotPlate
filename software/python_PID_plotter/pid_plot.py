import sys
import os
import serial
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import threading
import queue

# Set Qt backend to avoid GTK conflicts
os.environ["QT_QPA_PLATFORMTHEME"] = "qt5ct"
os.environ["NO_AT_BRIDGE"] = "1" 

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
NUM_POINTS = 2000
TEMP_MIN = 20
TEMP_MAX = 100
POWER_MIN = 0
POWER_MAX = 21

class SerialReader(QtCore.QObject):
    data_received = QtCore.pyqtSignal(float, float)  # Signal for temperature and power
    error_occurred = QtCore.pyqtSignal(str)         # Signal for errors

    def __init__(self, port, baud_rate):
        super().__init__()
        self.ser = None
        self.port = port
        self.baud_rate = baud_rate
        self.running = False
        self.queue = queue.Queue()

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.running = True
            self.thread = threading.Thread(target=self.read_serial, daemon=True)
            self.thread.start()
        except serial.SerialException as e:
            self.error_occurred.emit(f"Error opening port: {e}")

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()
            self.ser = None

    def read_serial(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        try:
                            temp, power = map(float, line.split(','))
                            self.data_received.emit(temp, power)
                        except ValueError:
                            self.error_occurred.emit(f"Invalid data format: {line}")
            except serial.SerialException:
                self.error_occurred.emit("Serial read error")
                self.running = False
                break

class PIDPlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Controller: Temperature and Power")
        self.resize(900, 600)

        self.serial_reader = None
        self.sample_index = 0
        self.temps = np.full(NUM_POINTS, np.nan)
        self.powers = np.full(NUM_POINTS, np.nan)

        self.init_ui()
        self.setup_serial()

    def init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        # Plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('left', 'Temperature (°C)', color='blue')
        self.plot_widget.setLabel('bottom', 'Sample')
        self.temp_curve = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name='Temperature')

        # Create second y-axis on the right (power)
        self.plot_widget.setLabel('right', 'Power', color='red')
        self.plot_widget.showAxis('right')
        self.plot_widget.getAxis('right').setPen(pg.mkPen('r'))

        # Link secondary viewbox
        self.power_viewbox = pg.ViewBox()
        self.plot_widget.scene().addItem(self.power_viewbox)
        self.plot_widget.getAxis('right').linkToView(self.power_viewbox)
        self.power_viewbox.setXLink(self.plot_widget)
        self.power_plot = pg.PlotCurveItem(pen=pg.mkPen('r', width=2), name='Power')
        self.power_viewbox.addItem(self.power_plot)

        # Sync resizing
        self.plot_widget.getViewBox().sigResized.connect(self.update_views)

        layout.addWidget(self.plot_widget)

        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("Start")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.exit_button = QtWidgets.QPushButton("Exit")
        self.stop_button.setEnabled(False)

        self.start_button.clicked.connect(self.start_serial)
        self.stop_button.clicked.connect(self.stop_serial)
        self.exit_button.clicked.connect(self.exit_app)

        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.exit_button)

        layout.addLayout(button_layout)

        # Axis limits
        self.plot_widget.setYRange(TEMP_MIN, TEMP_MAX)
        self.power_viewbox.setYRange(POWER_MIN, POWER_MAX)

    def update_views(self):
        self.power_viewbox.setGeometry(self.plot_widget.getViewBox().sceneBoundingRect())
        self.power_viewbox.linkedViewChanged(self.plot_widget.getViewBox(), self.power_viewbox.XAxis)

    def setup_serial(self):
        self.serial_reader = SerialReader(SERIAL_PORT, BAUD_RATE)
        self.serial_reader.data_received.connect(self.update_plot)
        self.serial_reader.error_occurred.connect(self.handle_error)

    def start_serial(self):
        self.serial_reader.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        print("Serial port opened")

    def stop_serial(self):
        if self.serial_reader:
            self.serial_reader.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        print("Serial port closed")

    def exit_app(self):
        self.stop_serial()
        QtWidgets.QApplication.quit()

    def update_plot(self, temp, power):
        power = int(power)
        self.temps[:-1] = self.temps[1:]
        self.temps[-1] = temp
        self.powers[:-1] = self.powers[1:]
        self.powers[-1] = power
        self.sample_index += 1

        xvals = np.arange(self.sample_index - NUM_POINTS + 1, self.sample_index + 1)
        self.temp_curve.setData(xvals, self.temps)
        self.power_plot.setData(xvals, self.powers)

        # Rescale temp axis if needed
        if np.nanmax(self.temps) > TEMP_MAX or np.nanmin(self.temps) < TEMP_MIN:
            self.plot_widget.setYRange(min(TEMP_MIN, np.nanmin(self.temps)),
                                       max(TEMP_MAX, np.nanmax(self.temps)))

    def handle_error(self, error_msg):
        print(error_msg)
        if "Serial read error" in error_msg:
            self.stop_serial()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = PIDPlotter()
    window.show()
    sys.exit(app.exec_())
