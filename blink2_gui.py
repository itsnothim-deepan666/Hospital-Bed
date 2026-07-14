import sys
import socket
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt

ESP_IP = "192.168.137.171" 
ESP_PORT = 12346

class UDP:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp_address = (ip, port)

    def send_command(self, command):
        self.sock.sendto(command.encode(), self.esp_address)
        print(f"Command sent: {command}")

udpclient = UDP(ESP_IP, ESP_PORT)

class AnglePage(QWidget):
    def __init__(self, option, back_callback):
        super().__init__()
        self.setWindowTitle(f"Select Angle for {option}")
        layout = QVBoxLayout()
        # Back button row
        back_row = QHBoxLayout()
        back_btn = QPushButton("Back")
        back_btn.clicked.connect(back_callback)
        back_row.addWidget(back_btn)
        back_row.addStretch()
        layout.addLayout(back_row)
        # Heading
        heading = QLabel(option)
        heading.setAlignment(Qt.AlignCenter)
        layout.addWidget(heading)
        # Angle buttons
        angles = ["0", "30", "45", "60"]
        btn_row = QHBoxLayout()
        for ang in angles:
            btn = QPushButton(ang)
            btn.setFixedSize(80, 40)
            btn.clicked.connect(lambda checked, a=ang: self.angle_selected(option,a))
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)
        self.setLayout(layout)
    def angle_selected(self, option, angle):
        udpclient.send_command(option + " " + angle)

class MainPage(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Select Option")
        layout = QVBoxLayout()
        heading = QLabel("Choose Position")
        heading.setAlignment(Qt.AlignCenter)
        layout.addWidget(heading)
        options = ["TOP", "RIGHT", "BOTTOM", "LEFT"]
        btn_row = QHBoxLayout()
        for opt in options:
            btn = QPushButton(opt)
            btn.setFixedSize(100, 50)
            btn.clicked.connect(lambda checked, o=opt: self.open_angle_page(o))
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)
        self.setLayout(layout)
        self.angle_page = None
    def open_angle_page(self, option):
        self.angle_page = AnglePage(option, self.show_main)
        self.angle_page.show()
        self.hide()
    def show_main(self):
        self.show()
        if self.angle_page:
            self.angle_page.close()

def main():
    app = QApplication(sys.argv)
    main_win = MainPage()
    main_win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
