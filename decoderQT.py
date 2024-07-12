import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QLineEdit, QTextEdit, QFileDialog, QMessageBox, QScrollBar
import subprocess

class LogDecoder(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Log Decoder")
        self.setGeometry(100, 100, 600, 400)
        self.setStyleSheet("background-color: black; color: white;")

        self.button_select_file = QPushButton("Select File", self)
        self.button_select_file.setGeometry(50, 50, 100, 30)
        self.button_select_file.clicked.connect(self.select_file)

        self.entry_input_file = QLineEdit(self)
        self.entry_input_file.setGeometry(200, 50, 300, 30)

        self.label_threshold = QLabel("Threshold Value:", self)
        self.label_threshold.setGeometry(50, 100, 150, 30)

        self.entry_threshold = QLineEdit(self)
        self.entry_threshold.setGeometry(200, 100, 300, 30)

        self.label_output_file = QLabel("Output File Name:", self)
        self.label_output_file.setGeometry(50, 150, 150, 30)

        self.entry_output_file = QLineEdit(self)
        self.entry_output_file.setGeometry(200, 150, 300, 30)

        self.button_run_program = QPushButton("Run Program", self)
        self.button_run_program.setGeometry(200, 200, 150, 30)
        self.button_run_program.clicked.connect(self.run_cpp_program)

        self.title_label = QLabel("Decoder Output", self)
        self.title_label.setGeometry(50, 250, 150, 30)
        self.title_label.setStyleSheet("font: bold; font-size: 14px;")

        self.text_widget = QTextEdit(self)
        self.text_widget.setGeometry(50, 290, 500, 100)
        self.text_widget.setStyleSheet("background-color: black; color: white;")
        self.text_widget.setReadOnly(True)

        self.scrollbar = QScrollBar(self)
        self.scrollbar.setGeometry(550, 290, 20, 100)
        self.scrollbar.setStyleSheet("background-color: black;")

    def select_file(self):
        file, _ = QFileDialog.getOpenFileName(self, "Select File", "", "Text files (*.txt)")
        if file:
            self.entry_input_file.setText(file)

    def run_cpp_program(self):
        input_file_path = self.entry_input_file.text()
        threshold_value = self.entry_threshold.text()
        output_file_name = self.entry_output_file.text()

        command = ["./Parser.exe", input_file_path, threshold_value, output_file_name + ".csv"]
        print(command)
        try:
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            while True:
                output_line = process.stdout.readline()
                if not output_line:
                    break
                self.text_widget.append(output_line)
                self.text_widget.ensureCursorVisible()
            process.wait()
            self.text_widget.append("CSV created.\n")
        except Exception as e:
            self.text_widget.append(f"Error running C++ program: {str(e)}\n")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = LogDecoder()
    window.show()
    sys.exit(app.exec_())
