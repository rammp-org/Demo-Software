import time
import os

DATA_DIR = "/home/herl/Desktop/Data"  # Ensure '~' is expanded correctly


class DataLogger:
    def __init__(self, serial_port):
        self.ser = serial_port
        self.logging = False
        os.makedirs(DATA_DIR, exist_ok=True)  # Ensure the data directory exists
        self.file_name = ""

    def start(self):
        # Open the log file for appending
        self.file_name = f'log-{time.strftime("%m_%d_%Y-%H_%M_%S")}.csv'
        file_path = os.path.join(DATA_DIR, self.file_name)
        self.log_file = open(file_path, "a")

        # Add header if the file is empty
        if os.stat(file_path).st_size == 0:
            self.log_file.write("Timestamp,Data\n")
        self.log_file.flush()

        # Reset the serial input buffer
        # self.ser.reset_input_buffer()
        self.logging = True

        # green text
        print(f"\033[92mStarted logging to {self.file_name}\033[0m")

    def log(self):
        if self.logging:
            try:
                # Read one complete line from the serial port
                line = self.ser.readline().decode("utf-8").strip()
                if line:  # Process non-empty lines
                    timestamp = time.strftime("%m_%d_%Y-%H:%M:%S")
                    log_entry = f"{timestamp},{line}\n"

                    # Write to the log file
                    self.log_file.write(log_entry)
                    self.log_file.flush()
                    # print(log_entry)  # Optional: Print to console
            except Exception as e:
                print(f"Error while logging data: {e}")
                print("Try seeing if Screen Command is using the port")
                print("run: sudo lsof /dev/ttyACM0")
                print("Kill PID: sudo kill <PID>")

    def stop(self):
        self.logging = False
        if hasattr(self, "log_file") and not self.log_file.closed:
            self.log_file.close()

        # red text
        print(f"\033[91mStopped logging to {self.file_name}\033[0m")
