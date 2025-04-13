import serial
import time

class SerialMonitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, min_interval=0.5):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.latest_data = "Not connected"
        self.last_check_time = 0  # Timestamp of the last check
        self.min_interval = min_interval  # Minimum interval between checks

    def open_serial_connection(self):
        """Open the serial connection."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            if self.ser.is_open:
                print(f"Connected to {self.port}")
            else:
                self.ser = None
        except serial.SerialException as e:
            self.ser = None

    def read_from_serial(self):
        """Read data from the serial connection."""
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8').strip()
                    self.latest_data = data
                return True
            except serial.SerialException as e:
                return False
            except OSError as e:
                return False
        return False

    def reconnect_if_needed(self):
        """Reconnect if connection is lost."""
        if not self.ser or not self.ser.is_open:
            self.open_serial_connection()

    def get_latest_data(self):
        """Return the latest data or Not connected."""
        return self.latest_data

    def close_connection(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Connection to {self.port} closed.")

    def monitor_non_blocking(self):
        """Check data with rate limiting."""
        current_time = time.time()
        # If the minimum interval has not passed since the last check, return early
        if current_time - self.last_check_time < self.min_interval:
            return
        
        self.last_check_time = current_time  # Update the last check time
        if self.ser and self.ser.is_open:
            if not self.read_from_serial():
                self.latest_data = "Not connected"
                self.close_connection()
                self.open_serial_connection()
        else:
            self.reconnect_if_needed()


# Example 
if __name__ == "__main__":
    serial_monitor = SerialMonitor('/dev/ttyACM0', 115200, min_interval=0.5)
    serial_monitor.open_serial_connection()
    while True:
        serial_monitor.monitor_non_blocking()
        print(serial_monitor.get_latest_data())
        time.sleep(0.1) 