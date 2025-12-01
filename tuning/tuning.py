import serial
import threading
import time
import keyboard  # pip install keyboard

# === USER CONFIGURATION ===
PORT = "COM3"        # Change to your Teensy's serial port
BAUDRATE = 115200
# ===========================

def read_from_teensy(ser):
    """Continuously read and print messages from the Teensy."""
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"[Teensy] {line}")
        time.sleep(0.05)

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print(f"Connected to Teensy on {PORT}")
        print("Press 'w' to increase, 's' to decrease, 'q' to quit.")
    except serial.SerialException:
        print(f"Error: could not open port {PORT}")
        return

    # Start serial listener thread
    threading.Thread(target=read_from_teensy, args=(ser,), daemon=True).start()

    try:
        while True:
            if keyboard.is_pressed('w'):
                ser.write(b'w')
                time.sleep(0.1)  # small delay to avoid flooding
            elif keyboard.is_pressed('s'):
                ser.write(b's')
                time.sleep(0.1)
            elif keyboard.is_pressed('q'):
                print("Exiting...")
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
