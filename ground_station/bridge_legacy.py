import serial
import threading
import time

# Βάλε εδώ τις πραγματικές θύρες COM των δύο boards
PICO_PORT = "COM8"    # Η θύρα του Pico 2 W
ESP32_PORT = "COM5"   # Η θύρα του FireBeetle 2 ESP32-C5 (τσέκαρέ την από το Arduino IDE)
BAUD_RATE = 115200

try:
    pico = serial.Serial(PICO_PORT, BAUD_RATE, timeout=0.1)
    esp32 = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=0.1)
    print(f"[*] Bridge connected: Pico ({PICO_PORT}) <---> ESP32 ({ESP32_PORT})")
except Exception as e:
    print(f"[!] Error opening serial ports: {e}")
    exit(1)

# Forwarding: Από Pico -> σε ESP32
def pico_to_esp():
    while True:
        try:
            data = pico.readline()
            if data:
                esp32.write(data)
        except Exception:
            break

# Forwarding: Από ESP32 -> σε Pico
def esp_to_pico():
    while True:
        try:
            data = esp32.readline()
            if data:
                pico.write(data)
        except Exception:
            break

# Ξεκινάμε τα δύο background threads
t1 = threading.Thread(target=pico_to_esp, daemon=True)
t2 = threading.Thread(target=esp_to_pico, daemon=True)

t1.start()
t2.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping bridge...")
    pico.close()
    esp32.close()