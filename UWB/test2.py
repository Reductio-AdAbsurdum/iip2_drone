import socket
import re
import os

# Set up the UDP listener
UDP_IP = "0.0.0.0"
UDP_PORT = 14550

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def clear_screen():
    # Clears the terminal (works on Mac/Linux 'clear' and Windows 'cls')
    os.system('clear' if os.name == 'posix' else 'cls')

clear_screen()
print(f"🚀 Waiting for Drone UWB signal on port {UDP_PORT}...")

while True:
    # 1. Catch the data over Wi-Fi
    data, addr = sock.recvfrom(1024)
    raw_string = data.decode('utf-8').strip()
    
    # 2. Use Regex to extract the first number inside range:(...)
    # This looks for "range:(", then captures any digits or minus signs until the first comma
    match = re.search(r'range:\(([-0-9]+)', raw_string)
    
    if match:
        # Extract the matched number
        distance = match.group(1)
        
        # 3. Redraw the UI
        clear_screen()
        print("==========================================")
        print("      🛰️ LIVE UWB DRONE TELEMETRY 🛰️      ")
        print("==========================================")
        print("\n")
        
        # Make the number stand out
        print(f"          Distance:  {distance} ")
        
        print("\n")
        print("==========================================")
        
        # IT people love seeing the raw data, so we leave it tiny at the bottom
        print(f"[Debug Packet] {raw_string}")
