import socket
import re
import os

# Set up the UDP listener
UDP_IP = "0.0.0.0"
UDP_PORT = 14550

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# A dictionary to draw giant numbers (ASCII Art)
DIGITS = {
    '0': ["  █████  ", " ██   ██ ", " ██   ██ ", " ██   ██ ", "  █████  "],
    '1': ["   ███   ", "  ████   ", "   ███   ", "   ███   ", "  █████  "],
    '2': [" ██████  ", "██    ██ ", "   ████  ", " ███     ", "████████ "],
    '3': [" ██████  ", "      ██ ", "  █████  ", "      ██ ", " ██████  "],
    '4': ["   ████  ", "  ██ ██  ", " ██  ██  ", "████████ ", "     ██  "],
    '5': ["████████ ", "██       ", "███████  ", "      ██ ", "███████  "],
    '6': ["  █████  ", " ██      ", " ██████  ", " ██   ██ ", "  █████  "],
    '7': ["████████ ", "      ██ ", "    ███  ", "   ███   ", "  ███    "],
    '8': ["  █████  ", " ██   ██ ", "  █████  ", " ██   ██ ", "  █████  "],
    '9': ["  █████  ", " ██   ██ ", "  ██████ ", "      ██ ", "  █████  "],
    '-': ["         ", "         ", " ███████ ", "         ", "         "],
    ' ': ["         ", "         ", "         ", "         ", "         "]
}

def get_big_text(text):
    # Combines the individual number blocks into one giant line
    lines = ["", "", "", "", ""]
    for char in text:
        if char in DIGITS:
            for i in range(5):
                lines[i] += DIGITS[char][i] + "  "
    return "\n".join(lines)

def clear_screen():
    # Clears the terminal cleanly
    os.system('clear' if os.name == 'posix' else 'cls')

clear_screen()
print(f"Waiting for Drone UWB signal on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    raw_string = data.decode('utf-8').strip()
    
    match = re.search(r'range:\(([-0-9]+)', raw_string)
    
    if match:
        distance = match.group(1)
        
        # Convert the standard number into the giant block text
        big_distance = get_big_text(distance)
        
        clear_screen()
        print("========================================================")
        print("                LIVE UWB DRONE TELEMETRY                ")
        print("========================================================\n")
        
        # Print the giant ASCII numbers centered
        print(big_distance)
        print("\n")
        
        print("\n========================================================")
        print(f"[Raw Packet] {raw_string}")
