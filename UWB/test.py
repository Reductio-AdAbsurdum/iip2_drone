import socket

# Set up the UDP listener
UDP_IP = "0.0.0.0" # Listen on all networks
UDP_PORT = 14550

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for Drone UWB data on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024) # Buffer size is 1024 bytes
    print(f"Received: {data.decode('utf-8').strip()}")
