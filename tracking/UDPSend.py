# import socket
from pythonosc import udp_client

UDP_IP = "192.168.1.1"
# UDP_IP = "143.215.63.250"
UDP_PORT = 7982
MESSAGE = "Hello, World!"

client = udp_client.SimpleUDPClient(UDP_IP, UDP_PORT)
client.send_message("/x", MESSAGE)

# print("UDP target IP: %s" % UDP_IP)
# print("UDP target port: %s" % UDP_PORT)
# print("message: %s" % MESSAGE)

# sock = socket.socket(socket.AF_INET, # Internet
#                      socket.SOCK_DGRAM) # UDP
# sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

