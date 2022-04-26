import socket

UDP_IP = "192.168.1.1"
# UDP_IP = "169.254.95.255"
UDP_PORT = 7982
MESSAGE = b"Hello, World!"
 
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

