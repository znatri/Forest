import struct
import socket

MAX_UDP_IP = "127.0.0.1"
MAX_UDP_PORT = 7983

if __name__ == "__main__":
    s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s.bind((MAX_UDP_IP, MAX_UDP_PORT))
    s.settimeout(None)

    try:
        while True:
            data, server = s.recvfrom(1024)
            coord = bytes.decode(data)
            print(coord[0], coord[1])
    except KeyboardInterrupt or socket.error:
        s.close()
    finally:
        s.close()
        print("Socket closed")
