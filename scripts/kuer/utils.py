# https://stackoverflow.com/questions/166506/finding-local-ip-addresses-using-pythons-stdlib
import socket
def find_ip_addr():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()

    if ip != "127.0.0.1":
        return ip
    return None