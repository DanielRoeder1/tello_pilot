from getmac import get_mac_address


def get_tello_ip(tello_mac):
    start = 100
    for i in range(12):
        ip_mac = get_mac_address(ip=f"192.168.0.{start}")
        if ip_mac == tello_mac:
            print(f"Found Tello IP: 192.168.0.{start}")
            return f"192.168.0.{start}"
        start += 1