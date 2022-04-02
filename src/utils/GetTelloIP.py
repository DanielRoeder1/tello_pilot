from getmac import get_mac_address
import time
import rospy


def get_tello_ip(tello_mac):
    search_ip_retries = 3
    for i in range(search_ip_retries):
        start = 100
        for i in range(12):
            ip_mac = get_mac_address(ip=f"192.168.0.{start}")
            if ip_mac == tello_mac:
                rospy.loginfo(f"Found Tello IP: 192.168.0.{start}")
                return f"192.168.0.{start}"
            start += 1
        time.sleep(3)
        rospy.loginfo("Couldn't find Tello in local network. Trying again...")