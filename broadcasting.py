import socket
from time import sleep, time

import netifaces as ni
ni.ifaddresses('wlp2s0')
ip = ni.ifaddresses('wlp2s0')[ni.AF_INET][0]['addr']
print ip

BROADCAST_PORT = 58083

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 0))
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while True:
    print "Broadcasting..."
    data = 'Hello World: ' + ip
    s.sendto(data, ('<broadcast>', BROADCAST_PORT))
    sleep(1)
