#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import socket

rospy.init_node('phone_call', anonymous=False)

def talker(fromDavid):
    pub = rospy.Publisher('/recognizer/output', String, queue_size=10)
    
    action = fromDavid
    #rospy.loginfo(action)
    pub.publish(action)

talker("")
# host = '192.168.5.40'
port = 7006

SERVER_IP = (
    [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [
        [(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in
         [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0])

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((SERVER_IP, port))

sock.listen(5)

print "server start with server ip: " + SERVER_IP + " with port: " + str(port)

while True:
    try:
        (clientSocket, address) = sock.accept()
        msg = clientSocket.recv(1024)
        if not msg:
            pass
        else:
            print "Client send: " + msg.rstrip()
            try:
                talker(msg.rstrip())
            except rospy.ROSInterruptException:
                pass

        clientSocket.close()
    except KeyboardInterrupt:
        print "keyboard interrupt"
        break


