#client-side
import socket
import serial
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("169.254.59.250",1234))
ser = serial.Serial("/dev/ttyS0", 115200)
while True:
    msg = s.recv(1024)
    print(msg)
    ser.write(msg)






