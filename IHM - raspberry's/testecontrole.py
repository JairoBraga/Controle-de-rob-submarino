#import evdev
from evdev import InputDevice, categorize, ecodes
import socket
#creates object 'gamepad' to store the data
#you can call it whatever you like
gamepad = InputDevice('/dev/input/event3')

#button code variables (change to suit your device)
aBtn = 304
bBtn = 305
xBtn = 307
yBtn = 308

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("169.254.59.250", 1234))
s.listen(5)

clientSocket, address = s.accept()
#clientSocket.send(bytes("connection from has been established", "utf-8"))
print('connection from has been established')
print(address)


#prints out device info at start
print(gamepad)

#loop and filter by event code and print the mapped label

for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        if event.value == 1:
            if event.code == yBtn:
                print("Y")
                clientSocket.send(bytes('s'))
            elif event.code == bBtn:
                print("B")
            elif event.code == aBtn:
                print("A")
                clientSocket.send(bytes('i'))
            elif event.code == xBtn:
                print("X")
                

    if event.type == ecodes.EV_ABS:
        if event.code == 1:
                if event.value == 0:
                    print("up")
                    clientSocket.send(bytes('f'))
                elif event.value == 255:
                    print('analogico down')
                    clientSocket.send(bytes('r'))
                elif event.value == 128:
                    print("Estado zero")
                    clientSocket.send(bytes('o'))
        
        if event.code == 0:
            if event.value == 0:
                print('analogico esquerdo')
                clientSocket.send(bytes('e'))
            elif event.value == 255:
                print("analogico direito")
                clientSocket.send(bytes('d'))
            
            elif event.value == 128:
                    print("Estado zero")
                    clientSocket.send(bytes('o'))
        if event.code == 5:
                if event.value == 0:
                    print("up")
                elif event.value == 255:
                    print('analogico down')
        if event.code == 2:
            if event.value == 0:
                print('analogico esquerdo')
            elif event.value == 255:
                print('analogico direito')
                                