import serial
import sys
import select

ser = serial.Serial("/dev/cu.usbmodem1101", 9600, timeout=1)
f = open("data.csv", "w")
f.write("timestamp_ms,roll,pitch,label\n")

while True:
    
    # Reads s to start collecting
    if select.select([sys.stdin], [], [], 0)[0]:
        ser.write(sys.stdin.read(1).encode())

    line = ser.readline().decode().strip()
    if line:
        print(line)
        if line[0].isdigit():
            f.write(line + "\n")
