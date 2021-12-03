import serial

serialcom = serial.Serial('COM8', 115200)
print("Menu:\n"
      "press 0 to toggle Blue LED\n"
      "press 1 to toggle Orange LED\n"
      "press 2 to toggle Red LED\n"
      "press 3 to toggle Green LED\n")
while True:
    packet = serialcom.readline()
    print(packet.decode('utf'))

    i = input("Enter the number: ")

    if i == '1':
        serialcom.write(i.encode())
    elif i == '2':
        serialcom.write(i.encode())
    elif i == '3':
        serialcom.write(i.encode())
    elif i == '4':
        serialcom.write(i.encode())
    else:
        print("Unexpected command")