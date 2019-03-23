import serial

# If this is false, read from HC-O6 and write to HB-O2
# If this is true,  read from HB-O2 and write to HC-O6
INVERT = False

while True:

    HBO2 = None
    HCO6 = None

    while True:
        try:
            HBO2 = serial.Serial('/dev/tty.HBO2-DevB', 115200, timeout=1)
            print("Connected to HB-O2")
            break
        except serial.serialutil.SerialException:
            print("Failed to connect to HB-O2, retrying")
            continue

    while True:
        try:
            HCO6 = serial.Serial('/dev/tty.HC-06-DevB', 9600, timeout=1)
            print("Connected to HC-O6")
            break
        except serial.serialutil.SerialException:
            print("Failed to connect to HC-O6, retrying")
            continue

    while True:
        try:
            while True:
                if not INVERT:
                    cmd = HCO6.read(1)
                else:
                    cmd = HBO2.read(1)

                if len(cmd) == 1:
                    print("CMD: %s" % str(cmd))

                    if not INVERT:
                        HBO2.write(cmd)
                    else:
                        HCO6.write(cmd)

        except serial.serialutil.SerialException:
            print("Lost connection, reconnecting")
            HBO2.close()
            HCO6.close()
            break
