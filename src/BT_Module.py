from pyb import UART, Pin
from time import sleep_ms

# Disclamer:
# This Bluetooth module was started, but due to the fast approaching deadline of the term project
# and the fact that Bluetooth wasn't a requirement, only to impove convenience, this module was left
# abandoned and unfinished

baudrate = 115200
stopbit = 1
parity = 0
pswd = "test"
name = "test"

allow_baudrate = [9600, 19200, 38400, 57600, 115200, 230400, 460800]
allow_stopbit = [1, 2]
allow_parity = [0, 1, 2]

#checking for setting errors
if baudrate not in allow_baudrate:
    raise ValueError(f"Invalid Baudrate Selected; choose one of the following {allow_baudrate}")
    
if stopbit not in allow_stopbit:
    raise ValueError(f"Invalid Stopbit Selected; choose one of the following {allow_stopbit}")
    
if parity not in allow_parity:
    raise ValueError(f"Invalid Parity Selected; choose one of the following {allow_parity}")
    
#UART 1 Uses B6 and B7 by default
BT_ser = UART(1, 38400, timeout=1000)

# # Make a serial port object from the UART class
# BT_ser = UART(1, 115200)

# # Deconfigure default pins
# Pin(Pin.B6,  mode=Pin.ANALOG)     # Set pin modes back to default
# Pin(Pin.B7,  mode=Pin.ANALOG)

# # Configure the selected pins in coordination with the alternate function table
# Pin(Pin.A9,  mode=Pin.ALT, alt=7) # Set pin modes to UART matching column 7 in alt. fcn. table
# Pin(Pin.A10, mode=Pin.ALT, alt=7)

input("Press enter to send configuration to Bluetooth Module\n")

# # Reset Device
# s = "AT+ORGL\r\n"
# print("Device Factory Reset")
# print(f"Sending command: {repr(s)}\n")
# BT_ser.write(s)
# s = BT_ser.readline()
# print(f"Device Response: {repr(s)}\n")
# if s !=b"OK\r\n":
#     raise Exception("Reset command not accepted")
#     sleep_ms(500)

# Rename Device
s = f"AT+NAME={name}\r\n"
print("Renaming Device")
print(f"Sending command: {repr(s)}")
BT_ser.write(s)
s = BT_ser.readline()
print(f"Device Response: {repr(s)}\n")
if s != b"OK\r\n":
    raise Exception("Rename command not accepted")
sleep_ms(500)

# Reset password
s = f"AT+PSWD={pswd}\r\n"
print("Resetting Password")
print(f"Sending command: {repr(s)}\n")
BT_ser.write(s)
s = BT_ser.readline()
print(f"Device Response: {repr(s)}\n")
if s != b"OK\r\n":
    raise Exception("Password reset command not accepted")
sleep_ms(500)

# Configure UART
s = f"AT+UART={baudrate},{stopbit},{parity}\r\n"
print("Configuring UART")
print(f"Sending command: {repr(s)}\n")
BT_ser.write(s)
s = BT_ser.readline()
print(f"Device Response: {repr(s)}\n")
if s != b"OK\r\n":
    raise Exception("UART config command not accepted")
sleep_ms(500)

# Reset
s = f"AT+RESET\r\n"
print("Resetting Module")
print(f"Sending command: {repr(s)}\n")
BT_ser.write(s)
s = BT_ser.readline()
print(f"Device Response: {repr(s)}\n")
if s != b"OK\r\n":
    raise Exception("Reset failed")