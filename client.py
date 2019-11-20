#!/usr/bin/env python3

## This script accepts a single command-line argument and then attempts
## to connect to the listener in car_control.py on socket 3002
## 
## If successful, the script then sends the argument over the socket and 
## then receive a response, which is printed to stdout

import socket, sys

# Check that there is a command-line argument to pass, exit if not
if len(sys.argv) < 2:
    sys.exit()

# Create socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Initiate connection
s.connect(('127.0.0.1', 3002))

# Send command-line argument
s.send(sys.argv[1].upper().encode())

# Receive response
data = s.recv(1024).decode()

# Print response to Node-RED debug
print(data)

# Close the socket
s.close()