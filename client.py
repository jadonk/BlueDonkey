#!/usr/bin/env python3
import socket, sys

if len(sys.argv) < 2:
    sys.exit()


print(sys.argv[1].upper())


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect(('127.0.0.1', 3002))

s.send(sys.argv[1].upper().encode())

data = s.recv(1024).decode()

print(data)

s.close()