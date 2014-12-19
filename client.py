import socket
import time
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--port-number', type=int, action='store', default=3000, help='Specify which server port you want to connect to')
args = parser.parse_args()
clientsocket= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientsocket.connect(('localhost',args.port_number))
while 1:
	clientsocket.send('h')
	time.sleep(10)
