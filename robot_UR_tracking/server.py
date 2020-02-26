
import socket
import struct



localIP     = "172.31.1.57"

localPort   = 30001

bufferSize  = 1024

 

msgFromServer       = "Hola"

bytesToSend         = str.encode(msgFromServer)

 

# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 

# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))

 

print("UDP server up and listening")

 

# Listen for incoming datagrams

while(True):

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

    message = bytesAddressPair[0]

    address = bytesAddressPair[1]

    clientMsg = "{}".format(message)
    clientIP  = "{}".format(address)
    
    print(clientMsg)
    print(clientIP)

    if clientMsg == "cmdx":
      print("Send Value")
      value = 10
      ba = bytearray(struct.pack("f", value))
    elif clientMsg == "cmdy":
      print("Send Value")
      value = 10.3
      ba = bytearray(struct.pack("f", value))    
    elif clientMsg == "cmdz":
      print("Send Value")
      value = 4.3
      ba = bytearray(struct.pack("f", value))  
    elif clientMsg == "Mode":
      print("Send Value")
      ba = bytearray("Man")    
    else :
      print("Send")



    UDPServerSocket.sendto(ba, address)
    print("Send Value")

    # Sending a reply to client

    