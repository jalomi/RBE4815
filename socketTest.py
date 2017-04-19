import socket
import numpy as np
from time import sleep
import abb


def main():
	 ### TEST SOCKET ###
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect('', 5001)
    s.send("Hello!")
    data = s.recv(1024)
    s.close()
    print(data)
    
    print("Done")
    return




if __name__ == "__main__":
	main()