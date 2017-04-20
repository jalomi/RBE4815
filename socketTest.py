import socket
import sys
from time import sleep
import abb



def main():
	 ### TEST SOCKET ###
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket created")

    try:
    	s.bind(('',5515))
    except socket.error as msg:
    	print(str(msg[0]) + ":" + msg[1])

    print("Socket binded")

    s.listen(10)

    print("Socket listening")


    conn, addr = s.accept()
    print("Connected: " + addr[0] + ":" + str(addr[1]))
   
    conn.send("jmove% 0, 0, 0, 0, 0, 0,#")

    data = conn.recv(1024)
    print(data)


    s.close()


    # s.connect('', 5001)
    # s.send("Hello!")
    # data = s.recv(1024)
    # s.close()
    # print(data)
    
    # print("Done")
    return




if __name__ == "__main__":
	main()
