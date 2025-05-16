import socket

def start_server():
    # 서버 소켓을 생성합니다. AF_INET은 주소 체계를, SOCK_STREAM은 TCP를 나타냅니다.
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 스트림 소켓 : 연결 지향적이며 신뢰성 있는 바이트 스트림을 제공하는 소켓

    # 서버의 IP 주소와 포트를 설정합니다. ''는 모든 사용 가능한 인터페이스를 나타냅니다.
    # netstat -a -n : 주소와 포트 확인, netstat -tn : 현재 열려있는 TCP 확인
    # 192.168.0.11::139, 127.0.0.1:40053
    server_address = ('192.168.0.25', 12346)
    server_socket.bind(server_address) # 소켓을 address와 binding
    server_socket.listen(1) # 서버가 클라이언트의 연결을 기다리고 있음. '1'은 동시에 처리 가능한 클라이언트 연결의 최대수
    
    #IP 주소와 포트 번호는 서로 다른 역할을 하는데, 
    #이 둘을 조합해 사용함으로써 특정 컴퓨터의 특정 프로세스(또는 서비스)에 데이터를 전송하거나 받을 수 있게 됩니다.
    #IP 주소는 인터넷에 연결된 장치(예: 컴퓨터, 스마트폰, 서버 등)를 식별하는 데 사용됩니다. 
    #예를 들어, 컴퓨터 A가 컴퓨터 B에 데이터를 보내려면 컴퓨터 B의 IP 주소를 알아야 합니다.
    #포트 번호는 해당 장치 내에서 실행 중인 특정 프로세스나 서비스를 식별하는 데 사용됩니다. 
    #예를 들어, 컴퓨터 A가 컴퓨터 B의 웹 서버에 접속하려면 웹 서버가 리스닝하고 있는 포트 번호(일반적으로는 80(HTTP) 또는 443(HTTPS))를 알아야 합니다.
    #따라서, IP 주소와 포트 번호를 함께 사용하면 인터넷 상의 특정 장치의 특정 서비스에 데이터를 전송할 수 있게 됩니다. 
    #이를 소켓이라고 부르기도 합니다.
    #그러니까, IP 주소를 설정할 때는 상대방 장치의 IP 주소를 정확하게 사용해야 합니다. 
    #이 IP 주소는 네트워크 설정이나 인터넷 서비스 제공자(ISP)에 따라 다르게 할당될 수 있습니다.
    #서버가 클라이언트의 연결을 기다리도록 합니다.


    server_socket.listen(1)

    print("서버가 클라이언트의 연결을 기다리는 중입니다.")
    connection, client_address = server_socket.accept() # 서버는 'accept' 메소드를 통해 클라이언트의 연결을 수락합니다. 
                                                        # 이 메소드는 클라이언트가 연결을 시도할 때까지 블록(block) 상태로 대기
                                                        # 'connection'은 클라이언트와 데이터를 주고 받을 수 있는 새로운 소켓 생성
                                                        # 'client_address'는 클라이언트의 주소를 담고 있는 튜플(IP:포트)
    print(f"연결: {client_address}")

    # 클라이언트로 메시지를 보냅니다.
    try:
        while client_address :
            message = input("보낼 메시지를 입력하세요 : ")

            connection.sendall(message.encode('utf-8'))

    # 클라이언트로부터 메시지를 받습니다.
            data = connection.recv(1024) # 클라이언트로부터 1024 bytes만큼의 데이터를 받습니다.
            if data :
                print(f"클라이언트로부터 받은 데이터: {data.decode('utf-8')}") # 받은 데이터를 출력합니다. 

    except KeyboardInterrupt :       
        connection.close()
        server_socket.close()

if __name__ == "__main__":
    start_server()