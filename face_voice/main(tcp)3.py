import threading
import time
from multi_voice4 import main_voice
import cv2
import os
from multi_face0 import Facerecognition0
import socket
import re
from gtts import gTTS
import playsound

lock = threading.Lock() # 공유 변수
shared_r_name_list = None
shared_r_locate = None
arrive = None
cur_send_location = None
lidar_signal = None
send_location = None
f_location = None

############################################################################
############################################################################
############################################################################

def send():
    global shared_r_locate, send_location, lidar_signal, arrive, f_location, msg_location_check, msg_cur_location_check

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET : 주소 체계, SOCK_STREAM : TCP 방식
    server_address = ('192.168.78.19', 12345) 
    server_socket.bind(server_address) # 소켓을 address와 binding
    server_socket.listen(1) # 동시에 처리 가능한 클라이언트 연결의 최대수
    print("Server is waiting for Client")
    connection, client_address = server_socket.accept() # 서버는 'accept' 메소드를 통해 클라이언트의 연결을 수락합니다. 
                                                        # 이 메소드는 클라이언트가 연결을 시도할 때까지 블록(block) 상태로 대기
                                                        # 'connection'은 클라이언트와 데이터를 주고 받을 수 있는 새로운 소켓 생성
                                                        # 'client_address'는 클라이언트의 주소를 담고 있는 튜플(IP:포트)
    print(f"연결 확인: {client_address}")

    thread4 = threading.Thread(target = recv, args=(connection,))
    thread4.start()
    

    try :
        #msg_location = None
        #prev_send_location = None

        while True :
            msg_location = None
            msg_cur_location = None
            prev_send_location = None
            prev_cur_send_location = None
            cur_send_location = send_location # 1-2) 전달 받은 send_location을 cur_send_locatio에 저장
            cur_cur_send_location = f_location
            arrv = None
            if client_address :

                with lock :
                    if lidar_signal or arrive :
                        signal = lidar_signal
                        arrv = arrive
                        
                    if prev_cur_send_location != cur_cur_send_location : # 현재 위치
                        prev_cur_send_location = cur_cur_send_location
                        msg_cur_location = cur_cur_send_location
                    
                    if prev_send_location != cur_send_location : # 목표 위치
                        prev_send_location = cur_send_location
                        msg_location = cur_send_location
                    
                if arrv and (signal == 'arrive') : # 라이다에서 목표점에 도착했을 때 신호를 받으면 실행
                    print(f"{arrv}에 도착했습니다!")
                    check = ' '.join(arrv)
                    connection.sendall(check.encode('utf-8'))
                    txt = f"목표지점에 도착했습니다. {check_name}님 카메라에 얼굴을 내밀어 주세요."
                    tts_kr = gTTS(txt, lang = 'ko', slow = False)
                    wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                    tts_kr.save(wav_path)
                    playsound.playsound(wav_path)
                    lidar_signal = 'finish'
                    time.sleep(1)
                    
                if (msg_cur_location and (msg_cur_location_check == "one")) or (msg_location and (msg_location_check == "once")) : # 현재 위치
                    msg = msg_cur_location + ' ' + msg_location
                    connection.sendall(msg.encode('utf-8'))
                    print(f"라이다에게 현재 {msg}의 좌표값을 보냈습니다. ")
                    msg_cur_location_check = "two"
                    msg_location_check = "twice"
                    time.sleep(1)
                    

                #if  msg_location and (msg_location_check == "once"): # 목표 위치
                #    connection.sendall(msg_location.encode('utf-8'))
                #    print(f"라이다에게 {msg_location}의 좌표값을 보냈습니다. ")
                #    msg_location_check = "twice"

            else :
                print("연결이 안됨")
            time.sleep(1)

    except KeyboardInterrupt :
        connection.close()
        server_socket.close()

############################################################################
############################################################################
############################################################################

def recv(connection): # lidar로부터 목표점에 도착했다는 신호를 받기 위한 함수
    global lidar_signal

    while True :
        try:

            message = connection.recv(1024)
            message = message.decode('utf-8')
            with lock :
                if message :
                    print(f"라이다로부터 받은 메세지 : {message}")
                    lidar_signal = message

        except Exception as e:
            print(f"Error occurred while receiving message: {e}")
            break

############################################################################
############################################################################
############################################################################

def camera(): 
    global shared_r_name_list, shared_r_locate, arrive

    face_recognition = Facerecognition0()
    complete_count = 0
    num = 0
    for names, location in face_recognition.video():
        str_names = ''.join(str(element) for element in names)
        str_location = ''.join(str(element1) for element1 in location)
        complete_count += 1

        if complete_count % 20 == 0 :
            print("location : ", str_location)

        if shared_r_name_list or shared_r_locate:
            with lock:
                if (str_names == shared_r_name_list) and (complete_count % 10 == 0):
                    print("일치합니다")
                    num += 1
                    if num % 2 == 0 :
                        txt = "일치합니다"
                        tts_kr = gTTS(txt, lang = 'ko', slow = False)
                        wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                        tts_kr.save(wav_path)
                        playsound.playsound(wav_path)

                if (str_location == shared_r_locate) and (complete_count % 20 == 0) :
                    print("{0}에 도착했습니다. ".format(str_location))
                    arrive = re.findall(r'\d+', str_location)
                    print("arrive :" ,arrive)
                    # shared_r_name_list = None
        # cv2.imwrite('captured_frame.jpg', frame)  # 사진 기능 captured_frame : 저장할 이름

############################################################################
############################################################################
############################################################################

def mic_speaker(): # Voice
    global shared_r_name_list, shared_r_locate, send_location, f_location, msg_location_check, msg_cur_location_check, check_name, g_name

    for r_name_list, r_locate_list, f_place in main_voice() :
        print ("r_name :" , r_name_list)
        print ("r_place :", r_locate_list)
        print ("f_place :", f_place)
        with lock:
            if (r_name_list != []) or (r_locate_list != [])  :
                shared_r_name_list = r_name_list # 카메라와 연동(얼굴)
                check_name = r_name_list # 
                shared_r_locate = r_locate_list # 카메라와 연동(장소)
                msg_location_check = "once"
                send_location = r_locate_list # 1-1) 음성인식으로 전달받은 데이터를 send_location으로 저장후 send()에 전송
                msg_cur_location_check = "one"
                f_location = f_place # 처음 위치

############################################################################
############################################################################
############################################################################

def main():
    #스레드 정의
    thread1 = threading.Thread(target=camera)
    thread2 = threading.Thread(target=mic_speaker)
    thread3 = threading.Thread(target=send)

    #스레드 시작
    thread1.start()
    thread2.start()
    thread3.start()

    thread1.join()
    thread2.join()
    thread3.join()

if __name__ == "__main__":
    main()