import threading
import time
from multi_voice import main_voice
import cv2
import os
from multi_face0 import Facerecognition0
import socket

lock = threading.Lock() # 공유 변수
shared_r_name_list = None
shared_r_locate = None
def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET : 주소 체계, SOCK_STREAM : TCP 방식
    server_address = ('192.168.0.11', 12345)
    server_socket.bind(server_address)


def func1(name): # Video
    global shared_r_name_list, shared_r_locate
    face_recognition = Facerecognition0()
    complete_count = 0
    for names, location in face_recognition.video():
        str_names = ''.join(str(element) for element in names)
        str_location = ''.join(str(element1) for element1 in location)
        complete_count += 1

        if complete_count % 20 == 0 :
            print("location : ", str_location)

        if shared_r_name_list or shared_r_locate:
            with lock:
                if (str_names == shared_r_name_list) and (complete_count % 20 == 0):
                    print("일치합니다")
                if (str_location == shared_r_locate) and (complete_count % 20 == 0) :
                    print("{0}에 도착했습니다. ".format(str_location))
        # cv2.imwrite('captured_frame.jpg', frame)  # 사진 기능 captured_frame : 저장할 이름

def func2(voice): # Voice
    global shared_r_name_list, shared_r_locate 
    for r_name_list, r_locate_list in main_voice() :
        print ("r_name :" , r_name_list)
        print ("r_place :", r_locate_list)
        with lock:
            if (r_name_list != []) or (r_locate_list != []) :
                shared_r_name_list = r_name_list
                shared_r_locate = r_locate_list

def main():
    #스레드 정의
    thread1 = threading.Thread(target=func1, args=(True,))
    thread2 = threading.Thread(target=func2, args=(True,))

    #스레드 시작
    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

if __name__ == "__main__":
    main()
