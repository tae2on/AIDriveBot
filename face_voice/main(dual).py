import threading
import time
from multi_voice import main_voice
import cv2
import os
from multi_face0 import Facerecognition0
from multi_face1 import Facerecognition1

lock = threading.Lock() # 공유 변수
shared_r_name_list = None
shared_r_locate = None

# 스레드 테스트를 위해 def 2개 생성
def face0():
    global shared_r_name_list, shared_r_locate
    face_recognition0 = Facerecognition0()
    complete_count0 = 0

    for names0, location0 in face_recognition0.video():
        str_names0 = ''.join(str(element) for element in names0)
        str_location0 = ''.join(str(element1) for element1 in location0)
        complete_count0 += 1
        
        if complete_count0 % 20 == 0 :
            print("location : ", str_location0)
            print("0번 카메라 인물 : {}".format(str_names0))

        if shared_r_name_list or shared_r_locate:
            with lock:
                if (str_names0 == shared_r_name_list) and (complete_count0 % 20 == 0):
                    print("0번 카메라 : 일치합니다")
                if (str_location0 == shared_r_locate) and (complete_count0 % 20 == 0) :
                    print("0번 카메라 : {0}에 도착했습니다. ".format(str_location0))
        # cv2.imwrite('captured_frame.jpg', frame)  # 사진 기능 captured_frame : 저장할 이름

def face1():
    global shared_r_name_list, shared_r_locate
    face_recognition1 = Facerecognition1()
    complete_count1 = 0

    for names1, location1 in face_recognition1.video():
        str_names1 = ''.join(str(element) for element in names1)
        str_location1 = ''.join(str(element1) for element1 in location1)
        complete_count1 += 1
        
        if complete_count1 % 20 == 0 :
            print("location : ", str_location1)
            print("1번 카메라 인물 : {}".format(str_names1))
            
        if shared_r_name_list or shared_r_locate:
            with lock:
                if (str_names1 == shared_r_name_list) and (complete_count1 % 20 == 0):
                    print("1번 카메라 : 일치합니다")
                if (str_location1 == shared_r_locate) and (complete_count1 % 20 == 0) :
                    print("1번 카메라 : {0}에 도착했습니다. ".format(str_location1))

def voice():
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
    thread1 = threading.Thread(target=face0)
    thread2 = threading.Thread(target=face1)
    thread3 = threading.Thread(target=voice)
    #스레드 시작
    thread1.start()
    thread2.start()
    thread3.start()

    thread1.join()
    thread2.join()
    thread3.join()

if __name__ == "__main__":
    main()
