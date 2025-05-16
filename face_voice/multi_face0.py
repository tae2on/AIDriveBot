import cv2
import os, sys
import numpy as np
import math
import glob
import time
import face_recognition as fr

MIN_CONTOUR_AREA = 4000

image_path = r'/home/hyeun/face_img/*.png'

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def color_recognition(frame) :
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([170, 70, 50])
    upper_red = np.array([180, 255, 255])
    #lower_red = np.array([0, 90, 80])
    #upper_red = np.array([10, 255, 255])

    #lower_red1 = np.array([130, 50, 50])
    #upper_red1 = np.array([160, 255, 255])
    
    #lower_White = np.array([25, 50, 70])
    #upper_White = np.array([35, 255, 255])

    lower_White = np.array([0,0,200], dtype=np.uint8)      
    upper_White = np.array([180,255,255], dtype=np.uint8) 

    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])

    lower_blue = np.array([100, 150, 50]) 
    upper_blue = np.array([140, 255, 255])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_White = cv2.inRange(hsv, lower_White, upper_White)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_White, _ = cv2.findContours(mask_White, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    roi_red = None
    roi_White = None
    roi_green = None
    roi_blue = None
    
    #for cnt_red in contours_red :
    #    x_r, y_r, w_r, h_r = cv2.boundingRect(cnt_red) # 외곽선 경계 사각형 구함 
    #    cv2.rectangle(frame, (x_r, y_r), (x_r + w_r, y_r + h_r), (0, 0, 255), 2) # w는 너비 h는 높이, 녹색, 두께
    #    roi_red = mask_red[y_r : y_r + h_r, x_r : x_r + w_r] # 해당 영역 이미지 잘라내고 확대`1`

    for cnt_red in contours_red:
        if cv2.contourArea(cnt_red) > MIN_CONTOUR_AREA:
            x_r, y_r, w_r, h_r = cv2.boundingRect(cnt_red)
            if abs(w_r - h_r) <= 5:
                cv2.rectangle(frame, (x_r, y_r), (x_r + w_r, y_r + h_r), (0, 0, 255), 2) # B G R
                roi_red = mask_red[y_r : y_r + h_r, x_r : x_r + w_r]

    for cnt_White in contours_White :
        if cv2.contourArea(cnt_White) > MIN_CONTOUR_AREA:
            x_b, y_b, w_b, h_b = cv2.boundingRect(cnt_White)
            if abs(w_b - h_b) <= 5:
                cv2.rectangle(frame, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 255, 255), 2)
                roi_White = mask_White[y_b:y_b + h_b, x_b:x_b + w_b]

    for cnt_green in contours_green :
        if cv2.contourArea(cnt_green) > MIN_CONTOUR_AREA:
            x_g, y_g, w_g, h_g = cv2.boundingRect(cnt_green)
            if abs(w_g - h_g) <= 5:
                cv2.rectangle(frame, (x_g, y_g), (x_g + w_g, y_g + h_g), (0, 255, 0), 2)
                roi_green = mask_green[y_g:y_g + h_g, x_g:x_g + w_g]

    for cnt_blue in contours_blue :
        if cv2.contourArea(cnt_blue) > MIN_CONTOUR_AREA:
            x_g, y_g, w_g, h_g = cv2.boundingRect(cnt_blue)
            if abs(w_g - h_g) <= 5:
                cv2.rectangle(frame, (x_g, y_g), (x_g + w_g, y_g + h_g), (255, 0, 0), 2)
                roi_blue = mask_blue[y_g:y_g + h_g, x_g:x_g + w_g]

    #remask_red = cv2.inRange(roi_red, lower_red,upper_red)
    #remask_White = cv2.inRange(roi_White, lower_White,upper_White)
    #remask_green = cv2.inRange(roi_green, lower_green,upper_green)

    red_pixels = cv2.countNonZero(roi_red) 
    White_pixels = cv2.countNonZero(roi_White) 
    green_pixels = cv2.countNonZero(roi_green) 
    blue_pixels = cv2.countNonZero(roi_blue) 
     

    color = {"not found":red_pixels, "620a": White_pixels, "elevatora": green_pixels, "610a": blue_pixels}
    loc_val = max(color, key = color.get, default= "*")
    
    return frame, loc_val

""" def locate_value(r_pix, b_pix, g_pix) :
    color = {620:r_pix, 602: b_pix, 611: g_pix}
    loc_val = max(color, key = color.get) """

def face_confidence(face_distance, face_match_threshold=0.6): # face_distance 값과 face_match 임계값을 설정한 사설함수
    range = (1.0 - face_match_threshold)
    linear_val = (1.0 - face_distance) / (range * 2.0)

    if face_distance > face_match_threshold:
        return str(round(linear_val * 100, 2)) + '%'
    else:
        value = (linear_val + ((1.0 - linear_val) * math.pow((linear_val - 0.5) * 2, 0.2))) * 100
        return str(round(value, 2)) + '%'

class Facerecognition0:
    face_location = []
    face_encoding = []
    face_names0 = []
    known_face_encoding = []
    known_face_names0 = []
    process_current_frame = True

    def __init__(self, callback=None):
        self.encode_faces()

    def encode_faces(self):
        os.chdir('/home/hyeun/face_img')
        file_names = os.listdir()
        for file_name in file_names :
            self.known_face_names0.append(os.path.splitext(file_name)[0])
        for image in glob.glob(image_path):
            face_image = fr.load_image_file(image)
            face_encoding = fr.face_encodings(face_image)[0]
            self.known_face_encoding.append(face_encoding)
        print(self.known_face_names0)

    
    def video(self, callback= None):
        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method = 0), cv2.CAP_GSTREAMER)

        if not cap.isOpened() :
            print('unable to open camera')
            sys.exit()

        while True :            
            ret, frame = cap.read() # fps  = frame per second 60 frame = 1초 60장을
            loc_name = []
            frame, location = color_recognition(frame)
            loc_name.append(location)
            if self.process_current_frame: # 인식처리를 더 빠르게 하기 위해 1/4 크기로 줄임
                small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

                # rgb_small_frame = small_frame[:, :, ::-1] # bgr => rgb
                rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
                # gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                # imgchar = pytesseract.image_to_string(gray, lang = 'eng')
                self.face_location = fr.face_locations(rgb_small_frame)
                self.face_encodings = fr.face_encodings(rgb_small_frame, self.face_location)

                self.face_names0 = []
                for face_encoding in self.face_encodings: # 저장된 얼굴과 캠에서 찍힌 얼굴과 비교
                    match = fr.compare_faces(self.known_face_encoding, face_encoding, 0.55)
                    name0 = "???"
                    match_percent = "??.?%"
                    face_distance = fr.face_distance(self.known_face_encoding, face_encoding) # 두 사진의 인Coding 거리 값을 비교

                    best_match_index = np.argmin(face_distance) # 최소 값을 가진 인덱스를 알려준다
                    if match[best_match_index] :
                        name0 = self.known_face_names0[best_match_index]
                        match_percent = face_confidence(face_distance[best_match_index])                          
                    self.face_names0.append(f'{name0}')
                
                # self.process_current_frame = not self.process_current_frame


            yield self.face_names0, loc_name

            for (top, right, bottom, left), name0 in zip(self.face_location, self.face_names0) : # 1/4로 축소된 얼굴 크기를 다시 되돌림
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                cv2.rectangle(frame, (left, top), (right, bottom), (0,0,0), 1)
                cv2.rectangle(frame, (left, bottom - 30), (right, bottom), (0,0,0), cv2.FILLED)
                cv2.putText(frame, name0, (left+ 10, bottom - 10), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255),1)

            cv2.imshow('Face Recognition0', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                 break

        cap.release()
        cv2.destroyAllWindows()



""" if __name__ == "__main__":
    run = Facerecognition()
    for names in run.video() :
        print(names) """


