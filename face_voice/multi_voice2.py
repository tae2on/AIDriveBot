import speech_recognition as sr
from gtts import gTTS
import playsound
import os
from hangul_romanize import Transliter
from hangul_romanize.rule import academic
import time

f_location = None

def recognition_rate(text, place, tae_eon, myung_hyun):
    for word in place + tae_eon + myung_hyun:
        if word in text:
            if word in place:
                text = text.replace(word, '620호')
            elif word in tae_eon:
                text = text.replace(word, '태언')
            elif word in myung_hyun :
                text = text.replace(word, '명현')
    return text

# 이름 인식 코드    
def speak_jetson():
            
    # 음성인식 객체 생성
    r = sr.Recognizer()
            
    with sr.Microphone() as source :
                
        # 마이크로부터 오디오 읽기
        print('"젯슨"을 불러주세요!')
        audio_data = r.record(source, duration = 3)
                
        try:
            # 구글 API로 인식 (하루에 50회 제한)
            text = r.recognize_google(audio_data, language = 'ko')
            
            # 음성인식 시 오류나는 단어
            jetson = ["잭슨", "넥슨", "넥센"]
            
            # 오류나는 젯슨 단어를 젯슨으로 바꿔주는 코드
            for i in jetson :
                if i in text :
                    text = text.replace(i, '젯슨')

            # 이름 인식 -> 음성 인식 코드로 넘어감
            if(text == "젯슨") :
                print("네! 부르셨나요?")
                txt = "네! 부르셨나요?"
                tts_kr = gTTS(txt, lang = 'ko', slow = False)
                wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                tts_kr.save(wav_path)
                playsound.playsound(wav_path)
                
                return respeak()
                    
            # 다른 단어 인식 -> 다시 이름 부르는 코드로 돌아감
            else:
                return speak_jetson()
            
        # 음성 인식 실패한 경우
        except sr.UnknownValueError:
            return speak_jetson()

# 음성 인식    
def respeak():
    global r_name, f_location
    
    # 음성인식 시 오류나는 단어
    place = ['620 4', '20%', '625']
    tae_eon = ['태연', '태현']
    myung_hyun = ['명 현', '영현', '영 현', '영영', '영 영', '명 연']
            
    # 음성인식 객체 생성
    r = sr.Recognizer()
    s = sr.Recognizer()
    m = sr.Recognizer()

    with sr.Microphone() as source :
            
        # 마이크로부터 오디오 읽기
        audio_data = r.record(source, duration = 5)
                
    try:
        # 음성을 문자열로 전환
        # 구글 API로 인식 (하루에 50회 제한)
        text = r.recognize_google(audio_data, language = 'ko')
        #테스트 후 이 위치로 변경
        text = recognition_rate(text, place, tae_eon, myung_hyun)
        
        print("<음성을 문자로 변환한 값을 아래에 표시했습니다.>")
        print(text)
        
        if(text == "현재 위치"):
            print("현재 위치를 말해주세요")
            text = "현재 위치를 말해주세요"
            tts_kr = gTTS(text, lang = 'ko', slow = False)
            wav_path = os.path.join("/home/hyeun/ssun/620_capstone", "voice.wav")
            tts_kr.save(wav_path)
            playsound.playsound(wav_path)
            
            with sr.Microphone() as source :
                audio_data2 = m.record(source, duration = 5)
                text = m.recognize_google(audio_data2, language = 'ko')
                text = recognition_rate(text, place, tae_eon, myung_hyun)
                
                if text :
                    f_location = text
                    f_location = f_location.replace('호', 'f')
                    print('"현재 위치"가 전송되었습니다. 내용을 말씀해주세요!')
                    text = '현재 위치가 전송되었습니다. 내용을 말씀해주세요!'
                    tts_kr = gTTS(text, lang = 'ko', slow = False)
                    wav_path = os.path.join("/home/hyeun/ssun/620_capstone", "voice.wav")
                    tts_kr.save(wav_path)
                    playsound.playsound(wav_path)
                    print(f_location)
                    return speak() if f_location else respeak()
                 
                else :
                    print("'현재 위치'라고 다시 말해주세요")
                    txt = "'현재 위치'라고 다시 말해주세요"
                    tts_kr = gTTS(txt, lang = 'ko', slow = False)
                    wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                    tts_kr.save(wav_path)
                    playsound.playsound(wav_path)
                    return respeak()
        else : 
            # 인식된 음성에 대한 대답
            print('"' + text + '"' + "라고 말한 것이 맞습니까?")
            txt = text + "라고 말한 것이 맞습니까?"
            tts_kr = gTTS(txt, lang = 'ko', slow = False)
            wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
            tts_kr.save(wav_path)
            playsound.playsound(wav_path)

            print("<네 / 아니요로 대답해주세요!")
            with sr.Microphone() as source :
                # 마이크로부터 오디오 읽기
                audio_data1 = s.record(source, duration = 3)
                text1 = s.recognize_google(audio_data1, language = 'ko')

            if (text1 == "네") :
                # 분리할 조사
                location = ['으로', '로', '이에게', '에게', '을', '를', '이한테', '한테', '에', '이']
                        
                # 문자열을 띄어쓰기 기준으로 분리
                text = text.split()
                    
                v_name = []
                v_place = []
                        
                # 문자열을 순회하면서 location이 포함된 단어를 찾고 제거하여 이름과 장소 분리
                for word in text :
                    for loc in location :
                        if loc in word :
                            # location의 단어를 제거한 후 이름 저장
                            if loc in ['이에게', '에게', '이한테', '한테', '이']:
                                v_name = word.replace(loc, '')
                            # location의 단어를 제거한 후 장소 저장
                            elif loc in ['으로', '로', '에']:
                                v_place = word.replace(loc, '')
                                v_place = v_place.replace('호', 'a')
                            # 613으로 같은 경우 '으로'와 '로'가 포함되어 2번 결과가 나오게 됨
                            # break문을 통해 겹치는 단어는 표시 X
                            break
                print(v_name)
                print(v_place)
                                
                # 로마자 변환을 위한 Transliter 클래스 객체 생성
                trans = Transliter(rule=academic)
                # 한글 이름을 로마자로 변환
                romanized_name = trans.translit(v_name)
                print(romanized_name)
                        
                r_name = []
                r_place = []
                r_location = []
                    
                r_name = romanized_name
                r_place = v_place
                r_location = f_location
                        
                print('이름은', r_name)
                print('장소는', r_place)
                print('현재 위치는', r_location)
  
                return r_name, r_place, r_location if r_name or r_place or r_location else respeak()
                
            elif (text1 == "아니요") :
                # 인식된 음성에 대한 대답
                print("내용을 다시 한 번 말씀해주시겠어요?")
                txt = "내용을 다시 한 번 말씀해주시겠어요?"
                tts_kr = gTTS(txt, lang = 'ko', slow = False)
                wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                tts_kr.save(wav_path)
                playsound.playsound(wav_path)
                return respeak()
            
            else :
                print("내용을 다시 한 번 말씀해주시겠어요?")
                txt = "내용을 다시 한 번 말씀해주시겠어요?"
                tts_kr = gTTS(txt, lang = 'ko', slow = False)
                wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                tts_kr.save(wav_path)
                playsound.playsound(wav_path)
                return respeak()
                    
    # 음성 인식 실패한 경우
    except sr.UnknownValueError:
        print("내용을 다시 한 번 말씀해주시겠어요?")
        txt = "내용을 다시 한 번 말씀해주시겠어요?"
        tts_kr = gTTS(txt, lang = 'ko', slow = False)
        wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
        tts_kr.save(wav_path)
        playsound.playsound(wav_path)
        return respeak()
    
def speak(): 
        global f_location 
        
        place = ['620 4', '20%', '625']
        tae_eon = ['태연', '태현']
        myung_hyun = ['명 현', '영현', '영 현', '영영', '영 영', '명 연']
            
        # 음성인식 객체 생성
        r = sr.Recognizer()
        s = sr.Recognizer()
          
        with sr.Microphone() as source :
            audio_data = r.record(source, duration = 5)
            text = r.recognize_google(audio_data, language = 'ko')
            
            # 오류난 단어를 원하는 단어로 변경(위치는 나중에 변경해야 함 !)
            text = recognition_rate(text, place, tae_eon, myung_hyun)
                
            # 인식된 음성에 대한 대답
            print('"' + text + '"' + "라고 말한 것이 맞습니까?")
            txt = text + "라고 말한 것이 맞습니까?"
            tts_kr = gTTS(txt, lang = 'ko', slow = False)
            wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
            tts_kr.save(wav_path)
            playsound.playsound(wav_path)

            print("<네 / 아니요로 대답해주세요!")
            with sr.Microphone() as source :
                # 마이크로부터 오디오 읽기
                audio_data1 = s.record(source, duration = 3)
                text1 = s.recognize_google(audio_data1, language = 'ko')

            if (text1 == "네") :
                # 분리할 조사
                location = ['으로', '로', '이에게', '에게', '을', '를', '이한테', '한테', '에', '이']
                        
                # 문자열을 띄어쓰기 기준으로 분리
                text = text.split()
                    
                v_name = []
                v_place = []
                        
                # 문자열을 순회하면서 location이 포함된 단어를 찾고 제거하여 이름과 장소 분리
                for word in text :
                    for loc in location :
                        if loc in word :
                            # location의 단어를 제거한 후 이름 저장
                            if loc in ['이에게', '에게', '이한테', '한테', '이']:
                                v_name = word.replace(loc, '')
                            # location의 단어를 제거한 후 장소 저장
                            elif loc in ['으로', '로', '에']:
                                v_place = word.replace(loc, '')
                                v_place = v_place.replace('호', 'a')
                            # 613으로 같은 경우 '으로'와 '로'가 포함되어 2번 결과가 나오게 됨
                            # break문을 통해 겹치는 단어는 표시 X
                            break
                print(v_name)
                print(v_place)
                                
                # 로마자 변환을 위한 Transliter 클래스 객체 생성
                trans = Transliter(rule=academic)
                # 한글 이름을 로마자로 변환
                romanized_name = trans.translit(v_name)
                print(romanized_name)
                        
                r_name = []
                r_place = []
                r_location = []
                    
                r_name = romanized_name
                r_place = v_place
                r_location = f_location
                        
                print('이름은', r_name)
                print('장소는', r_place)
                print('현재 위치는', r_location)
  
                return r_name, r_place, r_location if r_name or r_place or r_location else speak()
                
            elif (text1 == "아니요") :
                # 인식된 음성에 대한 대답
                print("내용을 다시 한 번 말씀해주시겠어요?")
                txt = "내용을 다시 한 번 말씀해주시겠어요?"
                tts_kr = gTTS(txt, lang = 'ko', slow = False)
                wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                tts_kr.save(wav_path)
                playsound.playsound(wav_path)
                return speak()
            
            else :
                print("내용을 다시 한 번 말씀해주시겠어요?")
                txt = "내용을 다시 한 번 말씀해주시겠어요?"
                tts_kr = gTTS(txt, lang = 'ko', slow = False)
                wav_path = os.path.join("/home/hyeun/2023-1-Capstone-/face_voice", "voice.wav")
                tts_kr.save(wav_path)
                playsound.playsound(wav_path)
                return speak()
    
def main_voice():

    while True:
        name = []
        locate = []
        ff_locate = []
        name, locate, ff_locate = speak_jetson()
        if (name != []) or (locate != []) or (ff_locate != []):
            r_name_list = name
            r_locate = locate
            first_location = ff_locate
            print("r_name: ", r_name_list)
            print("r_locate: ", r_locate)
            print("first_location: ", first_location)
            yield r_name_list, r_locate, first_location

        else:
            break

""" def get_r_name_list():
    r_name_list = main()  
    return r_name_list """

if __name__ == "__main__" :
    main_voice()