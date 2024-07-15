# import cv2
# import numpy as np


# if __name__ == "__main__":
#     board_type = cv2.aruco.DICT_6X6_250
#     marker_size = 400
#     id_info = 2

#     arucoDict = cv2.aruco.getPredefinedDictionary(board_type)
#     aruco_matker_img = cv2.aruco.generateImageMarker(arucoDict, id_info, marker_size)

#     cv2.imshow("aruco", aruco_matker_img)
#     cv2.waitKey(0)



from hi5_sound import Hi5_Sound

def main():
    # Hi5_Sound 클래스의 인스턴스를 생성합니다.
    hi5_sound_instance = Hi5_Sound()

    # 인스턴스를 사용하여 메서드를 호출합니다.
    hi5_sound_instance.sound_init()
    hi5_sound_instance.BGM_play()

    # 키 입력을 통한 효과음 테스트
    while True:
        user_input = input("Press 's' for start sound, 'w' for warning sound, or 'q' to quit: ")
        if user_input == 's':
            hi5_sound_instance.Effect_play('makestart.wav')
        elif user_input == 'w':
            hi5_sound_instance.Effect_play('warning.wav')
        elif user_input == 'q':
            break



if __name__ == "__main__":
    main()