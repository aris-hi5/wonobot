# import pygame

# class Hi5_Sound:
#     def __init__(self):
#         self.path = 'sound/'
        
#     def sound_init(self):
#         # pygame 에서 music vs sound
#         # music: 배경음악 재생을 위해 사용
#         # sound: 효과음을 위해 사용

#         # BG sound.
#         pygame.init()

#         # Effect sound
#         sounds = pygame.mixer
#         sounds.init()

#     def BGM_play(self):
#         pygame.mixer.music.load(self.path +'bgm.wav')  # Loading File Into Mixer
#         pygame.mixer.music.play(-1)  # Playing It In The Whole Device

#     def Effect_play(self, name):
#         effect = self.sounds.Sound(self.path+name)
#         effect.play()

#     def test_main(self):
#         self.sound_init()
#         self.BGM_play()
#         while True:
#             user_input = input()
#             if user_input == 's':
#                 self.Effect_play('makestart.wav')
#             elif user_input == 'w':
#                 self.Effect_play('warning.wav')

# # if __name__ == '__main__':
# #     sound_main = Hi5_Sound()
# #     sound_main.test_main()

import pygame
import json
import time
import random

class Hi5_Sound:
    def __init__(self):
        self.path = 'sound/'
        self.bgm_volume = 1.0  # BGM의 기본 볼륨 설정
        self.effect_volume = 1.0  # 효과음의 기본 볼륨 설정
        
    def sound_init(self):
        # pygame 에서 music vs sound
        # music: 배경음악 재생을 위해 사용
        # sound: 효과음을 위해 사용

        # BG sound.
        pygame.init()

        # Effect sound
        self.sounds = pygame.mixer
        self.sounds.init()

    def BGM_play(self):
        pygame.mixer.music.load(self.path +'bgm.mp3')  # Loading File Into Mixer
        pygame.mixer.music.play(-1)  # Playing It In The Whole Device
        pygame.mixer.music.set_volume(self.bgm_volume)

    def Effect_play(self, name):
        # 현재 BGM 볼륨 저장
        current_volume = pygame.mixer.music.get_volume()
        # BGM 볼륨 줄이기
        pygame.mixer.music.set_volume(0.3 * self.bgm_volume)

        effect = self.sounds.Sound(self.path + name)

        # 효과음 볼륨을 2배로 설정
        effect.set_volume(self.effect_volume * 2.0)

        channel = effect.play()
        print(f"Effect sound {name} played.")
        
        # 효과음 재생이 끝날 때까지 기다렸다가 BGM 볼륨 복원
        while channel.get_busy():
            pygame.time.wait(100)
        pygame.mixer.music.set_volume(current_volume)

        # effect.play()

        # effect.set_endevent(pygame.USEREVENT)
        # while True:
        #     for event in pygame.event.get():
        #         if event.type == pygame.USEREVENT:
        #             pygame.mixer.music.set_volume(current_volume)
        #             return
        

#     def test_main(self):
#         self.sound_init()
#         self.BGM_play()
#         # while True:
#         #     user_input = input()
#         #     if user_input == 's':
#         #         self.Effect_play('makestart.wav')
#         #     elif user_input == 'w':
#         #         self.Effect_play('warning.wav')



#         # JSON 메시지
#         json_message = '''
#         {
#             "OR": [
#                 {"orderId": "1", "icecream": "choco", "topping":"oreo,chocoball,cereal"}
#             ]
#         }
#         '''
#         # choco
#         # mint
#         # strawberry

#         # # JSON 메시지 파싱
#         data = json.loads(json_message)

#         for order in data['OR']:
#             icecream = order['icecream']
#             topping = order['topping']
#             print(f"icecream: {icecream}, topping: {topping}")

#             # 토핑의 갯수 계산
#             topping_list = [t.strip() for t in topping.split(',')]
#             # topping_count = len(topping_list)
            
#             print(topping_list)
#             # self.a = True

#         # # 필요한 정보 추출
#         # icecream = data["OR"]["icecream"]
#         # topping = data["OR"]["topping"]
#         # print(f"icecream: {icecream}, topping: {topping}")

#         # topping_list = [t.strip() for t in topping.split(',')]
#         # print(topping_list)

#         sound_main.Effect_play(f'order.mp3')
#         sound_main.Effect_play(f'{icecream}.mp3')
#         for i in topping_list:
#             sound_main.Effect_play(f'{i}.mp3')
#         sound_main.Effect_play(f'makestart.mp3')
#         time.sleep(1)

#         sound_main.Effect_play(f'making.mp3')
#         time.sleep(1)

#         sound_main.Effect_play(f'sealing.mp3')
#         time.sleep(1)

#         sound_main.Effect_play(f'warning.mp3')
#         time.sleep(1)

#         sound_main.Effect_play(f'Dondurma_start.mp3')
#         time.sleep(1)

#         Dondurma_count = random.randrange(2,5)
#         print(Dondurma_count)

#         for i in range(0,Dondurma_count):
#             sound_main.Effect_play(f'Dondurma_effect_{i}.mp3')
#             time.sleep(1)
        
#         sound_main.Effect_play(f'THX.mp3')
#         time.sleep(1)

#         sound_main.Effect_play(f'remove_capsule.mp3')
#         time.sleep(1)


# if __name__ == '__main__':
#     sound_main = Hi5_Sound()
#     sound_main.test_main()



