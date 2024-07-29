import socket
import threading
import queue
import json
import time

HOST = '192.168.1.13'
KIOSK_PORT = 9005

class KioskManager:
    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self.data_queue = queue.Queue()  # 데이터 처리를 위한 큐
        self.server_socket = None
        self.client_list = []
        self.running = True  # 노드 실행 여부 플래그

        # 서버 소켓 설정 및 수신 대기
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print(f"Listening on {self.host}:{self.port}")

        # 데이터 처리 스레드 시작
        self.data_thread = threading.Thread(target=self.process_data)
        self.data_thread.start()

        # tcp 통신 클라이언트 수락 스레드 시작
        self.accept_thread = threading.Thread(target=self.accept_clients)
        self.accept_thread.start()

    def accept_clients(self):
        while self.running:
            conn, addr = self.server_socket.accept()
            self.client_list.append(conn)
            client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
            client_thread.start()
            print(f"{addr}에 대한 스레드 시작됨")

    def handle_client(self, conn, addr):  # tcp로 kiosk gui에서 받은 data
        print(f"{addr}에서 연결됨")
        try:
            with conn:
                while self.running:
                    recv = conn.recv(1024)
                    if not recv:
                        break
                    print(f"{addr}로부터 수신: {recv.decode()}")
                    # 받은 데이터를 큐에 넣음
                    self.data_queue.put((recv.decode(), conn))
        except Exception as e:
            print(f"클라이언트 처리 중 오류: {e}")
        finally:
            if conn in self.client_list:
                self.client_list.remove(conn)
            conn.close()

    def process_data(self):  # queue에 쌓아놨던 order 처리
        while self.running:
            data, conn = self.data_queue.get()
            print(data)
            try:
                if "OR" in data:
                    print("CMD : OR 처리중")
                    self.client_send(conn, "OR 처리 완료")
                else:
                    print("Unknown CMD")
                    self.client_send(conn, "Unknown CMD")
            except json.JSONDecodeError as e:
                print(f"JSON 파싱 오류: {e}")
            except Exception as e:
                print(f"데이터 처리 중 오류: {e}")

    def client_send(self, conn, msg):
        try:
            conn.sendall(msg.encode())
            print(f"전송 완료: {conn.getpeername()[0]}")
        except Exception as e:
            print(f"전송 중 오류: {e}")

    def broadcast(self, msg):
        for conn in self.client_list:
            self.client_send(conn, msg)

    def shutdown(self):
        self.running = False  # 실행 플래그를 False로 설정하여 스레드 루프를 종료

        # 서버 소켓을 닫음
        if self.server_socket:
            self.server_socket.close()

        # 클라이언트 연결을 닫음
        for client in self.client_list:
            client.close()

def main(args=None):
    kiosk_manager = KioskManager(HOST, KIOSK_PORT)

    try:
        # accept_clients와 process_data는 스레드에서 이미 실행 중이므로 여기서 호출할 필요 없음
        while kiosk_manager.running:
            # 주기적으로 메시지를 브로드캐스트
            kiosk_manager.broadcast("정기 메시지")
            time.sleep(10)  # 10초마다 브로드캐스트 메시지 전송

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected, shutting down...")
    finally:
        kiosk_manager.shutdown()

if __name__ == '__main__':
    main()
