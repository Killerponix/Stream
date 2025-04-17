import asyncio
import base64
import struct
import socket
import time
import subprocess
import numpy as np
import multiprocessing
import cv2
from Server.ImageProvider import ImageProvider

class Server_tcp:
    def __init__(self, port=6139, ip="localhost"):
        self.port = port
        self.ip = ip
        self.socket = None

    def createSocket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**16)  # Puffergröße anpassen
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2**16)  # Puffergröße anpassen
        return self.socket

    async def sendVideo(self, queue_encoded):
        self.socket = self.createSocket()
        self.socket.connect((self.ip, self.port))

        frame_counter = 0
        last_time = time.time()

        while True:
            data = queue_encoded.get()  # Already JPEG-encoded
            self.socket.sendall(struct.pack("L", len(data)))  # Sende die Größe des Frames
            self.socket.sendall(data)  # Sende das Bild selbst

            frame_counter += 1
            current_time = time.time()
            if current_time - last_time >= 1.0:
                print(f"[SEND] FPS: {frame_counter} | Size: {len(data) // 1024} KB")
                frame_counter = 0
                last_time = current_time

            await asyncio.sleep(0)


    async def recVideo(self):
        self.socket = self.createSocket()
        self.socket.bind((self.ip, self.port))
        self.socket.listen(1)
        conn, addr = self.socket.accept()
        print(f"Connection from {addr}")

        while True:
            length_data = recv_exact(conn, 4)
            if not length_data:
                break
            length = struct.unpack("L", length_data)[0]
            img_data = recv_exact(conn, length)
            raw_img = base64.b64decode(img_data)
            img_array = np.frombuffer(raw_img,dtype=np.uint8)
            # img_array = np.frombuffer(img_data, dtype=np.uint8)

            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            cv2.imshow("Client", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



#UDP
# async def recVideo(self):
#     self.socket = self.createSocket()
#     self.socket.bind((self.ip, self.port))
#     print(f"Listening on {self.ip}:{self.port} via UDP")
#
#     while True:
#         data, _ = self.socket.recvfrom(65507)
#         img_array = np.frombuffer(data, dtype=np.uint8)
#         frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
#
#         if frame is not None:
#             cv2.imshow("Client", frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break


def recv_exact(sock, size):
    data = b""
    while len(data) < size:
        # packet = sock.recv(size - len(data))
        packet = sock.recv_string(size-len(data))
        if not packet:
            return None
        data += packet
    return data

# Worker für das Encoding mit FFmpeg
def encode_worker(input_q: multiprocessing.Queue, output_q: multiprocessing.Queue):
    while True:
        frame = input_q.get()
        if frame is None:
            break
######### Grayscale it to reduce data
        # H.264 Encoding mit FFmpeg und AMF (AMD Encoder)
        # Frame in JPEG kodieren
        encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        img = base64.b64encode(buffer)   #Beim client noch decodieren
        output_q.put(img)  # Das kodierte Bild an die nächste Queue weitergeben

# Video-Stream für das Senden
def run_video_sender(queue_encoded, ip, port):
    server = Server_tcp(port=port, ip=ip)
    asyncio.run(server.sendVideo(queue_encoded))

async def main():
    queue_raw = multiprocessing.Queue(maxsize=3)  # Unkomprimierte Frames
    queue_encoded = multiprocessing.Queue(maxsize=3)  # Komprimierte Frames

    ip_address = "192.168.178.27"
    port = 6139

    # Bild-Grabbing-Prozess
    process_grabber = multiprocessing.Process(target=ImageProvider(queue_raw).grab_opt)

    # Encoding-Prozess
    process_encoder = multiprocessing.Process(target=encode_worker, args=(queue_raw, queue_encoded))

    # Sender-Prozess
    process_sender = multiprocessing.Process(target=run_video_sender, args=(queue_encoded, ip_address, port))

    # Prozesse starten
    process_grabber.start()
    process_encoder.start()
    process_sender.start()

    process_grabber.join()
    process_encoder.join()
    process_sender.join()

if __name__ == "__main__":
    asyncio.run(main())
