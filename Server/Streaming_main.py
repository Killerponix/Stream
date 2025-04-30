import asyncio
import base64
import os
import struct
import socket
import time
import subprocess
import av
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
        self.socket.settimeout(5.0)
        return self.socket

    async def sendVideo(self, queue_encoded):
        self.socket = self.createSocket()
        self.socket.connect((self.ip, self.port))
        frame_counter = 0
        last_time = time.time()
        print(self.socket.getsockname())
        while True:
            # print("Vor DATA get encoded")
            data = queue_encoded.get(timeout=5)  #Bereits encodede Datei
            # print("nach get Encoded"+data)
            self.socket.sendall(struct.pack("L", len(data)))  # Sende die Größe des Frames
            self.socket.sendall(data)  # Sende das Bild selbst

            frame_counter += 1
            current_time = time.time()
            if current_time - last_time >= 1.0:
                # print(f"[SEND] FPS: {frame_counter} | Size: {len(data) // 1024} KB")
                frame_counter = 0
                last_time = current_time

            await asyncio.sleep(0)


    def recVideo(self):
        self.socket = self.createSocket()
        self.socket.bind((self.ip, self.port))
        self.socket.listen(1)
        conn, addr = self.socket.accept()
        print(f"Connection from {addr}")

        while True:
            length_data = recv_exact(conn, 4)
            # print(length_data)
            if not length_data:
                break
            length = struct.unpack("L", length_data)[0]
            img_data = recv_exact(conn, length)
            # ####h264 decode
            # # container = av.open(socket.makefile('rb'))
            # container = av.open(img_data, format="h264",mode='r')
            #
            # for frame in container.decode(video=0):
            #     img = img_data.to_ndarray(format="bgr24")
            #     cv2.imshow("Video", img)
            #     if cv2.waitKey(1) & 0xFF == ord('q'):
            #         break

            ###### Base 64 decode
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
    # print(f"[RECV] Size: {size}")
    try:
        while len(data) < size:
            packet = sock.recv(size - len(data))
            # print(f"[RECV] Packet: {packet}")
            # packet = sock.recv_string(size-len(data))
            if not packet:
                return None
            data += packet
    except sock.timeout:
        print(f"[WARN] recv_exact timed out while waiting for {size} bytes.")
    return data




# Worker für das Encoding mit FFmpeg
def encode_worker(input_q: multiprocessing.Queue, output_q: multiprocessing.Queue):
    # print(f"[WORKER] Input Queue: {input_q.get()}")

    ffmpeg = subprocess.Popen([
        'ffmpeg',
        '-loglevel', 'error',
        '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', '1920x1080',
        '-r', '30',
        '-i', '-',
        '-c:v', 'h264_amf',
        '-g', '1',  # Keyframe für jedes Frame
        '-b:v', '2M',
        '-f', 'mpegts',
        'out.ts'
        # 'pipe:1'
    ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE)

    while True:
        frame = input_q.get()
        if frame is None:
            break
        # if frame.shape[2] == 4:
        #     frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        # print(frame.shape)
######### Grayscale it to reduce data
        # ffmpeg.stdin.write(frame.tobytes())
        # print("nachWrite")
        # ffmpeg.stdin.flush()
        # print("Nach FLush")
        # try:
        #     h264_data = ffmpeg.stdout.read()
        #     if h264_data:
        #         output_q.put(h264_data)
        # except BlockingIOError:
        #     pass  # Kein Output gerade, aber Programm blockiert nicht
        # print("h264Data: "+h264_data)
        # output_q.put(h264_data)

###Base64 Encode oder Grayscale noch probieren
        encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        img = base64.b64encode(buffer)   #Beim client noch decodieren
        output_q.put(img)  # Das kodierte Bild an die nächste Queue weitergeben

# Video-Stream für das Senden
def run_video_sender(queue_encoded, ip, port):
    server = Server_tcp(port=port, ip=ip)
    print("Server started")
    asyncio.run(server.sendVideo(queue_encoded))

async def main():
    queue_raw = multiprocessing.Queue(maxsize=3)  # Unkomprimierte Frames
    queue_encoded = multiprocessing.Queue(maxsize=3)  # Komprimierte Frames

    # ip_address = "192.168.178.27"
    # ip_address ="192.168.179.18"
    # ip_address = "127.0.0.1"
    ip_address = "10.20.40.12"
    port = 6139
    client = Server_tcp(ip=ip_address, port=port)
    process_client = multiprocessing.Process(target=client.recVideo)
    # process_client = multiprocessing.Process(target=asyncio.run(client.recVideo()))
    process_client.start()
    time.sleep(2)




    # client = Server_tcp.recVideo()
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

    process_client.join()
    process_grabber.join()
    process_encoder.join()
    process_sender.join()



if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')
    asyncio.run(main())
