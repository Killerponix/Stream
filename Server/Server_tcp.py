import asyncio
from multiprocessing import Queue

import numpy as np
import multiprocessing
import socket
from socket import create_server
import struct
import time
import subprocess
import numpy as np
import cv2
from av import VideoFrame

from Server.ImageProvider import ImageProvider

# MAX_DGRAM = 65507  # Max UDP payload
# HEADER_SIZE = 4  # für Länge, Sequenz, etc. optional



class Server_tcp:


    def __init__(self,port=6139,ip="localhost"):
        self.port=port
        self.ip =ip
        self.socket = None

    def createSocket(self):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
        # self.socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        # self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF, 2**8)
        # self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF, 2**8)
        return self.socket

# Für TCP eigenes
#     async def sendVideo(self,queue):
#         self.socket= self.createSocket()
#         # self.socket.create_server(self.ip,self.port)
#         self.socket.connect((self.ip,self.port))
#         vgen = VideoStream(queue)
#         frame_counter = 0
#         last_time = time.time()
#         while True:
#             frame = await vgen.recv()
#             start = time.time()
#             buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])[1]
#             data = buffer.tobytes()
#             self.socket.sendall(struct.pack("L",len(data)))
#             self.socket.sendall(data)
#             # cv2.imshow("source",frame)
#                 # FPS zählen
#             frame_counter += 1
#             current_time = time.time()
#             if current_time - last_time >= 1.0:
#                 print(f"[SEND] FPS: {frame_counter} | Frame size: {len(data)//1024} KB")
#                 frame_counter = 0
#                 last_time = current_time
#
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#             # time.sleep(1/60)

#####Nur Senden
    async def sendVideo(self, queue_encoded):
        self.socket = self.createSocket()
        self.socket.connect((self.ip, self.port))

        frame_counter = 0
        last_time = time.time()

        while True:
            data = queue_encoded.get()  # Already JPEG-encoded

            self.socket.sendall(struct.pack("L", len(data)))
            self.socket.sendall(data)

            frame_counter += 1
            current_time = time.time()
            if current_time - last_time >= 1.0:
                print(f"[SEND] FPS: {frame_counter} | Size: {len(data) // 1024} KB")
                frame_counter = 0
                last_time = current_time

            await asyncio.sleep(0)





#UDP?
    # async def sendVideo(self, queue):
    #     self.socket = self.createSocket()
    #     vgen = VideoStream(queue)
    #     frame_counter = 0
    #     last_time = time.time()
    #     while True:
    #         frame = await vgen.recv()
    #         _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
    #         data = buffer.tobytes()
    #
    #         if len(data) > MAX_DGRAM:
    #             # print(f"⚠️ Frame too large for one datagram ({len(data)} bytes) – splitting...")
    #             # Frame splitten
    #             for i in range(0, len(data), MAX_DGRAM - HEADER_SIZE):
    #                 chunk = data[i:i + MAX_DGRAM - HEADER_SIZE]
    #                 self.socket.sendto(chunk, (self.ip, self.port))
    #         else:
    #             self.socket.sendto(data, (self.ip, self.port))
    #         # FPS zählen
    #         frame_counter += 1
    #         current_time = time.time()
    #         if current_time - last_time >= 1.0:
    #             print(f"[SEND] FPS: {frame_counter} | Frame size: {len(data)//1024} KB")
    #             frame_counter = 0
    #             last_time = current_time
    #         await asyncio.sleep(0)  # Kein Sleep für max FPS



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
            img_array = np.frombuffer(img_data, dtype=np.uint8)
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
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data


class VideoStream():
    def __init__(self, queue: multiprocessing.Queue):
        super().__init__()
        self.queue = queue
        self.frame_count = 0

    async def recv(self):
        # img = self.queue.get()
        loop = asyncio.get_running_loop()
        img = await loop.run_in_executor(None, self.queue.get)

# img_bgr = cv2.cvtColor(img,cv2.COLOR_BGRA2BGR)
        self.frame_count += 1
        if(self.frame_count%60==0):
            print(f"Server ist bei Frame: {self.frame_count}")
        return img


def encode_worker(input_q: Queue, output_q: Queue):
    while True:
        frame = input_q.get()
        if frame is None:
            break
        # frame = cv2.UMat(frame)
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        output_q.put(buffer)

def run_video_sender(queue_encoded, ip, port):
    server = Server_tcp(port=port, ip=ip)
    asyncio.run(server.sendVideo(queue_encoded))

async def main():
    queue = multiprocessing.Queue(maxsize=1)
    queue_raw = multiprocessing.Queue(maxsize=3)
    queue_encoded = multiprocessing.Queue(maxsize=3)
    ip_address = "192.168.178.27"
    port = 6139

    # image_provider = ImageProvider(queue)
    # process = multiprocessing.Process(target=image_provider.grab)
    server = Server_tcp(port=port, ip=ip_address)
    server.createSocket()

    # process_server = multiprocessing.Process(target=server.sendVideo(queue)) #Wrong?
    # process_server = multiprocessing.Process(target=run_video_sender, args=(queue, ip_address, port))
    #
    # process.start()
    # process_server.start()

    process_grabber = multiprocessing.Process(target=ImageProvider(queue_raw).grab_opt)   #grab_dx() oder grab
    process_encoder = multiprocessing.Process(target=encode_worker, args=(queue_raw, queue_encoded))
    process_sender = multiprocessing.Process(target=run_video_sender, args=(queue_encoded, ip_address, port))

    # Start
    process_grabber.start()
    process_encoder.start()
    process_sender.start()


    process_grabber.join()
    process_encoder.join()
    process_sender.join()

    # process.join()
    # process_server.join()


    # client = await server.recVideo()


def run_video_sender(queue, ip, port):
    server = Server_tcp(port=port, ip=ip)
    asyncio.run(server.sendVideo(queue))



if __name__ == "__main__":
    asyncio.run(main())