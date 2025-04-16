import asyncio
import numpy as np
import multiprocessing
import socket
from socket import create_server
import struct
import time

import cv2
from av import VideoFrame

from Server.ImageProvider import ImageProvider



class Server_tcp:
    def __init__(self,port=6139,ip="localhost"):
        self.port=port
        self.ip =ip
        self.socket = None

    def createSocket(self):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #TCP
        # self.socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF, 2**20)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF, 2**20)
        return self.socket


    async def sendVideo(self,queue):
        self.socket= self.createSocket()
        # self.socket.create_server(self.ip,self.port)
        self.socket.connect((self.ip,self.port))
        vgen = VideoStream(queue)
        while True:
            frame = await vgen.recv()
            buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 100])[1]
            data = buffer.tobytes()
            self.socket.sendall(struct.pack("L",len(data)))
            self.socket.sendall(data)
            # cv2.imshow("source",frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # time.sleep(1/60)


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
        img = self.queue.get()
        # img_bgr = cv2.cvtColor(img,cv2.COLOR_BGRA2BGR)
        self.frame_count += 1
        if(self.frame_count%60==0):
            print(f"Server ist bei Frame: {self.frame_count}")
        return img



async def main():
    queue = multiprocessing.Queue(maxsize=1)
    ip_address = "192.168.178.27"
    port = 6139

    image_provider = ImageProvider(queue)
    process = multiprocessing.Process(target=image_provider.grab)
    server = Server_tcp(port=port, ip=ip_address)
    server.createSocket()
    # process_server = multiprocessing.Process(target=server.sendVideo(queue)) #Wrong?

    process_server = multiprocessing.Process(target=run_video_sender, args=(queue, ip_address, port))

    process.start()
    process_server.start()
    # client = await server.recVideo()


    process.join()
    process_server.join()


def run_video_sender(queue, ip, port):
    server = Server_tcp(port=port, ip=ip)
    asyncio.run(server.sendVideo(queue))



if __name__ == "__main__":
    asyncio.run(main())