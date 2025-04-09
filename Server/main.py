import time
import cv2
import numpy as np
import mss
import multiprocessing
import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
from av import VideoFrame
import fractions
from datetime import datetime

from Server.ImageProvider import ImageProvider





class CustomVideoStreamTrack(VideoStreamTrack):
    def __init__(self, queue: multiprocessing.Queue):
        super().__init__()
        self.queue = queue
        self.frame_count = 0

    async def recv(self):
        img = self.queue.get()

        # Convert to RGB format (needed for WebRTC)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Prepare video frame for WebRTC
        video_frame = VideoFrame.from_ndarray(img_rgb, format="rgb24")
        video_frame.pts = self.frame_count
        video_frame.time_base = fractions.Fraction(1, 60)  # FPS control

        # Increment frame count for time-based video
        self.frame_count += 1
        if(self.frame_count%60==0):
            print(f"Server ist bei Frame: {self.frame_count}")

        return video_frame


async def setup_webrtc_and_run(ip_address, port, queue):
    signaling = TcpSocketSignaling(ip_address, port)
    pc = RTCPeerConnection()
    video_sender = CustomVideoStreamTrack(queue)
    pc.addTrack(video_sender)
    channel = pc.createDataChannel("keepalive")
    async def send_keepalive():
        while True:
            await asyncio.sleep(10)
            if channel.readyState == "open":
                channel.send("ping")
                print("Keep-Alive: Ping gesendet")

    # asyncio.create_task(send_keepalive())
    try:
        await signaling.connect()

        @pc.on("datachannel")
        def on_datachannel(channel):
            print(f"Data channel established: {channel.label}")
            # @channel.on("message")
            # def on_message(message):
            #     print("Keep-Alive")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"Connection state is {pc.connectionState}")
            if pc.connectionState == "connected":
                print("WebRTC connection established successfully")

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await signaling.send(pc.localDescription)

        while True:
            obj = await signaling.receive()
            if isinstance(obj, RTCSessionDescription):
                await pc.setRemoteDescription(obj)
                print("Remote description set")
            elif obj is None:
                print("Signaling ended")
                break
        print("Closing connection")
    finally:
        await pc.close()

class VideoStream(VideoStreamTrack):
    def __init__(self, queue: multiprocessing.Queue):
        super().__init__()
        self.queue = queue
        self.frame_count = 0

async def recv(self):
    img = self.queue.get()
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    video_frame = VideoFrame.from_ndarray(img_rgb, format="rgb24")
    video_frame.pts = self.frame_count
    video_frame.time_base = fractions.Fraction(1, 60)  # FPS control
    # Increment frame count for time-based video
    self.frame_count += 1
    if(self.frame_count%60==0):
        print(f"Server ist bei Frame: {self.frame_count}")
    return video_frame

class WebRTC:
    def __init__(self,ip="localhost",port=8080):
        self.port = port
        self.ip = ip

    def createServer(self,queue):
        signal = TcpSocketSignaling(self.ip,self.port)
        peer = RTCPeerConnection()
        videoGen = VideoStream(queue)
        peer.addTrack(videoGen)






async def main():
    queue = multiprocessing.Queue()
    ip_address = "192.168.178.24"
    port = 6139
    image_provider = ImageProvider(queue)
    process = multiprocessing.Process(target=image_provider.grab)
    process.start()

    webrtc = WebRTC(ip_address,port)
    await webrtc.createServer(queue)

    process.join()

# ////-------------------------- NEW MAIN ------------------------------- //////#
#     queue = multiprocessing.Queue()
#     ip_address = "192.168.178.24"  # IP Address of Remote Server/Machine
#     port = 9999
#
#     # Start image grabbing process
#     image_provider = ImageProvider(queue)
#     process = multiprocessing.Process(target=image_provider.grab)
#     process.start()
#
#     await setup_webrtc_and_run(ip_address, port, queue)
#
#     process.join()  # Ensure the image grabbing process continues until termination

if __name__ == "__main__":
    asyncio.run(main())