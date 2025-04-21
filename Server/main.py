import time
import cv2
import numpy as np
import mss
import multiprocessing
import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
from aiortc.rtcrtpsender import RTCRtpSender
from av import VideoFrame
import fractions
from datetime import datetime
from collections import deque

from Server.ImageProvider import ImageProvider







class VideoStream(VideoStreamTrack):
    def __init__(self, queue: multiprocessing.Queue):
        super().__init__()
        self.queue = queue
        self.frame_count = 0

    async def recv(self):
        # while not self.queue.empty():
        #     frame = self.queue.get()
        # img = frame
        img = self.queue.get()
        # img_rgb = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        img_bgr = cv2.cvtColor(img,cv2.COLOR_BGRA2BGR)
        # video_frame = VideoFrame.from_ndarray(img_rgb, format="rgb24")
        video_frame = VideoFrame.from_ndarray(img_bgr,format="bgr24")
        video_frame.pts = self.frame_count
        video_frame.time_base = fractions.Fraction(1, 60)  # FPS control
        # Increment frame count for time-based video
        self.frame_count += 1
        if(self.frame_count%60==0):
            print(f"Server ist bei Frame: {self.frame_count}")
        # cv2.imshow("source",video_frame)
        return video_frame

class WebRTC:
    def __init__(self,ip="localhost",port=6139):
        self.port = port
        self.ip = ip

    async def createServer(self,queue):
        signal = TcpSocketSignaling(self.ip,self.port)
        peer = RTCPeerConnection()
        # params = peer.getTransceivers()[0].sender.getParameters()
        # params.encodings[0].maxBitrate = 10_000_000
        # await peer.getTransceivers()[0].sender.setParameters(params)
        videoGen = VideoStream(queue)
        peer.addTrack(videoGen)


        sender = peer.getTransceivers()[0].sender
        params = sender.getParameters()

        if params.encodings:
            params.encodings[0].maxBitrate = 10_000_000  # 10 Mbps
            await sender.setParameters(params)
            print("Bitrate auf 10 Mbps gesetzt")


        await signal.connect()



        offer = await peer.createOffer()
        await peer.setLocalDescription(offer)
        await signal.send(peer.localDescription)


        while True:
            obj = await signal.receive()
            if isinstance(obj, RTCSessionDescription):
                await peer.setRemoteDescription(obj)
                print("Remote description set")
            elif obj is None:
                print("Signaling ended")
                break
        print("Closing connection")
        signal.close()




async def main():
    queue = multiprocessing.Queue(maxsize=1)
    ip_address = "192.168.178.24"
    port = 6139
    image_provider = ImageProvider(queue)
    process = multiprocessing.Process(target=image_provider.grab)
    process.start()

    webrtc = WebRTC(port=port,ip=ip_address)
    await webrtc.createServer(queue)
    # await setup_webrtc_and_run(ip_address,port,queue)
    process.join()



# class CustomVideoStreamTrack(VideoStreamTrack):
#     def __init__(self, queue: multiprocessing.Queue):
#         super().__init__()
#         self.queue = queue
#         self.frame_count = 0
#
#     async def recv(self):
#         img = self.queue.get()
#
#         # Convert to RGB format (needed for WebRTC)
#         img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#
#         # Prepare video frame for WebRTC
#         video_frame = VideoFrame.from_ndarray(img_rgb, format="rgb24")
#         video_frame.pts = self.frame_count
#         video_frame.time_base = fractions.Fraction(1, 60)  # FPS control
#
#         # Increment frame count for time-based video
#         self.frame_count += 1
#         if(self.frame_count%60==0):
#             print(f"Server ist bei Frame: {self.frame_count}")
#
#         return video_frame
#
#
# async def setup_webrtc_and_run(ip_address, port, queue):
#     signaling = TcpSocketSignaling(ip_address, port)
#     pc = RTCPeerConnection()
#     video_sender = VideoStreamTrack()
#     pc.addTrack(video_sender)
#     channel = pc.createDataChannel("keepalive")
#     async def send_keepalive():
#         while True:
#             await asyncio.sleep(10)
#             if channel.readyState == "open":
#                 channel.send("ping")
#                 print("Keep-Alive: Ping gesendet")
#
#     # asyncio.create_task(send_keepalive())
#     try:
#         await signaling.connect()
#
#         @pc.on("datachannel")
#         def on_datachannel(channel):
#             print(f"Data channel established: {channel.label}")
#             # @channel.on("message")
#             # def on_message(message):
#             #     print("Keep-Alive")
#
#
#         @pc.on("connectionstatechange")
#         async def on_connectionstatechange():
#             print(f"Connection state is {pc.connectionState}")
#             if pc.connectionState == "connected":
#                 print("WebRTC connection established successfully")
#
#         offer = await pc.createOffer()
#         await pc.setLocalDescription(offer)
#         await signaling.send(pc.localDescription)
#
#         while True:
#             obj = await signaling.receive()
#             if isinstance(obj, RTCSessionDescription):
#                 await pc.setRemoteDescription(obj)
#                 print("Remote description set")
#             elif obj is None:
#                 print("Signaling ended")
#                 break
#         print("Closing connection")
#     finally:
#         await pc.close()

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