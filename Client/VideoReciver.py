import asyncio
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
import numpy as np
import time
import fractions
from av import VideoFrame
#
# class VideoReceiverTrack(VideoStreamTrack):
#     def __init__(self):
#         super().__init__()
#         self.frame_count = 0
#
#     async def recv(self):
#         # Hier erhalten wir die Videodaten vom WebRTC-Server
#         frame = await self._receive_frame()  # Asynchroner Abruf von Frame (von WebRTC)
#         img = frame.to_ndarray(format("rgb24"))
#         img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#
#         # Erstellen eines VideoFrames für WebRTC
#         video_frame = VideoFrame.from_ndarray(img_rgb, format="rgb24")
#         video_frame.pts = self.frame_count
#         video_frame.time_base = fractions.Fraction(1, 60)  # FPS Steuerung
#
#         self.frame_count += 1
#
#
#
#         return video_frame
#
#     async def _receive_frame(self):
#         frame = await asyncio.wait_for(timeout=1)
#         frame = frame.to_ndarray(format("rgb24"))
#         return frame
#         # await asyncio.sleep(0.03)  # Zeit, um einen neuen Frame zu empfangen (simuliert)
#         # return None  # In einer echten Anwendung würde hier der Frame vom Server kommen
#
# async def run_client(ip_address, port):
#     signaling = TcpSocketSignaling(ip_address, port)
#     pc = RTCPeerConnection()
#
#     # Füge den VideoReceiverTrack hinzu
#     video_receiver = VideoReceiverTrack()
#     pc.addTrack(video_receiver)
#
#     try:
#         # Verbindet sich mit dem Server über Signalisierung
#         await signaling.connect()
#
#         # Warten auf das Angebot des Servers (SDP Offer)
#         offer = await signaling.receive()
#         if isinstance(offer, RTCSessionDescription):
#             await pc.setRemoteDescription(offer)
#             print("Remote description set.")
#
#             # Erstelle Antwort (SDP Answer)
#             answer = await pc.createAnswer()
#             await pc.setLocalDescription(answer)
#             await signaling.send(pc.localDescription)
#             print("Answer sent.")
#
#         # Video empfangen und anzeigen
#         while True:
#             # Hier wird ein Video-Frame empfangen und angezeigt
#             video_frame = await video_receiver.recv()
#
#             # Convert Frame to OpenCV format (NumPy array)
#             img = video_frame.to_ndarray(format="rgb24")
#             cv2.imshow("Received Video", img)
#
#             if cv2.waitKey(1) & 0xFF == ord("q"):
#                 break
#
#     finally:
#         await pc.close()
#         cv2.destroyAllWindows()
#
# async def main():
#     ip_address = "192.168.178.24"  # IP-Adresse des Servers (WebRTC-Server)
#     port = 9999
#     await run_client(ip_address, port)
#
# if __name__ == "__main__":
#     asyncio.run(main())
