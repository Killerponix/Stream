import asyncio
import fractions

import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
from av import VideoFrame
from datetime import datetime, timedelta
from collections import deque
frame_queue = deque(maxlen=1)
class VideoReceiver:

    def __init__(self,track=None):
        self.track = track


    async def recv(self):
            # Hier erhalten wir die Videodaten vom WebRTC-Server
            # while True:
            #     try:
            #         frame = self.track._queue.get_nowait()  # Holt das neueste Frame ohne zu blockieren
            #     except asyncio.QueueEmpty:
            #         await asyncio.sleep(0.001)
            #         continue
            #     break
            frame = asyncio.wait_for(self.track.recv())
            img = frame.to_ndarray(format("bgr24"))
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # Erstellen eines VideoFrames für WebRTC
            video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
            # video_frame.pts = self.frame_count
            video_frame.time_base = fractions.Fraction(1, 60)  # FPS Steuerung
            # self.frame_count += 1
            return video_frame

    async def handle_track(self, track):
        global frame_queue
        print("Inside handle track")
        self.track = track
        frame_count = 0
        while True:
            try:
                # # print("Waiting for frame...")
                frame = await asyncio.wait_for(track.recv(), timeout=10)
                # while True:
                #     try:
                #         frame = track._queue.get_nowait()  # Holt das neueste Frame ohne zu blockieren
                #     except asyncio.QueueEmpty:
                #         await asyncio.sleep(0.001)
                #         continue
                #     break
                frame_queue.append(frame)
                latest_frame = frame_queue[-1]
                latest_frame.time_base = fractions.Fraction(1,60)
                # frame = self.recv()
                frame_count += 1
                if (frame_count%60==0):
                    print(f"Received frame {frame_count}")

                if isinstance(frame, VideoFrame):
                    # print(f"Frame type: VideoFrame, pts: {frame.pts}, time_base: {frame.time_base}")
                    frame = latest_frame.to_ndarray(format="bgr24")
                elif isinstance(frame, np.ndarray):
                    print(f"Frame type: numpy array")
                else:
                    print(f"Unexpected frame type: {type(frame)}")
                    continue

                # Add timestamp to the frame
                # current_time = datetime.now()
                # new_time = current_time - timedelta( seconds=55)
                # timestamp = new_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                # cv2.putText(frame, timestamp, (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                # cv2.imwrite(f"imgs/received_frame_{frame_count}.jpg", frame)
                # print(f"Saved frame {frame_count} to file")
                # video_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cv2.imshow("Frame", frame)
                # cv2.imwrite("Client_frame.png",frame)

                # Exit on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except asyncio.TimeoutError:
                print("Timeout waiting for frame, continuing...")
            except Exception as e:
                print(f"Error in handle_track: {str(e)}")
                if "Connection" in str(e):
                    break
        print("Exiting handle_track")
async def run(pc, signaling):
    await signaling.connect()

    @pc.on("track")
    def on_track(track):
        if isinstance(track, MediaStreamTrack):
            print(f"Receiving {track.kind} track")
            asyncio.ensure_future(video_receiver.handle_track(track))


    @pc.on("datachannel")
    def on_datachannel(channel):
        print(f"Data channel established: {channel.label}")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state is {pc.connectionState}")
        if pc.connectionState == "connected":
            print("WebRTC connection established successfully")
        if pc.connectionState in ["disconnected", "failed"]:
            print("Connection lost! Attempting to restart...")
            await pc.close()
            await main()  # Verbindung neu aufbauen
    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        # print(f"ICE connection state: {pc.iceConnectionState}")
        if pc.iceConnectionState == "failed":
            print("ICE connection failed!")
        elif pc.iceConnectionState == "connected":
            print("ICE connection established.")


    print("Waiting for offer from sender...")
    offer = await signaling.receive()
    print("Offer received")
    await pc.setRemoteDescription(offer)
    print("Remote description set")

    # Antwort erstellen
    answer = await pc.createAnswer()


    print("Answer created")
    # await pc.setLocalDescription(modified)
    # await pc.setLocalDescription(patched_answer)
    await pc.setLocalDescription(answer)
    print("Local description set")
    # await signaling.send(pc.localDescription)
    # await signaling.send(modified)
    await signaling.send(pc.localDescription)
    print("Answer sent to sender")

    print("Waiting for connection to be established...")
    while pc.connectionState != "connected":
        await asyncio.sleep(0.1)

        print(f"Current state: {pc.connectionState}")

    print("Connection established, waiting for frames...")
    await asyncio.sleep(100)  # Wait for 35 seconds to receive frames

    print("Closing connection")

async def main():
    signaling = TcpSocketSignaling("192.168.178.24", 6139)
    # signaling = TcpSocketSignaling("127.0.0.1", 6139)
    pc = RTCPeerConnection()

    global video_receiver
    video_receiver = VideoReceiver()

    try:
        await run(pc, signaling)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        print("Closing peer connection")
        # await pc.close()

if __name__ == "__main__":
    asyncio.run(main())