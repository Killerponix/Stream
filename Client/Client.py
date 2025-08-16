import asyncio
import fractions

import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
from av import VideoFrame
from datetime import datetime, timedelta
from collections import deque

# Frame-Puffer (immer nur letztes Frame)
frame_queue = deque(maxlen=1)

class VideoReceiver:
    def __init__(self, track=None):
        self.track = track
        self.running = True

    async def handle_track(self, track):
        """
        Empfang und Anzeige von Videoframes.
        Bricht ab, wenn die Verbindung oder der Track endet.
        """
        global frame_queue
        print("Inside handle_track")
        self.track = track
        frame_count = 0

        while self.running:
            # Verbindung tot? -> Schleife beenden
            if self.track.readyState == "ended":
                print("Track ended — exiting handle_track")
                break

            try:
                frame = await asyncio.wait_for(track.recv(), timeout=10)
                frame_queue.append(frame)
                latest_frame = frame_queue[-1]
                latest_frame.time_base = fractions.Fraction(1, 60)
                frame_count += 1

                if frame_count % 60 == 0:
                    print(f"Received frame {frame_count}")

                if isinstance(latest_frame, VideoFrame):
                    img = latest_frame.to_ndarray(format="bgr24")
                elif isinstance(latest_frame, np.ndarray):
                    img = latest_frame
                else:
                    print(f"Unexpected frame type: {type(latest_frame)}")
                    continue

                cv2.imshow("Frame", img)

                # Exit on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Manual quit detected")
                    self.running = False
                    break

            except asyncio.TimeoutError:
                print("Timeout waiting for frame, continuing...")
            except Exception as e:
                print(f"Error in handle_track: {e}")
                traceback.print_exc()
                break

        print("Exiting handle_track")


async def run(pc, signaling):
    """
    Baut die WebRTC-Verbindung auf, verwaltet Keep-Alive und startet den Frame-Empfang.
    """
    await signaling.connect()

    # Keep-Alive-Channel erstellen
    keepalive_channel = pc.createDataChannel("keepalive")

    async def send_keepalive():
        """Sende alle 10 Sekunden einen Ping, um die Verbindung aktiv zu halten."""
        while True:
            await asyncio.sleep(10)
            if keepalive_channel.readyState == "open":
                keepalive_channel.send("ping")
                print("Keep-Alive: Ping gesendet")
            else:
                break

    asyncio.create_task(send_keepalive())

    @keepalive_channel.on("message")
    def on_keepalive_message(message):
        if message == "ping":
            keepalive_channel.send("pong")
            print("Keep-Alive: Pong gesendet")

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
        elif pc.connectionState in ["disconnected", "failed", "closed"]:
            print("Connection lost! Cleaning up...")
            await pc.close()

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
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
    await pc.setLocalDescription(answer)
    print("Local description set")
    await signaling.send(pc.localDescription)
    print("Answer sent to sender")

    print("Waiting for connection to be established...")
    while pc.connectionState != "connected":
        await asyncio.sleep(0.1)
        if pc.connectionState in ["failed", "closed"]:
            print("Connection failed before being established.")
            return

    print("Connection established, waiting for frames...")

    # Hier warten wir, bis Verbindung endet
    while pc.connectionState == "connected":
        await asyncio.sleep(1)

    print("Connection ended, closing...")


async def main():
    global video_receiver
    reconnect_delay = 5  # Sekunden warten vor Reconnect

    while True:
        signaling = TcpSocketSignaling("192.168.178.24", 9999)
        pc = RTCPeerConnection()
        video_receiver = VideoReceiver()

        try:
            await run(pc, signaling)
        except Exception as e:
            print(f"Error in main: {e}")
            traceback.print_exc()
        finally:
            print("Closing peer connection")
            await pc.close()
            cv2.destroyAllWindows()

        print(f"Reconnecting in {reconnect_delay} seconds...")
        await asyncio.sleep(reconnect_delay)


if __name__ == "__main__":
    asyncio.run(main())
