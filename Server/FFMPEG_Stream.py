import cv2
import subprocess
import multiprocessing
import time
from Server.ImageProvider import ImageProvider

class FFmpegStreamer:
    def __init__(self, width, height, host='192.168.178.24', port=6139):
        self.width = width
        self.height = height
        self.host = host
        self.port = port

        self.process = subprocess.Popen([
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', '60',
            '-i', '-',
            '-an',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-f', 'mpegts',
            f'udp://{self.host}:{self.port}'
        ], stdin=subprocess.PIPE)

    def send(self, frame):
        resized = cv2.resize(frame, (self.width, self.height))
        try:
            self.process.stdin.write(resized.tobytes())
        except BrokenPipeError:
            print("❌ FFmpeg Prozess hat sich beendet.")

def frame_sender(queue: multiprocessing.Queue):
    width, height = 1280, 720  # Ziel-Auflösung
    streamer = FFmpegStreamer(width, height)

    frame_count = 0
    start_time = time.time()

    while True:
        if not queue.empty():
            frame = queue.get()
            if frame is None:
                break

            streamer.send(frame)

            # FPS-Anzeige
            frame_count += 1
            if frame_count % 60 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"📡 Gesendete FPS: {fps:.2f}")

def main():
    queue = multiprocessing.Queue(maxsize=5)

    # Starte den ImageProvider, der Frames in die Queue legt
    image_provider = ImageProvider(queue)
    grabber_process = multiprocessing.Process(target=image_provider.grab)

    # Starte den Frame-Sender-Prozess
    sender_process = multiprocessing.Process(target=frame_sender, args=(queue,))

    grabber_process.start()
    sender_process.start()

    grabber_process.join()
    sender_process.join()

if __name__ == "__main__":
    main()
