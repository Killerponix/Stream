import queue
import threading
import time
import cv2
import numpy as np
import mss
import multiprocessing
from collections import deque
import dxcam


class ImageProvider:
    def __init__(self, queue: multiprocessing.Queue):
        self.queue = queue
        # self.running = False
    # def __init__(self, shared_queue: queue.Queue):
    #     self.queue = shared_queue
    #     self.running = False

    def grab(self):
        """Screenshot-Grabbing-Funktion"""
        last_time = time.time()
        frames = 0
        with mss.mss() as sct:
            monitor = {"top": 0, "left": 0, "width": 1920, "height": 1080}
            while True:
                img = np.array(sct.grab(monitor))
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                    except:
                        pass
                self.queue.put(img)
                frames += 1
                if time.time() - last_time >= 1:
                    print(f"[GRAB] FPS: {frames}")
                    frames = 0
                    last_time = time.time()
                # cv2.imwrite("Server_frame.png",img)



    def grab_dx (self):
        self.running = True
        thread = threading.Thread(target=self.loop_dx, daemon=True)
        thread.start()

    def loop_dx(self):
        cam = dxcam.create(output_idx=0)
        cam.start(target_fps=60)
        while self.running:
            frame = cam.get_latest_frame()
            if frame is not None:
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                    except queue.Empty:
                        pass
                self.queue.put(frame)


    def provide(self):
        """Provide image from queue"""
        # cv2.imshow("provider",self.queue[-1])
        return self.queue[-1]


# if __name__ == "__main__":
#     queue = multiprocessing.Queue()
#
#     # Prozesse starten
#     p1 = multiprocessing.Process(target=grab, args=(queue,))
#     p2 = multiprocessing.Process(target=provieIMG, args=(queue,))
#
#     p1.start()
#     p2.start()
#
#     p1.join()
#     p2.join()
