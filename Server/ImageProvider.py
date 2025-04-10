import time
import cv2
import numpy as np
import mss
import multiprocessing
from collections import deque


class ImageProvider:
    def __init__(self, queue: multiprocessing.Queue):
        self.queue = queue

    def grab(self):
        """Screenshot-Grabbing-Funktion"""
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

    def provide(self):
        """Provide image from queue"""
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
