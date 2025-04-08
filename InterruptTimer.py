import threading
import time

class InterruptTimer:
    def __init__(self, dt, callback_function):
        self.dt = dt
        self.callback_function = callback_function
  
    def work(self, target_time):
        t0 = time.perf_counter()
        self.callback_function()
        threading.Timer(target_time - t0, self.work, args=[target_time + self.dt]).start()

    def start(self):
        target_time = time.perf_counter() + self.dt
        self.work(target_time)


def interrupt():
    print(time.perf_counter())

myTimer = InterruptTimer(0.01, interrupt)
myTimer.start()

