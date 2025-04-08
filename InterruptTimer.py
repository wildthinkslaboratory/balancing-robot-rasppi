from threading import Timer
from time import perf_counter

class InterruptTimer:
    def __init__(self, dt, callback_function):
        self.dt = dt
        self.callback_function = callback_function
  
    def work(self, target_time):
        t0 = perf_counter()
        self.callback_function()
        Timer(target_time - t0, self.work, args=[target_time + self.dt]).start()

    def start(self):
        target_time = perf_counter() + self.dt
        self.work(target_time)

