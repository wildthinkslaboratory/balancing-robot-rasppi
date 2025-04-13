from threading import Timer
from time import perf_counter

class InterruptTimer:
    def __init__(self, dt, callback_function, timeout):
        self.dt = dt
        self.callback_function = callback_function
        self.timeout = perf_counter() + timeout
        self.running = False
  
    def work(self, target_time):
        t0 = perf_counter()
        self.callback_function()
        if self.timeout < t0:
            self.running = False
            return
        else: 
            Timer(target_time - t0, self.work, args=[target_time + self.dt]).start()

    def start(self):
        self.running = True
        target_time = perf_counter() + self.dt
        self.work(target_time)


########### test timer ############
def my_callback():
    print("called callback")

if __name__ == "__main__":
    timer = InterruptTimer(4, my_callback, 20)
    timer.start()