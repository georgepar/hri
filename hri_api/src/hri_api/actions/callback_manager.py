from functools import partial
from multiprocessing.dummy import Pool
pool = Pool(processes=1)

class CallbackManager():

    def __init__(self):
        self.lock = threading.RLock()
        self.callback_list = []

    def add_callback(self, callback):
        with self.lock:
            self.callback_list.append(callback)

    def remove_callback(self, callback):
        with self.lock:
            self.callback_list.remove(callback)

    def execute(self, *args, **kwargs):
        with self.lock:
            for callback in self.callback_list:
                pool.apply_async()
                pool.apply_async(callback, args, kwargs)

        pool.close()
        pool.join()

