from threading import Thread, RLock
import Queue

class AsyncJob(Thread):
    def __init__(self, name=None):
        Thread.__init__(self)
        self.name = name
        self.setDaemon(True)
        self.q = Queue.Queue()
        self.job_lock = RLock()
        self.abort = False
        self.working = False
        self.start()

    @property
    def working(self):
        with self.job_lock:
            return self._working
    
    @working.setter
    def working(self, val):
        with self.job_lock:
            self._working = val
        
    @property
    def abort(self):
        with self.job_lock:
            return self._abort
    
    @abort.setter
    def abort(self, val):
        with self.job_lock:
            self._abort = val

    def run(self):
        print "job thread " + str(self.name)
        while True:
            try:
                f = self.q.get(True, 0.33)
                try:
                    if self.abort == False:
                        f()
                except:
                    pass
                self.q.task_done()
            except Queue.Empty:
                with self.job_lock:
                    if self.q.empty():
                        self._working = False
                pass

    def addJob(self, f):
        with self.job_lock:
            self._working= True
            self.q.put(f)    

    def waitJob(self):
        self.q.join()
        self.abort = False
