from threading import Thread
import Queue

class AsyncJob(Thread):
    def __init__(self, name=None):
        Thread.__init__(self)
        self.name = name
        self.setDaemon(True)
        self.q = Queue.Queue()
        self.abort = False
        self.working = False
        self.start()

    def run(self):
        print "job thread " + str(self.name)
        while True:
            try:
                f = self.q.get(True, 2)
                self.working = True
                try:
                    f()
                except:
                    pass
                self.q.task_done()
            except Queue.Empty:
                self.working = False
                pass

    def addJob(self, f):
        self.q.put(f)      

    def waitJob(self):
        self.q.wait()
