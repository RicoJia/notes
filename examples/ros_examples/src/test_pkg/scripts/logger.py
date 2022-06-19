#!/usr/bin/python3 

import coloredlogs
import git
import logging
import logging.config
import os
import sys
import traceback
import uuid
import yaml
import time 
from multiprocessing import Queue, Process
START=1
END=2
STOP=3
class LogTimerMultiProcess(object):
    def __init__(self, name, level, extra=None):
        """
        Args:
            name (str): The name of the event being logged, it will be appended with _start or _end
            level (int or str): a logging level from the logging module, e.g "DEBUG", 20, "ERROR"
            extra (dict): Other JSON fields to be logged with both start and end events
        """
        self.logger = logging.getLogger(__name__)
        self._level = level if isinstance(level, int) else logging.getLevelName(level)
        self._name = name
        self._started = False
        self._uuid = None
        self._extra = extra or {}
        self.queue = Queue()
        self.worker_proc = Process(target=self.__worker, args=(self.queue,))
        self.worker_proc.daemon = True
        self.worker_proc.start()
        time.sleep(1)


    def __worker(self, queue):
        while True: 
            message, extra, flag = queue.get()
            if flag == END:
                break
            elif flag == START:
                extra = extra or {}
                if message is None:
                    message = '%s started'

                # if we have already logged a start event, mark it as a duplicate and keep the same uuid
                if self._started:
                    extra['duplicate'] = True
                # otherwise create a new one
                else:
                    self._uuid = uuid.uuid4()
                    self._extra['event'] = self._name.lower() + '_start'
                    self._extra['timer_id'] = self._uuid
                extra.update(self._extra)
                self.logger.log(self._level, message, extra=extra)
                self._started = True
            elif flag == STOP:
                if message is None:
                    message = '%s stopped'
                extra = extra or {}
                if not self._started:
                    extra['duplicate'] = True
                self._extra['event'] = self._name.lower() + '_end'
                self._extra['timer_id'] = self._uuid
                extra.update(self._extra)
                self.logger.log(self._level, message, extra=extra)
                self._started = False
            # queue.task_done()

    def start(self, message=None, extra=None):
        self.queue.put((message, extra, START), block=False)
    def stop(self, message=None, extra=None):
        self.queue.put((message, extra, STOP), block=False)
    def __del__(self):
        # self.queue.close()
        # print("2")
        # self.queue.join_thread()
        # print("3")
        # self.queue.put((None, None, END), block=False)
        self.worker_proc.terminate()
        print("sdf")

if __name__ == "__main__":
    # time.sleep(10)
    # print("hehe")
    logger=LogTimerMultiProcess('TestTimer', 'DEBUG')
    # time.sleep(10)
    # print("hehe")
    # for i in range(10):
    #     logger.start()
    #     logger.stop()
    print("done")
    # logger is not destructed?
    logger.__del__()

