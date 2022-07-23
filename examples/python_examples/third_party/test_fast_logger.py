from fastlogging import LogInit
import time
import sys
import copy
import orjson, datetime, numpy
import rapidjson
RUNS = 1000


from diligent.diligent_logging import DiligentLogFilter

class Record:
    node_name=None
    git=None

# modified source code to omit time logging
class TestFastLogger(object):
    def __init__(self):
        self.logger = LogInit(pathName="/var/log/diligent/rico_test.log", maxSize=0, backupCnt=10, console=False, colors=False)
            
        def _excepthook(exc_type, exc_value, exc_traceback):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            exc_info=str(exc_type) + str(exc_value) + str(exc_traceback)
            self.logger.exception('Uncaught exception!' + exc_info)
        sys.excepthook = _excepthook
        with open("/var/log/diligent/fake_json", 'r') as f:
            self.data = rapidjson.load(f)

        self.diligent_log_filter = DiligentLogFilter("rico_logger")
        record = Record()
        self.diligent_log_filter.filter(record)
        print("record: ", record.node_name, " git: ", record.git)

    def test_fast_logger(self):
        for _ in range(RUNS):
            
            data = copy.deepcopy(self.data)
            # data = orjson.dumps(copy.deepcopy(self.data))
            
            self.logger.info(self.data)
        # raise RuntimeError("rico error")

tfl = TestFastLogger()
tfl.test_fast_logger()
tfl.logger.shutdown()


