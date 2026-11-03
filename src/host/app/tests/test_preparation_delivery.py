"""Large result decoding and dead compiler must not block owner polling."""
import queue
import threading
import time
import unittest
from screwcell.preparation import NumericalPreparation

class PreparationDeliveryTests(unittest.TestCase):
    def relay(self,source,process):
        job=NumericalPreparation.__new__(NumericalPreparation)
        job.results=source;job.process=process;job.done=False;job.received=queue.Queue()
        job.reader=threading.Thread(target=job._receive,daemon=True);job.reader.start()
        return job
    def test_slow_unpickle_and_child_join_leave_status_poll_available(self):
        decoding=threading.Event();release=threading.Event();joining=threading.Event();finish=threading.Event()
        class Source:
            def get(self,timeout):
                decoding.set();release.wait(3);return dict(result={'verified':True})
            def close(self):pass
        class Process:
            def join(self,timeout):joining.set();finish.wait(3)
        job=self.relay(Source(),Process())
        try:
            self.assertTrue(decoding.wait(1))
            self.assertIsNone(job.poll(.001));self.assertFalse(job.done)
            release.set();self.assertTrue(joining.wait(1))
            self.assertIsNone(job.poll(.001));finish.set()
            self.assertEqual(job.poll(1),dict(result={'verified':True}));self.assertTrue(job.done)
        finally:release.set();finish.set();job.reader.join(1)
    def test_exited_compiler_is_reported_instead_of_poll_hanging(self):
        class Source:
            def get(self,timeout):raise queue.Empty
        class Process:
            def is_alive(self):return False
        job=self.relay(Source(),Process());value=job.poll(1)
        self.assertIn('NUMERICAL_PREPARATION_PROCESS_EXITED',value['error']);self.assertTrue(job.done)
        job.reader.join(1)
