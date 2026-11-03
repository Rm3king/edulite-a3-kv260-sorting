"""Warm planning workers: protocol, failure reporting, replacement of a stuck or dead worker, pool exclusivity."""
import sys,threading,time,unittest
from workcell.plan_service import PlanService,PlanPool,ServiceError

SERVER=r'''
import json,sys,time
print(json.dumps(dict(ready="fake")),flush=True)
for line in sys.stdin:
    r=json.loads(line)
    if r["op"]=="echo":reply=dict(id=r["id"],ok=True,result=r["args"])
    elif r["op"]=="fail":reply=dict(id=r["id"],ok=False,error="ValueError('boom')")
    elif r["op"]=="slow":time.sleep(30);continue
    elif r["op"]=="die":sys.exit(3)
    else:reply=dict(id=r["id"],ok=True,result=None)
    print(json.dumps(reply),flush=True)
'''
COMMAND=[sys.executable,'-u','-c',SERVER]


class Service(unittest.TestCase):
    def make(self):
        service=PlanService('sdk','unused',[],command=COMMAND);self.addCleanup(service.close);return service

    def test_round_trip_and_error_reporting(self):
        s=self.make()
        self.assertEqual(s.call('echo',['a',1],5),['a','1'])
        with self.assertRaisesRegex(ServiceError,'boom'):s.call('fail',[],5)
        self.assertEqual(s.call('echo',['again'],5),['again'])  # a reported error does not poison the worker

    def test_stuck_worker_is_killed_and_replaced(self):
        s=self.make();first=s.process.pid
        with self.assertRaisesRegex(ServiceError,'timed out'):s.call('slow',[],.5)
        self.assertEqual(s.call('echo',['ok'],5),['ok']);self.assertNotEqual(s.process.pid,first)

    def test_dead_worker_is_replaced_on_the_next_call(self):
        s=self.make()
        with self.assertRaises(ServiceError):s.call('die',[],5)
        self.assertEqual(s.call('echo',['x'],5),['x'])


class Pool(unittest.TestCase):
    def test_pairs_are_exclusive_and_returned(self):
        pool=PlanPool.__new__(PlanPool);pool.free=[('a1','a2'),('b1','b2')];pool.all=list(pool.free);pool.condition=threading.Condition()
        one=pool.acquire();two=pool.acquire();self.assertNotEqual(one,two)
        with self.assertRaisesRegex(ServiceError,'no free'):pool.acquire(timeout=.1)
        threading.Timer(.1,lambda:pool.release(one)).start();self.assertEqual(pool.acquire(timeout=2),one)


if __name__=='__main__':unittest.main()
