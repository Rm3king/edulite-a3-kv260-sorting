"""Atomic whole-buffer evidence writing; called only after verified disable."""
import gzip
import json
import os
import time
from pathlib import Path


def write_report(report, destination):
    destination=Path(destination)
    data=json.dumps(report,separators=(',',':'),allow_nan=False).encode('utf-8')
    data=gzip.compress(data,compresslevel=3,mtime=0)
    temporary=destination.with_name(destination.name+'.tmp')
    with temporary.open('wb') as f:
        f.write(data);f.flush();os.fsync(f.fileno())
    os.replace(temporary,destination)


def archive_disabled(report,destination):
    start=time.monotonic()
    try:
        write_report(report,destination)
        return dict(evidence_saved=True,archive_seconds=time.monotonic()-start)
    except Exception as exc:
        # The physical endpoint/reset has already completed. Logging failure
        # is an evidence error, never an actuator fault or a second motion.
        return dict(evidence_saved=False,evidence_error=repr(exc),archive_seconds=time.monotonic()-start)


def _archive_child(report,destination,pipe):
    try:
        report.get('result_summary',{}).update(evidence_pending=False)
        pipe.send(archive_disabled(report,destination))
    finally:pipe.close()


class DeferredArchives:
    """Fork only AFTER hardware handles and the motion lock are closed.

    Child owns a copy-on-write snapshot. The live owner can answer STATUS while
    JSON/gzip/fsync runs; completion is a separate correlated durable event.
    """
    def __init__(self):self.jobs=[]
    def submit(self,task,report,destination):
        import multiprocessing as mp
        ctx=mp.get_context('fork');parent,child=ctx.Pipe(duplex=False)
        p=ctx.Process(target=_archive_child,args=(report,destination,child));p.start();child.close()
        self.jobs.append((task,p,parent))
    def poll(self):
        completed=[]
        for task,process,pipe in self.jobs[:]:
            if process.is_alive():continue
            process.join()
            try:value=pipe.recv() if pipe.poll() else dict(evidence_saved=False,evidence_error='ARCHIVE_PROCESS_EXIT_'+str(process.exitcode))
            except EOFError:value=dict(evidence_saved=False,evidence_error='ARCHIVE_PROCESS_PIPE_CLOSED')
            pipe.close();self.jobs.remove((task,process,pipe));completed.append((task,value))
        return completed
    def close(self):
        for _,process,_ in self.jobs:process.join(10)
        return self.poll()
