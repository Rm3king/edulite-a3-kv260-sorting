#!/usr/bin/env python3
"""Recreate the KV260 build workspace from the submission's verified source bundle."""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tarfile
import tempfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_directory():
    for parent in Path(__file__).resolve().parents:
        for relative in ('build', 'fpga/build'):
            candidate = parent / relative
            if (candidate / 'source_bundle.tar.gz').is_file():
                return candidate
    raise RuntimeError('Keep this entry inside the complete submission package (build/source_bundle.tar.gz is required).')


def verified_bundle(directory):
    archive = directory / 'source_bundle.tar.gz'
    manifest = json.loads((directory / 'source_bundle.json').read_text())
    if digest(archive) != manifest['archive_sha256']:
        raise RuntimeError('Source bundle SHA-256 mismatch')
    with tarfile.open(archive, 'r:gz') as tf:
        members = tf.getmembers()
        names = [m.name for m in members]
        if len(set(names)) != len(names) or set(names) != set(manifest['files']):
            raise RuntimeError('Source bundle inventory mismatch')
        for member in members:
            path = PurePosixPath(member.name)
            if not member.isfile() or path.is_absolute() or '..' in path.parts:
                raise RuntimeError(f'Unsupported archive member: {member.name}')
            data = tf.extractfile(member).read()
            if hashlib.sha256(data).hexdigest() != manifest['files'][member.name]:
                raise RuntimeError(f'Source hash mismatch: {member.name}')
    return archive, manifest


def prepare(work, archive, manifest):
    marker = work / '.submission_bundle.json'
    if work.exists():
        if not marker.is_file() or json.loads(marker.read_text())['archive_sha256'] != manifest['archive_sha256']:
            raise RuntimeError('Work directory belongs to another source bundle; select a new --work-dir.')
        return
    work.parent.mkdir(parents=True, exist_ok=True)
    temp = Path(tempfile.mkdtemp(prefix='.kv260-prepare-', dir=work.parent))
    try:
        with tarfile.open(archive, 'r:gz') as tf:
            for member in tf.getmembers():
                dest = temp / member.name
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(tf.extractfile(member).read())
                dest.chmod(member.mode & 0o777)
        for name in ('contest/kv260_can/build', 'contest/kv260_can/result', '.cache'):
            (temp / name).mkdir(parents=True, exist_ok=True)
        (temp / '.submission_bundle.json').write_text(json.dumps({'archive_sha256': manifest['archive_sha256']}) + '\n')
        temp.rename(work)
    finally:
        if temp.exists():
            shutil.rmtree(temp)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='action', required=True)
    for action in ('check', 'prepare', 'build', 'test', 'package'):
        p = sub.add_parser(action)
        p.add_argument('--work-dir', type=Path, help='Default: build/work; use a new directory for a clean build')
        if action == 'build':
            p.add_argument('stage', choices=['create', 'ooc', 'synth', 'impl', 'bitstream'], nargs='?', default='create')
        if action == 'test':
            p.add_argument('--simulator', choices=['iverilog', 'xsim'], default='xsim')
    args = ap.parse_args()
    directory = build_directory()
    archive, manifest = verified_bundle(directory)
    print(f"PASS source bundle: {len(manifest['files'])} verified files", flush=True)
    if args.action == 'check':
        return
    work = (args.work_dir or directory / 'work').expanduser().resolve()
    work.parent.mkdir(parents=True, exist_ok=True)
    # The sibling lock covers extraction as well as every build/test/package operation.
    with work.with_name(work.name + '.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another command is using this work directory')
        prepare(work, archive, manifest)
        print(f'Workspace: {work}', flush=True)
        if args.action == 'prepare':
            return
        if args.action == 'build':
            command = ['bash', 'scripts/build_can.sh', args.stage]
        elif args.action == 'test':
            command = [sys.executable, 'contest/kv260_can/run_tests.py', '--simulator', args.simulator]
            if args.simulator == 'xsim':
                command = ['bash', '-c', 'source env/xilinx_2025_2.sh && exec "$@"', 'kv260-test', *command]
        else:
            command = [sys.executable, 'scripts/package_can.py']
        subprocess.run(command, cwd=work, check=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
