"""Content-bound MuJoCo cache; every new path still receives collision checks."""
import hashlib
import json
import os
import shutil
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


def model_key(xml, version):
    root = ET.fromstring(xml)
    digest = hashlib.sha256(version.encode() + xml.encode())
    if root.findall('.//include'):
        return None
    for element in root.iter():
        if 'file' not in element.attrib:
            continue
        path = Path(element.attrib['file'])
        if not path.is_absolute() or not path.is_file():
            return None
        digest.update(str(path).encode())
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


CACHE_LIMIT_BYTES = 3 * 2**30


def prune_cache(cache, incoming=0, limit=CACHE_LIMIT_BYTES, keep=()):
    """Oldest-first eviction so the RAM-backed cache can never fill /dev/shm.

    Every distinct scene writes a new ~220 MB compiled model; unbounded, 60 of them filled the 13 GB
    tmpfs and the next saves were silently truncated.
    """
    files = sorted((p for p in Path(cache).iterdir() if p.is_file()), key=lambda p: p.stat().st_mtime)
    total = sum(p.stat().st_size for p in files)
    free = shutil.disk_usage(cache).free
    for path in files:
        if total + incoming <= limit and free >= incoming * 1.2:
            break
        if path.name in keep:
            continue
        size = path.stat().st_size
        path.unlink(missing_ok=True)
        total -= size
        free += size


def cache_directory():
    base = Path('/dev/shm') if Path('/dev/shm').is_dir() else Path(tempfile.gettempdir())
    cache = base / f'aipc-mujoco-{os.getuid()}'
    cache.mkdir(mode=0o700, exist_ok=True)
    return cache


def load_model(mj, xml):
    key = model_key(xml, mj.__version__)
    if key is None:
        return mj.MjModel.from_xml_string(xml), False
    try:
        cache = cache_directory()
    except OSError:
        return mj.MjModel.from_xml_string(xml), False
    path = cache / (key + '.mjb')
    if path.is_file():
        try:
            # Fresh private model: changed payload poses never leak to callers.
            return mj.MjModel.from_binary_path(str(path)), True
        except ValueError:
            path.unlink(missing_ok=True)
    model = mj.MjModel.from_xml_string(xml)
    tmp = None
    try:
        expected = mj.mj_sizeModel(model)
        prune_cache(cache, incoming=expected, keep={key + '.contacts.json'})
        fd, tmp = tempfile.mkstemp(dir=cache, suffix='.mjb')
        os.close(fd)
        mj.mj_saveModel(model, tmp)
        if Path(tmp).stat().st_size != expected:
            raise OSError('compiled model was not written completely')  # e.g. tmpfs full: never publish a truncated cache entry
        os.replace(tmp, path)
    except OSError:
        pass  # An unavailable cache must not invalidate a freshly built model.
    finally:
        if tmp is not None:
            Path(tmp).unlink(missing_ok=True)
    return model, False


def contact_store(xml, version):
    """Exact collision states scoped to model/assets, never cached PASS results."""
    key = model_key(xml, version)
    if key is None:
        return {}, None
    try:
        path = cache_directory() / (key + '.contacts.json')
    except OSError:
        return {}, None
    try:
        data = json.loads(path.read_text())
        return (data if isinstance(data, dict) else {}), path
    except (OSError, ValueError):
        return {}, path


def save_contacts(cache, path):
    if path is None:
        return
    tmp = None
    try:
        fd, tmp = tempfile.mkstemp(dir=path.parent, suffix='.json')
        with os.fdopen(fd, 'w') as stream:
            json.dump(cache, stream, separators=(',', ':'), allow_nan=False)
        os.replace(tmp, path)
    except OSError:
        pass  # Contact computation and classification already completed.
    finally:
        if tmp is not None:
            Path(tmp).unlink(missing_ok=True)
