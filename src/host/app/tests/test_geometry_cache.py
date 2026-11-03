import tempfile
from pathlib import Path
from workcell.geometry_cache import model_key
from unittest.mock import patch
from workcell.geometry_cache import save_contacts


def test_key_tracks_asset_content_even_with_same_size():
    with tempfile.TemporaryDirectory() as directory:
        asset=Path(directory)/'mesh.stl';asset.write_bytes(b'aaaa')
        xml=f'<mujoco><asset><mesh file="{asset}"/></asset></mujoco>'
        old=model_key(xml,'v1')
        asset.write_bytes(b'bbbb')
        assert model_key(xml,'v1') != old
        assert model_key(xml,'v2') != model_key(xml,'v1')
        assert model_key(xml.replace('<mujoco>','<mujoco model="changed">'),'v1') != model_key(xml,'v1')


def test_unknown_asset_resolution_is_not_cached():
    assert model_key('<mujoco><include file="scene.xml"/></mujoco>','v1') is None
    assert model_key('<mujoco><asset><mesh file="relative.stl"/></asset></mujoco>','v1') is None


def test_cache_storage_full_does_not_abort_verified_geometry():
    with patch('workcell.geometry_cache.tempfile.mkstemp', side_effect=OSError('full')):
        save_contacts({'verified-state': []}, Path('/unused/contacts.json'))


def test_cache_eviction_is_oldest_first_and_keeps_the_requested_entry():
    import os,time
    from workcell.geometry_cache import prune_cache
    with tempfile.TemporaryDirectory() as directory:
        cache=Path(directory)
        for i,name in enumerate(['old.mjb','mid.mjb','new.mjb','keep.contacts.json']):
            (cache/name).write_bytes(b'x'*1000);os.utime(cache/name,(1000+i,1000+i))
        prune_cache(cache,incoming=1500,limit=3600,keep={'keep.contacts.json'})
        left=sorted(p.name for p in cache.iterdir())
        assert left==['keep.contacts.json','new.mjb'],left
        assert sum((cache/n).stat().st_size for n in left)+1500<=3600


def test_nothing_is_evicted_while_the_cache_has_room():
    from workcell.geometry_cache import prune_cache
    with tempfile.TemporaryDirectory() as directory:
        cache=Path(directory);(cache/'a.mjb').write_bytes(b'x'*10)
        prune_cache(cache,incoming=10,limit=10**9)
        assert (cache/'a.mjb').exists()
