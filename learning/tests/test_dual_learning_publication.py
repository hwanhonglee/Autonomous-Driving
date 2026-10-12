"""HH_260906 - Validate publication shape and checksum failures without SSH/GPU use."""
import hashlib
import numpy as np
import pytest
from PIL import Image
from scripts.e2e import collect_dual_learning_results as r


def test_gallery_keeps_aspect_and_marks_learned_output(tmp_path):
    data=tmp_path/'data';data.mkdir();out=tmp_path/'out';out.mkdir()
    np.savez(data/'dev.npz',rgb=np.zeros((2,180,320,3),np.uint8),target=np.zeros((2,180,320),np.uint8))
    np.savez(tmp_path/'pred.npz',prediction=np.zeros((2,180,320),np.uint8))
    r.render_mapping(data,tmp_path/'pred.npz',out)
    assert Image.open(out/'perception_0000.png').size==(1280,260)
    assert (out/'learned_perception.gif').exists()


def test_checksum_failure_does_not_publish(tmp_path,monkeypatch):
    monkeypatch.setattr(r,'ssh',lambda _:hashlib.sha256(b'expected').hexdigest()+' remote')
    def copy(*args,**kwargs):(tmp_path/'result.json.partial').write_bytes(b'wrong')
    monkeypatch.setattr(r.subprocess,'run',copy)
    with pytest.raises(ValueError):r.fetch('/fake/remote',tmp_path/'result.json')
    assert not (tmp_path/'result.json').exists()
