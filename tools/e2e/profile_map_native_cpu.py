"""HH_260906 - Bounded local CPU-only RGB inference timing, never a ROS or driving FPS claim."""
import hashlib
import inspect
import json
from pathlib import Path
import platform
import time
import numpy as np
import torch
from portable_e2e.map_perception import MapPerception


def main():
    root=Path(__file__).resolve().parents[2]
    report=root/'docs/assets/validation/2026-09-26/dual_model_recovery_ah'
    checkpoint=root/'storage/experiments/training/2026-09-26/dual_recovery_ah/mapping/checkpoint.pt'
    summary=json.loads((report/'results/mapping/summary.json').read_text())
    expected=summary['checkpoint_sha256']
    if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=expected:raise ValueError('checkpoint changed')
    data=root/'storage/datasets/prepared/map_native_20260926_ah_v1'
    manifest=json.loads((data/'manifest.json').read_text())
    if hashlib.sha256((data/'dev.npz').read_bytes()).hexdigest()!=manifest['splits']['dev']['sha256']:
        raise ValueError('DEV archive changed')
    source=root/'learning/portable_e2e/map_perception.py'
    if hashlib.sha256(source.read_bytes()).hexdigest()!='1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b':
        raise ValueError('model implementation differs from trained source')
    output=report/'diagnostics/mapping_cpu_timing.json'
    if output.exists():raise ValueError('fresh measurement output required')
    torch.set_num_threads(2);torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    with np.load(data/'dev.npz',allow_pickle=False) as d:images=d['rgb'][:10].copy()
    model=MapPerception().cpu().eval().requires_grad_(False)
    # HH_260906 - Use the existing legacy local Torch without installing or modifying its environment.
    # HH_260906 - This is only our SHA-verified owned checkpoint, never an untrusted pickle.
    options={'map_location':'cpu'}
    if 'weights_only' in inspect.signature(torch.load).parameters:options['weights_only']=False
    model.load_state_dict(torch.load(checkpoint,**options)['model_state_dict'],strict=True)
    elapsed=[]
    with getattr(torch,'inference_mode',torch.no_grad)():
        for step in range(60):
            start=time.perf_counter()
            image=torch.from_numpy(images[step%len(images)].transpose(2,0,1).copy()).float().unsqueeze(0)/255
            result=model(image).argmax(1).numpy()
            seconds=time.perf_counter()-start
            if result.shape!=(1,360,640):raise ValueError('inference shape differs')
            if step>=10:elapsed.append(seconds*1000)
    payload=dict(device='CPU',gpu_used=False,threads=2,interop_threads=1,warmup=10,measured=50,batch_size=1,
        input_shape=[3,360,640],latency_ms=dict(p50=float(np.percentile(elapsed,50)),p95=float(np.percentile(elapsed,95)),
        maximum=float(max(elapsed)),minimum=float(min(elapsed))),checkpoint_sha256=expected,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),torch_version=torch.__version__,
        platform=platform.platform(),host=platform.node(),timestamp_unix=time.time(),
        samples_ms=elapsed,excludes=['disk decoding','sensor transport','synchronization','ROS','map fusion','vehicle control'],
        includes=['in-memory RGB layout and normalization','model forward','class argmax'],
        target_period_ms=100,complete_10hz_pipeline_verified=False,deployment_approved=False)
    with output.open('x') as f:json.dump(payload,f,indent=2)
    print(json.dumps(payload['latency_ms']),flush=True)


if __name__=='__main__':main()
