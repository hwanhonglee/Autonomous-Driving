"""HH_261001 - Exact geometry-source intervention over immutable paired appearance data."""

import hashlib
import json
from pathlib import Path
import numpy as np

try:
    from . import map_appearance_matched_contract_20261001 as base
    from . import prepare_map_route_geometry_remote_20261001 as package
except ImportError:
    import map_appearance_matched_contract_20261001 as base
    import prepare_map_route_geometry_remote_20261001 as package

NAME = package.NAME
ARMS = ('control','geometry')
PLAN_SHA = '5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68'
PACKAGE_APPROVAL_SHA = '844bbc685eb0e49c427687d53432d622d3d8123e519e870423c460375bdfe43e'
DATA_SHA = 'db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d'
GEOMETRY_FILES = {'new_false480.npz':'f1fd23b786aa4dee7373054ed1580a4ad7a87715e05a62d2ec31b4ae35591f05',
    'training_schedule.npz':'80659aa9eeecd3de590623a88f357a615dd39f74f268c756ee18ed46ff9ef5d9'}
BASE_REGISTRY_SHA = '456b8b31ef2a3426d4c732236e7c549792ebe9d9c1cb246e0b055496441718af'
CLASS_WEIGHTS = [0.13594788312911987,0.13577374815940857,0.2083463817834854,
                 1.4355568885803223,1.9548143148422241,2.183657646179199,0.9459031224250793]
PINNED_SOURCE = {**base.ARCHITECTURE_SHA,
    'map_appearance_matched_contract_20261001.py':'9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96',
    'prepare_map_route_geometry_remote_20261001.py':'8c656bb1324049e27c9ec0c6fc2a533f465f529beb84ab9ae733ec9443d4d5e0',
    'train_map_photometric_pilot.py':package.POLICY_SHA}
SOURCE_FILES = {*PINNED_SOURCE,'map_route_geometry_contract_20261001.py',
    'train_map_route_geometry_matched_20261001.py','training_plan.md','weights_correction.md'}
FIT_SECONDS = 2100
EVALUATION_SECONDS = 300


def validate_plan(value):
    base.require(value == package.plan(), 'Frozen source, replay, exposure or photo plan differs')
    return value


def slot_source(slot,arm,replay):
    base.require(arm in ARMS and 0 <= int(slot) < 1689, 'Unknown arm or slot')
    slot = int(slot)
    if slot < 729:
        return 'original',slot
    if slot < 1209:
        return 'paired',slot-729
    return ('original',int(replay[slot-1209])) if arm == 'control' else ('geometry',slot-1209)


def validate_fit_authorization(value,data_sha,source_sha):
    base.require(value.get('schema') == 'portable_e2e.map_geometry_fit_authorization.v1'
        and value.get('study_id') == NAME and value.get('fit_authorized') is True
        and value.get('prepared_manifest_sha256') == data_sha and value.get('source_bundle_sha256') == source_sha
        and value.get('reference_plan_sha256') == PLAN_SHA
        and value.get('package_authorization_sha256') == PACKAGE_APPROVAL_SHA
        and value.get('steps_per_arm') == 10000 and value.get('arms') == list(ARMS)
        and value.get('common_class_weights') == CLASS_WEIGHTS
        and value.get('no_dev_checkpoint_selection') is True and value.get('ctrack_train_eligible') is False,
        'Separate exact parent fit authorization required')


def source_binding(root,expected):
    base.require(base.digest(root/'bundle.json') == expected, 'Frozen source bundle changed')
    bundle = base.read_json(root/'bundle.json')
    base.require(bundle.get('schema') == 'portable_e2e.map_geometry_training_source.v1'
        and bundle.get('study_id') == NAME and set(bundle.get('files',{})) == SOURCE_FILES
        and bundle.get('reference_plan_sha256') == PLAN_SHA
        and bundle.get('prepared_manifest_sha256') == DATA_SHA
        and bundle.get('package_authorization_sha256') == PACKAGE_APPROVAL_SHA, 'Source bundle contract differs')
    for name,sha in bundle['files'].items():
        base.require(base.digest(root/name) == sha, 'Frozen source changed: '+name)
    for name,sha in PINNED_SOURCE.items():
        base.require(bundle['files'][name] == sha, 'Existing architecture/policy/packager changed: '+name)
    return bundle


def validate_prepared_manifest(manifest,ref):
    base.require(manifest.get('schema') == 'portable_e2e.map_geometry_remote_package.v1'
        and manifest.get('study_id') == NAME and manifest.get('candidate_manifest_sha256') == package.CANDIDATE_SHA
        and manifest.get('reference_plan_sha256') == PLAN_SHA and manifest.get('approval_sha256') == PACKAGE_APPROVAL_SHA
        and manifest.get('reference_root') == str(package.REFERENCE)
        and manifest.get('reference_manifest_sha256') == package.BASE_SHA
        and manifest.get('reference_files') == ref['files'] and manifest.get('source_receipts_preserved') is True
        and manifest.get('parent_weights_sha256') == base.PARENT_SHA and manifest.get('fit_authorized') is False
        and manifest.get('builder_sha256') == PINNED_SOURCE['prepare_map_route_geometry_remote_20261001.py'],
        'Remote geometry package contract differs')
    base.require(manifest.get('common_class_weights') == ref['common_class_weights'] == CLASS_WEIGHTS,
        'Use actual preceding paired-study weights unchanged; never recompute from geometry labels')
    expected_ids = [f'{route}:frames/{i:04d}' for route in ('p02','p04') for i in range(240)]
    base.require(manifest.get('source_ids') == expected_ids and manifest.get('files') == GEOMETRY_FILES,
        'Full480 source order/files differ')


def package_binding(study,reference,expected_manifest):
    """HH_261001 - Verify receipts and byte identity without loading protected evaluation arrays."""
    base.require(expected_manifest == DATA_SHA,'Only the reviewed actual remote package is permitted')
    prepared = study/'prepared'
    checks = {prepared/'manifest.json':expected_manifest,study/'reference_plan.json':PLAN_SHA,
        study/'package_authorization.json':PACKAGE_APPROVAL_SHA,study/'candidate_manifest.json':package.CANDIDATE_SHA,
        reference/'manifest.json':package.BASE_SHA,reference/'study_train_registry.json':BASE_REGISTRY_SHA}
    for path,sha in checks.items():
        base.require(base.digest(path) == sha,'Study/reference bytes changed: '+str(path))
    manifest,ref = base.read_json(prepared/'manifest.json'),base.read_json(reference/'manifest.json')
    plan = validate_plan(base.read_json(study/'reference_plan.json'))
    package.validate_authorization(base.read_json(study/'package_authorization.json'),PLAN_SHA)
    base.validate_manifest(ref,base.read_json(reference/'study_train_registry.json'))
    validate_prepared_manifest(manifest,ref)
    checks[reference/'paired_samples.json'] = ref['paired_samples_sha256']
    checks.update({reference/name:r['sha256'] for name,r in ref['files'].items()})
    checks.update({prepared/name:sha for name,sha in manifest['files'].items()})
    for path,sha in checks.items():
        base.require(base.digest(path) == sha,'Input artifact changed: '+str(path))
    return manifest,ref,plan,checks


def load_arrays(study,reference,expected_manifest):
    manifest,ref,plan,checks = package_binding(study,reference,expected_manifest)
    arrays = {}
    for key,path,n,rgb_key in [('original',reference/'original_train729.npz',729,'rgb'),
        ('paired',reference/'paired_train480.npz',480,'rgb_true'),
        ('geometry',study/'prepared/new_false480.npz',480,'rgb')]:
        with np.load(path,allow_pickle=False) as data:
            base.require(set(data.files) == ({'rgb_false','rgb_true','target'} if key == 'paired' else {'rgb','target'}),
                         'RGB/target archive ABI differs')
            rgb,target = data[rgb_key],data['target']
        base.require(rgb.shape == (n,360,640,3) and target.shape == (n,360,640)
            and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7,'Native seven-class ABI differs')
        arrays[key] = (rgb,target)
    candidate = base.read_json(study/'candidate_manifest.json')
    for offset,route in [(0,'p02'),(240,'p04')]:
        capture = candidate['captures'][route]
        base.require(capture['raw_frames'] == capture['proposed_frames'] == 240
            and capture['invalid_frames'] == capture['dropped_frames'] == [],'Full valid capture required')
        for i,frame in enumerate(capture['frames']):
            base.require(frame['capture_index'] == i and frame['proposed_include'] is True
                and np.bincount(arrays['geometry'][1][offset+i].ravel(),minlength=7).tolist() == frame['class_pixels'],
                'Per-frame geometry target/selection differs')
    with np.load(study/'prepared/training_schedule.npz',allow_pickle=False) as data:
        base.require(set(data.files) == {'indices','crops','control_original_indices'},'Schedule archive ABI differs')
        indices,crops,replay = data['indices'],data['crops'],data['control_original_indices']
    expected_indices,expected_crops = package.schedule()
    base.require(indices.dtype == np.int64 and crops.dtype == np.int16 and replay.dtype == np.int64
        and np.array_equal(indices,expected_indices) and np.array_equal(crops,expected_crops)
        and np.array_equal(replay,package.control_replay()),'Exact matched schedule/replay/crops differ')
    return manifest,ref,plan,checks,arrays,indices,crops,replay


def batch_arrays(arrays,ids,boxes,arm,replay):
    pairs = []
    for slot,box in zip(ids,boxes):
        name,index = slot_source(slot,arm,replay)
        rgb,target = arrays[name]
        pairs.append(base.apply_crop(rgb[index],target[index],box))
    return np.stack([pair[0] for pair in pairs]),np.stack([pair[1] for pair in pairs])


class PhotoTrace:
    def __init__(self):
        self.sha = hashlib.sha256()
        self.eligible = self.applied = 0

    def receipt(self,rng):
        return {'eligible_calls':self.eligible,'applied_calls':self.applied,
                'sample_gate_trace_sha256':self.sha.hexdigest(),'final_rng_state':rng.bit_generator.state}


def augment_both_arms(rgb,ids,rng,photometric,trace):
    result = rgb.copy()
    for i,slot in enumerate(ids):
        if package.eligible_for_photo(slot):
            result[i],active = photometric(rgb[i],rng)
            trace.eligible += 1
            trace.applied += int(active)
            trace.sha.update(np.asarray([slot,int(active)],np.int64).tobytes())
    return result


def exposure_receipt(indices,replay,arm):
    slots = np.bincount(indices.ravel(),minlength=1689)
    actual = {'original':np.zeros(729,np.int64),'paired':np.zeros(480,np.int64),'geometry':np.zeros(480,np.int64)}
    for slot,n in enumerate(slots):
        name,index = slot_source(slot,arm,replay)
        actual[name][index] += n
    return {'slot_exposures':slots.tolist(),'physical_source_exposures':{k:v.tolist() for k,v in actual.items()},
            'physical_source_totals':{k:int(v.sum()) for k,v in actual.items()}}
