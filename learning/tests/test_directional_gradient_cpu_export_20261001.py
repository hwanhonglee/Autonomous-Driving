"""HH_261001 - Reject modified endpoints while keeping original inference identity scoped."""

from copy import deepcopy
import pytest
import torch
from scripts.e2e import export_directional_gradient_cpu_20261001 as export


def payload_fixture():
    study = export.study
    tensors = {'test': torch.ones(3)}
    state = export.base.tensor_digest(tensors)
    declaration = {'fixture': True}
    payload = dict(checkpoint_schema=study.SCHEMA, arm=study.ARMS[1],
        parent_checkpoint_sha256=study.previous.AH_CHECKPOINT_SHA256, training_plan_sha256=study.PLAN_SHA256,
        training_dataset_fingerprint_sha256=study.reference.DATASET_FINGERPRINT,
        exposure_order_sha256=study.reference.ORDER_SHA, warmup_mask_sha256=study.reference.MASK_SHA,
        fit=deepcopy(study.COUNTS), research_objective=declaration, initial_model_state_sha256='initial',
        inference_uses_original_model=True, training_data_approved=False, vehicle_control_approved=False,
        model_promoted=False, optimizer_state_dict={'state': {0: {}}},
        training_only_gradient_recipe=study.directional.RECIPE, model_state_dict=tensors,
        final_model_state_sha256=state)
    return payload, declaration, {'final_model_state_sha256': state}


def test_valid_b_and_tensor_identity():
    payload, declaration, receipt = payload_fixture()
    assert export.validate_payload(payload, export.study.ARMS[1], declaration, receipt, 'initial') == receipt['final_model_state_sha256']
    payload['model_state_dict']['test'][0] = 2
    with pytest.raises(ValueError):
        export.validate_payload(payload, export.study.ARMS[1], declaration, receipt, 'initial')


@pytest.mark.parametrize('key,value', [('fit', {}), ('inference_uses_original_model', False),
    ('training_only_gradient_recipe', 'other'), ('optimizer_state_dict', {}),
    ('model_promoted', True), ('training_plan_sha256', 'wrong'), ('warmup_mask_sha256', 'wrong')])
def test_reject_changed_endpoint(key, value):
    payload, declaration, receipt = payload_fixture()
    payload[key] = value
    with pytest.raises(ValueError):
        export.validate_payload(payload, export.study.ARMS[1], declaration, receipt, 'initial')


def test_identity_restored_after_failure():
    old = export.research.AH_CHECKPOINT_SHA256, export.research.CHECKPOINT_SCHEMA
    with pytest.raises(RuntimeError):
        with export.endpoint_identity('a' * 64):
            assert export.research.CHECKPOINT_SCHEMA == export.study.SCHEMA
            raise RuntimeError('test')
    assert (export.research.AH_CHECKPOINT_SHA256, export.research.CHECKPOINT_SCHEMA) == old


def test_native_must_reproduce_w1():
    payload, declaration, receipt = payload_fixture()
    payload.update(arm=export.study.ARMS[0], training_only_gradient_recipe='native')
    with pytest.raises(ValueError, match='native control'):
        export.validate_payload(payload, export.study.ARMS[0], declaration, receipt, 'initial')


def test_current_and_archived_sources_required(tmp_path):
    training, current = tmp_path / 'training', tmp_path / 'current'
    (training / 'source').mkdir(parents=True)
    current.mkdir()
    archived = training / 'source' / 'example.py'
    loaded = current / 'example.py'
    archived.write_text('original')
    loaded.write_text('original')
    sources = {'example.py': export.base.digest(archived)}
    export.verify_training_sources(training, current, sources)
    loaded.write_text('changed')
    with pytest.raises(ValueError, match='current or archived'):
        export.verify_training_sources(training, current, sources)
    loaded.write_text('original')
    archived.write_text('changed')
    with pytest.raises(ValueError, match='current or archived'):
        export.verify_training_sources(training, current, sources)
