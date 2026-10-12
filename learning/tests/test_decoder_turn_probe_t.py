"""HH_260906 - Keep separate STOP candidates out of the DRIVE latent tensor."""
import numpy as np
import pytest
from scripts.e2e.run_decoder_turn_probe_t import drive_latents


def test_stopmix_drive_head_has_six_not_twelve_candidates():
    assert drive_latents(np.zeros((6,768)),6).shape==(6,6,64,2)


def test_invalid_head_is_rejected():
    with pytest.raises(ValueError):drive_latents(np.zeros((6,769)),6)
    with pytest.raises(ValueError):drive_latents(np.zeros((5,768)),6)
