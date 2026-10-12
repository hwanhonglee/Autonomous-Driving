"""HH_261001 - Research-only inward lower-bound surrogate; forward physics and runtime ABI stay unchanged."""

import ast
from contextlib import contextmanager
from contextvars import ContextVar
import hashlib
import inspect
from pathlib import Path
import textwrap
import torch
from . import model as original

MODEL_SOURCE_SHA = 'fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e'
RECIPE = 'directional_lower_speed_gradient_v1'
_TRACE = ContextVar('portable_e2e_directional_speed_gradient_trace', default=None)


class GradientTrace:
    """HH_261001 - Optional detached counters; enable on prospectively fixed diagnostic batches only."""
    def __init__(self):
        self.calls = 0
        self.totals = {}
        self.minimum_z = None

    def add(self, z, incoming, lower, allowed):
        self.calls += 1
        with torch.no_grad():
            for key, mask in dict(lower_saturated=lower, lower_inward=allowed,
                                  lower_outward=lower & (incoming > 0), lower_zero=lower & (incoming == 0)).items():
                value = mask.sum(dtype=torch.int64).detach()
                self.totals[key] = self.totals.get(key, 0) + value
            value = z.detach().min()
            self.minimum_z = value if self.minimum_z is None else torch.minimum(self.minimum_z, value)

    def receipt(self):
        return dict(backward_calls=self.calls, **{k: int(v.cpu()) for k, v in self.totals.items()},
                    minimum_preclamp_speed=None if self.minimum_z is None else float(self.minimum_z.cpu()))


@contextmanager
def trace_gradients(trace):
    token = _TRACE.set(trace)
    try:
        yield
    finally:
        _TRACE.reset(token)


class _InwardLowerClamp(torch.autograd.Function):
    @staticmethod
    def forward(ctx, z):
        ctx.save_for_backward(z)
        ctx.trace = _TRACE.get()
        # HH_261001 - Exactly the original clamp, including its upper bound and floating-point behavior.
        return z.clamp(0.0, original.PHYSICAL_MAXIMUM_SPEED_MPS)

    @staticmethod
    def backward(ctx, incoming):
        z, = ctx.saved_tensors
        lower = z < 0
        inward = lower & (incoming < 0)
        native = (z >= 0) & (z <= original.PHYSICAL_MAXIMUM_SPEED_MPS)
        if ctx.trace is not None:
            ctx.trace.add(z, incoming, lower, inward)
        # HH_261001 - This nonlinear surrogate is not the mathematical derivative of clamp.
        # HH_261001 - Only an inward lower-bound descent signal is added; upper saturation stays native zero.
        return torch.where(native | inward, incoming, torch.zeros_like(incoming))


def directional_lower_speed_clamp(z):
    return _InwardLowerClamp.apply(z)


def _decoder():
    """HH_261001 - Replace exactly one pinned recurrent clamp, never the observed-speed clamp or STOP decoder."""
    source_file = Path(inspect.getsourcefile(original.PerspectiveTrajectoryModel))
    if hashlib.sha256(source_file.read_bytes()).hexdigest() != MODEL_SOURCE_SHA:
        raise ValueError('reviewed original model source differs')
    tree = ast.parse(textwrap.dedent(inspect.getsource(original.PerspectiveTrajectoryModel._decode_physical_v1)))
    expected = ast.dump(ast.parse('(previous_speed + acceleration[:, :, point_index] * PHYSICAL_TIME_STEP_S).clamp(0.0, PHYSICAL_MAXIMUM_SPEED_MPS)', mode='eval').body)

    class ReplaceOne(ast.NodeTransformer):
        count = 0

        def visit_Call(self, node):
            if ast.dump(node) == expected:
                self.count += 1
                return ast.copy_location(ast.Call(func=ast.Name(id='_directional_lower_speed_clamp', ctx=ast.Load()),
                                                  args=[node.func.value], keywords=[]), node)
            return self.generic_visit(node)

    transform = ReplaceOne()
    changed = transform.visit(tree)
    if transform.count != 1:
        raise ValueError('exactly one reviewed recurrent speed clamp required')
    ast.fix_missing_locations(changed)
    namespace = dict(vars(original), _directional_lower_speed_clamp=directional_lower_speed_clamp)
    exec(compile(changed, '<pinned_directional_speed_decoder>', 'exec'), namespace)
    return namespace['_decode_physical_v1']


class DirectionalLowerSpeedGradientModel(original.PerspectiveTrajectoryModel):
    """HH_261001 - Same parameters/config/output values; changed backward used only by an explicit new training protocol."""
    _decode_physical_v1 = _decoder()

