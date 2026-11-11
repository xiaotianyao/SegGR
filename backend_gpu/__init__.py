# Python wrapper for the IntervalAccum CUDA operator (approach B).
#
# The extension is JIT-compiled with torch.utils.cpp_extension.load on first
# use; requires nvcc (CUDA >= 11) and a visible GPU at build time (or set
# TORCH_CUDA_ARCH_LIST, which we default to 8.0 for A100/A800).
import os

import torch

_EXT = None


def _ext():
    global _EXT
    if _EXT is None:
        from torch.utils.cpp_extension import load
        os.environ.setdefault("TORCH_CUDA_ARCH_LIST", "8.0")
        src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "interval_accum_ext.cu")
        _EXT = load(name="interval_accum_ext", sources=[src],
                    extra_cuda_cflags=["-O3"], verbose=False)
    return _EXT


class IntervalAccumFn(torch.autograd.Function):
    """demand = IntervalAccum(p): differentiable interval-wise accumulation.

    Non-tensor operands: C (chunk size), E (edge count)."""

    @staticmethod
    def forward(ctx, p, chunk_ptr, seg_l, seg_r, seg_cand, C, E):
        demand = _ext().forward(p, chunk_ptr, seg_l, seg_r, seg_cand, C, E)
        ctx.save_for_backward(chunk_ptr, seg_l, seg_r, seg_cand)
        ctx.C = C
        ctx.E = E
        ctx.N = p.numel()
        return demand

    @staticmethod
    def backward(ctx, grad_out):
        chunk_ptr, seg_l, seg_r, seg_cand = ctx.saved_tensors
        grad_p = _ext().backward(grad_out.contiguous(), chunk_ptr, seg_l,
                                 seg_r, seg_cand, ctx.C, ctx.E, ctx.N)
        return grad_p, None, None, None, None, None, None


def interval_accum_cuda(p, chunk_table):
    """chunk_table: dict with chunk_ptr (int64), seg_l/seg_r (int32),
    seg_cand (int64), C, E -- see interval_ops.chunks_from_segments."""
    return IntervalAccumFn.apply(p, chunk_table["chunk_ptr"], chunk_table["seg_l"],
                                 chunk_table["seg_r"], chunk_table["seg_cand"],
                                 chunk_table["C"], chunk_table["E"])
