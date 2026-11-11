// IntervalAccum custom CUDA operator (approach B).
//
// forward : demand = IntervalAccum(p)   -- per chunk: scatter segment weights
//           into a shared-memory difference array, inclusive-scan it, write
//           the chunk's demand to global memory.
// backward: grad_p = IntervalAccum^T (grad_out) -- per chunk: load the upstream
//           gradient, inclusive-scan it, then each segment gathers its range
//           sum (incl[r] - incl[l-1]) and atomicAdds into grad_p.
//
// One thread block per chunk; the O(C) intermediates never leave shared
// memory. Chunks are independent, so no inter-block synchronization exists.
// Scan is a double-buffered Hillis-Steele pass (works for arbitrary len);
// cub::BlockScan can replace it later if profiling justifies it.
//
// Target: sm_80 (A100/A800), CUDA 12.1. Shared usage = 2*(C+1)*4 B
// (32.8 KB at C=4096), below the 48 KB static limit -> no opt-in needed.

#include <torch/extension.h>
#include <ATen/cuda/CUDAContext.h>
#include <cuda.h>
#include <cuda_runtime.h>

#define THREADS 256

// Inclusive scan of buf_a[0..n-1]; buf_b is scratch. Returns the buffer that
// holds the result (alternates between buf_a and buf_b per pass).
__device__ float* block_inclusive_scan(float* buf_a, float* buf_b, int n) {
    float* src = buf_a;
    float* dst = buf_b;
    for (int off = 1; off < n; off <<= 1) {
        for (int i = threadIdx.x; i < n; i += blockDim.x) {
            dst[i] = (i >= off) ? src[i] + src[i - off] : src[i];
        }
        __syncthreads();
        float* t = src; src = dst; dst = t;
    }
    return src;
}

__global__ void interval_accum_fwd_kernel(
    const float* __restrict__ p,
    const long*  __restrict__ chunk_ptr,   // (n_chunks+1,)
    const int*   __restrict__ seg_l,       // chunk-local, inclusive
    const int*   __restrict__ seg_r,
    const long*  __restrict__ seg_cand,
    float* __restrict__ demand,            // (E,)
    const int C, const long E)
{
    extern __shared__ float smem[];
    float* buf_a = smem;
    float* buf_b = smem + (C + 1);

    const long base = (long)blockIdx.x * C;
    if (base >= E) return;
    const int len = (int)min((long)C, E - base); // last chunk may be partial

    for (int i = threadIdx.x; i < len + 1; i += blockDim.x) buf_a[i] = 0.f;
    __syncthreads();

    const long s0 = chunk_ptr[blockIdx.x], s1 = chunk_ptr[blockIdx.x + 1];
    for (long s = s0 + threadIdx.x; s < s1; s += blockDim.x) {
        const float w = p[seg_cand[s]];
        atomicAdd(&buf_a[seg_l[s]], w);
        atomicAdd(&buf_a[seg_r[s] + 1], -w); // r+1 <= len; entry never scanned
    }
    __syncthreads();

    const float* scanned = block_inclusive_scan(buf_a, buf_b, len);
    for (int i = threadIdx.x; i < len; i += blockDim.x) demand[base + i] = scanned[i];
}

__global__ void interval_accum_bwd_kernel(
    const float* __restrict__ grad_out,    // (E,)
    const long*  __restrict__ chunk_ptr,
    const int*   __restrict__ seg_l,
    const int*   __restrict__ seg_r,
    const long*  __restrict__ seg_cand,
    float* __restrict__ grad_p,            // (N,) pre-zeroed
    const int C, const long E)
{
    extern __shared__ float smem[];
    float* buf_a = smem;
    float* buf_b = smem + (C + 1);

    const long base = (long)blockIdx.x * C;
    if (base >= E) return;
    const int len = (int)min((long)C, E - base);

    for (int i = threadIdx.x; i < len; i += blockDim.x) buf_a[i] = grad_out[base + i];
    __syncthreads();

    const float* incl = block_inclusive_scan(buf_a, buf_b, len);

    const long s0 = chunk_ptr[blockIdx.x], s1 = chunk_ptr[blockIdx.x + 1];
    for (long s = s0 + threadIdx.x; s < s1; s += blockDim.x) {
        const int l = seg_l[s], r = seg_r[s];
        const float hi = incl[r];
        const float lo = (l > 0) ? incl[l - 1] : 0.f;
        atomicAdd(&grad_p[seg_cand[s]], hi - lo);
    }
}

torch::Tensor interval_accum_forward(
    torch::Tensor p, torch::Tensor chunk_ptr, torch::Tensor seg_l,
    torch::Tensor seg_r, torch::Tensor seg_cand, long C, long E)
{
    TORCH_CHECK(p.is_cuda() && p.dtype() == torch::kFloat32, "p must be CUDA float32");
    TORCH_CHECK(chunk_ptr.dtype() == torch::kInt64, "chunk_ptr must be int64");
    TORCH_CHECK(seg_l.dtype() == torch::kInt32 && seg_r.dtype() == torch::kInt32,
                "seg_l/seg_r must be int32");
    TORCH_CHECK(seg_cand.dtype() == torch::kInt64, "seg_cand must be int64");

    auto demand = torch::empty({E}, p.options());
    const long n_chunks = chunk_ptr.numel() - 1;
    if (n_chunks == 0) return demand;
    const int smem = (int)(2 * (C + 1) * sizeof(float));
    interval_accum_fwd_kernel<<<(int)n_chunks, THREADS, smem, at::cuda::getCurrentCUDAStream()>>>(
        p.data_ptr<float>(), chunk_ptr.data_ptr<long>(), seg_l.data_ptr<int>(),
        seg_r.data_ptr<int>(), seg_cand.data_ptr<long>(), demand.data_ptr<float>(),
        (int)C, E);
    return demand;
}

torch::Tensor interval_accum_backward(
    torch::Tensor grad_out, torch::Tensor chunk_ptr, torch::Tensor seg_l,
    torch::Tensor seg_r, torch::Tensor seg_cand, long C, long E, long N)
{
    TORCH_CHECK(grad_out.is_cuda() && grad_out.dtype() == torch::kFloat32,
                "grad_out must be CUDA float32");
    auto grad_p = torch::zeros({N}, grad_out.options());
    const long n_chunks = chunk_ptr.numel() - 1;
    if (n_chunks == 0) return grad_p;
    const int smem = (int)(2 * (C + 1) * sizeof(float));
    interval_accum_bwd_kernel<<<(int)n_chunks, THREADS, smem, at::cuda::getCurrentCUDAStream()>>>(
        grad_out.data_ptr<float>(), chunk_ptr.data_ptr<long>(), seg_l.data_ptr<int>(),
        seg_r.data_ptr<int>(), seg_cand.data_ptr<long>(), grad_p.data_ptr<float>(),
        (int)C, E);
    return grad_p;
}

PYBIND11_MODULE(TORCH_EXTENSION_NAME, m) {
    m.def("forward", &interval_accum_forward, "IntervalAccum forward (CUDA)");
    m.def("backward", &interval_accum_backward, "IntervalAccum backward (CUDA)");
}
