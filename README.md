<div align="center">

# ⚡ SegGR

## 🌐 Fast and Energy-Efficient Hardware-Accelerated Parallel Global Routing

**Segments as primitives. Parallel optimization. Hardware-aware execution.**

A hardware-accelerated global-routing framework that combines the solution quality of parallel routing with the execution efficiency of custom GPU operators and FPGA accelerators.

</div>

---

## 🚀 Why SegGR?

Parallel global routers explore a large solution space and deliver strong solution quality, but their edge-wise sparse-tensor representation creates substantial construction, memory-bandwidth, and scheduling overheads. Sequential routers are faster, but their net-by-net optimization sacrifices the global congestion view.

SegGR addresses this quality-speed tradeoff with one shared idea:

> A rectilinear route is determined by its turning points, not by an explicit list of every edge. Process maximal straight segments throughout the optimization loop instead of repeatedly expanding paths into edges.

SegGR is **one parallel-routing framework with two hardware backends**. Both backends optimize the same probabilistic parallel-routing objective using the same compact segment representation and the same range-update/range-sum primitives.

- **Shared algorithm:** candidate probabilities, expected-demand accumulation, overflow gradients, and logit updates follow one common formulation.
- **Shared task representation:** routing paths remain compact segments across the frontend and both hardware backends.
- **Platform-specialized execution:** the GPU maps these primitives to shared-memory scatter/scan CUDA kernels and autograd, while the FPGA maps them to a pipelined segment tree, streaming softmax, and explicit block/data movement scheduling.

This provides **algorithm-level portability across GPU and FPGA** without forcing either platform into the other's execution model. CUDA/PyTorch handles GPU memory and dependency management, while the FPGA backend exploits custom fixed-point pipelines, banked on-chip storage, and adaptive partitioning for severely resource-constrained hardware.

```text
                Candidate routes
                        |
                        v
            Compact segment representation
                        |
        +-------------------------------+
        |                               |
        v                               v
    GPU backend                       FPGA backend
    Segment tasks               Segment tasks + adaptive partitioning
        |                               |
        v                               v
SegmentAccum CUDA operator      Gradient engine + streaming softmax
        |                               |
        +---------------+---------------+
                        v
                Global routing result
```

## Reported Results

The following results are reported in our manuscript.

| Backend | Runtime and energy | Evaluation scope |
|---|---|---|
| NVIDIA A100 GPU | **1.91x-6.85x speedup over DGR**, with **3.6x lower energy consumption on average** | 10 ISPD 2018 cases and 6 congested ICCAD 2019 cases |
| Xilinx Kintex UltraScale FPGA (66.67MHz) | **4.10x-7.95x speedup over DGR**, with an estimated **45.7x-50.6x higher energy efficiency** | Same 16 cases |

The solution quality of both backends **matches DGR and improves over CUGR2 in aggregate** after common post-processing.

## 🧭 Code Guide

This repository includes the core implementation of SegGR to make the algorithms and hardware architecture easy to inspect.
The implementation can be reviewed in the same order as the manuscript: partitioning, GPU backend, FPGA segment core, and streaming softmax.

### 1. Adaptive Partitioning

**Code:** [`frontend_partitioning/adaptive_partition.py`](frontend_partitioning/adaptive_partition.py)

The partitioner computes global x cuts followed by independent y cuts for each column. `_dp_cut_1d()` implements the one-dimensional dynamic program that balances group counts while minimizing boundary crossings. It uses prefix sums and difference arrays for constant-time queries and automatically relaxes the balance band when necessary. `compute_adaptive_cuts()` applies this recurrence in both directions, and `make_block_fn()` converts the cuts into a task-to-block mapping.

### 2. GPU Backend: SegmentAccum

**Code:** [`backend_gpu/interval_accum_ext.cu`](backend_gpu/interval_accum_ext.cu), with the PyTorch wrapper in [`backend_gpu/__init__.py`](backend_gpu/__init__.py)

The forward kernel performs two shared-memory atomic scatters per segment and an inclusive scan to produce per-edge demand. The backward kernel scans the upstream gradients, gathers range sums, and atomically accumulates candidate gradients. One thread block handles one chunk, keeping intermediate per-edge state in shared memory. Softmax, the nonlinear objective, and the optimizer remain in the PyTorch autograd domain.

### 3. FPGA Segment Core

**Code:** [`segcore_core.sv`](backend_fpga/router.srcs/sources_1/imports/rtl/segcore/segcore_core.sv)

The segment core is a recursively halved segment tree with registered command and response paths at every level. It accepts one segment operation per cycle and supports `CLEAR`, `UPD`, `SETW`, `FLUSH`, `ACT`, `QUERY`, and `EMPTY_QUERY`. Fully covered updates are absorbed into lazy internal-node registers. Queries always descend to the leaves, producing fixed-latency, in-order responses that can be paired directly with coefficient FIFOs.

### 4. FPGA Streaming Softmax

**Code:** [`softmax_bank_engine.sv`](backend_fpga/router.srcs/sources_1/imports/rtl/softmax/softmax_bank_engine.sv)

The engine pipelines across small, irregularly sized candidate groups rather than processing one group at a time. Input/group-descriptor accumulation, raw-gradient and RMSprop update, and collect-exp-emit can operate on different groups in flight. Fixed-point lookup tables, pipelined CORDIC/divider IP, and burst-oriented DDR row readers/writers complete the probability-update path.

## 🗺️ Detailed RTL Reading Map

The hand-written RTL is rooted at `backend_fpga/router.srcs/sources_1/imports/rtl/`. The detailed map below indexes each module. Files under `backend_fpga/ip/` and generated IP under `router.srcs/` are implementation dependencies rather than the main architectural contribution.

### A. `segcore/`: segment operations at one operation per cycle

| File | What to look for |
|---|---|
| `segcore/segcore_core.sv` | Recursive tree structure and level-by-level command/result pipelining |
| `segcore/segcore_highlevel_nonleaf_pe.sv` | Range splitting, coordinate rebasing, lazy updates, and query reduction |
| `segcore/segcore_nonleaf_pe.sv` | Internal-node command propagation and response handling |
| `segcore/segcore_leaf_pe.sv` | Per-edge demand and weight state |
| `segcore/segcore_actgroup8.sv` | Activation scheduling across a leaf group |
| `segcore/act_pe.sv` | Fixed-point activation-derivative LUT and multiply pipeline |
| `segcore/act_pe_lut_1024x8.svh` | Default 1024-entry activation-derivative table |
| `common/segcore_pkg.sv` | Instruction set, banked addresses, and fixed-point formats |

### B. `grad_p/`: from segment tasks to candidate gradients

| File | What to look for |
|---|---|
| `grad_p/grad_overflow_core_xN.sv` | Ingress FIFOs, probability prefetch, dispatch pipeline, and segment-core integration |
| `grad_p/dispatcher_core.sv` | Automatic query generation, coefficient bookkeeping, and task phases |
| `grad_p/grad_wlvia.sv` | Independent linear wirelength/via gradient lane |
| `grad_p/grad_p.sv` | Banked probability access and read-modify-write gradient accumulation |
| `grad_p/grad_engine.sv` | Multi-core organization, banked storage, and softmax integration |
| `grad_p/net_fetcher.sv` | Static NET-image fetching and lane-stream generation |
| `grad_p/blocked_ctrl.sv` | Iteration management, block sequencing, carry refill/spill, and update overlap |

### C. `softmax/`: pipelined probability updates

| File | What to look for |
|---|---|
| `softmax/softmax_bank_engine.sv` | Three-stage group pipeline, RMSprop, logit update, and softmax refresh |
| `softmax/softmax_ddr.sv` | Per-bank engine integration and probability writeback |
| `softmax/softmax_row_reader.sv` | Burst-aligned DDR reads and row unpacking |
| `softmax/softmax_row_writer.sv` | Buffered, masked DDR write bursts |
| `softmax/p_line_asm.sv` | Probability-line assembly and writeback scheduling |
| `softmax/softmax_exp_lut_512x8.svh` | Fixed-point exponential lookup table |

### D. `router/`: system integration

| File | What to look for |
|---|---|
| `router/router.sv` | Compute, storage, DDR, and scheduler integration |
| `router/router_pcie_bram_adapter.sv` | Command mailbox, descriptor validation, initialization, and interrupt handling |
| `router/router_wrapper.sv` | Board-level wrapper, PCIe/DDR integration, and clock generation |

## License

SegGR is released under the Apache License, Version 2.0. See [`LICENSE`](LICENSE) for the full license text.
