# Deterministic BlockCholesky Hessian construction

## Contract

The optimizer overlay leaves `Optimizer2`, the trust-region policy, LM
parameters, error-term order, and the CHOLMOD/SPQR solve unchanged.

- `nThreads == 0` and `nThreads == 1` execute the original
  `ErrorTerm::buildHessian()` loop. This is the native reference path and is
  bitwise unchanged.
- `nThreads > 1` partitions `_errorTerms[0..N)` into a fixed set of contiguous
  index ranges. Every worker calls each error term's unmodified
  `buildHessian()` method, in increasing error index, into one chunk-local
  sparse block matrix and RHS.
- Completed chunks are merged into the shared Hessian/RHS in increasing chunk
  index. Scheduling cannot change chunk boundaries, arithmetic inside a chunk,
  or reduction order, so repeated builds with the same thread count are
  bitwise deterministic.
- Chunk grouping changes floating-point parentheses relative to the serial
  per-error accumulation. Serial and parallel results are therefore required
  to agree to a strict numerical tolerance, not bit-for-bit. Different thread
  counts have different fixed chunk boundaries and are not promised to be
  bitwise identical to each other.
- Kalibr's `BSplineMotionError` is a direct quadratic error and intentionally
  has no Jacobian representation. It works through the same unchanged
  `buildHessian()` path; no type check or exception-string fallback is used.

Each local chunk uses the exact native expression path, including square-root
covariance, M-estimator weighting, design-variable scaling, and direct
quadratic contributions. The parallel implementation changes only where a
range accumulates its normal equations and when that range is reduced.

## Why chunks replace per-error buffers

The first deterministic implementation built one local sparse matrix and one
full-length RHS for every error result, then reduced results one by one. Even
with a bounded ring, every error caused all of the following:

1. allocate and free every dense sparse-matrix block it touched;
2. scan the complete global block-column layout during reduction; and
3. add a parameter-dimension RHS vector, most of whose entries were zero.

For the ROS2 calibrated dataset this work repeats for roughly 300,000 error
terms on every optimizer iteration. It dominated the useful Jacobian/Hessian
math and made four threads much slower than native serial execution.

The fixed-chunk implementation performs sparse allocation into an accumulating
chunk, scans the block layout once per chunk, and adds one full RHS per chunk.
The unchanged per-error math still runs in original sequence within each
range. For `T` threads, layout scans and full-RHS reductions fall from `O(N)`
to exactly `T` per Hessian build.

## Thread safety and exceptions

The fixed- and dynamic-size `ErrorTerm` implementations in this Kalibr snapshot
alias their internal timer type to `sm::timing::DummyTimer`; their real shared
timing accumulators are explicitly disabled in the headers. Calling distinct
error terms' `buildHessian()` implementations concurrently is therefore safe
under the same read-only design-variable contract already required by Kalibr's
threaded dense and sparse solvers. If upstream switches those aliases back to
`sm::timing::Timer`, synchronization must be added around the shared timer.

As in the other native Kalibr threaded solvers, custom error terms must permit
concurrent read-only Jacobian evaluation across distinct objects. A worker
captures any exception and the caller rethrows it after all workers join;
partial normal equations are never passed to the linear solver.

## Memory bound

Each worker owns one sparse matrix for its contiguous range and one full RHS.
Temporary memory is

`O(nThreads * parameter_dimension + sum(chunk_sparse_patterns))`.

Time-local spline blocks tend to partition across contiguous ranges. Global
camera/IMU calibration blocks are replicated once per chunk. Increasing thread
count therefore trades additional normal-equation storage for lower assembly
latency; it does not duplicate a dense global Hessian.

## Regression test and benchmark

`tests/block_cholesky_parallel_test.cpp` is built and registered by the
top-level CMake configuration. Its default mode covers:

- bitwise identity of the native `nThreads=0/1` path;
- strict numerical agreement between native and 2/4/8-thread builds;
- bitwise repeatability at each fixed parallel thread count;
- a 10,000-error accumulation-order stress case;
- observed concurrent Jacobian evaluation;
- M-estimator enabled and disabled;
- non-unit design-variable scaling;
- native direct-Hessian support for quadratic errors; and
- worker exception propagation without deadlock.

The same executable provides an opt-in, Kalibr-shaped assembly benchmark:

```text
kalibr_block_cholesky_parallel_test --benchmark THREADS [ERRORS] [REPETITIONS]
```

It uses 6-dimensional design variables, four time-local and four global blocks
per 2D error, non-unit scaling, correlated covariance, and a Huber M-estimator.
Benchmark mode is not a pass/fail CI assertion because scheduler and thermal
behavior are machine-dependent.

On the 32-logical-CPU Ryzen 9 8940HX validation host, 200,000 errors and seven
measured repetitions produced:

| Threads | Median build | Speedup | Peak RSS | RSS over serial |
|---:|---:|---:|---:|---:|
| 1 | 0.880 s | 1.00x | 265.5 MiB | - |
| 2 | 0.562 s | 1.57x | 277.1 MiB | +11.6 MiB |
| 4 | 0.308 s | 2.85x | 290.2 MiB | +24.7 MiB |
| 8 | 0.218 s | 4.04x | 313.7 MiB | +48.2 MiB |

Peak RSS includes the retained 200,000-error synthetic graph; the last column
isolates the observed process-level increase relative to serial. On a smaller
30,000-error comparison, the removed per-error-ring implementation took
1.680/0.910/0.936 s at 2/4/8 threads, while fixed chunks took
0.109/0.073/0.087 s. This isolates the allocation, full-layout scan, and
full-RHS reduction regression that triggered the redesign.

For race detection, compile the library and test with
`-fsanitize=thread -fno-omit-frame-pointer`, then run the default test with no
other Kalibr process in the same build tree.
