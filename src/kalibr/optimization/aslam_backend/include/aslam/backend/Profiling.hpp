#ifndef ASLAM_BACKEND_PROFILING_HPP
#define ASLAM_BACKEND_PROFILING_HPP

#include <sm/timing/Timer.hpp>

namespace aslam {
namespace backend {

// The release solver keeps the same numerical path without reading clocks.
#ifdef KALIBR_NATIVE_ENABLE_PROFILING
using ProfilingTimer = sm::timing::Timer;
#else
using ProfilingTimer = sm::timing::DummyTimer;
#endif

}  // namespace backend
}  // namespace aslam

#endif
