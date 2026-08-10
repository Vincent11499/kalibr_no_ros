#pragma once

// GCC 7+ in C++17 mode can instantiate Eigen's generic operators while Boost
// Serialization probes an intentionally incomplete placeholder type named U.
// Giving that placeholder inert Eigen traits prevents the probe from becoming
// a hard error. This header is force-included for all C++ targets so the
// upstream snapshot can remain byte-for-byte unchanged.
#if defined(__GNUC__) && __GNUC__ >= 7 && __cplusplus >= 201703L
namespace boost {
namespace serialization {
struct U;
}  // namespace serialization
}  // namespace boost

namespace Eigen {
namespace internal {
template <typename T>
struct traits;

template <>
struct traits<boost::serialization::U> {
  enum { Flags = 0 };
};
}  // namespace internal
}  // namespace Eigen
#endif
