#pragma once

#include <boost/predef/other/endian.h>

#if BOOST_ENDIAN_BIG_BYTE
#define BOOST_BIG_ENDIAN
#elif BOOST_ENDIAN_LITTLE_BYTE
#define BOOST_LITTLE_ENDIAN
#else
#error "Unable to determine host byte order with Boost.Predef"
#endif
