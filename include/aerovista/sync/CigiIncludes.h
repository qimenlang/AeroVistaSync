#pragma once

// CIGI CCL (CigiOutgoingMsg → Windows.h; CigiMessage → `using namespace std`)
// collides with C++17 std::byte on MSVC STL / clang-cl. Disable std::byte in TUs
// that include CIGI pack/session headers.
#if defined(_WIN32) && !defined(_HAS_STD_BYTE)
#    define _HAS_STD_BYTE 0 // NOLINT(readability-identifier-naming)
#endif

#ifndef NOMINMAX
#    define NOMINMAX
#endif
