#pragma once

// CIGI CCL（CigiOutgoingMsg → Windows.h；CigiMessage → `using namespace std`）
// 在 MSVC STL / clang-cl 上与 C++17 std::byte 冲突。在包含 CIGI pack/session
// 头的 TU 中禁用 std::byte。
#if defined(_WIN32) && !defined(_HAS_STD_BYTE)
#    define _HAS_STD_BYTE 0 // NOLINT(readability-identifier-naming)
#endif

#ifndef NOMINMAX
#    define NOMINMAX
#endif
