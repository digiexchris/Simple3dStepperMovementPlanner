#pragma once
#include <array>
#include <bits/stdint-uintn.h>
#include <cstdint>
#include <vector>

template <typename T> using Vector = std::array<T, 3>;

using Vector3Int32 = Vector<int32_t>;

using Vector3UInt16 = Vector<uint16_t>;

using Vector3Int32List = std::vector<Vector3Int32>;