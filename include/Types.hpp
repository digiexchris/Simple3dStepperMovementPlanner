#pragma once
#include <array>
#include <cstdint>
#include <stdint.h>
#include <vector>

template <typename T>
using Vector = std::array<T, 3>;

using Vec3Uint8 = Vector<uint8_t>;
using Vec3Int8 = Vector<int8_t>;

using Vec3Int32 = Vector<int32_t>;

using Vecr3UInt16 = Vector<uint16_t>;

using Vec3Int8List = std::vector<Vec3Int8>;

using Vec3Int32List = std::vector<Vec3Int32>;