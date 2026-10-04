#pragma once

#include <cstdint>
#include <vector>

namespace qmec
{
    struct ImageData
    {
        std::uint32_t width{};
        std::uint32_t height{};
        std::vector<std::uint8_t> pixels{};
    };

    [[nodiscard]] bool LoadImageRgba8(const char* filePath,ImageData& output);
}