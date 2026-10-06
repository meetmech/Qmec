#pragma once

#include <cstdint>
#include <vector>

namespace qmec::assets
{
    struct ImageData
    {
        std::uint32_t width{};
        std::uint32_t height{};
        std::vector<std::uint8_t> pixels{};
    };

    [[nodiscard]] bool LoadImageRgba8(const char* filePath,ImageData& output);
}
namespace qmec
{
    using assets::ImageData;
    using assets::LoadImageRgba8;
}
