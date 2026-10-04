#include "QMEC/Assets/ImageData.h"

#include <stb_image.h>

#include <cstddef>
#include <memory>

namespace qmec
{
    bool LoadImageRgba8(const char* filePath, ImageData& output)
    {
        output = {};

        if(filePath == nullptr || filePath[0] == '\0')
        {
            return false;
        }

        int width{};
        int height{};
        int sourceChannelCount{};

        using StbiPixels = std::unique_ptr<stbi_uc, decltype(&stbi_image_free)>;

        StbiPixels decodedPixels{stbi_load(filePath,&width,&height,&sourceChannelCount,STBI_rgb_alpha),&stbi_image_free};

        if(decodedPixels == nullptr || width <= 0 || height <= 0)
        {
            return false;
        }

        constexpr std::size_t RgbaChannelCount = 4U;
        const std::size_t byteCount = static_cast<std::size_t>(width) *static_cast<std::size_t>(height) * RgbaChannelCount;

        output.width = static_cast<std::uint32_t>(width);
        output.height = static_cast<std::uint32_t>(height);
        output.pixels.assign(decodedPixels.get(),decodedPixels.get() + byteCount);

        return true;
    }
}
