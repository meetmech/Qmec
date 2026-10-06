#pragma once
#include <d3d11.h>
#include <wrl/client.h>
#include <cstdint>


namespace qmec::graphics
{
struct GPUBuffer
{
        Microsoft::WRL::ComPtr<ID3D11Buffer> vertexBuffer{};
        Microsoft::WRL::ComPtr<ID3D11Buffer> indexBuffer{};
        std::uint32_t indexCount{};

};

}
namespace qmec
{
    using graphics::GPUBuffer;
}
