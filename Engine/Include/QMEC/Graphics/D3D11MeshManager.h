#pragma once

#include <d3d11.h>
#include <wrl/client.h>

#include <cstdint>
#include <vector>

#include "QMEC/Assets/AssetManager.h"
#include "QMEC/Graphics/GPUBuffer.h"
#include "QMEC/Graphics/MeshData.h"

namespace qmec::graphics
{
    class D3D11MeshManager
    {
    public:
        explicit D3D11MeshManager(
            Microsoft::WRL::ComPtr<ID3D11Device> device);

        bool CreateMesh(MeshHandle handle,const MeshData& meshData);

        const GPUBuffer* GetMesh(MeshHandle handle) const noexcept;

    private:
        Microsoft::WRL::ComPtr<ID3D11Device> device_;
        std::vector<GPUBuffer> meshes_;
    };
}

namespace qmec
{
    using graphics::D3D11MeshManager;
}
