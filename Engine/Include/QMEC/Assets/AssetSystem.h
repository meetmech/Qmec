#pragma once


#include "QMEC/Assets/AssetManager.h"
#include "QMEC/Graphics/D3D11MeshManager.h"
#include "QMEC/Graphics/IMeshGenerator.h"

namespace qmec::assets
{
    class AssetSystem
    {
    public:
        AssetSystem(AssetManager& assetManager,D3D11MeshManager& gpuMeshManager): assetManager_(assetManager),
                                                                                gpuMeshManager_(gpuMeshManager)
        {
        }

        [[nodiscard]] MeshHandle CreateAsset(MeshData meshData);
        [[nodiscard]] MeshHandle CreateAsset(const IMeshGenerator& generator);

        [[nodiscard]] const MeshData* GetMesh(MeshHandle handle) const noexcept;
        [[nodiscard]] const GPUBuffer* GetGPUMesh(MeshHandle handle) const noexcept;
        [[nodiscard]] std::size_t GetMeshCount() const noexcept;

    private:
        AssetManager& assetManager_;
        D3D11MeshManager& gpuMeshManager_;
    };
}

namespace qmec
{
    using assets::AssetSystem;
}
