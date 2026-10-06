#include "QMEC/Assets/AssetSystem.h"

#include <utility>

namespace qmec::assets
{
    MeshHandle AssetSystem::CreateAsset(MeshData meshData)
    {
        const MeshHandle handle = assetManager_.AddMesh(std::move(meshData));
        if (!handle.IsValid())
        {
            return {};
        }

        try
        {
            const MeshData* storedMesh = assetManager_.GetMesh(handle);
            if (gpuMeshManager_.CreateMesh(handle, *storedMesh))
            {
                return handle;
            }
        }
        catch (...)
        {
            assetManager_.RollbackLastMesh(handle);
            throw;
        }

        assetManager_.RollbackLastMesh(handle);
        return {};
    }

    MeshHandle AssetSystem::CreateAsset(const IMeshGenerator& generator)
    {
        return CreateAsset(generator.Generate());
    }

    const MeshData* AssetSystem::GetMesh(MeshHandle handle) const noexcept
    {
        return assetManager_.GetMesh(handle);
    }

    const GPUBuffer* AssetSystem::GetGPUMesh(MeshHandle handle) const noexcept
    {
        return gpuMeshManager_.GetMesh(handle);
    }

    std::size_t AssetSystem::GetMeshCount() const noexcept
    {
        return assetManager_.GetMeshCount();
    }
}
