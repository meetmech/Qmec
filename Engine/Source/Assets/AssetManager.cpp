#include "QMEC/Assets/AssetManager.h"

#include <utility>

namespace qmec
{
    MeshHandle AssetManager::AddMesh(MeshData mesh)
    {
        if (meshes_.size() >= InvalidMeshIndex)
        {
            return {};
        }

        const MeshHandle handle{static_cast<std::uint32_t>(meshes_.size())};

        meshes_.push_back(std::move(mesh));
        return handle;
    }

    void AssetManager::RollbackLastMesh(MeshHandle handle) noexcept
    {
        if (handle.IsValid() && !meshes_.empty() && handle.index == meshes_.size() - 1U)
        {
            meshes_.pop_back();
        }
    }

    MeshData* AssetManager::GetMesh(MeshHandle handle) noexcept
    {
        if (!handle.IsValid() || handle.index >= meshes_.size())
        {
            return nullptr;
        }

        return &meshes_[handle.index];
    }

    const MeshData* AssetManager::GetMesh(MeshHandle handle) const noexcept
    {
        if (!handle.IsValid() || handle.index >= meshes_.size())
        {
            return nullptr;
        }

        return &meshes_[handle.index];
    }

    std::size_t AssetManager::GetMeshCount() const noexcept
    {
        return meshes_.size();
    }
}
