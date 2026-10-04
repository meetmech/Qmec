#pragma once

#include "QMEC/Graphics/MeshData.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace qmec
{
    inline constexpr std::uint32_t InvalidMeshIndex = (std::numeric_limits<std::uint32_t>::max)();

    struct MeshHandle
    {
        std::uint32_t index{InvalidMeshIndex};

        [[nodiscard]] bool IsValid() const noexcept
        {
            return index != InvalidMeshIndex;
        }

        bool operator==(const MeshHandle&) const noexcept = default;
    };

    class AssetManager final
    {
    public:
        [[nodiscard]] MeshHandle AddMesh(MeshData mesh);
        [[nodiscard]] MeshData* GetMesh(MeshHandle handle) noexcept;
        [[nodiscard]] const MeshData* GetMesh(MeshHandle handle) const noexcept;

        [[nodiscard]] std::size_t GetMeshCount() const noexcept;

    private:
        friend class AssetSystem;
        void RollbackLastMesh(MeshHandle handle) noexcept;
        std::vector<MeshData> meshes_{};
    };
}
