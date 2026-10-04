#include "QMEC/Graphics/D3D11MeshManager.h"

#include <utility>
#include <limits>

namespace qmec
{
    D3D11MeshManager::D3D11MeshManager(Microsoft::WRL::ComPtr<ID3D11Device> device) : device_(std::move(device))
    {
    }

    bool D3D11MeshManager::CreateMesh(MeshHandle handle,const MeshData& meshData)
    {
        if (!handle.IsValid() || device_ == nullptr
            || meshData.vertices.empty() || meshData.indices.empty()
            || meshData.indices.size() % 3U != 0U
            || meshData.vertices.size() > (std::numeric_limits<UINT>::max)() / sizeof(Vertex)
            || meshData.indices.size() > (std::numeric_limits<UINT>::max)() / sizeof(std::uint32_t))
        {
            return false;
        }

        for (const std::uint32_t index : meshData.indices)
        {
            if (index >= meshData.vertices.size())
            {
                return false;
            }
        }

        GPUBuffer gpuBuffer{};

        D3D11_BUFFER_DESC vertexBufferDescription{};
        vertexBufferDescription.ByteWidth = static_cast<UINT>(meshData.vertices.size() * sizeof(Vertex));

        vertexBufferDescription.Usage = D3D11_USAGE_IMMUTABLE;
        vertexBufferDescription.BindFlags = D3D11_BIND_VERTEX_BUFFER;

        D3D11_SUBRESOURCE_DATA vertexData{};
        vertexData.pSysMem = meshData.vertices.data();

        const HRESULT vertexBufferResult = device_->CreateBuffer(&vertexBufferDescription,&vertexData,gpuBuffer.vertexBuffer.GetAddressOf());

        if (FAILED(vertexBufferResult))
        {
            return false;
        }

        D3D11_BUFFER_DESC indexBufferDescription{};
        indexBufferDescription.ByteWidth =static_cast<UINT>( meshData.indices.size() * sizeof(std::uint32_t));

        indexBufferDescription.Usage = D3D11_USAGE_IMMUTABLE;
        indexBufferDescription.BindFlags = D3D11_BIND_INDEX_BUFFER;

        D3D11_SUBRESOURCE_DATA indexData{};
        indexData.pSysMem = meshData.indices.data();

        const HRESULT indexBufferResult = device_->CreateBuffer(&indexBufferDescription,&indexData,gpuBuffer.indexBuffer.GetAddressOf());

        if (FAILED(indexBufferResult))
        {
            return false;
        }

        gpuBuffer.indexCount = static_cast<std::uint32_t>(meshData.indices.size());
        if (handle.index >= meshes_.size())
        {
            meshes_.resize(static_cast<std::size_t>(handle.index) + 1U);
        }
        meshes_[handle.index] = std::move(gpuBuffer);
        return true;
    }

    const GPUBuffer* D3D11MeshManager::GetMesh(MeshHandle handle) const noexcept
    {
        if (!handle.IsValid() ||
            handle.index >= meshes_.size())
        {
            return nullptr;
        }

        const GPUBuffer& mesh = meshes_[handle.index];
        return mesh.vertexBuffer != nullptr && mesh.indexBuffer != nullptr? &mesh : nullptr;
    }
}
