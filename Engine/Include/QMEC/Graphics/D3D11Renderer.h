#pragma once

#include "QMEC/ECS/Entity.h"
#include "QMEC/Graphics/Debug/DebugLine.h"

#include <cstdint>
#include <memory>
#include <span>
#include <wrl/client.h>

struct ID3D11Device;

namespace qmec::scene { class Camera; class Scene; }
namespace qmec::assets { class AssetSystem; }

namespace qmec::graphics
{

    class D3D11Renderer final
    {
    public:
        D3D11Renderer();
        ~D3D11Renderer() noexcept;

        D3D11Renderer(const D3D11Renderer&) = delete;
        D3D11Renderer& operator=(const D3D11Renderer&) = delete;

        [[nodiscard]] bool Initialize(void* nativeWindowHandle,std::uint32_t width,std::uint32_t height) noexcept;
        [[nodiscard]] bool Initialize(
            void* nativeWindowHandle,
            std::uint32_t width,
            std::uint32_t height,
            ID3D11Device* sharedDevice) noexcept;
        [[nodiscard]] bool Resize(std::uint32_t width, std::uint32_t height) noexcept;
        [[nodiscard]] bool ClearFrame(float red, float green, float blue, float alpha) noexcept;

        [[nodiscard]] Microsoft::WRL::ComPtr<ID3D11Device> GetDevice() const noexcept;

        [[nodiscard]] bool RenderFrame(float red,float green,float blue,float alpha,const qmec::scene::Scene& scene, const qmec::assets::AssetSystem& assets,
                                       const qmec::scene::Camera& camera, std::span<const DebugLine> debugLines = {}) noexcept;
        [[nodiscard]] bool RenderFrame(float red,float green,float blue,float alpha,const qmec::scene::Scene& scene, const qmec::assets::AssetSystem& assets,
                                       qmec::ecs::Entity cameraEntity, std::span<const DebugLine> debugLines = {}) noexcept;

    private:
        struct Implementation;
        std::unique_ptr<Implementation> implementation_;
    };
}

namespace qmec
{
    using graphics::D3D11Renderer;
}
