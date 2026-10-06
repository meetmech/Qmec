#pragma once
#include "QMEC/Graphics/Debug/DebugRenderer.h"
#include "QMEC/Graphics/D3D11Renderer.h"
#include "QMEC/Scene/Scene.h"

#include <QMainWindow>
#include <QString>
#include <cstdint>
#include <memory>

class QTimer;

namespace qmec::editor
{
    class ViewportSurface;
    class Viewport;
    class GameViewport;
    class GameViewportSurface;
    class HierarchyPanel;
    class InspectorPanel;
    enum class InspectorComponentType : std::uint8_t;
}

namespace qmec::assets
{
    class AssetManager;
    class AssetSystem;
}

namespace qmec::graphics
{
    class D3D11MeshManager;
}

namespace qmec::scene::factory
{
    class EntityFactory;
}

namespace qmec::scene::components
{
    struct ColliderComponent;
    struct CameraComponent;
    struct DirectionalLightComponent;
    struct MaterialComponent;
    struct RigidBodyComponent;
    struct TransformComponent;
}

namespace qmec::editor
{
    class Editor final : public QMainWindow
    {
    public:
        Editor();
        ~Editor() override;
   

    private:
        void InitializeEngine();
        void InitializeGameRenderer();
        [[nodiscard]] bool StartPlayMode();
        [[nodiscard]] bool StopPlayMode();
        void SaveScene();
        void DeleteSelectedEntity();
        void OnViewportResized(int width, int height);
        void OnGameViewportResized(int width, int height);
        void OnEntitySelected(qmec::ecs::Entity entity);
        void OnEntityReparentRequested(qmec::ecs::Entity child, qmec::ecs::Entity parent);
        void OnComponentAddRequested(qmec::ecs::Entity entity, InspectorComponentType componentType);
        void OnComponentRemoveRequested(qmec::ecs::Entity entity, InspectorComponentType componentType);
        void OnScriptAddRequested(qmec::ecs::Entity entity, const QString& typeId);
        void OnScriptRemoveRequested(qmec::ecs::Entity entity, const QString& typeId);
        void OnScriptEntityReferenceEdited(qmec::ecs::Entity owner, const QString& typeId,
            const QString& fieldId, qmec::ecs::Entity referencedEntity);
        void OnTransformEdited(qmec::ecs::Entity entity,
        const qmec::scene::components::TransformComponent& transform);
        void OnDirectionalLightEdited(qmec::ecs::Entity entity,const qmec::scene::components::DirectionalLightComponent& light);
        void OnCameraEdited(qmec::ecs::Entity entity,const qmec::scene::components::CameraComponent& camera);
        void OnRigidBodyEdited(qmec::ecs::Entity entity, const qmec::scene::components::RigidBodyComponent& rigidbody);
        void OnColliderEdited(qmec::ecs::Entity entity,const qmec::scene::components::ColliderComponent& collider);
        void OnMaterialEdited(qmec::ecs::Entity entity, const qmec::scene::components::MaterialComponent& material);
        void Tick();

        qmec::graphics::D3D11Renderer renderer_{};
        qmec::graphics::D3D11Renderer gameRenderer_{};
        qmec::graphics::debug::DebugRenderer debugRenderer_{};
        qmec::scene::Scene scene_{};
        qmec::ecs::Entity selectedEntity_{};
        std::unique_ptr<qmec::assets::AssetManager> assetManager_;
        std::unique_ptr<qmec::graphics::D3D11MeshManager> gpuMeshManager_;
        std::unique_ptr<qmec::assets::AssetSystem> assetSystem_;
        std::unique_ptr<qmec::scene::factory::EntityFactory> entityFactory_;
        HierarchyPanel* hierarchyPanel_{nullptr};
        InspectorPanel* inspectorPanel_{nullptr};
        Viewport* viewport_{nullptr};
        ViewportSurface* viewportSurface_{nullptr};
        GameViewport* gameViewport_{nullptr};
        GameViewportSurface* gameViewportSurface_{nullptr};
        bool initializing_{false};
        bool rendererInitialized_{false};
        bool gameRendererInitialized_{false};
        bool gameRendererInitializing_{false};
        bool playing_{false};
        QString sceneFilePath_{};
        QString playSnapshotPath_{};
        std::uint64_t lastHierarchyRevision_{};
        QTimer* timer_{nullptr};
    };

}
