#pragma once
#include "QMEC/Graphics/Debug/DebugRenderer.h"
#include "QMEC/Graphics/D3D11Renderer.h"
#include "QMEC/Scene/Scene.h"

#include <QMainWindow>
#include <QString>
#include <cstdint>
#include <memory>

class QTimer;
class ViewportSurface;
class Viewport;
class GameViewport;
class GameViewportSurface;
class HierarchyPanel;
class InspectorPanel;
enum class InspectorComponentType : std::uint8_t;

namespace qmec
{
    class AssetManager;
    class AssetSystem;
    class D3D11MeshManager;
    class EntityFactory;
    struct ColliderComponent;
    struct CameraComponent;
    struct DirectionalLightComponent;
    struct MaterialComponent;
    struct RigidBodyComponent;
    struct TransformComponent;
}

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
    void OnEntitySelected(qmec::Entity entity);
    void OnEntityReparentRequested(qmec::Entity child, qmec::Entity parent);
    void OnComponentAddRequested(qmec::Entity entity, InspectorComponentType componentType);
    void OnComponentRemoveRequested(qmec::Entity entity, InspectorComponentType componentType);
    void OnScriptAddRequested(qmec::Entity entity, const QString& typeId);
    void OnScriptRemoveRequested(qmec::Entity entity, const QString& typeId);
    void OnScriptEntityReferenceEdited(qmec::Entity owner, const QString& typeId,
        const QString& fieldId, qmec::Entity referencedEntity);
    void OnTransformEdited(qmec::Entity entity,
    const qmec::TransformComponent& transform);
    void OnDirectionalLightEdited(qmec::Entity entity,const qmec::DirectionalLightComponent& light);
    void OnCameraEdited(qmec::Entity entity,const qmec::CameraComponent& camera);
    void OnRigidBodyEdited(qmec::Entity entity, const qmec::RigidBodyComponent& rigidbody);
    void OnColliderEdited(qmec::Entity entity,const qmec::ColliderComponent& collider);
    void OnMaterialEdited(qmec::Entity entity, const qmec::MaterialComponent& material);
    void Tick();

    qmec::D3D11Renderer renderer_{};
    qmec::D3D11Renderer gameRenderer_{};
    qmec::DebugRenderer debugRenderer_{};
    qmec::Scene scene_{};
    qmec::Entity selectedEntity_{};
    std::unique_ptr<qmec::AssetManager> assetManager_;
    std::unique_ptr<qmec::D3D11MeshManager> gpuMeshManager_;
    std::unique_ptr<qmec::AssetSystem> assetSystem_;
    std::unique_ptr<qmec::EntityFactory> entityFactory_;
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
