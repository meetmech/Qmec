#include "Editor.h"
#include "panels/Hierarchy/HierarchyPanel.h"
#include "panels/Inspector/InspectorPanel.h"
#include "panels/Viewport/Viewport.h"
#include "panels/GameViewport/GameViewport.h"
#include "QMEC/Assets/AssetManager.h"
#include "QMEC/Assets/AssetSystem.h"
#include "QMEC/Graphics/D3D11MeshManager.h"
#include "QMEC/Graphics/Debug/DebugRenderer.h"
#include "QMEC/Physics/Collider/WorldShapes.h"
#include "QMEC/Scene/Factory/EntityFactory.h"
#include "QMEC/Scene/Components/CameraComponent.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/DirectionalLightComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/MeshRendererComponent.h"
#include "QMEC/Scene/SceneSerializer.h"
#include "QMEC/Scripting/Script.h"
#include "Game/RegisterSandboxScripts.h"

#include <QAction>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeySequence>
#include <QLineEdit>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QSettings>
#include <QStatusBar>
#include <QTemporaryFile>
#include <QToolBar>
#include <QTimer>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace qmec::editor
{
    namespace
    {
        struct EntityWorldBasis
        {
            qmec::math::Mat4 matrix{};
            qmec::math::Vec3 position{};
            qmec::math::Vec3 axisX{};
            qmec::math::Vec3 axisY{};
            qmec::math::Vec3 axisZ{};
            qmec::math::Vec3 scale{};
        };

        bool GetEntityWorldBasis(const qmec::scene::Scene& scene, qmec::ecs::Entity entity, EntityWorldBasis& basis) noexcept
        {
            basis.matrix = scene.GetWorldMatrix(entity);
            const qmec::math::Vec3 rawX = basis.matrix.TransformDirection({ 1.0f, 0.0f, 0.0f });
            const qmec::math::Vec3 rawY = basis.matrix.TransformDirection({ 0.0f, 1.0f, 0.0f });
            const qmec::math::Vec3 rawZ = basis.matrix.TransformDirection({ 0.0f, 0.0f, 1.0f });
            basis.scale = { rawX.Length(), rawY.Length(), rawZ.Length() };
            if (basis.scale.x <= 1.0e-6f || basis.scale.y <= 1.0e-6f || basis.scale.z <= 1.0e-6f)
                return false;

            basis.position = scene.GetWorldPosition(entity);
            basis.axisX = rawX / basis.scale.x;
            basis.axisY = rawY / basis.scale.y;
            basis.axisZ = rawZ / basis.scale.z;
            return true;
        }

        qmec::math::Vec3 TransformLocalPoint(const EntityWorldBasis& basis, const qmec::math::Vec3& point) noexcept
        {
            return basis.matrix.TransformDirection(point) + basis.position;
        }

        void DrawSelectedEntityOutline(
            qmec::graphics::debug::DebugRenderer& debugRenderer,
            const qmec::scene::Scene& scene,
            const qmec::assets::AssetSystem& assets,
            qmec::ecs::Entity entity)
        {
            constexpr qmec::math::Vec3 selectedColor{ 1.0f, 0.72f, 0.05f };
            constexpr float outlinePadding = 0.01f;

            EntityWorldBasis basis{};
            if (!GetEntityWorldBasis(scene, entity, basis))
                return;

            const qmec::ecs::Registry& registry = scene.GetRegistry();
            const auto* collider = registry.GetComponent<qmec::scene::components::ColliderComponent>(entity);
            if (collider != nullptr)
            {
                if (const auto* box = std::get_if<qmec::physics::BoxShape>(&collider->shape))
                {
                    qmec::physics::WorldBox worldBox{};
                    worldBox.centre = TransformLocalPoint(basis, box->centre);
                    worldBox.halfExtents = {
                        std::abs(box->halfExtents.x) * basis.scale.x + outlinePadding,
                        std::abs(box->halfExtents.y) * basis.scale.y + outlinePadding,
                        std::abs(box->halfExtents.z) * basis.scale.z + outlinePadding };
                    worldBox.axisX = basis.axisX;
                    worldBox.axisY = basis.axisY;
                    worldBox.axisZ = basis.axisZ;
                    debugRenderer.DrawBox(worldBox, selectedColor);
                    return;
                }

                if (const auto* sphere = std::get_if<qmec::physics::SphereShape>(&collider->shape))
                {
                    const float worldRadius = std::abs(sphere->radius) *
                        (std::max)({ basis.scale.x, basis.scale.y, basis.scale.z });
                    debugRenderer.DrawSphere(TransformLocalPoint(basis, sphere->centre),
                        worldRadius + outlinePadding, selectedColor);
                    return;
                }

                if (const auto* cylinder = std::get_if<qmec::physics::CylinderShape>(&collider->shape))
                {
                    qmec::physics::WorldCylinder worldCylinder{};
                    worldCylinder.centre = TransformLocalPoint(basis, cylinder->centre);
                    worldCylinder.radius = std::abs(cylinder->radius) *
                        (std::max)(basis.scale.x, basis.scale.z) + outlinePadding;
                    worldCylinder.halfHeight = std::abs(cylinder->halfHeight) * basis.scale.y + outlinePadding;
                    worldCylinder.axisX = basis.axisX;
                    worldCylinder.axisY = basis.axisY;
                    worldCylinder.axisZ = basis.axisZ;
                    debugRenderer.DrawCylinder(worldCylinder, selectedColor);
                    return;
                }

                if (const auto* plane = std::get_if<qmec::physics::PlaneShape>(&collider->shape))
                {
                    qmec::physics::WorldPlane worldPlane{};
                    worldPlane.centre = TransformLocalPoint(basis, plane->centre);
                    worldPlane.halfWidthX = std::abs(plane->halfWidthX) * basis.scale.x + outlinePadding;
                    worldPlane.halfLengthZ = std::abs(plane->halfLengthZ) * basis.scale.z + outlinePadding;
                    worldPlane.axisX = basis.axisX;
                    worldPlane.normal = basis.axisY;
                    worldPlane.axisZ = basis.axisZ;
                    debugRenderer.DrawPlane(worldPlane, selectedColor);
                    return;
                }
            }

            const auto* meshRenderer = registry.GetComponent<qmec::scene::components::MeshRendererComponent>(entity);
            const qmec::graphics::MeshData* mesh = meshRenderer != nullptr ? assets.GetMesh(meshRenderer->mesh) : nullptr;
            if (mesh != nullptr && !mesh->vertices.empty())
            {
                qmec::math::Vec3 minimum = mesh->vertices.front().position;
                qmec::math::Vec3 maximum = minimum;
                for (const qmec::graphics::Vertex& vertex : mesh->vertices)
                {
                    minimum.x = (std::min)(minimum.x, vertex.position.x);
                    minimum.y = (std::min)(minimum.y, vertex.position.y);
                    minimum.z = (std::min)(minimum.z, vertex.position.z);
                    maximum.x = (std::max)(maximum.x, vertex.position.x);
                    maximum.y = (std::max)(maximum.y, vertex.position.y);
                    maximum.z = (std::max)(maximum.z, vertex.position.z);
                }

                const qmec::math::Vec3 localCentre = (minimum + maximum) * 0.5f;
                const qmec::math::Vec3 localHalfExtents = (maximum - minimum) * 0.5f;
                qmec::physics::WorldBox worldBox{};
                worldBox.centre = TransformLocalPoint(basis, localCentre);
                worldBox.halfExtents = {
                    localHalfExtents.x * basis.scale.x + outlinePadding,
                    localHalfExtents.y * basis.scale.y + outlinePadding,
                    localHalfExtents.z * basis.scale.z + outlinePadding };
                worldBox.axisX = basis.axisX;
                worldBox.axisY = basis.axisY;
                worldBox.axisZ = basis.axisZ;
                debugRenderer.DrawBox(worldBox, selectedColor);
                return;
            }

            constexpr float markerHalfLength = 0.18f;
            debugRenderer.DrawLine(basis.position - basis.axisX * markerHalfLength,
                basis.position + basis.axisX * markerHalfLength, selectedColor);
            debugRenderer.DrawLine(basis.position - basis.axisY * markerHalfLength,
                basis.position + basis.axisY * markerHalfLength, selectedColor);
            debugRenderer.DrawLine(basis.position - basis.axisZ * markerHalfLength,
                basis.position + basis.axisZ * markerHalfLength, selectedColor);
        }
    }

    Editor::Editor() : QMainWindow(nullptr), timer_(new QTimer(this))
    {
        qmec::game::RegisterSandboxScripts();

        setWindowTitle("QMEC Editor");
        resize(1280, 800);

        auto* primitiveToolbar = addToolBar("Primitives");
        primitiveToolbar->setObjectName("PrimitiveToolbar");
        primitiveToolbar->setMovable(false);
        primitiveToolbar->setFloatable(false);
        primitiveToolbar->toggleViewAction()->setEnabled(false);

        QAction* playModeAction = primitiveToolbar->addAction("Play");
        playModeAction->setCheckable(true);
        connect(playModeAction, &QAction::toggled, this,
            [this, playModeAction](bool playing)
            {
                const bool succeeded = playing
                    ? StartPlayMode()
                    : StopPlayMode();
                if (!succeeded)
                {
                    const QSignalBlocker blocker(playModeAction);
                    playModeAction->setChecked(playing_);
                }
                playModeAction->setText(playing_ ? "Stop" : "Play");
            });

        auto* saveSceneAction = new QAction("Save Scene", this);
        saveSceneAction->setShortcut(QKeySequence::Save);
        saveSceneAction->setShortcutContext(Qt::WindowShortcut);
        addAction(saveSceneAction);
        connect(saveSceneAction, &QAction::triggered, this,
            [this]() { SaveScene(); });

        auto* deleteEntityAction = new QAction("Delete Entity", this);
        deleteEntityAction->setShortcut(QKeySequence::Delete);
        deleteEntityAction->setShortcutContext(Qt::WindowShortcut);
        addAction(deleteEntityAction);
        connect(deleteEntityAction, &QAction::triggered, this,
            [this]()
            {
                QWidget* const focusedWidget = QApplication::focusWidget();
                if (qobject_cast<QAbstractSpinBox*>(focusedWidget) != nullptr ||
                    qobject_cast<QLineEdit*>(focusedWidget) != nullptr)
                {
                    return;
                }
                DeleteSelectedEntity();
            });

        const auto addPrimitiveAction = [this, primitiveToolbar](
            const QString& label,
            auto createEntity)
        {
            QAction* action = primitiveToolbar->addAction(label);
            connect(action, &QAction::triggered, this,
                [this, createEntity]()
                {
                    if (entityFactory_ == nullptr)
                        return;

                    const qmec::ecs::Entity entity = createEntity(*entityFactory_);
                    if (!entity.IsValid())
                        return;

                    hierarchyPanel_->Refresh();
                    lastHierarchyRevision_ = scene_.GetHierarchyRevision();
                });
        };

        addPrimitiveAction("Cube", [](qmec::scene::factory::EntityFactory& factory)
            { return factory.createCubeEntity(); });
        addPrimitiveAction("Sphere", [](qmec::scene::factory::EntityFactory& factory)
            { return factory.createSphereEntity(); });
        addPrimitiveAction("Cylinder", [](qmec::scene::factory::EntityFactory& factory)
            { return factory.createCylinderEntity(); });
        addPrimitiveAction("Plane", [](qmec::scene::factory::EntityFactory& factory)
            { return factory.createPlaneEntity(); });

        hierarchyPanel_ = new HierarchyPanel(scene_, this);
        inspectorPanel_ = new InspectorPanel(this);
        viewport_ = new Viewport(this);
        viewportSurface_ = viewport_->Surface();
        gameViewport_ = new GameViewport(this);
        gameViewportSurface_ = gameViewport_->Surface();

        addDockWidget(Qt::LeftDockWidgetArea, hierarchyPanel_);
        addDockWidget(Qt::RightDockWidgetArea, viewport_);
        splitDockWidget(hierarchyPanel_, viewport_, Qt::Horizontal);
        splitDockWidget(viewport_, inspectorPanel_, Qt::Horizontal);
        tabifyDockWidget(viewport_, gameViewport_);
        viewport_->raise();
        resizeDocks({hierarchyPanel_, viewport_, inspectorPanel_}, {1, 2, 1}, Qt::Horizontal);

        connect(viewportSurface_, &ViewportSurface::Resized,this, &Editor::OnViewportResized);
        connect(gameViewportSurface_, &GameViewportSurface::Resized,
            this, &Editor::OnGameViewportResized);
        connect(hierarchyPanel_, &HierarchyPanel::EntitySelected,this, &Editor::OnEntitySelected);
        connect(hierarchyPanel_, &HierarchyPanel::EntityReparentRequested,this, &Editor::OnEntityReparentRequested);
        connect(inspectorPanel_, &InspectorPanel::TransformEdited,this, &Editor::OnTransformEdited);
        connect(inspectorPanel_, &InspectorPanel::DirectionalLightEdited,this, &Editor::OnDirectionalLightEdited);
        connect(inspectorPanel_, &InspectorPanel::CameraEdited,this, &Editor::OnCameraEdited);
        connect(inspectorPanel_, &InspectorPanel::ColliderEdited,this, &Editor::OnColliderEdited);
        connect(inspectorPanel_, &InspectorPanel::MaterialEdited, this, &Editor::OnMaterialEdited);
        connect(inspectorPanel_, &InspectorPanel::RigidBodyEdited, this, &Editor::OnRigidBodyEdited);
        connect(inspectorPanel_, &InspectorPanel::AddComponentRequested,
            this, &Editor::OnComponentAddRequested);
        connect(inspectorPanel_, &InspectorPanel::RemoveComponentRequested,
            this, &Editor::OnComponentRemoveRequested);
        connect(inspectorPanel_, &InspectorPanel::ScriptAddRequested,
            this, &Editor::OnScriptAddRequested);
        connect(inspectorPanel_, &InspectorPanel::ScriptRemoveRequested,
            this, &Editor::OnScriptRemoveRequested);
        connect(inspectorPanel_, &InspectorPanel::ScriptEntityReferenceEdited,
            this, &Editor::OnScriptEntityReferenceEdited);
        connect(timer_, &QTimer::timeout, this, &Editor::Tick);
        timer_->start(16);

        QTimer::singleShot(0, this, &Editor::InitializeEngine);
    }

    Editor::~Editor()
    {
        if (!playSnapshotPath_.isEmpty())
        {
            QFile::remove(playSnapshotPath_);
        }
    }

    bool Editor::StartPlayMode()
    {
        if (playing_)
        {
            return true;
        }

        if (!rendererInitialized_)
        {
            QMessageBox::information(this, "Play Mode",
                "The editor is still initializing. Try again in a moment.");
            return false;
        }

        QTemporaryFile snapshotFile{};
        snapshotFile.setAutoRemove(false);
        if (!snapshotFile.open())
        {
            QMessageBox::critical(this, "Play Mode",
                "Could not create a temporary scene snapshot.");
            return false;
        }

        const QString snapshotPath = snapshotFile.fileName();
        snapshotFile.close();
        if (!qmec::scene::SceneSerializer::Save(scene_, snapshotPath.toStdString()))
        {
            QFile::remove(snapshotPath);
            QMessageBox::critical(this, "Play Mode",
                "Could not snapshot the current scene. Play mode was not started.");
            return false;
        }

        playSnapshotPath_ = snapshotPath;
        playing_ = true;
        qmec::ecs::Registry& registry = scene_.GetRegistry();
        for (const qmec::ecs::Entity entity : registry.GetEntitiesWith<qmec::scripting::ScriptComponent>())
        {
            auto* scripts = registry.GetComponent<qmec::scripting::ScriptComponent>(entity);
            if (scripts == nullptr)
            {
                continue;
            }
            const qmec::scripting::ScriptContext context{scene_, entity, 0.0f};
            for (const qmec::scripting::ScriptInstance& instance : scripts->instances)
            {
                if (instance.object != nullptr)
                {
                    instance.object->OnCreate(context);
                }
            }
        }
        return true;
    }

    bool Editor::StopPlayMode()
    {
        if (!playing_)
        {
            return true;
        }

        qmec::ecs::Registry& registry = scene_.GetRegistry();
        for (const qmec::ecs::Entity entity : registry.GetEntitiesWith<qmec::scripting::ScriptComponent>())
        {
            auto* scripts = registry.GetComponent<qmec::scripting::ScriptComponent>(entity);
            if (scripts == nullptr)
            {
                continue;
            }
            const qmec::scripting::ScriptContext context{scene_, entity, 0.0f};
            for (const qmec::scripting::ScriptInstance& instance : scripts->instances)
            {
                if (instance.object != nullptr)
                {
                    instance.object->OnDestroy(context);
                }
            }
        }

        if (playSnapshotPath_.isEmpty()
            || !qmec::scene::SceneSerializer::Load(scene_, playSnapshotPath_.toStdString()))
        {
            QMessageBox::critical(this, "Play Mode",
                "Could not restore the pre-play scene. Play mode remains active.");
            return false;
        }

        playing_ = false;
        QFile::remove(playSnapshotPath_);
        playSnapshotPath_.clear();

        OnEntitySelected({});
        hierarchyPanel_->Refresh();
        hierarchyPanel_->ClearSelection();
        lastHierarchyRevision_ = scene_.GetHierarchyRevision();
        return true;
    }

    void Editor::SaveScene()
    {
        if (!rendererInitialized_)
            return;

        QString path = sceneFilePath_;
        if (path.isEmpty())
        {
            path = QFileDialog::getSaveFileName(
                this,
                "Save Scene",
                QDir::current().filePath("Untitled.qmecscene"),
                "QMEC Scene (*.qmecscene)");
            if (path.isEmpty())
                return;

            if (QFileInfo(path).suffix().isEmpty())
                path += ".qmecscene";
        }

        if (!qmec::scene::SceneSerializer::Save(scene_, path.toStdString()))
        {
            QMessageBox::critical(this, "Save Scene", "Could not save the scene file.");
            return;
        }

        sceneFilePath_ = path;
        QSettings settings;
        settings.setValue("scene/lastPath", sceneFilePath_);
        statusBar()->showMessage(
            "Scene saved: " + QFileInfo(path).fileName(), 3000);
    }

    void Editor::DeleteSelectedEntity()
    {
        if (!selectedEntity_.IsValid())
            return;

        qmec::ecs::Registry& registry = scene_.GetRegistry();
        if (!registry.IsAlive(selectedEntity_))
            return;

        // Deleting an entity removes its entire hierarchy subtree.
        for (const qmec::ecs::Entity descendant : scene_.GetDescendants(selectedEntity_))
            registry.DestroyEntity(descendant);

        if (!registry.DestroyEntity(selectedEntity_))
            return;

        selectedEntity_ = {};
        hierarchyPanel_->Refresh();
        hierarchyPanel_->ClearSelection();
        lastHierarchyRevision_ = scene_.GetHierarchyRevision();
    }

    void Editor::InitializeEngine()
    {
        if (rendererInitialized_ || initializing_ || viewportSurface_ == nullptr || viewportSurface_->width() <= 0 || viewportSurface_->height() <= 0)
        {
            return;
        }
        initializing_ = true;

        const auto width = static_cast<std::uint32_t>(viewportSurface_->width());
        const auto height = static_cast<std::uint32_t>(viewportSurface_->height());
        const auto nativeWindow = reinterpret_cast<void*>(viewportSurface_->winId());

        if (!renderer_.Initialize(nativeWindow, width, height))
        {
            initializing_ = false;
            QMessageBox::critical(this, "QMEC Editor", "Failed to initialize Direct3D 11 for the viewport.");
            return;
        }
        rendererInitialized_ = true;
        InitializeGameRenderer();

        assetManager_ = std::make_unique<qmec::assets::AssetManager>();
        gpuMeshManager_ = std::make_unique<qmec::graphics::D3D11MeshManager>(renderer_.GetDevice());
        assetSystem_ = std::make_unique<qmec::assets::AssetSystem>(*assetManager_, *gpuMeshManager_);
        entityFactory_ = std::make_unique<qmec::scene::factory::EntityFactory>(scene_.GetRegistry(), *assetSystem_);

        qmec::scene::factory::EntityFactory& factory = *entityFactory_;

        qmec::scene::components::TransformComponent firstCubeTransform{};
        firstCubeTransform.position = {-1.0f, 0.0f, 4.0f};
        const qmec::ecs::Entity firstCube = factory.createCubeEntity(firstCubeTransform, "Cube - Box A");

        qmec::scene::components::TransformComponent secondCubeTransform{};
        secondCubeTransform.position = {-1.0f, 10.0f, 3.5f};
        const qmec::ecs::Entity secondCube = factory.createCubeEntity(secondCubeTransform, "Cube - Box B");

        qmec::scene::components::TransformComponent movingSphereTransform{};
        movingSphereTransform.position = {5.0f, 0.0f, 5.0f};
        const qmec::ecs::Entity movingSphere = factory.createSphereEntity(movingSphereTransform, "Sphere - Moving");

        qmec::scene::components::TransformComponent fallingSphereTransform{};
        fallingSphereTransform.position = {2.0f, 4.0f, 5.0f};
        const qmec::ecs::Entity fallingSphere = factory.createSphereEntity(fallingSphereTransform, "Sphere - Falling");

        qmec::scene::components::TransformComponent cylinderTransform{};
        cylinderTransform.position = {2.0f, 0.0f, 5.0f};
        const qmec::ecs::Entity cylinder = factory.createCylinderEntity(cylinderTransform, "Cylinder");

        qmec::scene::components::TransformComponent planeTransform{};
        planeTransform.position = {8.0f, -3.0f, 4.0f};
        planeTransform.scale = {30.0f, 30.0f, 30.0f};
        const qmec::ecs::Entity ground = factory.createPlaneEntity(planeTransform, "Ground Plane");

        const auto dynamicBody = [](float mass)
        {
            qmec::scene::components::RigidBodyComponent body{};
            body.mass = mass;
            body.inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f;
            return body;
        };

        qmec::scene::components::RigidBodyComponent firstCubeBody = dynamicBody(1.0f);
        const bool firstCubePhysicsAdded = factory.addComponent(firstCube, firstCubeBody)
            && scene_.GetRegistry().HasComponent<qmec::scene::components::ColliderComponent>(firstCube);

        qmec::scene::components::RigidBodyComponent secondCubeBody = dynamicBody(1.0f);
        const bool secondCubePhysicsAdded = factory.addComponent(secondCube, secondCubeBody)
            && scene_.GetRegistry().HasComponent<qmec::scene::components::ColliderComponent>(secondCube);

        qmec::scene::components::RigidBodyComponent movingSphereBody = dynamicBody(1.0f);
        movingSphereBody.velocity = {-2.0f, 0.0f, 0.0f};
        const bool movingSpherePhysicsAdded = factory.addComponent(movingSphere, movingSphereBody)
            && scene_.GetRegistry().HasComponent<qmec::scene::components::ColliderComponent>(movingSphere);

        qmec::scene::components::RigidBodyComponent fallingSphereBody = dynamicBody(10.0f);
        const bool fallingSpherePhysicsAdded = factory.addComponent(fallingSphere, fallingSphereBody)
            && scene_.GetRegistry().HasComponent<qmec::scene::components::ColliderComponent>(fallingSphere);

        qmec::scene::components::RigidBodyComponent cylinderBody = dynamicBody(1.0f);
        const bool cylinderPhysicsAdded = factory.addComponent(cylinder, cylinderBody)
            && scene_.GetRegistry().HasComponent<qmec::scene::components::ColliderComponent>(cylinder);

        qmec::scene::components::RigidBodyComponent groundBody{};
        groundBody.mass = 1.0f;
        groundBody.inverseMass = 0.0f;
        groundBody.isKinematic = true;
        const bool groundPhysicsAdded = factory.addComponent(ground, groundBody)
            && scene_.GetRegistry().HasComponent<qmec::scene::components::ColliderComponent>(ground);

        qmec::scene::components::CameraComponent camera{};
        camera.primary = true;
        qmec::scene::components::DirectionalLightComponent sunlight{};
        sunlight.color = {1.0f, 0.95f, 0.85f};
        sunlight.intensity = 1.0f;
        qmec::scene::components::TransformComponent lightTransform{};
        lightTransform.rotation = qmec::math::Quat::FromYawPitch(-0.45f, -0.65f);
        const bool sceneCreated = firstCube.IsValid() && secondCube.IsValid()
            && movingSphere.IsValid() && fallingSphere.IsValid() && cylinder.IsValid() && ground.IsValid()
            && firstCubePhysicsAdded && secondCubePhysicsAdded && movingSpherePhysicsAdded
            && fallingSpherePhysicsAdded && cylinderPhysicsAdded && groundPhysicsAdded
            && factory.createCameraEntity(camera, {}, "Game Camera").IsValid()
            && factory.createDirectionalLightEntity(sunlight, lightTransform, "Directional Light").IsValid();
        if (!sceneCreated)
        {
            entityFactory_.reset();
            assetSystem_.reset();
            gpuMeshManager_.reset();
            assetManager_.reset();
            initializing_ = false;
            QMessageBox::critical(this, "QMEC Editor", "Failed to create the editor's starter scene.");
            return;
        }

        QSettings settings;
        const QString savedScenePath = settings.value("scene/lastPath").toString();
        if (!savedScenePath.isEmpty())
        {
            if (!QFileInfo::exists(savedScenePath))
            {
                settings.remove("scene/lastPath");
                QMessageBox::warning(this, "Load Scene",
                    "The previously saved scene file could not be found. The starter scene was loaded instead.");
            }
            else if (qmec::scene::SceneSerializer::Load(scene_, savedScenePath.toStdString()))
            {
                sceneFilePath_ = savedScenePath;
            }
            else
            {
                settings.remove("scene/lastPath");
                QMessageBox::warning(this, "Load Scene",
                    "The previously saved scene could not be loaded. The starter scene was loaded instead.");
            }
        }

        hierarchyPanel_->Refresh();
        lastHierarchyRevision_ = scene_.GetHierarchyRevision();
        initializing_ = false;
    }

    void Editor::InitializeGameRenderer()
    {
        if (gameRendererInitialized_ || gameRendererInitializing_
            || !rendererInitialized_ || gameViewportSurface_ == nullptr
            || gameViewportSurface_->width() <= 0 || gameViewportSurface_->height() <= 0)
        {
            return;
        }

        gameRendererInitializing_ = true;
        const auto width = static_cast<std::uint32_t>(gameViewportSurface_->width());
        const auto height = static_cast<std::uint32_t>(gameViewportSurface_->height());
        const auto nativeWindow = reinterpret_cast<void*>(gameViewportSurface_->winId());
        const auto sharedDevice = renderer_.GetDevice();
        if (!gameRenderer_.Initialize(nativeWindow, width, height, sharedDevice.Get()))
        {
            gameRendererInitializing_ = false;
            QMessageBox::warning(this, "QMEC Editor",
                "Failed to initialize Direct3D 11 for the Game viewport.");
            return;
        }

        gameRendererInitialized_ = true;
        gameRendererInitializing_ = false;
    }

    void Editor::OnViewportResized(int width, int height)
    {
        if (width <= 0 || height <= 0)
        {
            return;
        }

        if (!rendererInitialized_)
        {
            InitializeEngine();
            return;
        }

        if (!renderer_.Resize(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height)))
        {
            QMessageBox::warning(this, "QMEC Editor", "Failed to resize the Direct3D viewport.");
        }
    }

    void Editor::OnGameViewportResized(int width, int height)
    {
        if (width <= 0 || height <= 0)
        {
            return;
        }

        if (!rendererInitialized_)
        {
            InitializeEngine();
            return;
        }

        if (!gameRendererInitialized_)
        {
            InitializeGameRenderer();
            return;
        }

        if (!gameRenderer_.Resize(
            static_cast<std::uint32_t>(width),
            static_cast<std::uint32_t>(height)))
        {
            QMessageBox::warning(this, "QMEC Editor",
                "Failed to resize the Direct3D Game viewport.");
        }
    }

    void Editor::OnEntitySelected(qmec::ecs::Entity entity)
    {
        selectedEntity_ = entity;
        const auto& registry = scene_.GetRegistry();
        inspectorPanel_->SetSelectedEntity(entity, registry.GetComponent<qmec::scene::components::TransformComponent>(entity),
            registry.GetComponent<qmec::scene::components::DirectionalLightComponent>(entity),
            registry.GetComponent<qmec::scene::components::CameraComponent>(entity),
            registry.GetComponent<qmec::scene::components::RigidBodyComponent>(entity),
            registry.GetComponent<qmec::scene::components::ColliderComponent>(entity),
            registry.GetComponent<qmec::scene::components::MaterialComponent>(entity),
            registry.GetComponent<qmec::scripting::ScriptComponent>(entity));
    }

    void Editor::OnEntityReparentRequested(qmec::ecs::Entity child, qmec::ecs::Entity parent)
    {
        scene_.SetParent(child, parent);
    }

    void Editor::OnComponentAddRequested(
        qmec::ecs::Entity entity,
        InspectorComponentType componentType)
    {
        qmec::ecs::Registry& registry = scene_.GetRegistry();
        if (!registry.IsAlive(entity))
            return;

        bool added = false;
        switch (componentType)
        {
        case InspectorComponentType::RigidBody:
            if (!registry.HasComponent<qmec::scene::components::RigidBodyComponent>(entity))
            {
                qmec::scene::components::RigidBodyComponent rigidBody{};
                rigidBody.mass = 1.0f;
                rigidBody.inverseMass = 1.0f / rigidBody.mass;
                added = registry.AddComponent(entity, rigidBody);
            }
            break;
        case InspectorComponentType::Collider:
            if (!registry.HasComponent<qmec::scene::components::ColliderComponent>(entity))
            {
                if (entityFactory_ != nullptr &&
                    registry.HasComponent<qmec::scene::components::MeshRendererComponent>(entity))
                    added = entityFactory_->addMatchingCollider(entity);
                else
                    added = registry.AddComponent(entity, qmec::scene::components::ColliderComponent{});
            }
            break;
        case InspectorComponentType::Camera:
            if (!registry.HasComponent<qmec::scene::components::CameraComponent>(entity))
                added = registry.AddComponent(entity, qmec::scene::components::CameraComponent{});
            break;
        case InspectorComponentType::DirectionalLight:
            if (!registry.HasComponent<qmec::scene::components::DirectionalLightComponent>(entity))
                added = registry.AddComponent(entity, qmec::scene::components::DirectionalLightComponent{});
            break;
        case InspectorComponentType::Material:
            if (!registry.HasComponent<qmec::scene::components::MaterialComponent>(entity))
                added = registry.AddComponent(entity, qmec::scene::components::MaterialComponent{});
            break;
        }

        if (added)
            OnEntitySelected(entity);
    }

    void Editor::OnComponentRemoveRequested(
        qmec::ecs::Entity entity,
        InspectorComponentType componentType)
    {
        qmec::ecs::Registry& registry = scene_.GetRegistry();
        if (!registry.IsAlive(entity))
            return;

        bool removed = false;
        switch (componentType)
        {
        case InspectorComponentType::RigidBody:
            removed = registry.RemoveComponent<qmec::scene::components::RigidBodyComponent>(entity);
            break;
        case InspectorComponentType::Collider:
            removed = registry.RemoveComponent<qmec::scene::components::ColliderComponent>(entity);
            break;
        case InspectorComponentType::Camera:
            removed = registry.RemoveComponent<qmec::scene::components::CameraComponent>(entity);
            break;
        case InspectorComponentType::DirectionalLight:
            removed = registry.RemoveComponent<qmec::scene::components::DirectionalLightComponent>(entity);
            break;
        case InspectorComponentType::Material:
            removed = registry.RemoveComponent<qmec::scene::components::MaterialComponent>(entity);
            break;
        }

        if (removed)
            OnEntitySelected(entity);
    }

    void Editor::OnScriptAddRequested(qmec::ecs::Entity entity, const QString& typeId)
    {
        qmec::ecs::Registry& registry = scene_.GetRegistry();
        if (!registry.IsAlive(entity))
        {
            return;
        }

        std::shared_ptr<qmec::scripting::Script> script =
            qmec::scripting::ScriptTypeRegistry::Instance().Create(typeId.toStdString());
        if (script == nullptr)
        {
            return;
        }

        auto* scripts = registry.GetComponent<qmec::scripting::ScriptComponent>(entity);
        if (scripts == nullptr)
        {
            if (!registry.AddComponent(entity, qmec::scripting::ScriptComponent{}))
            {
                return;
            }
            scripts = registry.GetComponent<qmec::scripting::ScriptComponent>(entity);
        }

        if (scripts == nullptr)
        {
            return;
        }

        for (const qmec::scripting::ScriptInstance& instance : scripts->instances)
        {
            if (instance.typeId == typeId.toStdString())
            {
                return;
            }
        }

        scripts->instances.push_back({typeId.toStdString(), std::move(script)});
        OnEntitySelected(entity);
    }

    void Editor::OnScriptRemoveRequested(qmec::ecs::Entity entity, const QString& typeId)
    {
        qmec::ecs::Registry& registry = scene_.GetRegistry();
        auto* scripts = registry.GetComponent<qmec::scripting::ScriptComponent>(entity);
        if (scripts == nullptr)
        {
            return;
        }

        const std::string id = typeId.toStdString();
        auto& instances = scripts->instances;
        for (auto it = instances.begin(); it != instances.end(); ++it)
        {
            if (it->typeId == id)
            {
                instances.erase(it);
                break;
            }
        }

        if (instances.empty())
        {
            registry.RemoveComponent<qmec::scripting::ScriptComponent>(entity);
        }
        OnEntitySelected(entity);
    }

    void Editor::OnScriptEntityReferenceEdited(qmec::ecs::Entity owner,
        const QString& typeId, const QString& fieldId,
        qmec::ecs::Entity referencedEntity)
    {
        qmec::ecs::Registry& registry = scene_.GetRegistry();
        if (!registry.IsAlive(owner))
            return;

        if (referencedEntity.IsValid() && !registry.IsAlive(referencedEntity))
            referencedEntity = {};

        auto* scripts = registry.GetComponent<qmec::scripting::ScriptComponent>(owner);
        if (scripts == nullptr)
            return;

        const std::string scriptTypeId = typeId.toStdString();
        const std::string propertyId = fieldId.toStdString();
        const qmec::scripting::ScriptTypeInfo* type =
            qmec::scripting::ScriptTypeRegistry::Instance().Find(scriptTypeId);
        if (type == nullptr)
            return;

        const auto field = std::find_if(type->entityReferenceFields.begin(),
            type->entityReferenceFields.end(), [&propertyId](const auto& candidate)
            {
                return candidate.id == propertyId;
            });
        if (field == type->entityReferenceFields.end())
            return;

        for (qmec::scripting::ScriptInstance& instance : scripts->instances)
        {
            if (instance.typeId == scriptTypeId && instance.object != nullptr)
            {
                field->set(*instance.object, referencedEntity);
                return;
            }
        }
    }

    void Editor::OnDirectionalLightEdited(qmec::ecs::Entity entity, const qmec::scene::components::DirectionalLightComponent& light)
    {
        auto* current = scene_.GetRegistry().GetComponent<qmec::scene::components::DirectionalLightComponent>(entity);
        if (current != nullptr) *current = light;
    }

    void Editor::OnCameraEdited(qmec::ecs::Entity entity,const qmec::scene::components::CameraComponent& camera)
    {
        auto* current = scene_.GetRegistry().GetComponent<qmec::scene::components::CameraComponent>(entity);
        if (current != nullptr) *current = camera;
    }

    void Editor::OnRigidBodyEdited(qmec::ecs::Entity entity,const qmec::scene::components::RigidBodyComponent& rigidbody)
    {
        auto* current = scene_.GetRegistry().GetComponent<qmec::scene::components::RigidBodyComponent>(entity);
        if (current != nullptr) *current = rigidbody;
    }

    void Editor::OnColliderEdited(qmec::ecs::Entity entity,const qmec::scene::components::ColliderComponent& collider)
    {
        auto* current = scene_.GetRegistry().GetComponent<qmec::scene::components::ColliderComponent>(entity);
        if (current != nullptr) *current = collider;
    }

    void Editor::OnMaterialEdited(qmec::ecs::Entity entity,const qmec::scene::components::MaterialComponent& material)
    {
        auto* current = scene_.GetRegistry().GetComponent<qmec::scene::components::MaterialComponent>(entity);
        if (current != nullptr) *current = material;
    }

    void Editor::OnTransformEdited(qmec::ecs::Entity entity, const qmec::scene::components::TransformComponent& transform)
    {
        auto* currentTransform = scene_.GetRegistry().GetComponent<qmec::scene::components::TransformComponent>(entity);
        if (currentTransform != nullptr)
        {
            *currentTransform = transform;
        }
    }

    void Editor::Tick()
    {
        if (!rendererInitialized_ || assetSystem_ == nullptr || viewport_ == nullptr)
        {
            return;
        }

        constexpr float deltaTime = 1.0f / 60.0f;
        viewport_->UpdateCamera(deltaTime);
        if (playing_)
        {
            qmec::ecs::Registry& registry = scene_.GetRegistry();
            for (const qmec::ecs::Entity entity : registry.GetEntitiesWith<qmec::scripting::ScriptComponent>())
            {
                auto* scripts = registry.GetComponent<qmec::scripting::ScriptComponent>(entity);
                if (scripts == nullptr)
                {
                    continue;
                }
                const qmec::scripting::ScriptContext context{scene_, entity, deltaTime};
                for (const qmec::scripting::ScriptInstance& instance : scripts->instances)
                {
                    if (instance.object != nullptr)
                    {
                        instance.object->OnUpdate(context);
                    }
                }
            }
            scene_.runPhysics(deltaTime);
        }
        const std::uint64_t currentHierarchyRevision = scene_.GetHierarchyRevision();
        if (currentHierarchyRevision != lastHierarchyRevision_)
        {
            hierarchyPanel_->Refresh();
            lastHierarchyRevision_ = currentHierarchyRevision;
        }

        if (selectedEntity_.IsValid())
        {
            OnEntitySelected(selectedEntity_);
        }

        debugRenderer_.Clear();
        if (selectedEntity_.IsValid() && scene_.GetRegistry().IsAlive(selectedEntity_))
        {
            DrawSelectedEntityOutline(debugRenderer_, scene_, *assetSystem_, selectedEntity_);
        }

        const bool editorFrameRendered = renderer_.RenderFrame(
            0.05F, 0.10F, 0.20F, 1.0F, scene_, *assetSystem_,
            viewport_->EditorCamera(), debugRenderer_.GetLines());
        if (!editorFrameRendered)
        {
            timer_->stop();
            QMessageBox::critical(this, "QMEC Editor", "Failed to render the viewport frame.");
            return;
        }

        if (gameRendererInitialized_)
        {
            qmec::ecs::Entity primaryCamera{};
            const qmec::ecs::Registry& registry = scene_.GetRegistry();
            for (const qmec::ecs::Entity entity : registry.GetEntitiesWith<qmec::scene::components::CameraComponent>())
            {
                const auto* camera = registry.GetComponent<qmec::scene::components::CameraComponent>(entity);
                if (camera != nullptr && camera->primary)
                {
                    primaryCamera = entity;
                    break;
                }
            }

            const bool gameFrameRendered = primaryCamera.IsValid() ? gameRenderer_.RenderFrame( 0.0F, 0.0F, 0.0F, 1.0F, scene_, *assetSystem_, primaryCamera) : gameRenderer_.ClearFrame(0.0F, 0.0F, 0.0F, 1.0F);
            if (!gameFrameRendered)
            {
                timer_->stop();
                QMessageBox::critical(this, "QMEC Editor", "Failed to render the Game viewport frame.");
            }
        }
    }

}
