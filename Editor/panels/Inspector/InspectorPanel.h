#pragma once

#include "QMEC/ECS/Entity.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/CameraComponent.h"
#include "QMEC/Scene/Components/DirectionalLightComponent.h"
#include "QMEC/Scene/Components/MaterialComponent.h"
#include "QMEC/Scripting/Script.h"
#include <QDockWidget>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

class QDoubleSpinBox;
class QGroupBox;
class QCheckBox;
class QComboBox;
class QLineEdit;
class QPushButton;
class QMenu;
class QVBoxLayout;

namespace qmec::editor
{
    enum class InspectorComponentType : std::uint8_t
    {
        RigidBody,
        Collider,
        Camera,
        DirectionalLight,
        Material
    };

    class InspectorPanel final : public QDockWidget
    {
        Q_OBJECT

    public:
        explicit InspectorPanel(QWidget* parent = nullptr);

        void SetSelectedEntity(qmec::ecs::Entity entity,const qmec::scene::components::TransformComponent* transform,const qmec::scene::components::DirectionalLightComponent* light,
            const qmec::scene::components::CameraComponent* camera,
            const qmec::scene::components::RigidBodyComponent* rigidbody,
            const qmec::scene::components::ColliderComponent* collider,
            const qmec::scene::components::MaterialComponent* material,
            const qmec::scripting::ScriptComponent* scripts);

    signals:
        void TransformEdited(qmec::ecs::Entity entity, qmec::scene::components::TransformComponent transform);
        void ColliderEdited(qmec::ecs::Entity entity, qmec::scene::components::ColliderComponent collider);
        void RigidBodyEdited(qmec::ecs::Entity entity, qmec::scene::components::RigidBodyComponent rigidBody);
        void DirectionalLightEdited(qmec::ecs::Entity entity, qmec::scene::components::DirectionalLightComponent light);
        void CameraEdited(qmec::ecs::Entity entity, qmec::scene::components::CameraComponent camera);
        void MaterialEdited(qmec::ecs::Entity entity, qmec::scene::components::MaterialComponent material);
        void AddComponentRequested(qmec::ecs::Entity entity, InspectorComponentType componentType);
        void RemoveComponentRequested(qmec::ecs::Entity entity, InspectorComponentType componentType);
        void ScriptAddRequested(qmec::ecs::Entity entity, QString typeId);
        void ScriptRemoveRequested(qmec::ecs::Entity entity, QString typeId);
        void ScriptEntityReferenceEdited(qmec::ecs::Entity owner, QString typeId,
            QString fieldId, qmec::ecs::Entity referencedEntity);

    private:
        void RefreshTransformFields() noexcept;
        void CommitTransformEdit() noexcept;
        void RefreshLightFields() noexcept;
        void CommitLightEdit() noexcept;
        void RefreshRigidbodyFields() noexcept;
        void CommitRigidbodyEdit() noexcept;
        void RefreshCameraFields() noexcept;
        void CommitCameraEdit() noexcept;
        void RefreshMaterialFields() noexcept;
        void CommitMaterialEdit() noexcept;
        void OnChooseAlbedoTexture();
        void OnChooseNormalTexture();
        void RefreshColliderFields() noexcept;
        void CommitColliderEdit() noexcept;
        void OnColliderShapeChanged(int shapeIndex) noexcept;
        void RefreshAddComponentMenu();
        void RefreshScriptFields();

        qmec::ecs::Entity selectedEntity_{};
        qmec::scene::components::TransformComponent displayedTransform_{};
        qmec::math::Vec3 displayedEulerDegrees_{};
        qmec::scene::components::ColliderComponent displayedCollider_{};
        qmec::scene::components::RigidBodyComponent displayedRigidbody{};
        qmec::scene::components::DirectionalLightComponent displayedLight{};
        qmec::scene::components::CameraComponent displayedCamera{};
        qmec::scene::components::MaterialComponent displayedMaterial{};
        std::array<QDoubleSpinBox*, 3> positionFields_{};
        std::array<QDoubleSpinBox*, 3> rotationFields_{};
        std::array<QDoubleSpinBox*, 3> scaleFields_{};
        std::array<QDoubleSpinBox*, 3> lightColorFields_{};
        std::array<QDoubleSpinBox*, 3> colliderCenterFields_{};
        std::array<QDoubleSpinBox*, 3> boxHalfExtentsFields_{};

        QDoubleSpinBox* lightIntensityField_{nullptr};
        QCheckBox* lightEnabledField_{nullptr};
        QDoubleSpinBox* cameraFovField_{nullptr};
        QDoubleSpinBox* cameraNearField_{nullptr};
        QDoubleSpinBox* cameraFarField_{nullptr};
        QCheckBox* cameraPrimaryField_{nullptr};


        QComboBox* colliderShapeField_{nullptr};
        QCheckBox* colliderTriggerField_{nullptr};
        QDoubleSpinBox* planeHalfWidthField_{nullptr};
        QDoubleSpinBox* planeHalfLengthField_{nullptr};
        QDoubleSpinBox* sphereRadiusField_{nullptr};
        QDoubleSpinBox* cylinderRadiusField_{nullptr};
        QDoubleSpinBox* cylinderHalfHeightField_{nullptr};


        std::array<QDoubleSpinBox*, 3> albedoColorFields_{};
        QDoubleSpinBox* normalStrengthField_{nullptr};
        QDoubleSpinBox* metallicField_{nullptr};
        QDoubleSpinBox* roughnessField_{nullptr};
        QDoubleSpinBox* specularLevelField_{nullptr};
        QLineEdit* albedoTexturePathField_{nullptr};
        QPushButton* albedoTextureButton_{nullptr};
        QLineEdit* normalTexturePathField_{nullptr};
        QPushButton* normalTextureButton_{ nullptr };
        QCheckBox* rigidBodyIsKinematic_{ nullptr };
        QDoubleSpinBox* rigidBodyMass_{ nullptr };
        QDoubleSpinBox* staticFrictionField_{nullptr};
        QDoubleSpinBox* dynamicFrictionField_{nullptr};
        QDoubleSpinBox* rollingResistanceField_{nullptr};
        QDoubleSpinBox* elasticityField_{nullptr};
        std::array<QDoubleSpinBox*, 3> velocity_{ nullptr };
        std::array<QDoubleSpinBox*, 3> angularVelocity_{ nullptr };



        QGroupBox* transformGroup_{nullptr};
        QGroupBox* lightGroup_{nullptr};
        QGroupBox* cameraGroup_{nullptr};
        QGroupBox* rigidBodyGroup_{ nullptr };
        QGroupBox* colliderGroup_{nullptr};
        QGroupBox* materialGroup_{nullptr};
        QGroupBox* scriptsGroup_{nullptr};
        QVBoxLayout* scriptsLayout_{nullptr};
        QGroupBox* boxShapeGroup_{nullptr};
        QGroupBox* planeShapeGroup_{nullptr};
        QGroupBox* sphereShapeGroup_{nullptr};
        QGroupBox* cylinderShapeGroup_{nullptr};
        QPushButton* addComponentButton_{nullptr};
        QMenu* addComponentMenu_{nullptr};
        qmec::ecs::Entity addComponentMenuEntity_{};
        std::uint8_t addComponentMenuComponentMask_{};
        qmec::ecs::Entity displayedScriptsEntity_{};
        std::vector<std::string> selectedScriptTypeIds_{};
        std::vector<qmec::scripting::ScriptInstance> selectedScriptInstances_{};
        std::vector<std::string> displayedScriptTypeIds_{};
        std::vector<std::string> addComponentMenuAvailableScripts_{};
        bool hasTransform_{false};
        bool hasDisplayedEuler_{false};
        bool hasLight_{false};
        bool hasRigidBody_{ false };
        bool hasCamera_{false};
        bool hasCollider_{false};
        bool hasMaterial_{false};
        bool updatingUi_{false};
    };

}
