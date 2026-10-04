#include "InspectorPanel.h"
#include "DraggableDoubleSpinBox.h"

#include "QMEC/Math/Quat.h"
#include "QMEC/Assets/ImageData.h"

#include <QByteArray>
#include <QAction>
#include <QDataStream>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QFileDialog>
#include <QFile>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMenu>
#include <QMimeData>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <numbers>
#include <variant>

namespace
{
    constexpr char EntityMimeType[] = "application/x-qmec-entity";

    class EntityReferenceDropLabel final : public QLabel
    {
    public:
        using DropCallback = std::function<void(qmec::Entity, const QString&)>;

        explicit EntityReferenceDropLabel(QWidget* parent) : QLabel(parent)
        {
            setAcceptDrops(true);
            setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
            setMinimumWidth(140);
            setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
            SetEntity({});
        }

        void SetEntity(qmec::Entity entity, const QString& displayName = {})
        {
            entity_ = entity;
            if (!entity.IsValid())
            {
                setText("None — drop an entity here");
                return;
            }

            const QString prefix = displayName.isEmpty()
                ? QStringLiteral("Entity")
                : displayName;
            setText(QStringLiteral("%1 (#%2)").arg(prefix).arg(entity.index));
        }

        DropCallback onDrop{};

    protected:
        void dragEnterEvent(QDragEnterEvent* event) override
        {
            if (event->mimeData()->hasFormat(EntityMimeType))
                event->acceptProposedAction();
            else
                event->ignore();
        }

        void dragMoveEvent(QDragMoveEvent* event) override
        {
            if (event->mimeData()->hasFormat(EntityMimeType))
                event->acceptProposedAction();
            else
                event->ignore();
        }

        void dropEvent(QDropEvent* event) override
        {
            if (!event->mimeData()->hasFormat(EntityMimeType))
            {
                event->ignore();
                return;
            }

            QByteArray bytes = event->mimeData()->data(EntityMimeType);
            QDataStream stream(&bytes, QIODevice::ReadOnly);
            quint32 index{};
            quint32 generation{};
            stream >> index >> generation;
            if (stream.status() != QDataStream::Ok)
            {
                event->ignore();
                return;
            }

            entity_ = {index, generation};
            const QString entityName = event->mimeData()->text();
            SetEntity(entity_, entityName);
            if (onDrop)
                onDrop(entity_, entityName);
            event->acceptProposedAction();
        }

    private:
        qmec::Entity entity_{};
    };

    QWidget* MakeVec3Row(std::array<QDoubleSpinBox*, 3>& fields,QWidget* parent, double minimum, double maximum, double step)
    {
        auto* row = new QWidget(parent);
        auto* layout = new QHBoxLayout(row);
        layout->setContentsMargins(0, 0, 0, 0);

        constexpr std::array<const char*, 3> axisNames{"X", "Y", "Z"};
        for (std::size_t axis = 0; axis < fields.size(); ++axis)
        {
            fields[axis] = new DraggableDoubleSpinBox(row);
            fields[axis]->setRange(minimum, maximum);
            fields[axis]->setSingleStep(step);
            fields[axis]->setDecimals(3);
            fields[axis]->setPrefix(QString::fromLatin1(axisNames[axis]) + " ");
            fields[axis]->setKeyboardTracking(false);
            layout->addWidget(fields[axis]);
        }

        return row;
    }

    QDoubleSpinBox* MakeScalarField(QWidget* parent, double minimum, double maximum, double step)
    {
        auto* field = new DraggableDoubleSpinBox(parent);
        field->setRange(minimum, maximum);
        field->setSingleStep(step);
        field->setDecimals(3);
        field->setKeyboardTracking(false);
        return field;
    }

    float NearestEquivalentDegrees(float degrees, float reference) noexcept
    {
        constexpr float fullTurn = 360.0f;
        return degrees + fullTurn * std::round((reference - degrees) / fullTurn);
    }

    qmec::Vec3 ToEulerDegrees(const qmec::Quat& rotation, const qmec::Vec3& reference, bool hasReference) noexcept
    {
        const qmec::Quat q = rotation.Normalized();

        const float sinX = 2.0f * (q.w * q.x + q.y * q.z);
        const float cosX = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        const float x = std::atan2(sinX, cosX);

        const float sinY = 2.0f * (q.w * q.y - q.z * q.x);
        const float y = std::fabs(sinY) >= 1.0f ? std::copysign(std::numbers::pi_v<float> * 0.5f, sinY): std::asin(sinY);

        const float sinZ = 2.0f * (q.w * q.z + q.x * q.y);
        const float cosZ = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        const float z = std::atan2(sinZ, cosZ);

        constexpr float radiansToDegrees = 180.0f / std::numbers::pi_v<float>;
        const qmec::Vec3 primary{x * radiansToDegrees, y * radiansToDegrees, z * radiansToDegrees};

        if (!hasReference)
        {
            return primary;
        }

        // Euler angles have a second equivalent representation. Choosing the
        // one nearest the previous inspector values avoids the apparent
        // 180-degree X/Z jump when pitch crosses +/-90 degrees.
        qmec::Vec3 alternate{primary.x + 180.0f,180.0f - primary.y,primary.z + 180.0f};

        qmec::Vec3 nearestPrimary{
            NearestEquivalentDegrees(primary.x, reference.x),
            NearestEquivalentDegrees(primary.y, reference.y),
            NearestEquivalentDegrees(primary.z, reference.z)};
        alternate = {
            NearestEquivalentDegrees(alternate.x, reference.x),
            NearestEquivalentDegrees(alternate.y, reference.y),
            NearestEquivalentDegrees(alternate.z, reference.z)};

        const qmec::Vec3 primaryDelta = nearestPrimary - reference;
        const qmec::Vec3 alternateDelta = alternate - reference;
        return qmec::Dot(alternateDelta, alternateDelta) <
            qmec::Dot(primaryDelta, primaryDelta) ? alternate : nearestPrimary;
    }

    qmec::Quat FromEulerDegrees(const qmec::Vec3& eulerDegrees) noexcept
    {
        constexpr float degreesToRadians = std::numbers::pi_v<float> / 180.0f;
        const qmec::Quat xRotation = qmec::Quat::FromAxisAngle({1.0f, 0.0f, 0.0f}, eulerDegrees.x * degreesToRadians);
        const qmec::Quat yRotation = qmec::Quat::FromAxisAngle({0.0f, 1.0f, 0.0f}, eulerDegrees.y * degreesToRadians);
        const qmec::Quat zRotation = qmec::Quat::FromAxisAngle({0.0f, 0.0f, 1.0f}, eulerDegrees.z * degreesToRadians);
        return (zRotation * yRotation * xRotation).Normalized();
    }
}

InspectorPanel::InspectorPanel(QWidget* parent): QDockWidget("Inspector", parent)
{
    setObjectName("InspectorPanel");

    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);

    auto* content = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(content);
    const auto makeRemoveButtonRow = [this](
        QWidget* parent,
        InspectorComponentType componentType,
        const QString& tooltip)
    {
        auto* row = new QWidget(parent);
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->addStretch();

        auto* removeButton = new QPushButton(QString::fromUtf8("×"), row);
        removeButton->setFixedSize(24, 22);
        removeButton->setToolTip(tooltip);
        rowLayout->addWidget(removeButton);
        connect(removeButton, &QPushButton::clicked, this,
            [this, componentType]()
            {
                if (selectedEntity_.IsValid())
                    emit RemoveComponentRequested(selectedEntity_, componentType);
            });
        return row;
    };

    transformGroup_ = new QGroupBox("Transform", content);
    auto* transformLayout = new QFormLayout(transformGroup_);
    transformLayout->addRow("Position", MakeVec3Row(positionFields_, transformGroup_, -1000000.0, 1000000.0, 0.1));
    transformLayout->addRow("Rotation (degrees)", MakeVec3Row(rotationFields_, transformGroup_, -3600.0, 3600.0, 1.0));
    transformLayout->addRow("Scale", MakeVec3Row(scaleFields_, transformGroup_, -1000.0, 1000.0, 0.1));
    contentLayout->addWidget(transformGroup_);

    materialGroup_ = new QGroupBox("Material", content);
    auto* materialLayout = new QFormLayout(materialGroup_);
    materialLayout->addRow(makeRemoveButtonRow(materialGroup_, InspectorComponentType::Material, "Remove Material component"));
    materialLayout->addRow("Albedo Color", MakeVec3Row(albedoColorFields_, materialGroup_, 0.0, 1.0, 0.05));

    auto* albedoTextureRow = new QWidget(materialGroup_);
    auto* albedoTextureLayout = new QHBoxLayout(albedoTextureRow);
    albedoTextureLayout->setContentsMargins(0, 0, 0, 0);
    albedoTexturePathField_ = new QLineEdit(materialGroup_);
    albedoTexturePathField_->setReadOnly(true);
    albedoTexturePathField_->setPlaceholderText("No albedo texture");
    albedoTextureLayout->addWidget(albedoTexturePathField_);
    albedoTextureButton_ = new QPushButton("Select...", albedoTextureRow);
    albedoTextureLayout->addWidget(albedoTextureButton_);
    materialLayout->addRow("Albedo Texture", albedoTextureRow);

    auto* normalTextureRow = new QWidget(materialGroup_);
    auto* normalTextureLayout = new QHBoxLayout(normalTextureRow);
    normalTextureLayout->setContentsMargins(0, 0, 0, 0);
    normalTexturePathField_ = new QLineEdit(normalTextureRow);
    normalTexturePathField_->setReadOnly(true);
    normalTexturePathField_->setPlaceholderText("No normal texture");
    normalTextureLayout->addWidget(normalTexturePathField_);
    normalTextureButton_ = new QPushButton("Select...", normalTextureRow);
    normalTextureLayout->addWidget(normalTextureButton_);
    materialLayout->addRow("Normal Texture", normalTextureRow);

    normalStrengthField_ = MakeScalarField(materialGroup_, 0.0, 2.0, 0.05);
    materialLayout->addRow("Normal Strength", normalStrengthField_);
    metallicField_ = MakeScalarField(materialGroup_, 0.0, 1.0, 0.05);
    materialLayout->addRow("Metallic", metallicField_);
    roughnessField_ = MakeScalarField(materialGroup_, 0.04, 1.0, 0.05);
    materialLayout->addRow("Roughness", roughnessField_);
    specularLevelField_ = MakeScalarField(materialGroup_, 0.0, 1.0, 0.05);
    materialLayout->addRow("Specular Level", specularLevelField_);
    contentLayout->addWidget(materialGroup_);

    lightGroup_ = new QGroupBox("Directional Light", content);
    auto* lightLayout = new QFormLayout(lightGroup_);
    lightLayout->addRow(makeRemoveButtonRow(lightGroup_, InspectorComponentType::DirectionalLight, "Remove Directional Light component"));
    lightLayout->addRow("Color", MakeVec3Row(lightColorFields_, lightGroup_, 0.0, 100.0, 0.05));
    lightIntensityField_ = MakeScalarField(lightGroup_, 0.0, 100000.0, 0.1);
    lightLayout->addRow("Intensity", lightIntensityField_);
    lightEnabledField_ = new QCheckBox("Enabled", lightGroup_);
    lightLayout->addRow(lightEnabledField_);
    contentLayout->addWidget(lightGroup_);

    cameraGroup_ = new QGroupBox("Camera", content);
    auto* cameraLayout = new QFormLayout(cameraGroup_);
    cameraLayout->addRow(makeRemoveButtonRow(cameraGroup_, InspectorComponentType::Camera, "Remove Camera component"));
    cameraFovField_ = MakeScalarField(cameraGroup_, 1.0, 179.0, 1.0);
    cameraFovField_->setSuffix(" deg");
    cameraLayout->addRow("Vertical FOV", cameraFovField_);
    cameraNearField_ = MakeScalarField(cameraGroup_, 0.001, 100000.0, 0.1);
    cameraLayout->addRow("Near Plane", cameraNearField_);
    cameraFarField_ = MakeScalarField(cameraGroup_, 0.002, 1000000.0, 1.0);
    cameraLayout->addRow("Far Plane", cameraFarField_);
    cameraPrimaryField_ = new QCheckBox("Primary Camera", cameraGroup_);
    cameraLayout->addRow(cameraPrimaryField_);
    contentLayout->addWidget(cameraGroup_);

    colliderGroup_ = new QGroupBox("Collider", content);
    auto* colliderLayout = new QVBoxLayout(colliderGroup_);
    colliderLayout->addWidget(makeRemoveButtonRow(colliderGroup_, InspectorComponentType::Collider, "Remove Collider component"));
    auto* colliderForm = new QFormLayout();
    colliderShapeField_ = new QComboBox(colliderGroup_);
    colliderShapeField_->addItems({"Box", "Plane", "Sphere", "Cylinder"});
    colliderShapeField_->setEnabled(false);
    colliderForm->addRow("Shape", colliderShapeField_);
    colliderTriggerField_ = new QCheckBox("Is Trigger", colliderGroup_);
    colliderTriggerField_->setEnabled(false);
    colliderForm->addRow(colliderTriggerField_);
    colliderForm->addRow("Center", MakeVec3Row(colliderCenterFields_, colliderGroup_, -1000000.0, 1000000.0, 0.1));
    for (QDoubleSpinBox* field : colliderCenterFields_)
        field->setEnabled(false);
    colliderLayout->addLayout(colliderForm);

    rigidBodyGroup_ = new QGroupBox("Rigidbody", content);
    auto* rigidbodyLayout = new QFormLayout(rigidBodyGroup_);
    rigidbodyLayout->addRow(makeRemoveButtonRow(
        rigidBodyGroup_, InspectorComponentType::RigidBody, "Remove Rigidbody component"));
    rigidbodyLayout->addRow("Velocity", MakeVec3Row(velocity_, rigidBodyGroup_, -1000000.0, 1000000.0, 0.1));
    rigidbodyLayout->addRow("Angular Velocity", MakeVec3Row(angularVelocity_, rigidBodyGroup_, -1000000.0, 1000000.0, 0.1));
    for (QDoubleSpinBox* field : velocity_)
        field->setEnabled(false);
    for (QDoubleSpinBox* field : angularVelocity_)
        field->setEnabled(false);
    rigidBodyMass_ = MakeScalarField(rigidBodyGroup_, 0.0, 100000.0, 0.1);
    rigidbodyLayout->addRow("Mass", rigidBodyMass_);
    rigidBodyIsKinematic_ = new QCheckBox(rigidBodyGroup_);
    rigidbodyLayout->addRow("IsKinematic", rigidBodyIsKinematic_);

    auto* physicsMaterialGroup = new QGroupBox("Physics Material", rigidBodyGroup_);
    auto* physicsMaterialLayout = new QFormLayout(physicsMaterialGroup);
    staticFrictionField_ = MakeScalarField(physicsMaterialGroup, 0.0, 10.0, 0.05);
    physicsMaterialLayout->addRow("Static Friction", staticFrictionField_);
    dynamicFrictionField_ = MakeScalarField(physicsMaterialGroup, 0.0, 10.0, 0.05);
    physicsMaterialLayout->addRow("Dynamic Friction", dynamicFrictionField_);
    rollingResistanceField_ = MakeScalarField(physicsMaterialGroup, 0.0, 1.0, 0.005);
    physicsMaterialLayout->addRow("Rolling Resistance", rollingResistanceField_);
    elasticityField_ = MakeScalarField(physicsMaterialGroup, 0.0, 1.0, 0.05);
    physicsMaterialLayout->addRow("Elasticity", elasticityField_);
    rigidbodyLayout->addRow(physicsMaterialGroup);
    contentLayout->addWidget(rigidBodyGroup_);


    boxShapeGroup_ = new QGroupBox("Box", colliderGroup_);
    auto* boxLayout = new QFormLayout(boxShapeGroup_);
    boxLayout->addRow("Half Extents", MakeVec3Row(boxHalfExtentsFields_, boxShapeGroup_, 0.001, 1000000.0, 0.1));
    for (QDoubleSpinBox* field : boxHalfExtentsFields_)
        field->setEnabled(false);
    colliderLayout->addWidget(boxShapeGroup_);
    planeShapeGroup_ = new QGroupBox("Plane", colliderGroup_);
    auto* planeLayout = new QFormLayout(planeShapeGroup_);
    planeHalfWidthField_ = MakeScalarField(planeShapeGroup_, 0.001, 1000000.0, 0.1);
    planeHalfLengthField_ = MakeScalarField(planeShapeGroup_, 0.001, 1000000.0, 0.1);
    planeLayout->addRow("Half Width X", planeHalfWidthField_);
    planeLayout->addRow("Half Length Z", planeHalfLengthField_);
    planeHalfWidthField_->setEnabled(false);
    planeHalfLengthField_->setEnabled(false);
    colliderLayout->addWidget(planeShapeGroup_);
    sphereShapeGroup_ = new QGroupBox("Sphere", colliderGroup_);
    auto* sphereLayout = new QFormLayout(sphereShapeGroup_);
    sphereRadiusField_ = MakeScalarField(sphereShapeGroup_, 0.001, 1000000.0, 0.1);
    sphereRadiusField_->setEnabled(false);
    sphereLayout->addRow("Radius", sphereRadiusField_);
    colliderLayout->addWidget(sphereShapeGroup_);
    cylinderShapeGroup_ = new QGroupBox("Cylinder", colliderGroup_);
    auto* cylinderLayout = new QFormLayout(cylinderShapeGroup_);
    cylinderRadiusField_ = MakeScalarField(cylinderShapeGroup_, 0.001, 1000000.0, 0.1);
    cylinderHalfHeightField_ = MakeScalarField(cylinderShapeGroup_, 0.001, 1000000.0, 0.1);
    cylinderRadiusField_->setEnabled(false);
    cylinderHalfHeightField_->setEnabled(false);
    cylinderLayout->addRow("Radius", cylinderRadiusField_);
    cylinderLayout->addRow("Half Height", cylinderHalfHeightField_);
    colliderLayout->addWidget(cylinderShapeGroup_);
    contentLayout->addWidget(colliderGroup_);

    scriptsGroup_ = new QGroupBox("Scripts", content);
    scriptsLayout_ = new QVBoxLayout(scriptsGroup_);
    contentLayout->addWidget(scriptsGroup_);

    addComponentMenu_ = new QMenu(this);
    addComponentButton_ = new QPushButton("Add Component", content);
    addComponentButton_->setMenu(addComponentMenu_);
    contentLayout->addStretch();
    contentLayout->addWidget(addComponentButton_);

    scrollArea->setWidget(content);
    setWidget(scrollArea);
    transformGroup_->setVisible(false);
    materialGroup_->setVisible(false);
    lightGroup_->setVisible(false);
    cameraGroup_->setVisible(false);
    colliderGroup_->setVisible(false);
    scriptsGroup_->setVisible(false);
    addComponentButton_->setEnabled(false);

    for (QDoubleSpinBox* field : positionFields_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitTransformEdit(); });

    for (QDoubleSpinBox* field : rotationFields_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { CommitTransformEdit(); });

    for (QDoubleSpinBox* field : scaleFields_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitTransformEdit(); });

    for (QDoubleSpinBox* field : albedoColorFields_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitMaterialEdit(); });

    for (QDoubleSpinBox* field : {normalStrengthField_, metallicField_, roughnessField_, specularLevelField_})
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitMaterialEdit(); });

    connect(albedoTextureButton_, &QPushButton::clicked,this, &InspectorPanel::OnChooseAlbedoTexture);
    connect(normalTextureButton_, &QPushButton::clicked, this, &InspectorPanel::OnChooseNormalTexture);

    for (QDoubleSpinBox* field : lightColorFields_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitLightEdit(); });
    connect(lightIntensityField_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { CommitLightEdit(); });
    connect(lightEnabledField_, &QCheckBox::toggled, this,[this](bool) { CommitLightEdit(); });

    for (QDoubleSpinBox* field : {cameraFovField_, cameraNearField_, cameraFarField_})
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { CommitCameraEdit(); });
    connect(cameraPrimaryField_, &QCheckBox::toggled, this,[this](bool) { CommitCameraEdit(); });


    for (QDoubleSpinBox* field : velocity_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double) { CommitRigidbodyEdit(); });

    for (QDoubleSpinBox* field : angularVelocity_)
    {
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { CommitRigidbodyEdit(); });
        connect(rigidBodyMass_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { CommitRigidbodyEdit(); });
        connect(rigidBodyIsKinematic_, &QCheckBox::toggled, this, [this](bool) { CommitRigidbodyEdit(); });
    }
    for (QDoubleSpinBox* field : { staticFrictionField_, dynamicFrictionField_,rollingResistanceField_, elasticityField_ })
    {
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { CommitRigidbodyEdit(); });
        connect(colliderShapeField_, qOverload<int>(&QComboBox::currentIndexChanged),this, &InspectorPanel::OnColliderShapeChanged);
        connect(colliderTriggerField_, &QCheckBox::toggled, this,[this](bool) { CommitColliderEdit(); });
    }
    for (QDoubleSpinBox* field : colliderCenterFields_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitColliderEdit(); });
    for (QDoubleSpinBox* field : boxHalfExtentsFields_)
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitColliderEdit(); });
    for (QDoubleSpinBox* field : {planeHalfWidthField_, planeHalfLengthField_, sphereRadiusField_, cylinderRadiusField_, cylinderHalfHeightField_})
        connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this,[this](double) { CommitColliderEdit(); });
}

void InspectorPanel::SetSelectedEntity(qmec::Entity entity,
    const qmec::TransformComponent* transform,
    const qmec::DirectionalLightComponent* light,
    const qmec::CameraComponent* camera,
    const qmec::RigidBodyComponent* rigidbody,
    const qmec::ColliderComponent* collider,
    const qmec::MaterialComponent* material,
    const qmec::ScriptComponent* scripts)
{
    const bool sameSelectedEntity = selectedEntity_ == entity;
    if (!sameSelectedEntity)
    {
        hasDisplayedEuler_ = false;
    }
    selectedEntity_ = entity;
    hasTransform_ = entity.IsValid() && transform != nullptr;
    hasRigidBody_ = entity.IsValid() && rigidbody != nullptr;
    hasLight_ = entity.IsValid() && light != nullptr;
    hasCamera_ = entity.IsValid() && camera != nullptr;
    hasCollider_ = entity.IsValid() && collider != nullptr;
    hasMaterial_ = entity.IsValid() && material != nullptr;
    if (hasTransform_)displayedTransform_ = *transform;
    if (hasLight_)displayedLight = *light;
    if (hasCamera_)displayedCamera = *camera;
    if (hasCollider_)displayedCollider_ = *collider;
    if (hasMaterial_)displayedMaterial = *material;
    if (hasRigidBody_)displayedRigidbody = *rigidbody;

    selectedScriptTypeIds_.clear();
    if (entity.IsValid() && scripts != nullptr)
    {
        selectedScriptTypeIds_.reserve(scripts->instances.size());
        selectedScriptInstances_ = scripts->instances;
        for (const qmec::ScriptInstance& instance : scripts->instances)
        {
            selectedScriptTypeIds_.push_back(instance.typeId);
        }
    }
    else
    {
        selectedScriptInstances_.clear();
    }

    transformGroup_->setVisible(hasTransform_);
    lightGroup_->setVisible(hasLight_);
    cameraGroup_->setVisible(hasCamera_);
    colliderGroup_->setVisible(hasCollider_);
    materialGroup_->setVisible(hasMaterial_);
    rigidBodyGroup_->setVisible(hasRigidBody_);
    scriptsGroup_->setVisible(!selectedScriptTypeIds_.empty());
    RefreshScriptFields();
    RefreshAddComponentMenu();
    RefreshTransformFields();
    RefreshLightFields();
    RefreshCameraFields();
    RefreshMaterialFields();
    RefreshColliderFields();
    RefreshRigidbodyFields();
}

void InspectorPanel::RefreshAddComponentMenu()
{
    const std::uint8_t componentMask =
        static_cast<std::uint8_t>((hasRigidBody_ ? 1U : 0U) |
        (hasCollider_ ? 2U : 0U) |
        (hasCamera_ ? 4U : 0U) |
        (hasLight_ ? 8U : 0U) |
        (hasMaterial_ ? 16U : 0U));

    std::vector<std::string> availableScripts;
    for (const qmec::ScriptTypeInfo& scriptType :
        qmec::ScriptTypeRegistry::Instance().GetTypes())
    {
        const bool alreadyAttached = std::find(selectedScriptTypeIds_.begin(),
            selectedScriptTypeIds_.end(), scriptType.id) != selectedScriptTypeIds_.end();
        if (!alreadyAttached)
        {
            availableScripts.push_back(scriptType.id);
        }
    }

    if (selectedEntity_ == addComponentMenuEntity_ &&
        componentMask == addComponentMenuComponentMask_ &&
        availableScripts == addComponentMenuAvailableScripts_)
    {
        return;
    }

    addComponentMenuEntity_ = selectedEntity_;
    addComponentMenuComponentMask_ = componentMask;
    addComponentMenuAvailableScripts_ = availableScripts;
    addComponentMenu_->clear();
    if (!selectedEntity_.IsValid())
    {
        addComponentButton_->setEnabled(false);
        return;
    }

    const auto addOption = [this](const QString& label, InspectorComponentType componentType)
    {
        QAction* action = addComponentMenu_->addAction(label);
        connect(action, &QAction::triggered, this,
            [this, componentType]()
            {
                if (selectedEntity_.IsValid())
                    emit AddComponentRequested(selectedEntity_, componentType);
            });
    };

    if (!hasRigidBody_)
        addOption("Rigidbody", InspectorComponentType::RigidBody);
    if (!hasCollider_)
        addOption("Collider", InspectorComponentType::Collider);
    if (!hasCamera_)
        addOption("Camera", InspectorComponentType::Camera);
    if (!hasLight_)
        addOption("Directional Light", InspectorComponentType::DirectionalLight);
    if (!hasMaterial_)
        addOption("Material", InspectorComponentType::Material);

    if (!availableScripts.empty())
    {
        QMenu* scriptsMenu = addComponentMenu_->addMenu("Scripts");
        for (const std::string& typeId : availableScripts)
        {
            const qmec::ScriptTypeInfo* type =
                qmec::ScriptTypeRegistry::Instance().Find(typeId);
            if (type == nullptr)
            {
                continue;
            }

            QAction* action = scriptsMenu->addAction(QString::fromStdString(type->displayName));
            connect(action, &QAction::triggered, this,
                [this, typeId]()
                {
                    if (selectedEntity_.IsValid())
                    {
                        emit ScriptAddRequested(selectedEntity_, QString::fromStdString(typeId));
                    }
                });
        }
    }

    addComponentButton_->setEnabled(!addComponentMenu_->isEmpty());
}

void InspectorPanel::RefreshScriptFields()
{
    if (displayedScriptsEntity_ == selectedEntity_ && displayedScriptTypeIds_ == selectedScriptTypeIds_)
    {
        return;
    }

    while (QLayoutItem* item = scriptsLayout_->takeAt(0))
    {
        if (QWidget* widget = item->widget())
        {
            widget->deleteLater();
        }
        delete item;
    }

    displayedScriptsEntity_ = selectedEntity_;
    displayedScriptTypeIds_ = selectedScriptTypeIds_;
    for (const std::string& typeId : displayedScriptTypeIds_)
    {
        const qmec::ScriptTypeInfo* type = qmec::ScriptTypeRegistry::Instance().Find(typeId);
        const QString displayName = type != nullptr ? QString::fromStdString(type->displayName) : QString::fromStdString(typeId);

        auto* row = new QWidget(scriptsGroup_);
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->addWidget(new QLabel(displayName, row));
        rowLayout->addStretch();
        auto* removeButton = new QPushButton(QString::fromUtf8("×"), row);
        removeButton->setFixedSize(24, 22);
        removeButton->setToolTip("Remove script");
        rowLayout->addWidget(removeButton);
        connect(removeButton, &QPushButton::clicked, this,
            [this, typeId]()
            {
                if (selectedEntity_.IsValid())
                {
                    emit ScriptRemoveRequested(selectedEntity_, QString::fromStdString(typeId));
                }
            });
        scriptsLayout_->addWidget(row);

        const auto instanceIt = std::find_if(selectedScriptInstances_.begin(),
            selectedScriptInstances_.end(), [&typeId](const qmec::ScriptInstance& instance)
            {
                return instance.typeId == typeId;
            });
        if (type == nullptr || instanceIt == selectedScriptInstances_.end() ||
            instanceIt->object == nullptr)
        {
            continue;
        }

        for (const qmec::ScriptTypeInfo::EntityReferenceField& field : type->entityReferenceFields)
        {
            auto* fieldRow = new QWidget(scriptsGroup_);
            auto* fieldLayout = new QHBoxLayout(fieldRow);
            fieldLayout->setContentsMargins(12, 0, 0, 0);
            fieldLayout->addWidget(new QLabel(QString::fromStdString(field.displayName), fieldRow));

            auto* dropTarget = new EntityReferenceDropLabel(fieldRow);
            dropTarget->SetEntity(field.get(*instanceIt->object));
            dropTarget->onDrop = [this, dropTarget, typeId, fieldId = field.id]( qmec::Entity referencedEntity, const QString& displayName)
            {
                dropTarget->SetEntity(referencedEntity, displayName);
                emit ScriptEntityReferenceEdited(selectedEntity_, QString::fromStdString(typeId), QString::fromStdString(fieldId), referencedEntity);
            };
            fieldLayout->addWidget(dropTarget, 1);

            auto* clearButton = new QPushButton(QString::fromUtf8("×"), fieldRow);
            clearButton->setFixedSize(24, 22);
            clearButton->setToolTip("Clear entity reference");
            fieldLayout->addWidget(clearButton);
            connect(clearButton, &QPushButton::clicked, this, [this, dropTarget, typeId, fieldId = field.id]()
                {
                    dropTarget->SetEntity({});
                    emit ScriptEntityReferenceEdited(selectedEntity_,QString::fromStdString(typeId), QString::fromStdString(fieldId), {});
                });
            scriptsLayout_->addWidget(fieldRow);
        }
    }
}



void InspectorPanel::RefreshTransformFields() noexcept
{
    if (!hasTransform_)
    {
        return;
    }

    updatingUi_ = true;
    const qmec::Vec3 rotation = ToEulerDegrees(displayedTransform_.rotation, displayedEulerDegrees_, hasDisplayedEuler_);
    const std::array<float, 3> position{displayedTransform_.position.x, displayedTransform_.position.y, displayedTransform_.position.z};
    const std::array<float, 3> euler{rotation.x, rotation.y, rotation.z};
    const std::array<float, 3> scale{displayedTransform_.scale.x, displayedTransform_.scale.y, displayedTransform_.scale.z};

    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!positionFields_[axis]->hasFocus()) positionFields_[axis]->setValue(position[axis]);
        if (!rotationFields_[axis]->hasFocus()) rotationFields_[axis]->setValue(euler[axis]);
        if (!scaleFields_[axis]->hasFocus()) scaleFields_[axis]->setValue(scale[axis]);
    }
    displayedEulerDegrees_ = {
        static_cast<float>(rotationFields_[0]->value()),
        static_cast<float>(rotationFields_[1]->value()),
        static_cast<float>(rotationFields_[2]->value())};
    hasDisplayedEuler_ = true;
    updatingUi_ = false;
}

void InspectorPanel::CommitTransformEdit() noexcept
{
    if (updatingUi_ || !hasTransform_ || !selectedEntity_.IsValid())
    {
        return;
    }

    displayedTransform_.position = {
        static_cast<float>(positionFields_[0]->value()),
        static_cast<float>(positionFields_[1]->value()),
        static_cast<float>(positionFields_[2]->value())};
    displayedTransform_.rotation = FromEulerDegrees({
        static_cast<float>(rotationFields_[0]->value()),
        static_cast<float>(rotationFields_[1]->value()),
        static_cast<float>(rotationFields_[2]->value())});
    displayedEulerDegrees_ = {
        static_cast<float>(rotationFields_[0]->value()),
        static_cast<float>(rotationFields_[1]->value()),
        static_cast<float>(rotationFields_[2]->value())};
    hasDisplayedEuler_ = true;
    displayedTransform_.scale = {
        static_cast<float>(scaleFields_[0]->value()),
        static_cast<float>(scaleFields_[1]->value()),
        static_cast<float>(scaleFields_[2]->value())};

    emit TransformEdited(selectedEntity_, displayedTransform_);
}


void InspectorPanel::CommitRigidbodyEdit() noexcept
{
    if (updatingUi_ || !hasRigidBody_ || !selectedEntity_.IsValid())
    {
        return;
    }
   
    displayedRigidbody.isKinematic = rigidBodyIsKinematic_->isChecked();
    displayedRigidbody.mass = static_cast<float>(rigidBodyMass_->value());
    displayedRigidbody.velocity = { static_cast<float>(velocity_[0]->value()),
        static_cast<float>(velocity_[1]->value()),
        static_cast<float>(velocity_[2]->value())};
    displayedRigidbody.angularVelocity = {
        static_cast<float>(angularVelocity_[0]->value()),
        static_cast<float>(angularVelocity_[1]->value()),
        static_cast<float>(angularVelocity_[2]->value())};
    displayedRigidbody.material.staticFriction =
        static_cast<float>(staticFrictionField_->value());
    displayedRigidbody.material.dynamicFriction =
        static_cast<float>(dynamicFrictionField_->value());
    displayedRigidbody.material.rollingResistance = std::clamp(
        static_cast<float>(rollingResistanceField_->value()), 0.0f, 1.0f);
    displayedRigidbody.material.Elasticity = std::clamp(
        static_cast<float>(elasticityField_->value()), 0.0f, 1.0f);
    displayedRigidbody.inverseMass = displayedRigidbody.isKinematic || displayedRigidbody.mass <= 0.0f
        ? 0.0f : 1.0f / displayedRigidbody.mass;
    rigidBodyMass_->setEnabled(!displayedRigidbody.isKinematic);
    emit RigidBodyEdited(selectedEntity_, displayedRigidbody);
}



void InspectorPanel::RefreshLightFields() noexcept
{
    if (!hasLight_) return;
    updatingUi_ = true;
    const std::array<float, 3> color{displayedLight.color.x, displayedLight.color.y, displayedLight.color.z};
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (!lightColorFields_[axis]->hasFocus()) lightColorFields_[axis]->setValue(color[axis]);
    if (!lightIntensityField_->hasFocus()) lightIntensityField_->setValue(displayedLight.intensity);
    lightEnabledField_->setChecked(displayedLight.enabled);
    updatingUi_ = false;
}

void InspectorPanel::CommitLightEdit() noexcept
{
    if (updatingUi_ || !hasLight_ || !selectedEntity_.IsValid()) return;
    displayedLight.color = { static_cast<float>(lightColorFields_[0]->value()),static_cast<float>(lightColorFields_[1]->value()),
        static_cast<float>(lightColorFields_[2]->value())};
    displayedLight.intensity = static_cast<float>(lightIntensityField_->value());
    displayedLight.enabled = lightEnabledField_->isChecked();
    emit DirectionalLightEdited(selectedEntity_, displayedLight);
}

void InspectorPanel::RefreshRigidbodyFields() noexcept
{
    if (!hasRigidBody_) return;
    updatingUi_ = true;
    const std::array<float, 3> linear{displayedRigidbody.velocity.x, displayedRigidbody.velocity.y, displayedRigidbody.velocity.z};
    const std::array<float, 3> angular{displayedRigidbody.angularVelocity.x, displayedRigidbody.angularVelocity.y, displayedRigidbody.angularVelocity.z};
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!velocity_[axis]->hasFocus()) velocity_[axis]->setValue(linear[axis]);
        if (!angularVelocity_[axis]->hasFocus()) angularVelocity_[axis]->setValue(angular[axis]);
    }
    rigidBodyIsKinematic_->setChecked(displayedRigidbody.isKinematic);
    if (!rigidBodyMass_->hasFocus()) rigidBodyMass_->setValue(displayedRigidbody.mass);
    if (!staticFrictionField_->hasFocus())
        staticFrictionField_->setValue(displayedRigidbody.material.staticFriction);
    if (!dynamicFrictionField_->hasFocus())
        dynamicFrictionField_->setValue(displayedRigidbody.material.dynamicFriction);
    if (!rollingResistanceField_->hasFocus())
        rollingResistanceField_->setValue(displayedRigidbody.material.rollingResistance);
    if (!elasticityField_->hasFocus())
        elasticityField_->setValue(displayedRigidbody.material.Elasticity);
    rigidBodyMass_->setEnabled(!displayedRigidbody.isKinematic);
    updatingUi_ = false;
}

void InspectorPanel::RefreshCameraFields() noexcept
{
    if (!hasCamera_) return;
    constexpr double radiansToDegrees = 180.0 / std::numbers::pi;
    updatingUi_ = true;
    if (!cameraFovField_->hasFocus()) cameraFovField_->setValue(displayedCamera.verticalFieldOfViewRadians * radiansToDegrees);
    if (!cameraNearField_->hasFocus()) cameraNearField_->setValue(displayedCamera.nearPlane);
    if (!cameraFarField_->hasFocus()) cameraFarField_->setValue(displayedCamera.farPlane);
    cameraPrimaryField_->setChecked(displayedCamera.primary);
    updatingUi_ = false;
}

void InspectorPanel::CommitCameraEdit() noexcept
{
    if (updatingUi_ || !hasCamera_ || !selectedEntity_.IsValid()) return;
    constexpr float degreesToRadians = std::numbers::pi_v<float> / 180.0f;
    displayedCamera.verticalFieldOfViewRadians = static_cast<float>(cameraFovField_->value()) * degreesToRadians;
    displayedCamera.nearPlane = static_cast<float>(cameraNearField_->value());
    displayedCamera.farPlane = std::max(static_cast<float>(cameraFarField_->value()), displayedCamera.nearPlane + 0.001f);
    displayedCamera.primary = cameraPrimaryField_->isChecked();
    updatingUi_ = true;
    cameraFarField_->setValue(displayedCamera.farPlane);
    updatingUi_ = false;
    emit CameraEdited(selectedEntity_, displayedCamera);
}

void InspectorPanel::RefreshMaterialFields() noexcept
{
    if (!hasMaterial_) return;
    updatingUi_ = true;
    const std::array<float, 3> albedo{displayedMaterial.albedoColor.x, displayedMaterial.albedoColor.y,displayedMaterial.albedoColor.z};
    for (std::size_t axis = 0; axis < albedo.size(); ++axis)
        if (!albedoColorFields_[axis]->hasFocus()) albedoColorFields_[axis]->setValue(albedo[axis]);

    if (!normalStrengthField_->hasFocus()) normalStrengthField_->setValue(displayedMaterial.normalStrength);
    if (!metallicField_->hasFocus()) metallicField_->setValue(displayedMaterial.metallic);
    if (!roughnessField_->hasFocus()) roughnessField_->setValue(displayedMaterial.roughness);
    if (!specularLevelField_->hasFocus()) specularLevelField_->setValue(displayedMaterial.specularLevel);
    albedoTexturePathField_->setText(QFile::decodeName(QByteArray::fromStdString(displayedMaterial.albedoTexturePath)));
    normalTexturePathField_->setText(QString::fromStdString(displayedMaterial.normalTexturePath));
    updatingUi_ = false;
}

void InspectorPanel::CommitMaterialEdit() noexcept
{
    if (updatingUi_ || !hasMaterial_ || !selectedEntity_.IsValid()) return;
    displayedMaterial.albedoColor = {static_cast<float>(albedoColorFields_[0]->value()),static_cast<float>(albedoColorFields_[1]->value()),
        static_cast<float>(albedoColorFields_[2]->value())};
    displayedMaterial.normalStrength = static_cast<float>(normalStrengthField_->value());
    displayedMaterial.metallic = static_cast<float>(metallicField_->value());
    displayedMaterial.roughness = static_cast<float>(roughnessField_->value());
    displayedMaterial.specularLevel = static_cast<float>(specularLevelField_->value());
    emit MaterialEdited(selectedEntity_, displayedMaterial);
}

void InspectorPanel::OnChooseAlbedoTexture()
{
    if (!hasMaterial_ || !selectedEntity_.IsValid())
        return;

    const QString filePath = QFileDialog::getOpenFileName(this,"Select Albedo Texture",QString{},"Albedo Images (*.png *.jpg *.jpeg *.bmp *.tga)");
    if (filePath.isEmpty())
        return;

    const QByteArray encodedPath = QFile::encodeName(filePath);
    qmec::ImageData decodedImage{};
    if (!qmec::LoadImageRgba8(encodedPath.constData(), decodedImage))
    {
        QMessageBox::warning(this, "Invalid Albedo Texture","The selected file could not be decoded as a supported image.");
        return;
    }

    displayedMaterial.albedoTexturePath = encodedPath.toStdString();
    RefreshMaterialFields();
    emit MaterialEdited(selectedEntity_, displayedMaterial);
}

void InspectorPanel::OnChooseNormalTexture()
{
    if (!hasMaterial_ || !selectedEntity_.IsValid())
        return;

    const QString filePath = QFileDialog::getOpenFileName(this, "Select Normal Texture", QString{}, "Normal Images (*.png *.jpg *.jpeg *.bmp *.tga)");
    if (filePath.isEmpty())
        return;

    const QByteArray encodedPath = QFile::encodeName(filePath);
    qmec::ImageData decodedImage{};
    if (!qmec::LoadImageRgba8(encodedPath.constData(), decodedImage))
    {
        QMessageBox::warning(this, "Invalid Normal Texture", "The selected file could not be decoded as a supported image.");
        return;
    }

    displayedMaterial.normalTexturePath = encodedPath.toStdString();
    RefreshMaterialFields();
    emit MaterialEdited(selectedEntity_, displayedMaterial);
}

void InspectorPanel::RefreshColliderFields() noexcept
{
    if (!hasCollider_) return;
    updatingUi_ = true;
    const int shapeIndex = static_cast<int>(displayedCollider_.shape.index());
    colliderShapeField_->setCurrentIndex(shapeIndex);
    colliderTriggerField_->setChecked(displayedCollider_.isTrigger);

    qmec::Vec3 centre{};
    if (const auto* box = std::get_if<qmec::BoxShape>(&displayedCollider_.shape))
    {
        centre = box->centre;
        if (!boxHalfExtentsFields_[0]->hasFocus()) boxHalfExtentsFields_[0]->setValue(box->halfExtents.x);
        if (!boxHalfExtentsFields_[1]->hasFocus()) boxHalfExtentsFields_[1]->setValue(box->halfExtents.y);
        if (!boxHalfExtentsFields_[2]->hasFocus()) boxHalfExtentsFields_[2]->setValue(box->halfExtents.z);
    }
    else if (const auto* plane = std::get_if<qmec::PlaneShape>(&displayedCollider_.shape))
    {
        centre = plane->centre;
        if (!planeHalfWidthField_->hasFocus()) planeHalfWidthField_->setValue(plane->halfWidthX);
        if (!planeHalfLengthField_->hasFocus()) planeHalfLengthField_->setValue(plane->halfLengthZ);
    }
    else if (const auto* sphere = std::get_if<qmec::SphereShape>(&displayedCollider_.shape))
    {
        centre = sphere->centre;
        if (!sphereRadiusField_->hasFocus()) sphereRadiusField_->setValue(sphere->radius);
    }
    else if (const auto* cylinder = std::get_if<qmec::CylinderShape>(&displayedCollider_.shape))
    {
        centre = cylinder->centre;
        if (!cylinderRadiusField_->hasFocus()) cylinderRadiusField_->setValue(cylinder->radius);
        if (!cylinderHalfHeightField_->hasFocus()) cylinderHalfHeightField_->setValue(cylinder->halfHeight);
    }

    if (!colliderCenterFields_[0]->hasFocus()) colliderCenterFields_[0]->setValue(centre.x);
    if (!colliderCenterFields_[1]->hasFocus()) colliderCenterFields_[1]->setValue(centre.y);
    if (!colliderCenterFields_[2]->hasFocus()) colliderCenterFields_[2]->setValue(centre.z);
    boxShapeGroup_->setVisible(shapeIndex == 0);
    planeShapeGroup_->setVisible(shapeIndex == 1);
    sphereShapeGroup_->setVisible(shapeIndex == 2);
    cylinderShapeGroup_->setVisible(shapeIndex == 3);
    updatingUi_ = false;
}

void InspectorPanel::CommitColliderEdit() noexcept
{
    if (updatingUi_ || !hasCollider_ || !selectedEntity_.IsValid()) return;
    const qmec::Vec3 centre{
        static_cast<float>(colliderCenterFields_[0]->value()),
        static_cast<float>(colliderCenterFields_[1]->value()),
        static_cast<float>(colliderCenterFields_[2]->value())};
    displayedCollider_.isTrigger = colliderTriggerField_->isChecked();

    if (auto* box = std::get_if<qmec::BoxShape>(&displayedCollider_.shape))
    {
        box->centre = centre;
        box->halfExtents = {static_cast<float>(boxHalfExtentsFields_[0]->value()),
            static_cast<float>(boxHalfExtentsFields_[1]->value()),
            static_cast<float>(boxHalfExtentsFields_[2]->value())};
    }
    else if (auto* plane = std::get_if<qmec::PlaneShape>(&displayedCollider_.shape))
    {
        plane->centre = centre;
        plane->halfWidthX = static_cast<float>(planeHalfWidthField_->value());
        plane->halfLengthZ = static_cast<float>(planeHalfLengthField_->value());
    }
    else if (auto* sphere = std::get_if<qmec::SphereShape>(&displayedCollider_.shape))
    {
        sphere->centre = centre;
        sphere->radius = static_cast<float>(sphereRadiusField_->value());
    }
    else if (auto* cylinder = std::get_if<qmec::CylinderShape>(&displayedCollider_.shape))
    {
        cylinder->centre = centre;
        cylinder->radius = static_cast<float>(cylinderRadiusField_->value());
        cylinder->halfHeight = static_cast<float>(cylinderHalfHeightField_->value());
    }
    emit ColliderEdited(selectedEntity_, displayedCollider_);
}

void InspectorPanel::OnColliderShapeChanged(int shapeIndex) noexcept
{
    if (updatingUi_ || !hasCollider_ || shapeIndex < 0 || shapeIndex > 3) return;
    const qmec::Vec3 centre{
        static_cast<float>(colliderCenterFields_[0]->value()),
        static_cast<float>(colliderCenterFields_[1]->value()),
        static_cast<float>(colliderCenterFields_[2]->value())};
    switch (shapeIndex)
    {
    case 0: displayedCollider_.shape = qmec::BoxShape{centre}; break;
    case 1: displayedCollider_.shape = qmec::PlaneShape{centre}; break;
    case 2: displayedCollider_.shape = qmec::SphereShape{centre, 0.5f}; break;
    case 3: displayedCollider_.shape = qmec::CylinderShape{centre, 0.5f, 0.5f}; break;
    }
    RefreshColliderFields();
    CommitColliderEdit();
}
