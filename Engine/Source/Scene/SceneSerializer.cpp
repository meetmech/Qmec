#include "QMEC/Scene/SceneSerializer.h"

#include "QMEC/Assets/AssetManager.h"
#include "QMEC/Scene/Components/CameraComponent.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Scene/Components/DirectionalLightComponent.h"
#include "QMEC/Scene/Components/MaterialComponent.h"
#include "QMEC/Scene/Components/MeshRendererComponent.h"
#include "QMEC/Scene/Components/NameComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scripting/Script.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace qmec
{
    namespace
    {
        using json = nlohmann::json;
        constexpr std::uint32_t SceneFileVersion = 2U;

        struct PendingScriptEntityReference
        {
            std::size_t scriptIndex{};
            std::string fieldId{};
            std::optional<std::uint64_t> targetFileId{};
        };

        struct EntityRecord
        {
            std::uint64_t fileId{};
            std::optional<std::uint64_t> parentId{};
            std::optional<NameComponent> name{};
            std::optional<TransformComponent> transform{};
            std::optional<MeshRendererComponent> meshRenderer{};
            std::optional<MaterialComponent> material{};
            std::optional<CameraComponent> camera{};
            std::optional<DirectionalLightComponent> directionalLight{};
            std::optional<RigidBodyComponent> rigidBody{};
            std::optional<ColliderComponent> collider{};
            std::optional<ScriptComponent> scripts{};
            std::vector<PendingScriptEntityReference> scriptEntityReferences{};
        };

        std::uint64_t ToKey(Entity entity) noexcept
        {
            return (static_cast<std::uint64_t>(entity.generation) << 32U) |
                static_cast<std::uint64_t>(entity.index);
        }

        json ToJson(const Vec3& value)
        {
            return json::array({value.x, value.y, value.z});
        }

        json ToJson(const Quat& value)
        {
            return json::array({value.x, value.y, value.z, value.w});
        }

        float ReadFloat(const json& value)
        {
            const float result = value.get<float>();
            if (!std::isfinite(result))
                throw json::type_error::create(302, "expected a finite number", &value);
            return result;
        }

        Vec3 ReadVec3(const json& value)
        {
            if (!value.is_array() || value.size() != 3U)
                throw json::type_error::create(302, "expected a three-number array", &value);

            return {ReadFloat(value[0]), ReadFloat(value[1]), ReadFloat(value[2])};
        }

        Quat ReadQuat(const json& value)
        {
            if (!value.is_array() || value.size() != 4U)
                throw json::type_error::create(302, "expected a four-number array", &value);

            Quat result{
                ReadFloat(value[0]), ReadFloat(value[1]),
                ReadFloat(value[2]), ReadFloat(value[3])};
            if (result.LengthSquared() <= 1.0e-12f)
                throw json::type_error::create(302, "expected a non-zero quaternion", &value);
            return result.Normalized();
        }

        json WriteColliderShape(const ColliderComponent& collider)
        {
            return std::visit([](const auto& shape) -> json
            {
                using Shape = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<Shape, BoxShape>)
                {
                    return json{
                        {"type", "box"},
                        {"centre", ToJson(shape.centre)},
                        {"halfExtents", ToJson(shape.halfExtents)}};
                }
                else if constexpr (std::is_same_v<Shape, PlaneShape>)
                {
                    return json{
                        {"type", "plane"},
                        {"centre", ToJson(shape.centre)},
                        {"halfWidthX", shape.halfWidthX},
                        {"halfLengthZ", shape.halfLengthZ}};
                }
                else if constexpr (std::is_same_v<Shape, SphereShape>)
                {
                    return json{
                        {"type", "sphere"},
                        {"centre", ToJson(shape.centre)},
                        {"radius", shape.radius}};
                }
                else
                {
                    return json{
                        {"type", "cylinder"},
                        {"centre", ToJson(shape.centre)},
                        {"radius", shape.radius},
                        {"halfHeight", shape.halfHeight}};
                }
            }, collider.shape);
        }

        ColliderComponent ReadCollider(const json& value)
        {
            if (!value.is_object() || !value.contains("type"))
                throw json::type_error::create(302, "expected a collider shape object", &value);

            ColliderComponent result{};
            const std::string type = value.at("type").get<std::string>();
            if (type == "box")
            {
                result.shape = BoxShape{
                    ReadVec3(value.at("centre")),
                    ReadVec3(value.at("halfExtents"))};
            }
            else if (type == "plane")
            {
                result.shape = PlaneShape{
                    ReadVec3(value.at("centre")),
                    ReadFloat(value.at("halfWidthX")),
                    ReadFloat(value.at("halfLengthZ"))};
            }
            else if (type == "sphere")
            {
                result.shape = SphereShape{
                    ReadVec3(value.at("centre")), ReadFloat(value.at("radius"))};
            }
            else if (type == "cylinder")
            {
                result.shape = CylinderShape{
                    ReadVec3(value.at("centre")),
                    ReadFloat(value.at("radius")),
                    ReadFloat(value.at("halfHeight"))};
            }
            else
            {
                throw json::type_error::create(302, "unsupported collider shape type", &value);
            }

            result.isTrigger = value.value("isTrigger", false);
            return result;
        }

        EntityRecord ReadEntityRecord(const json& value)
        {
            if (!value.is_object() || !value.contains("id") || !value.contains("components"))
                throw json::type_error::create(302, "invalid entity record", &value);

            EntityRecord result{};
            result.fileId = value.at("id").get<std::uint64_t>();
            if (value.contains("parent") && !value.at("parent").is_null())
                result.parentId = value.at("parent").get<std::uint64_t>();

            const json& components = value.at("components");
            if (!components.is_object())
                throw json::type_error::create(302, "entity components must be an object", &components);

            if (components.contains("name"))
                result.name = NameComponent{components.at("name").get<std::string>()};

            if (components.contains("transform"))
            {
                const json& data = components.at("transform");
                TransformComponent transform{};
                transform.position = ReadVec3(data.at("position"));
                transform.rotation = ReadQuat(data.at("rotation"));
                transform.scale = ReadVec3(data.at("scale"));
                result.transform = transform;
            }

            if (result.parentId && !result.transform)
                throw json::type_error::create(302, "parented entities need a transform", &value);

            if (components.contains("meshRenderer"))
            {
                const json& data = components.at("meshRenderer");
                MeshRendererComponent component{};
                component.mesh.index = data.at("meshIndex").get<std::uint32_t>();
                component.materialIndex = data.at("materialIndex").get<std::uint32_t>();
                component.visible = data.at("visible").get<bool>();
                result.meshRenderer = component;
            }

            if (components.contains("material"))
            {
                const json& data = components.at("material");
                MaterialComponent component{};
                component.albedoColor = ReadVec3(data.at("albedoColor"));
                component.albedoTexturePath = data.at("albedoTexturePath").get<std::string>();
                component.normalTexturePath = data.at("normalTexturePath").get<std::string>();
                component.normalStrength = ReadFloat(data.at("normalStrength"));
                component.metallic = ReadFloat(data.at("metallic"));
                component.roughness = ReadFloat(data.at("roughness"));
                component.specularLevel = ReadFloat(data.at("specularLevel"));
                result.material = std::move(component);
            }

            if (components.contains("camera"))
            {
                const json& data = components.at("camera");
                CameraComponent component{};
                component.verticalFieldOfViewRadians = ReadFloat(data.at("verticalFieldOfViewRadians"));
                component.nearPlane = ReadFloat(data.at("nearPlane"));
                component.farPlane = ReadFloat(data.at("farPlane"));
                component.primary = data.at("primary").get<bool>();
                result.camera = component;
            }

            if (components.contains("directionalLight"))
            {
                const json& data = components.at("directionalLight");
                DirectionalLightComponent component{};
                component.color = ReadVec3(data.at("color"));
                component.intensity = ReadFloat(data.at("intensity"));
                component.enabled = data.at("enabled").get<bool>();
                result.directionalLight = component;
            }

            if (components.contains("rigidBody"))
            {
                const json& data = components.at("rigidBody");
                RigidBodyComponent component{};
                component.position = ReadVec3(data.at("position"));
                component.rotation = ReadQuat(data.at("rotation"));
                component.mass = ReadFloat(data.at("mass"));
                component.velocity = ReadVec3(data.at("velocity"));
                component.angularVelocity = ReadVec3(data.at("angularVelocity"));
                component.isKinematic = data.at("isKinematic").get<bool>();
                component.inverseMass = !component.isKinematic && component.mass > 0.0f
                    ? 1.0f / component.mass
                    : 0.0f;
                if (data.contains("staticFriction"))
                    component.material.staticFriction =
                        (std::max)(0.0f, ReadFloat(data.at("staticFriction")));
                if (data.contains("dynamicFriction"))
                    component.material.dynamicFriction =
                        (std::max)(0.0f, ReadFloat(data.at("dynamicFriction")));
                if (data.contains("rollingResistance"))
                    component.material.rollingResistance =
                        std::clamp(ReadFloat(data.at("rollingResistance")), 0.0f, 1.0f);
                if (data.contains("elasticity"))
                    component.material.Elasticity =
                        std::clamp(ReadFloat(data.at("elasticity")), 0.0f, 1.0f);
                result.rigidBody = component;
            }

            if (components.contains("collider"))
            {
                const json& data = components.at("collider");
                ColliderComponent component = ReadCollider(data.at("shape"));
                component.isTrigger = data.at("isTrigger").get<bool>();
                result.collider = std::move(component);
            }

            if (components.contains("scripts"))
            {
                const json& data = components.at("scripts");
                if (!data.is_array())
                    throw json::type_error::create(302, "scripts must be an array", &data);

                ScriptComponent component{};
                for (const json& scriptValue : data)
                {
                    std::string typeId{};
                    const json* references = nullptr;
                    if (scriptValue.is_string())
                    {
                        // Version 1 stored only the script type ID.
                        typeId = scriptValue.get<std::string>();
                    }
                    else if (scriptValue.is_object() && scriptValue.contains("typeId"))
                    {
                        typeId = scriptValue.at("typeId").get<std::string>();
                        if (scriptValue.contains("entityReferences"))
                            references = &scriptValue.at("entityReferences");
                    }
                    else
                    {
                        throw json::type_error::create(302, "invalid script record", &scriptValue);
                    }

                    std::shared_ptr<Script> script =
                        ScriptTypeRegistry::Instance().Create(typeId);
                    if (script == nullptr)
                        throw json::type_error::create(302, "unregistered script type", &scriptValue);

                    const std::size_t scriptIndex = component.instances.size();
                    component.instances.push_back({typeId, std::move(script)});

                    if (references != nullptr)
                    {
                        if (!references->is_object())
                            throw json::type_error::create(302,
                                "script entity references must be an object", references);

                        const ScriptTypeInfo* type =
                            ScriptTypeRegistry::Instance().Find(typeId);
                        if (type != nullptr)
                        {
                            for (const ScriptTypeInfo::EntityReferenceField& field :
                                type->entityReferenceFields)
                            {
                                if (!references->contains(field.id))
                                    continue;

                                const json& target = references->at(field.id);
                                std::optional<std::uint64_t> targetFileId{};
                                if (!target.is_null())
                                    targetFileId = target.get<std::uint64_t>();
                                result.scriptEntityReferences.push_back(
                                    {scriptIndex, field.id, targetFileId});
                            }
                        }
                    }
                }
                result.scripts = std::move(component);
            }

            return result;
        }

        void ValidateParentLinks(const std::vector<EntityRecord>& records)
        {
            std::unordered_map<std::uint64_t, std::optional<std::uint64_t>> parents{};
            parents.reserve(records.size());
            for (const EntityRecord& record : records)
                parents.emplace(record.fileId, record.parentId);

            for (const EntityRecord& record : records)
            {
                if (record.parentId && !parents.contains(*record.parentId))
                    throw json::type_error::create(302, "parent ID does not refer to an entity", nullptr);

                std::unordered_set<std::uint64_t> visited{};
                std::uint64_t current = record.fileId;
                while (parents.at(current))
                {
                    current = *parents.at(current);
                    if (!visited.insert(current).second || current == record.fileId)
                        throw json::type_error::create(302, "parent hierarchy contains a cycle", nullptr);
                }
            }
        }
    }

    bool SceneSerializer::Save(const Scene& scene, const std::string& path)
    {
        try
        {
            const Registry& registry = scene.GetRegistry();
            const std::vector<Entity> entities = registry.GetEntities();
            std::unordered_map<std::uint64_t, std::uint64_t> fileIds{};
            fileIds.reserve(entities.size());
            for (std::size_t i = 0; i < entities.size(); ++i)
                fileIds.emplace(ToKey(entities[i]), static_cast<std::uint64_t>(i));

            json document{
                {"format", "QMECScene"},
                {"version", SceneFileVersion},
                {"entities", json::array()}};

            for (const Entity entity : entities)
            {
                json entry{
                    {"id", fileIds.at(ToKey(entity))},
                    {"parent", nullptr},
                    {"components", json::object()}};
                json& components = entry["components"];

                if (const auto* name = registry.GetComponent<NameComponent>(entity))
                    components["name"] = name->name;

                if (const auto* transform = registry.GetComponent<TransformComponent>(entity))
                {
                    components["transform"] = {
                        {"position", ToJson(transform->position)},
                        {"rotation", ToJson(transform->rotation)},
                        {"scale", ToJson(transform->scale)}};

                    if (transform->HasParent())
                    {
                        const auto parentId = fileIds.find(ToKey(transform->GetParent()));
                        if (parentId == fileIds.end())
                            return false;
                        entry["parent"] = parentId->second;
                    }
                }

                if (const auto* mesh = registry.GetComponent<MeshRendererComponent>(entity))
                {
                    components["meshRenderer"] = {
                        {"meshIndex", mesh->mesh.index},
                        {"materialIndex", mesh->materialIndex},
                        {"visible", mesh->visible}};
                }

                if (const auto* material = registry.GetComponent<MaterialComponent>(entity))
                {
                    components["material"] = {
                        {"albedoColor", ToJson(material->albedoColor)},
                        {"albedoTexturePath", material->albedoTexturePath},
                        {"normalTexturePath", material->normalTexturePath},
                        {"normalStrength", material->normalStrength},
                        {"metallic", material->metallic},
                        {"roughness", material->roughness},
                        {"specularLevel", material->specularLevel}};
                }

                if (const auto* camera = registry.GetComponent<CameraComponent>(entity))
                {
                    components["camera"] = {
                        {"verticalFieldOfViewRadians", camera->verticalFieldOfViewRadians},
                        {"nearPlane", camera->nearPlane},
                        {"farPlane", camera->farPlane},
                        {"primary", camera->primary}};
                }

                if (const auto* light = registry.GetComponent<DirectionalLightComponent>(entity))
                {
                    components["directionalLight"] = {
                        {"color", ToJson(light->color)},
                        {"intensity", light->intensity},
                        {"enabled", light->enabled}};
                }

                if (const auto* body = registry.GetComponent<RigidBodyComponent>(entity))
                {
                    components["rigidBody"] = {
                        {"position", ToJson(body->position)},
                        {"rotation", ToJson(body->rotation)},
                        {"mass", body->mass},
                        {"velocity", ToJson(body->velocity)},
                        {"angularVelocity", ToJson(body->angularVelocity)},
                        {"isKinematic", body->isKinematic},
                        {"staticFriction", body->material.staticFriction},
                        {"dynamicFriction", body->material.dynamicFriction},
                        {"rollingResistance", std::clamp(body->material.rollingResistance, 0.0f, 1.0f)},
                        {"elasticity", std::clamp(body->material.Elasticity, 0.0f, 1.0f)}};
                }

                if (const auto* collider = registry.GetComponent<ColliderComponent>(entity))
                {
                    components["collider"] = {{"shape", WriteColliderShape(*collider)},{"isTrigger", collider->isTrigger}};
                }

                if (const auto* scripts = registry.GetComponent<ScriptComponent>(entity))
                {
                    json serializedScripts = json::array();
                    for (const ScriptInstance& script : scripts->instances)
                    {
                        json serializedScript{
                            {"typeId", script.typeId},
                            {"entityReferences", json::object()}};

                        const ScriptTypeInfo* type =
                            ScriptTypeRegistry::Instance().Find(script.typeId);
                        if (type != nullptr && script.object != nullptr)
                        {
                            for (const ScriptTypeInfo::EntityReferenceField& field :
                                type->entityReferenceFields)
                            {
                                const Entity target = field.get(*script.object);
                                if (!target.IsValid() || !registry.IsAlive(target))
                                {
                                    serializedScript["entityReferences"][field.id] = nullptr;
                                    continue;
                                }

                                const auto targetId = fileIds.find(ToKey(target));
                                serializedScript["entityReferences"][field.id] =
                                    targetId != fileIds.end()
                                    ? json(targetId->second)
                                    : json(nullptr);
                            }
                        }
                        serializedScripts.push_back(std::move(serializedScript));
                    }
                    components["scripts"] = std::move(serializedScripts);
                }

                document["entities"].push_back(std::move(entry));
            }

            std::ofstream output{path, std::ios::binary | std::ios::trunc};
            if (!output)
                return false;
            output << document.dump(4);
            return output.good();
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    bool SceneSerializer::Load(Scene& scene, const std::string& path)
    {
        std::vector<EntityRecord> records{};
        try
        {
            std::ifstream input{path, std::ios::binary};
            if (!input)
                return false;

            json document;
            input >> document;
            const std::uint32_t fileVersion = document.is_object()
                ? document.value("version", 0U)
                : 0U;
            if (!document.is_object() || document.value("format", std::string{}) != "QMECScene" ||
                (fileVersion != 1U && fileVersion != SceneFileVersion) ||
                !document.contains("entities") || !document["entities"].is_array())
            {
                return false;
            }

            records.reserve(document["entities"].size());
            std::unordered_set<std::uint64_t> fileIds{};
            for (const json& entity : document["entities"])
            {
                EntityRecord record = ReadEntityRecord(entity);
                if (!fileIds.insert(record.fileId).second)
                    return false;
                records.push_back(std::move(record));
            }
            ValidateParentLinks(records);

            for (const EntityRecord& record : records)
            {
                for (const PendingScriptEntityReference& reference :
                    record.scriptEntityReferences)
                {
                    if (reference.targetFileId && !fileIds.contains(*reference.targetFileId))
                        return false;
                }
            }
        }
        catch (const std::exception&)
        {
            return false;
        }

        Registry& registry = scene.GetRegistry();
        scene.Clear();

        std::unordered_map<std::uint64_t, Entity> entitiesByFileId{};
        entitiesByFileId.reserve(records.size());
        try
        {
            for (const EntityRecord& record : records)
            {
                const Entity entity = registry.CreateEntity();
                entitiesByFileId.emplace(record.fileId, entity);

                const auto addComponent = [&registry, entity](const auto& component)
                {
                    return registry.AddComponent(entity, component);
                };

                if ((record.name && !addComponent(*record.name)) ||
                    (record.transform && !addComponent(*record.transform)) ||
                    (record.meshRenderer && !addComponent(*record.meshRenderer)) ||
                    (record.material && !addComponent(*record.material)) ||
                    (record.camera && !addComponent(*record.camera)) ||
                    (record.directionalLight && !addComponent(*record.directionalLight)) ||
                    (record.rigidBody && !addComponent(*record.rigidBody)) ||
                    (record.collider && !addComponent(*record.collider)) ||
                    (record.scripts && !addComponent(*record.scripts)))
                {
                    scene.Clear();
                    return false;
                }
            }

            for (const EntityRecord& record : records)
            {
                if (!record.parentId)
                    continue;

                if (!scene.SetParent(
                    entitiesByFileId.at(record.fileId),
                    entitiesByFileId.at(*record.parentId),
                    false))
                {
                    scene.Clear();
                    return false;
                }
            }

            for (const EntityRecord& record : records)
            {
                if (record.scriptEntityReferences.empty())
                    continue;

                const Entity owner = entitiesByFileId.at(record.fileId);
                auto* scripts = registry.GetComponent<ScriptComponent>(owner);
                if (scripts == nullptr)
                    throw std::runtime_error("script component missing during reference fix-up");

                for (const PendingScriptEntityReference& reference :
                    record.scriptEntityReferences)
                {
                    if (reference.scriptIndex >= scripts->instances.size())
                        throw std::runtime_error("script reference index is invalid");

                    ScriptInstance& instance = scripts->instances[reference.scriptIndex];
                    const ScriptTypeInfo* type =
                        ScriptTypeRegistry::Instance().Find(instance.typeId);
                    if (type == nullptr || instance.object == nullptr)
                        continue;

                    const auto field = std::find_if(type->entityReferenceFields.begin(),
                        type->entityReferenceFields.end(), [&reference](const auto& candidate)
                        {
                            return candidate.id == reference.fieldId;
                        });
                    if (field == type->entityReferenceFields.end())
                        continue;

                    const Entity target = reference.targetFileId
                        ? entitiesByFileId.at(*reference.targetFileId)
                        : Entity{};
                    field->set(*instance.object, target);
                }
            }
        }
        catch (const std::exception&)
        {
            scene.Clear();
            return false;
        }

        return true;
    }
}
