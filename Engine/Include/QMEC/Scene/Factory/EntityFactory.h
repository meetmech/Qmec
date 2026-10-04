#pragma once

#include "QMEC/ECS/Registry.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/MeshRendererComponent.h"
#include "QMEC/Graphics/PrimitiveMesh/PrimitiveMeshData.h"
#include "QMEC/Scene/Components/CameraComponent.h"
#include "QMEC/Scene/Components/DirectionalLightComponent.h"
#include "QMEC/Scene/Components/MaterialComponent.h"
#include "QMEC/Scene/Components/NameComponent.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Assets/AssetSystem.h"
#include "QMEC/ECS/Entity.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

namespace qmec
{
    class EntityFactory
    {
    public:
        explicit EntityFactory(Registry& registry, AssetSystem& assetsystem) noexcept : registry_{registry}, assetSystem_{ assetsystem }
        {
        }

        Entity createMeshEntity(MeshHandle mesh, const TransformComponent& transform = {}, std::string name = "Mesh")
        {
            if (assetSystem_.GetMesh(mesh) == nullptr || assetSystem_.GetGPUMesh(mesh) == nullptr)
            {
                return Entity{};
            }
            MeshRendererComponent meshrender{};
            meshrender.mesh = mesh;
           
            Entity entity = registry_.CreateEntity();
            registry_.AddComponent(entity, transform);
            registry_.AddComponent(entity, meshrender);
            registry_.AddComponent(entity, MaterialComponent{});
            registry_.AddComponent(entity, NameComponent{std::move(name)});
            registry_.AddComponent(entity, CreateColliderForMesh(mesh));

            return entity;
        }

        bool addMatchingCollider(const Entity entity)
        {
            if (!registry_.IsAlive(entity) || registry_.HasComponent<ColliderComponent>(entity))
                return false;

            const auto* meshRenderer = registry_.GetComponent<MeshRendererComponent>(entity);
            if (meshRenderer == nullptr)
                return false;

            return registry_.AddComponent(entity, CreateColliderForMesh(meshRenderer->mesh));
        }

        Entity createCubeEntity(const TransformComponent& transform = {}, std::string name = "Cube")
        {
            if (!cubeMesh_.IsValid())
            {
                cubeMesh_ = assetSystem_.CreateAsset(CubeMesh{});
            }

            return createMeshEntity(cubeMesh_, transform, std::move(name));
        }

        Entity createSphereEntity(const TransformComponent& transform = {}, std::string name = "Sphere")
        {
            if (!sphereMesh_.IsValid())
            {
                sphereMesh_ = assetSystem_.CreateAsset(SphereMesh{});
            }

            return createMeshEntity(sphereMesh_, transform, std::move(name));
        }

        Entity createCylinderEntity(const TransformComponent& transform = {}, std::string name = "Cylinder")
        {
            if (!cylinderMesh_.IsValid())
            {
                cylinderMesh_ = assetSystem_.CreateAsset(CylinderMesh{});
            }

            return createMeshEntity(cylinderMesh_, transform, std::move(name));
        }

        Entity createPlaneEntity(const TransformComponent& transform = {}, std::string name = "Plane")
        {
            if (!planeMesh_.IsValid())
            {
                planeMesh_ = assetSystem_.CreateAsset(PlaneMesh{});
            }

            return createMeshEntity(planeMesh_, transform, std::move(name));
        }

        Entity createCameraEntity(const CameraComponent& cam, const TransformComponent& transform = {}, std::string name = "Camera")
        {
            Entity entity = registry_.CreateEntity();
            registry_.AddComponent(entity, cam);
            registry_.AddComponent(entity, transform);
            registry_.AddComponent(entity, NameComponent{std::move(name)});
            return entity;
        }
        

        Entity createDirectionalLightEntity( const DirectionalLightComponent& light = {},const TransformComponent& transform = {},std::string name = "Directional Light")
        {
            Entity entity = registry_.CreateEntity();
            registry_.AddComponent(entity, transform);
            registry_.AddComponent(entity, light);
            registry_.AddComponent(entity, NameComponent{std::move(name)});
            return entity;
        }


        template<typename Component>
        bool addComponent(const Entity entity, const Component& component);

    private:
        [[nodiscard]] ColliderComponent CreateColliderForMesh(MeshHandle mesh) const
        {
            ColliderComponent collider{};
            if (cubeMesh_.IsValid() && mesh == cubeMesh_)
                collider.shape = BoxShape{{}, {0.5f, 0.5f, 0.5f}};
            else if (sphereMesh_.IsValid() && mesh == sphereMesh_)
                collider.shape = SphereShape{{}, 1.0f};
            else if (cylinderMesh_.IsValid() && mesh == cylinderMesh_)
                collider.shape = CylinderShape{{}, 1.0f, 1.0f};
            else if (planeMesh_.IsValid() && mesh == planeMesh_)
                collider.shape = PlaneShape{{}, 0.5f, 0.5f};
            else
            {
                const MeshData* meshData = assetSystem_.GetMesh(mesh);
                if (meshData == nullptr || meshData->vertices.empty())
                    return collider;

                const float largest = (std::numeric_limits<float>::max)();
                Vec3 minimum{largest, largest, largest};
                Vec3 maximum{-largest, -largest, -largest};
                for (const Vertex& vertex : meshData->vertices)
                {
                    minimum.x = (std::min)(minimum.x, vertex.position.x);
                    minimum.y = (std::min)(minimum.y, vertex.position.y);
                    minimum.z = (std::min)(minimum.z, vertex.position.z);
                    maximum.x = (std::max)(maximum.x, vertex.position.x);
                    maximum.y = (std::max)(maximum.y, vertex.position.y);
                    maximum.z = (std::max)(maximum.z, vertex.position.z);
                }

                collider.shape = BoxShape{(minimum + maximum) * 0.5f,(maximum - minimum) * 0.5f};
            }

            return collider;
        }

        Registry& registry_;
        AssetSystem& assetSystem_;
        MeshHandle cubeMesh_{};
        MeshHandle sphereMesh_{};
        MeshHandle cylinderMesh_{};
        MeshHandle planeMesh_{};
    };
}

#include "QMEC/ECS/EntityFactory.inl"
