#include "QMEC/Graphics/D3D11Renderer.h"
#include "QMEC/Graphics/Debug/DebugRenderer.h"
#include "QMEC/Graphics/Platform/Win32Window.h"
#include "QMEC/Scene/Camera.h"
#include "QMEC/Assets/AssetSystem.h"
#include "QMEC/Graphics/D3D11MeshManager.h"
#include "QMEC/Scene/Scene.h"
#include "QMEC/Scene/Factory/EntityFactory.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/CameraComponent.h"
#include "QMEC/Scene/Components/MeshRendererComponent.h"
#include "QMEC/Scene/Components/DirectionalLightComponent.h"
#include "QMEC/Scene/Components/RigidBodyComponent.h"
#include "QMEC/Scene/Components/ColliderComponent.h"
#include "QMEC/Physics/Collider/ColliderShapes.h"
#include "QMEC/Physics/Collider/WorldShapes.h"
#include "QMEC/Physics/PhyscisWorld.h"
#include "Game/RegisterSandboxScripts.h"
#include <chrono>
#include <iostream>
#include <type_traits>
#include <variant>

int main()
{
    qmec::game::RegisterSandboxScripts();
    std::cout << "QMEC starting...\n";

    qmec::platform::windows::Win32Window window{};
    if(!window.Initialize(L"QMEC", 1280U, 720U))
    {
        std::cerr << "Failed to create the QMEC window.\n";
        return 1;
    }

    qmec::graphics::D3D11Renderer renderer{};
    if(!renderer.Initialize(
           window.NativeHandle(),
           window.ClientWidth(),
           window.ClientHeight()))
    {
        std::cerr << "Failed to initialize Direct3D 11.\n";
        return 1;
    }

    qmec::graphics::D3D11MeshManager gpuMeshManager{renderer.GetDevice()};
    qmec::assets::AssetManager assetManager{};
    qmec::assets::AssetSystem assetSystem{assetManager, gpuMeshManager};
    qmec::scene::Scene scene{};
    qmec::scene::factory::EntityFactory factory{scene.GetRegistry(), assetSystem};


    qmec::scene::components::TransformComponent firstTransform{};
    firstTransform.position = { -1.0f, 0.0f, 4.0f };

    const qmec::ecs::Entity firstCube = factory.createCubeEntity(firstTransform);

    qmec::scene::components::RigidBodyComponent firstRigidbody{};
    firstRigidbody.mass = 1.0f;
    firstRigidbody.inverseMass = 1.0f / firstRigidbody.mass;

    qmec::scene::components::ColliderComponent firstCollider{};
    firstCollider.shape = qmec::physics::BoxShape{.centre = {0.0f, 0.0f, 0.0f},.halfExtents = {0.5f, 0.5f, 0.5f}};

    factory.addComponent(firstCube, firstRigidbody);
    factory.addComponent(firstCube, firstCollider);


    // ------------------------------------------------------------
    // Second cube
    // ------------------------------------------------------------

    qmec::scene::components::TransformComponent secondTransform{};
    secondTransform.position = { -1.0f, 10.0f, 3.5f };

    const qmec::ecs::Entity secondCube = factory.createCubeEntity(secondTransform);

    qmec::scene::components::RigidBodyComponent secondRigidbody{};
    secondRigidbody.mass = 1.0f;
    secondRigidbody.inverseMass = 1.0f / secondRigidbody.mass;

    qmec::scene::components::ColliderComponent secondCollider{};
    secondCollider.shape = qmec::physics::BoxShape{.centre = {0.0f, 0.0f, 0.0f},.halfExtents = {0.5f, 0.5f, 0.5f}};

    factory.addComponent(secondCube, secondRigidbody);
    factory.addComponent(secondCube, secondCollider);

    // ------------------------------------------------------------
   // Sphere
   // ------------------------------------------------------------

    qmec::scene::components::TransformComponent sphereTransformTwo{};
    sphereTransformTwo.position = { 5.0f, 0.0f, 5.0f }; // Moves into the cylinder's side.

    const qmec::ecs::Entity sphereTwo = factory.createSphereEntity(sphereTransformTwo);

    qmec::scene::components::RigidBodyComponent sphereRigidbodyTwo{};
    sphereRigidbodyTwo.mass = 1.0f;
    sphereRigidbodyTwo.inverseMass = 1.0f / sphereRigidbodyTwo.mass;
    sphereRigidbodyTwo.isKinematic = false;
    sphereRigidbodyTwo.velocity = {-2.0f, 0.0f, 0.0f};

    qmec::scene::components::ColliderComponent sphereColliderTwo{};
    sphereColliderTwo.shape = qmec::physics::SphereShape{.centre = {0.0f, 0.0f, 0.0f},.radius = 1.0f};

    factory.addComponent(sphereTwo, sphereRigidbodyTwo);
    factory.addComponent(sphereTwo, sphereColliderTwo);

    qmec::scene::components::TransformComponent sphereTransform{};
    sphereTransform.position = { 2.0f, 4.0f, 5.0f }; 
    const qmec::ecs::Entity sphere = factory.createSphereEntity(sphereTransform);

    qmec::scene::components::RigidBodyComponent sphereRigidbody{};
    sphereRigidbody.mass = 10.0f;
    sphereRigidbody.inverseMass = 1.0f / sphereRigidbody.mass;
    sphereRigidbody.isKinematic = false;

    qmec::scene::components::ColliderComponent sphereCollider{};
    sphereCollider.shape = qmec::physics::SphereShape{
        .centre = {0.0f, 0.0f, 0.0f},
        .radius = 1.0f
    };

    factory.addComponent(sphere, sphereRigidbody);
    factory.addComponent(sphere, sphereCollider);


    qmec::scene::components::TransformComponent cylinderTransform{};
    cylinderTransform.position = { 2.0f, 0.0f, 5.0f }; 

    const qmec::ecs::Entity cylinder = factory.createCylinderEntity(cylinderTransform);

    qmec::scene::components::RigidBodyComponent cylinderRigidbody{};
    cylinderRigidbody.mass = 1.0f;
    cylinderRigidbody.inverseMass = 1.0f / cylinderRigidbody.mass;
    cylinderRigidbody.isKinematic = false;

    qmec::scene::components::ColliderComponent cylinderCollider{};
    cylinderCollider.shape = qmec::physics::CylinderShape{
        .centre = {0.0f, 0.0f, 0.0f},
        .radius = 1.0f,
        .halfHeight = 1.0f // Mesh total height is 2.
    };

    factory.addComponent(cylinder, cylinderRigidbody);
    factory.addComponent(cylinder, cylinderCollider);



    // ------------------------------------------------------------
    // Ground plane
    // ------------------------------------------------------------

    qmec::scene::components::TransformComponent thirdTransform{};
    thirdTransform.position = { 8.0f, -3.0f, 4.0f };
    thirdTransform.scale = { 30.0f, 30.0f, 30.0f };

    const qmec::ecs::Entity plane = factory.createPlaneEntity(thirdTransform);

    qmec::scene::components::RigidBodyComponent thirdRigidbody{};
     thirdRigidbody.mass = 1.0f;

    thirdRigidbody.inverseMass = 0.0f;
    thirdRigidbody.isKinematic = true;

    qmec::scene::components::ColliderComponent thirdCollider{};
    thirdCollider.shape = qmec::physics::PlaneShape{
        .centre = {0.0f, 0.0f, 0.0f},
        .halfWidthX = 0.5f,
        .halfLengthZ = 0.5f
    };

    factory.addComponent(plane, thirdRigidbody);
    factory.addComponent(plane, thirdCollider);

    qmec::scene::components::CameraComponent cameraSettings{};
    cameraSettings.primary = true;
    const qmec::ecs::Entity cameraEntity = factory.createCameraEntity(cameraSettings);
    qmec::scene::components::DirectionalLightComponent sunlight{};
    sunlight.color = {1.0f, 0.95f, 0.85f};
    sunlight.intensity = 1.0f;
    qmec::scene::components::TransformComponent lightTransform{};
    lightTransform.rotation = qmec::math::Quat::FromYawPitch(-0.45f, -0.65f);
    const qmec::ecs::Entity lightEntity =
    factory.createDirectionalLightEntity(sunlight, lightTransform);

    if (!firstCube.IsValid() || !secondCube.IsValid() || !cameraEntity.IsValid()
        || !lightEntity.IsValid() || !sphere.IsValid() || !sphereTwo.IsValid()
        || !cylinder.IsValid() || !plane.IsValid())
    {
        std::cerr << "Failed to create the demo scene.\n";
        return 1;
    }
    std::cout << "Scene ready: two cubes, two spheres, one cylinder, one plane, one camera, one directional light, "
        << assetSystem.GetMeshCount() << " shared meshes.\n";

    const qmec::math::Vec3 cubePosition{0.0f, 0.0f, 4.0f};
    qmec::math::Vec3 orbitPivot = cubePosition;

    qmec::scene::Camera camera{};
    camera.SetAspectRatio(static_cast<float>(window.ClientWidth())/static_cast<float>(window.ClientHeight()));
    camera.SetPosition({0.0f, 0.0f, 0.0f});
    camera.LookAt(orbitPivot);

    qmec::graphics::debug::DebugRenderer debug{};

    auto previousFrameTime = std::chrono::steady_clock::now();

    while(window.ProcessMessages())
    {
        if (window.IsKeyDown(qmec::input::Key::Escape))
        {
            break;
        }
        float physicsStepTime = 0.016667;
        float physicsAccumulator = 0;
        const auto currentFrameTime = std::chrono::steady_clock::now();
        float deltaTime = std::chrono::duration<float>(currentFrameTime - previousFrameTime).count();
        previousFrameTime = currentFrameTime;

        if(deltaTime > 0.1f)
        {
            deltaTime = 0.1f;
        }
        if (physicsAccumulator > physicsStepTime)
        {
            physicsAccumulator -= physicsStepTime;
        }

        const qmec::input::MouseDelta mouseDelta = window.ConsumeMouseDelta();
        const float mouseWheelDelta = window.ConsumeMouseWheelDelta();

        const bool altDown = window.IsKeyDown(qmec::input::Key::Alt);
        const bool orbiting = altDown && window.IsMouseButtonDown(qmec::input::MouseButton::Left);
        const bool panning = window.IsMouseButtonDown(qmec::input::MouseButton::Middle);
        const bool zoomDragging = altDown && window.IsMouseButtonDown(qmec::input::MouseButton::Right);
        const bool flyLooking = !altDown && window.IsMouseButtonDown(qmec::input::MouseButton::Right);
        const bool movingFast = window.IsKeyDown(qmec::input::Key::Shift);

        (void)mouseDelta;
        if (mouseWheelDelta)
        {
            qmec::math::Vec3 newPosition =camera.Position() +camera.Forward() * mouseWheelDelta * 0.5;

            camera.SetPosition(newPosition);
        }
        if (orbiting)
        {
            constexpr float sensitivity = 0.003f;

            const float orbitRadius =(camera.Position() - orbitPivot).Length();

            camera.AddYawPitch(mouseDelta.x * sensitivity,-mouseDelta.y * sensitivity);

            camera.SetPosition(orbitPivot - camera.Forward() * orbitRadius);
            camera.LookAt(orbitPivot);
        }
        if (panning)
        {
            constexpr float sensitivity = 0.003f;

            const float horizontal = mouseDelta.x * sensitivity;
            const float vertical = mouseDelta.y * sensitivity;

            qmec::math::Vec3 pan = camera.Right() * horizontal + camera.Up() * vertical;
            camera.SetPosition(camera.Position() + pan);
            orbitPivot += pan;

        }
        (void)zoomDragging;
        if (flyLooking)
        {
            constexpr float sensitivity = 0.003f;

            camera.AddYawPitch(mouseDelta.x * sensitivity,-mouseDelta.y * sensitivity);
        }
        (void)movingFast;
        qmec::math::Vec3 movement{};

        if (window.IsKeyDown(qmec::input::Key::W)) movement += camera.Forward();
        if (window.IsKeyDown(qmec::input::Key::S)) movement += camera.Forward() * -1.0f;
        if (window.IsKeyDown(qmec::input::Key::D)) movement += camera.Right();
        if (window.IsKeyDown(qmec::input::Key::A)) movement += camera.Right() * -1.0f;
        if (window.IsKeyDown(qmec::input::Key::E)) movement += camera.Up();
        if (window.IsKeyDown(qmec::input::Key::Q)) movement += camera.Up() * -1.0f;

        if(movement.Length() > 0.0f)
        {
            constexpr float CameraSpeed = 5.0f;
            movement = movement.Normalized() * CameraSpeed * deltaTime;
            camera.SetPosition(camera.Position() + movement);
        }

        auto* cameraTransform = scene.GetRegistry().GetComponent<qmec::scene::components::TransformComponent>(cameraEntity);
        cameraTransform->position = camera.Position();
        cameraTransform->rotation = camera.Orientation();
        scene.runPhysics(deltaTime);
        debug.Clear();
       
        const auto& registry = scene.GetRegistry();
        const qmec::math::Vec3 colliderColor{0.0f, 1.0f, 0.0f};
        for (const qmec::ecs::Entity entity : registry.GetEntitiesWith<qmec::scene::components::ColliderComponent>())
        {
            const auto* collider = registry.GetComponent<qmec::scene::components::ColliderComponent>(entity);
            const auto* transform = registry.GetComponent<qmec::scene::components::TransformComponent>(entity);
            if (collider == nullptr || transform == nullptr)
                continue;

            std::visit([&](const auto& shape)
                {
                
                    const auto worldShape = qmec::physics::ToWorldShape(shape, *transform);
                    using Shape = std::decay_t<decltype(shape)>;
                    if constexpr (std::is_same_v<Shape, qmec::physics::BoxShape>)
                        debug.DrawBox(worldShape, colliderColor);
                    else if constexpr (std::is_same_v<Shape, qmec::physics::SphereShape>)
                        debug.DrawSphere(worldShape.centre, worldShape.radius, colliderColor);
                    else if constexpr (std::is_same_v<Shape, qmec::physics::PlaneShape>)
                        debug.DrawPlane(worldShape, colliderColor);
                    else if constexpr (std::is_same_v<Shape, qmec::physics::CylinderShape>)
                        debug.DrawCylinder(worldShape, colliderColor);
                }, collider->shape);
        }



        if(!renderer.RenderFrame(0.05F, 0.10F,0.20F,1.0F,scene, assetSystem,cameraEntity, debug.GetLines()))
        {
            std::cerr << "Direct3D failed to present the frame.\n";
            return 1;
        }
    }

    return 0;
}
