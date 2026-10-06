cmake_minimum_required(VERSION 3.24)

project(QMEC VERSION 0.1.0 LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
)

FetchContent_MakeAvailable(nlohmann_json)

set(CMAKE_AUTOMOC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(QMECSandbox Sandbox/Source/main.cpp)

if(NOT WIN32)
    message(FATAL_ERROR "QMEC currently supports Windows only.")
endif()

set(QMEC_QT_ROOT "C:/Qt" CACHE PATH "Qt installation root")
file(GLOB QMEC_QT_MSVC_PREFIXES LIST_DIRECTORIES TRUE
    "${QMEC_QT_ROOT}/*/msvc2022_64")
if(QMEC_QT_MSVC_PREFIXES)
    list(PREPEND CMAKE_PREFIX_PATH ${QMEC_QT_MSVC_PREFIXES})
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets)

qt_add_executable(
    QMECEditor
    Editor/main.cpp
    Editor/Editor.cpp
    Editor/Editor.h
    Editor/panels/Hierarchy/HierarchyPanel.cpp
    Editor/panels/Hierarchy/HierarchyPanel.h
    Editor/panels/Inspector/InspectorPanel.cpp
    Editor/panels/Inspector/InspectorPanel.h
    Editor/panels/Viewport/Viewport.cpp
    Editor/panels/Viewport/Viewport.h
    Editor/panels/GameViewport/GameViewport.cpp
    Editor/panels/GameViewport/GameViewport.h
)

target_link_libraries(QMECEditor PRIVATE Qt6::Widgets)

get_filename_component(QMEC_QT_PREFIX "${Qt6_DIR}/../../.." ABSOLUTE)
find_program(QMEC_WINDEPLOYQT_EXECUTABLE
    NAMES windeployqt.exe
    PATHS "${QMEC_QT_PREFIX}/bin"
    NO_DEFAULT_PATH
    REQUIRED
)

add_custom_command(TARGET QMECEditor POST_BUILD
    COMMAND "${QMEC_WINDEPLOYQT_EXECUTABLE}"
        "--$<IF:$<CONFIG:Debug>,debug,release>"
        "$<TARGET_FILE:QMECEditor>"
    VERBATIM
)

add_library(stb_image STATIC
    ThirdParty/stb/stb_image.cpp
)

target_include_directories(stb_image
    PUBLIC ThirdParty/stb
)

add_library(QMEC STATIC
    Engine/Include/QMEC/Scripting/Script.h
    Engine/Source/Scripting/Script.cpp
    Engine/Source/Platform/Windows/Win32Window.cpp
    Engine/Source/Graphics/Direct3D11/D3D11Renderer.cpp
    Engine/Include/QMEC/Graphics/Debug/DebugLine.h
    Engine/Include/QMEC/Graphics/Debug/DebugRenderer.h
    Engine/Source/Graphics/Debug/DebugRenderer.cpp
    Engine/Source/Math/Mat4.cpp
    Engine/Include/QMEC/Math/Mat3.h
    Engine/Source/Math/Quat.cpp
    Engine/Source/Math/Vec3.cpp
    Engine/Source/Scene/Camera.cpp
    Engine/Source/Assets/ImageData.cpp
    Engine/Source/Assets/AssetManager.cpp
    Engine/Source/ECS/Registry.cpp
    "Engine/Include/QMEC/Input/MouseDelta.h"
    "Engine/Include/QMEC/Assets/ImageData.h"
    Engine/Source/Graphics/Direct3D11/D3D11MeshManager.cpp
    Engine/Source/Assets/AssetSystem.cpp
    Engine/Include/QMEC/Graphics/PrimitiveMesh/PrimitiveMeshData.h
    Engine/Include/QMEC/Graphics/D3D11MeshManager.h
    Engine/Include/QMEC/Assets/AssetSystem.h
    Engine/Include/QMEC/Scene/Scene.h
    Engine/Source/Scene/Scene.cpp
    Engine/Include/QMEC/Scene/Components/DirectionalLightComponent.h
    Engine/Include/QMEC/Scene/Components/MaterialComponent.h
    Engine/Include/QMEC/Scene/Components/NameComponent.h
    Engine/Include/QMEC/Physics/PhyscisWorld.h
    Engine/Include/QMEC/Scene/Components/RigidBodyComponent.h
    Engine/Include/QMEC/Physics/AABB.h
    Engine/Include/QMEC/Physics/Collider/ColliderShapes.h
    Engine/Include/QMEC/Physics/Collider/WorldShapes.h
    Engine/Source/Physics/WorldShapes.cpp
    Engine/Include/QMEC/Scene/Components/ColliderComponent.h
    Engine/Shaders/MaterialTemplate.hlsl
    Engine/Source/Physics/PhysicsWorld.cpp
    Engine/Include/QMEC/Physics/BroadPhase.h
    Engine/Include/QMEC/Physics/NarrowPhase.h
    
    Engine/Include/QMEC/Physics/Solver/ContactSolver.h
    Engine/Source/Physics/BroadPhase.cpp
    Engine/Include/QMEC/Physics/CollisionPair.h
    Engine/Source/Physics/NarrowPhase.cpp
    Engine/Include/QMEC/Physics/ContactManifold.h
    Engine/Source/Physics/ContactSolver.cpp
 "Engine/Include/QMEC/Scene/SceneSerializer.h" "Engine/Source/Scene/SceneSerializer.cpp" "Engine/Source/Physics/ConvexCollision.cpp" "Engine/Include/QMEC/Physics/ConvexCollision.h")

set_source_files_properties(Engine/Shaders/MaterialTemplate.hlsl PROPERTIES HEADER_FILE_ONLY TRUE)

target_include_directories(QMEC
    PUBLIC Engine/Include
)

# Game-specific scripts live in Sandbox, not in the reusable engine library.
# Both the editor (Play mode) and sandbox executable link this game module.
add_library(QMECGame STATIC
    Sandbox/Game/VehicleController.h
    Sandbox/Game/VehicleController.cpp
    Sandbox/Game/WheelController.h
    Sandbox/Game/WheelController.cpp
    Sandbox/Game/RegisterSandboxScripts.h
    Sandbox/Game/RegisterSandboxScripts.cpp)

target_include_directories(QMECGame
    PUBLIC Sandbox
)

target_link_libraries(QMECGame PUBLIC QMEC)

target_compile_definitions(QMEC PRIVATE
    UNICODE
    _UNICODE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

target_link_libraries(QMEC PRIVATE
    nlohmann_json::nlohmann_json
    stb_image
    user32
    d3d11
    dxgi
    d3dcompiler

)

target_link_libraries(QMECSandbox PRIVATE QMEC QMECGame)
target_link_libraries(QMECEditor PRIVATE QMEC QMECGame)

add_custom_target(QMECAssets
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:QMECSandbox>/Shaders"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/ColorTriangle.hlsl"
        "$<TARGET_FILE_DIR:QMECSandbox>/Shaders/ColorTriangle.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/MaterialTemplate.hlsl"
        "$<TARGET_FILE_DIR:QMECSandbox>/Shaders/MaterialTemplate.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/DebugLines.hlsl"
        "$<TARGET_FILE_DIR:QMECSandbox>/Shaders/DebugLines.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/PostProcess.hlsl"
        "$<TARGET_FILE_DIR:QMECSandbox>/Shaders/PostProcess.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/Assets"
        "$<TARGET_FILE_DIR:QMECSandbox>/Assets"
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:QMECEditor>/Shaders"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/ColorTriangle.hlsl"
        "$<TARGET_FILE_DIR:QMECEditor>/Shaders/ColorTriangle.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/MaterialTemplate.hlsl"
        "$<TARGET_FILE_DIR:QMECEditor>/Shaders/MaterialTemplate.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/DebugLines.hlsl"
        "$<TARGET_FILE_DIR:QMECEditor>/Shaders/DebugLines.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/Engine/Shaders/PostProcess.hlsl"
        "$<TARGET_FILE_DIR:QMECEditor>/Shaders/PostProcess.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/Assets"
        "$<TARGET_FILE_DIR:QMECEditor>/Assets"
    VERBATIM
)

add_dependencies(QMECSandbox QMECAssets)
add_dependencies(QMECEditor QMECAssets)

set_property(TARGET QMECSandbox PROPERTY
    VS_DEBUGGER_WORKING_DIRECTORY
    "$<TARGET_FILE_DIR:QMECSandbox>"
)

set_property(TARGET QMECEditor PROPERTY
    VS_DEBUGGER_WORKING_DIRECTORY
    "$<TARGET_FILE_DIR:QMECEditor>"
)

if(MSVC)
    target_compile_options(stb_image PRIVATE /W0)
    target_compile_options(QMEC PRIVATE /W4 /permissive-)
    target_compile_options(QMECSandbox PRIVATE /W4 /permissive-)
    target_compile_options(QMECEditor PRIVATE /W4 /permissive-)
endif()

  
