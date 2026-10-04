#pragma once

#include "QMEC/ECS/Entity.h"

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace qmec
{
    class Scene;

    struct ScriptContext
    {
        Scene& scene;
        Entity entity{};
        float deltaTime{};
    };

    class Script
    {
    public:
        virtual ~Script() = default;

        virtual void OnCreate(const ScriptContext&) {}
        virtual void OnUpdate(const ScriptContext&) {}
        virtual void OnDestroy(const ScriptContext&) {}
    };

    struct ScriptTypeInfo
    {
        struct EntityReferenceField
        {
            std::string id{};
            std::string displayName{};
            std::function<Entity(const Script&)> get{};
            std::function<void(Script&, Entity)> set{};
        };

        std::string id{};
        std::string displayName{};
        std::function<std::shared_ptr<Script>()> create{};
        std::vector<EntityReferenceField> entityReferenceFields{};
    };

    template<typename TScript>
    [[nodiscard]] ScriptTypeInfo::EntityReferenceField MakeEntityReferenceField(
        std::string id,
        std::string displayName,
        Entity TScript::* member)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return {
            std::move(id),
            std::move(displayName),
            [member](const Script& script)
            {
                return static_cast<const TScript&>(script).*member;
            },
            [member](Script& script, Entity value)
            {
                static_cast<TScript&>(script).*member = value;
            }};
    }

    class ScriptTypeRegistry final
    {
    public:
        [[nodiscard]] static ScriptTypeRegistry& Instance() noexcept;

        bool Register(std::string id, std::string displayName,
            std::function<std::shared_ptr<Script>()> create,
            std::vector<ScriptTypeInfo::EntityReferenceField> entityReferenceFields = {});

        template<typename TScript>
        bool Register(std::string id, std::string displayName,
            std::vector<ScriptTypeInfo::EntityReferenceField> entityReferenceFields = {})
        {
            static_assert(std::is_base_of_v<Script, TScript>);
            return Register(std::move(id), std::move(displayName),
                []() { return std::make_shared<TScript>(); },
                std::move(entityReferenceFields));
        }

        [[nodiscard]] const std::vector<ScriptTypeInfo>& GetTypes() const noexcept;
        [[nodiscard]] const ScriptTypeInfo* Find(std::string_view id) const noexcept;
        [[nodiscard]] std::shared_ptr<Script> Create(std::string_view id) const;

    private:
        std::vector<ScriptTypeInfo> types_{};
    };

    struct ScriptInstance
    {
        std::string typeId{};
        std::shared_ptr<Script> object{};
    };


    struct ScriptComponent
    {
        std::vector<ScriptInstance> instances{};
    };
}
