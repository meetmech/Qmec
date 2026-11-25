#pragma once

#include "QMEC/ECS/Entity.h"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace qmec::scene { class Scene; }

namespace qmec::scripting
{

    struct ScriptContext
    {
        qmec::scene::Scene& scene;
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

        struct FloatField
        {
            std::string id{};
            std::string displayName{};
            float minimum{};
            float maximum{ 1.0f };
            float step{ 0.1f };
            std::function<float(const Script&)> get{};
            std::function<void(Script&, float)> set{};
        };

        struct BoolField
        {
            std::string id{};
            std::string displayName{};
            std::function<bool(const Script&)> get{};
            std::function<void(Script&, bool)> set{};
        };

        struct IntField
        {
            std::string id{};
            std::string displayName{};
            int minimum{};
            int maximum{};
            std::function<int(const Script&)> get{};
            std::function<void(Script&, int)> set{};
        };

        std::string id{};
        std::string displayName{};
        std::function<std::shared_ptr<Script>()> create{};
        std::vector<EntityReferenceField> entityReferenceFields{};
        std::vector<FloatField> floatFields{};
        std::vector<BoolField> boolFields{};
        std::vector<IntField> intFields{};
    };

    template<typename TScript>
    [[nodiscard]] ScriptTypeInfo::EntityReferenceField MakeEntityReferenceField(std::string id,std::string displayName,Entity TScript::* member)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return {std::move(id),std::move(displayName),[member](const Script& script)
            {
                return static_cast<const TScript&>(script).*member;
            },
            [member](Script& script, Entity value)
            {
                static_cast<TScript&>(script).*member = value;
            }};
    }

    template<typename TScript>
    [[nodiscard]] ScriptTypeInfo::FloatField MakeFloatField(std::string id, std::string displayName, float TScript::* member,float minimum, float maximum, float step)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return { std::move(id), std::move(displayName), minimum, maximum, step,
            [member](const Script& script)
            {
                return static_cast<const TScript&>(script).*member;
            },
            [member](Script& script, float value)
            {
                static_cast<TScript&>(script).*member = value;
            } };
    }

    template<typename TScript, typename TNested>
    [[nodiscard]] ScriptTypeInfo::FloatField MakeNestedFloatField(std::string id, std::string displayName, TNested TScript::* nestedMember,float TNested::* member, float minimum, float maximum, float step)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return { std::move(id), std::move(displayName), minimum, maximum, step,
            [nestedMember, member](const Script& script)
            {
                return (static_cast<const TScript&>(script).*nestedMember).*member;
            },
            [nestedMember, member](Script& script, float value)
            {
                (static_cast<TScript&>(script).*nestedMember).*member = value;
            } };
    }

    template<typename TScript, typename TNested>
    [[nodiscard]] ScriptTypeInfo::BoolField MakeNestedBoolField(std::string id, std::string displayName, TNested TScript::* nestedMember,bool TNested::* member)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return { std::move(id), std::move(displayName),
            [nestedMember, member](const Script& script)
            {
                return (static_cast<const TScript&>(script).*nestedMember).*member;
            },
            [nestedMember, member](Script& script, bool value)
            {
                (static_cast<TScript&>(script).*nestedMember).*member = value;
            } };
    }

    template<typename TScript, typename TNested>
    [[nodiscard]] ScriptTypeInfo::IntField MakeNestedIntField(std::string id, std::string displayName, TNested TScript::* nestedMember,int TNested::* member, int minimum, int maximum)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return { std::move(id), std::move(displayName), minimum, maximum,
            [nestedMember, member](const Script& script)
            {
                return (static_cast<const TScript&>(script).*nestedMember).*member;
            },
            [nestedMember, member](Script& script, int value)
            {
                (static_cast<TScript&>(script).*nestedMember).*member = value;
            } };
    }

    template<typename TScript, typename TNested, std::size_t Size>
    [[nodiscard]] ScriptTypeInfo::FloatField MakeNestedFloatArrayField(std::string id, std::string displayName, TNested TScript::* nestedMember,float (TNested::* arrayMember)[Size], std::size_t index,float minimum, float maximum, float step)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return { std::move(id), std::move(displayName), minimum, maximum, step,
            [nestedMember, arrayMember, index](const Script& script)
            {
                return ((static_cast<const TScript&>(script).*nestedMember).*arrayMember)[index];
            },
            [nestedMember, arrayMember, index](Script& script, float value)
            {
                ((static_cast<TScript&>(script).*nestedMember).*arrayMember)[index] = value;
            } };
    }

    template<typename TScript, typename TNested, typename TElement, std::size_t Size>
    [[nodiscard]] ScriptTypeInfo::FloatField MakeNestedArrayElementFloatField(std::string id, std::string displayName, TNested TScript::* nestedMember,std::array<TElement, Size> TNested::* arrayMember, std::size_t index,
        float TElement::* valueMember, float minimum, float maximum, float step)
    {
        static_assert(std::is_base_of_v<Script, TScript>);
        return { std::move(id), std::move(displayName), minimum, maximum, step,
            [nestedMember, arrayMember, index, valueMember](const Script& script)
            {
                return ((static_cast<const TScript&>(script).*nestedMember).*arrayMember)[index].*valueMember;
            },
            [nestedMember, arrayMember, index, valueMember](Script& script, float value)
            {
                ((static_cast<TScript&>(script).*nestedMember).*arrayMember)[index].*valueMember = value;
            } };
    }

    class ScriptTypeRegistry final
    {
    public:
        [[nodiscard]] static ScriptTypeRegistry& Instance() noexcept;

        bool Register(std::string id, std::string displayName,std::function<std::shared_ptr<Script>()> create,std::vector<ScriptTypeInfo::EntityReferenceField> entityReferenceFields = {},
            std::vector<ScriptTypeInfo::FloatField> floatFields = {},
            std::vector<ScriptTypeInfo::BoolField> boolFields = {},
            std::vector<ScriptTypeInfo::IntField> intFields = {});

        template<typename TScript>
        bool Register(std::string id, std::string displayName,std::vector<ScriptTypeInfo::EntityReferenceField> entityReferenceFields = {},
            std::vector<ScriptTypeInfo::FloatField> floatFields = {},
            std::vector<ScriptTypeInfo::BoolField> boolFields = {},
            std::vector<ScriptTypeInfo::IntField> intFields = {})
        {
            static_assert(std::is_base_of_v<Script, TScript>);
            return Register(std::move(id), std::move(displayName),[]() { return std::make_shared<TScript>(); },std::move(entityReferenceFields), std::move(floatFields),std::move(boolFields), std::move(intFields));
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

namespace qmec
{
    using scripting::ScriptContext;
    using scripting::Script;
    using scripting::ScriptTypeInfo;
    using scripting::MakeEntityReferenceField;
    using scripting::MakeFloatField;
    using scripting::MakeNestedFloatField;
    using scripting::MakeNestedBoolField;
    using scripting::MakeNestedIntField;
    using scripting::MakeNestedFloatArrayField;
    using scripting::MakeNestedArrayElementFloatField;
    using scripting::ScriptTypeRegistry;
    using scripting::ScriptInstance;
    using scripting::ScriptComponent;
}
