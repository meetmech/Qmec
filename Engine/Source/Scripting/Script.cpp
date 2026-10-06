#include "QMEC/Scripting/Script.h"

#include <algorithm>
#include <utility>

namespace qmec::scripting
{
    ScriptTypeRegistry& ScriptTypeRegistry::Instance() noexcept
    {
        static ScriptTypeRegistry registry{};
        return registry;
    }

    bool ScriptTypeRegistry::Register(std::string id, std::string displayName,
        std::function<std::shared_ptr<Script>()> create,
        std::vector<ScriptTypeInfo::EntityReferenceField> entityReferenceFields)
    {
        if (id.empty() || displayName.empty() || !create || Find(id) != nullptr)
        {
            return false;
        }

        types_.push_back({std::move(id), std::move(displayName), std::move(create),
            std::move(entityReferenceFields)});
        return true;
    }

    const std::vector<ScriptTypeInfo>& ScriptTypeRegistry::GetTypes() const noexcept
    {
        return types_;
    }

    const ScriptTypeInfo* ScriptTypeRegistry::Find(std::string_view id) const noexcept
    {
        const auto found = std::find_if(types_.begin(), types_.end(),
            [id](const ScriptTypeInfo& type) { return type.id == id; });
        return found != types_.end() ? &*found : nullptr;
    }

    std::shared_ptr<Script> ScriptTypeRegistry::Create(std::string_view id) const
    {
        const ScriptTypeInfo* type = Find(id);
        return type != nullptr ? type->create() : nullptr;
    }
}
