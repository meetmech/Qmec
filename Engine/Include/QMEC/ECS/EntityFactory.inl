#pragma once


namespace qmec
{
    template<typename Component>
    bool EntityFactory::addComponent(const Entity entity, const Component& component)
    {
       return registry_.AddComponent(entity, component);
    }
}