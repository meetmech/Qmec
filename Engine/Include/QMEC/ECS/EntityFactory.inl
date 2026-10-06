#pragma once


namespace qmec::scene::factory
{
    template<typename Component>
    bool EntityFactory::addComponent(const Entity entity, const Component& component)
    {
       return registry_.AddComponent(entity, component);
    }
}