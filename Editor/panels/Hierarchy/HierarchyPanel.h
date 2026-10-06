#pragma once
#include "QMEC/ECS/Entity.h"

#include <QDockWidget>

class QTreeWidget;
namespace qmec::scene { class Scene; }

namespace qmec::editor
{
    class HierarchyPanel final : public QDockWidget
    {
        Q_OBJECT

    public :
        explicit HierarchyPanel(const qmec::scene::Scene& scene, QWidget* parent = nullptr);
        void Refresh();
        void ClearSelection();

    signals:
        void EntitySelected(qmec::ecs::Entity entity);
        void EntityReparentRequested(qmec::ecs::Entity child, qmec::ecs::Entity parent);

    private :
        QTreeWidget* tree_;
        const qmec::scene::Scene& scene_;
    };

}
