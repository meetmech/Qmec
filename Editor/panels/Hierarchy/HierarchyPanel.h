#pragma once
#include "QMEC/ECS/Entity.h"

#include <QDockWidget>

class QTreeWidget;
namespace qmec { class Scene; }

class HierarchyPanel final : public QDockWidget
{
    Q_OBJECT

public :
    explicit HierarchyPanel(const qmec::Scene& scene, QWidget* parent = nullptr);
    void Refresh();
    void ClearSelection();

signals:
    void EntitySelected(qmec::Entity entity);
    void EntityReparentRequested(qmec::Entity child, qmec::Entity parent);

private :
    QTreeWidget* tree_;
    const qmec::Scene& scene_;
};
