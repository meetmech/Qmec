#include "HierarchyPanel.h"
#include "QMEC/Scene/Scene.h"
#include <QAbstractItemView>
#include <QDataStream>
#include <QMimeData>
#include <QDropEvent>
#include <QMouseEvent>
#include <QSignalBlocker>
#include <QTreeWidget>

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace
{
    constexpr char EntityMimeType[] = "application/x-qmec-entity";

    qmec::Entity GetItemEntity(const QTreeWidgetItem* item)
    {
        if (item == nullptr)
            return {};

        return
        {
            static_cast<qmec::EntityIndex>(item->data(0, Qt::UserRole).toUInt()),
            static_cast<qmec::EntityGeneration>(item->data(0, Qt::UserRole + 1).toUInt())
        };
    }

    class HierarchyTreeWidget final : public QTreeWidget
    {
    public:
        using ReparentCallback = std::function<void(qmec::Entity, qmec::Entity)>;

        explicit HierarchyTreeWidget(QWidget* parent): QTreeWidget(parent)
        {
            setDragEnabled(true);
            setAcceptDrops(true);
            viewport()->setAcceptDrops(true);
            setDropIndicatorShown(true);
            setDragDropMode(QAbstractItemView::InternalMove);
            setDefaultDropAction(Qt::MoveAction);
        }

        ReparentCallback onReparentRequested{};

        bool IsMousePressInProgress() const noexcept
        {
            return mousePressInProgress_;
        }

    protected:
        void mousePressEvent(QMouseEvent* event) override
        {
            mousePressInProgress_ = true;
            QTreeWidget::mousePressEvent(event);
        }

        void mouseReleaseEvent(QMouseEvent* event) override
        {
            QTreeWidget::mouseReleaseEvent(event);
            mousePressInProgress_ = false;
        }

        QMimeData* mimeData(const QList<QTreeWidgetItem*>& items) const override
        {
            QMimeData* mime = QTreeWidget::mimeData(items);
            if (mime == nullptr || items.empty())
                return mime;

            const qmec::Entity entity = GetItemEntity(items.front());
            QByteArray encodedEntity{};
            QDataStream stream(&encodedEntity, QIODevice::WriteOnly);
            stream << static_cast<quint32>(entity.index)
                   << static_cast<quint32>(entity.generation);
            mime->setData(EntityMimeType, encodedEntity);
            mime->setText(items.front()->text(0));
            return mime;
        }

        void startDrag(Qt::DropActions supportedActions) override
        {
            draggedEntity_ = GetItemEntity(currentItem());
            QTreeWidget::startDrag(supportedActions);
            draggedEntity_ = {};
        }

        void dropEvent(QDropEvent* event) override
        {
            if (!draggedEntity_.IsValid())
            {
                event->ignore();
                return;
            }
            const qmec::Entity newParent = GetItemEntity(itemAt(event->position().toPoint()));
            if (onReparentRequested)
                onReparentRequested(draggedEntity_, newParent);

            event->acceptProposedAction();
        }

    private:
        qmec::Entity draggedEntity_{};
        bool mousePressInProgress_ = false;
    };
}

HierarchyPanel::HierarchyPanel(const qmec::Scene& scene, QWidget* parent): QDockWidget("Hierarchy", parent), tree_(new HierarchyTreeWidget(this)), scene_(scene)
{
    setObjectName("HierarchyPanel");
    tree_->setHeaderLabel("Scene");
    setWidget(tree_);

    static_cast<HierarchyTreeWidget*>(tree_)->onReparentRequested =[this](qmec::Entity child, qmec::Entity parentEntity)
        {
            emit EntityReparentRequested(child, parentEntity);
        };

    auto* hierarchyTree = static_cast<HierarchyTreeWidget*>(tree_);
    connect(tree_, &QTreeWidget::currentItemChanged, this,
        [this, hierarchyTree](QTreeWidgetItem* current, QTreeWidgetItem*)
        {
            // Mouse selection changes on press. Wait for itemClicked (release)
            // so starting a drag doesn't redirect the inspector selection.
            if (hierarchyTree->IsMousePressInProgress())
                return;

            if (current == nullptr)
            {
                emit EntitySelected(qmec::Entity{});
                return;
            }

            emit EntitySelected(qmec::Entity{current->data(0, Qt::UserRole).toUInt(), current->data(0, Qt::UserRole + 1).toUInt()});
        });

    connect(tree_, &QTreeWidget::itemClicked, this,
        [this](QTreeWidgetItem* item, int)
        {
            emit EntitySelected(GetItemEntity(item));
        });
}

void HierarchyPanel::Refresh()
{
    const qmec::Entity selectedBeforeRefresh = GetItemEntity(tree_->currentItem());
    const std::vector<qmec::SceneEntityEntry> entries = scene_.GetHierarchyEntities();
    std::vector<QTreeWidgetItem*> items{};
    items.reserve(entries.size());
    std::unordered_map<qmec::EntityIndex, QTreeWidgetItem*> itemsByIndex{};
    itemsByIndex.reserve(entries.size());

    const QSignalBlocker signalBlocker(tree_);
    tree_->clear();

    for (const qmec::SceneEntityEntry& entry : entries)
    {
        auto* item = new QTreeWidgetItem();
        item->setText(0, QString::fromStdString(entry.name));
        item->setData(0, Qt::UserRole, static_cast<qulonglong>(entry.entity.index));
        item->setData(0, Qt::UserRole + 1, static_cast<qulonglong>(entry.entity.generation));
        item->setFlags(item->flags() | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
        items.push_back(item);
        itemsByIndex.emplace(entry.entity.index, item);
    }

    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        const qmec::SceneEntityEntry& entry = entries[i];
        QTreeWidgetItem* const item = items[i];
        const auto parentIt = itemsByIndex.find(entry.parent.index);
        if (entry.parent.IsValid() && parentIt != itemsByIndex.end())
        {
            const qmec::Entity actualParent = GetItemEntity(parentIt->second);
            if (actualParent == entry.parent && actualParent != entry.entity)
            {
                parentIt->second->addChild(item);
                continue;
            }
        }
        tree_->addTopLevelItem(item);
    }

    QTreeWidgetItem* itemToSelect = nullptr;
    for (QTreeWidgetItem* item : items)
    {
        if (GetItemEntity(item) == selectedBeforeRefresh)
        {
            itemToSelect = item;
            break;
        }
    }
    if (itemToSelect == nullptr && !items.empty())
        itemToSelect = items.front();
    tree_->setCurrentItem(itemToSelect);
    emit EntitySelected(GetItemEntity(itemToSelect));
}

void HierarchyPanel::ClearSelection()
{
    const QSignalBlocker signalBlocker(tree_);
    tree_->setCurrentItem(nullptr);
    tree_->clearSelection();
    emit EntitySelected({});
}
