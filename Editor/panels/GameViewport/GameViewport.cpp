#include "GameViewport.h"

#include <QResizeEvent>

namespace qmec::editor
{
    GameViewportSurface::GameViewportSurface(QWidget* parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_NativeWindow);
        setAttribute(Qt::WA_NoSystemBackground);
        setAutoFillBackground(false);
        setFocusPolicy(Qt::NoFocus);
    }

    void GameViewportSurface::resizeEvent(QResizeEvent* event)
    {
        QWidget::resizeEvent(event);
        emit Resized(event->size().width(), event->size().height());
    }

    GameViewport::GameViewport(QWidget* parent)
        : QDockWidget("Game", parent),
          surface_(new GameViewportSurface(this))
    {
        setObjectName("GameViewportPanel");
        setWidget(surface_);
    }

}
