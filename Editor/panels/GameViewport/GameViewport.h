#pragma once

#include <QDockWidget>
#include <QWidget>

class QResizeEvent;

namespace qmec::editor
{
    class GameViewportSurface final : public QWidget
    {
        Q_OBJECT

    public:
        explicit GameViewportSurface(QWidget* parent = nullptr);

    signals:
        void Resized(int width, int height);

    protected:
        void resizeEvent(QResizeEvent* event) override;
    };

    class GameViewport final : public QDockWidget
    {
    public:
        explicit GameViewport(QWidget* parent = nullptr);
        [[nodiscard]] GameViewportSurface* Surface() const noexcept { return surface_; }

    private:
        GameViewportSurface* surface_{};
    };

}
