#pragma once
#include "QMEC/Scene/Camera.h"

#include <QDockWidget>
#include <QPointF>
#include <QSet>
#include <QWidget>

class QFocusEvent;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;

namespace qmec::editor
{
    class ViewportSurface final : public QWidget
    {
        Q_OBJECT

    public:
        explicit ViewportSurface(qmec::scene::Camera& camera, QWidget* parent = nullptr);
        void UpdateCamera(float deltaTime) noexcept;

    signals:
        void Resized(int width, int height);

    protected:
        void focusOutEvent(QFocusEvent* event) override;
        void keyPressEvent(QKeyEvent* event) override;
        void keyReleaseEvent(QKeyEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void resizeEvent(QResizeEvent* event) override;
        void wheelEvent(QWheelEvent* event) override;

    private:
        [[nodiscard]] bool IsKeyDown(int key) const noexcept;
        void ReleaseMouseIfIdle() noexcept;

        qmec::scene::Camera& camera_;
        qmec::math::Vec3 orbitPivot_{0.0f, 0.0f, 4.0f};
        QPointF lastMousePosition_{};
        QSet<int> pressedKeys_{};
        bool leftMouseDown_{false};
        bool middleMouseDown_{false};
        bool rightMouseDown_{false};
    };

    class Viewport final : public QDockWidget
    {
    public:
        explicit Viewport(QWidget* parent = nullptr);
        [[nodiscard]] ViewportSurface* Surface() const noexcept { return viewport_; }
        [[nodiscard]] qmec::scene::Camera& EditorCamera() noexcept { return editorCamera_; }
        [[nodiscard]] const qmec::scene::Camera& EditorCamera() const noexcept { return editorCamera_; }
        void UpdateCamera(float deltaTime) noexcept { viewport_->UpdateCamera(deltaTime); }

    private:
        qmec::scene::Camera editorCamera_{};
        ViewportSurface* viewport_;
    };

}
