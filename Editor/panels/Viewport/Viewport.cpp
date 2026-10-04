#include "Viewport.h"

#include <QFocusEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QWheelEvent>

ViewportSurface::ViewportSurface(qmec::Camera& camera, QWidget* parent) : QWidget(parent), camera_(camera)
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
}

bool ViewportSurface::IsKeyDown(int key) const noexcept
{
    return pressedKeys_.contains(key);
}

void ViewportSurface::ReleaseMouseIfIdle() noexcept
{
    if (!leftMouseDown_ && !middleMouseDown_ && !rightMouseDown_ && QWidget::mouseGrabber() == this)
    {
        releaseMouse();
    }
}

void ViewportSurface::UpdateCamera(float deltaTime) noexcept
{
    qmec::Vec3 movement{};
    if (IsKeyDown(Qt::Key_W)) movement += camera_.Forward();
    if (IsKeyDown(Qt::Key_S)) movement += camera_.Forward() * -1.0f;
    if (IsKeyDown(Qt::Key_D)) movement += camera_.Right();
    if (IsKeyDown(Qt::Key_A)) movement += camera_.Right() * -1.0f;
    if (IsKeyDown(Qt::Key_E)) movement += camera_.Up();
    if (IsKeyDown(Qt::Key_Q)) movement += camera_.Up() * -1.0f;

    if (movement.Length() > 0.0f)
    {
        constexpr float cameraSpeed = 5.0f;
        camera_.SetPosition(camera_.Position() + movement.Normalized() * cameraSpeed * deltaTime);
    }
}

void ViewportSurface::focusOutEvent(QFocusEvent* event)
{
    pressedKeys_.clear();
    leftMouseDown_ = false;
    middleMouseDown_ = false;
    rightMouseDown_ = false;
    ReleaseMouseIfIdle();
    QWidget::focusOutEvent(event);
}

void ViewportSurface::keyPressEvent(QKeyEvent* event)
{
    pressedKeys_.insert(event->key());
    event->accept();
}

void ViewportSurface::keyReleaseEvent(QKeyEvent* event)
{
    pressedKeys_.remove(event->key());
    event->accept();
}

void ViewportSurface::mousePressEvent(QMouseEvent* event)
{
    setFocus(Qt::MouseFocusReason);
    lastMousePosition_ = event->position();

    if (event->button() == Qt::LeftButton) leftMouseDown_ = true;
    if (event->button() == Qt::MiddleButton) middleMouseDown_ = true;
    if (event->button() == Qt::RightButton) rightMouseDown_ = true;

    const bool altDown = event->modifiers().testFlag(Qt::AltModifier);
    const bool cameraDrag = middleMouseDown_ || rightMouseDown_
        || (leftMouseDown_ && altDown);
    if (cameraDrag)
    {
        grabMouse();
    }
    event->accept();
}

void ViewportSurface::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) leftMouseDown_ = false;
    if (event->button() == Qt::MiddleButton) middleMouseDown_ = false;
    if (event->button() == Qt::RightButton) rightMouseDown_ = false;

    ReleaseMouseIfIdle();
    event->accept();
}

void ViewportSurface::mouseMoveEvent(QMouseEvent* event)
{
    const QPointF mouseDelta = event->position() - lastMousePosition_;
    lastMousePosition_ = event->position();

    constexpr float sensitivity = 0.003f;
    const bool altDown = event->modifiers().testFlag(Qt::AltModifier);
    const bool orbiting = altDown && leftMouseDown_;
    const bool panning = middleMouseDown_;
    const bool zoomDragging = altDown && rightMouseDown_;
    const bool flyLooking = !altDown && rightMouseDown_;
    const bool movingFast = IsKeyDown(Qt::Key_Shift);
    (void)zoomDragging;
    (void)movingFast;

    if (orbiting)
    {
        const float orbitRadius = (camera_.Position() - orbitPivot_).Length();
        if (orbitRadius > 0.0001f)
        {
            camera_.AddYawPitch(static_cast<float>(mouseDelta.x()) * sensitivity, -static_cast<float>(mouseDelta.y()) * sensitivity);
            camera_.SetPosition(orbitPivot_ - camera_.Forward() * orbitRadius);
            camera_.LookAt(orbitPivot_);
        }
    }

    if (panning)
    {
        const float horizontal = static_cast<float>(mouseDelta.x()) * sensitivity;
        const float vertical = static_cast<float>(mouseDelta.y()) * sensitivity;
        const qmec::Vec3 pan = camera_.Right() * horizontal + camera_.Up() * vertical;
        camera_.SetPosition(camera_.Position() + pan);
        orbitPivot_ += pan;
    }

    if (flyLooking)
    {
        camera_.AddYawPitch(static_cast<float>(mouseDelta.x()) * sensitivity,-static_cast<float>(mouseDelta.y()) * sensitivity);
    }

    event->accept();
}

void ViewportSurface::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    emit Resized(event->size().width(), event->size().height());
}

void ViewportSurface::wheelEvent(QWheelEvent* event)
{
    const float wheelSteps = static_cast<float>(event->angleDelta().y()) / 120.0f;
    camera_.SetPosition(camera_.Position() + camera_.Forward() * wheelSteps * 0.5f);
    event->accept();
}

Viewport::Viewport(QWidget* parent): QDockWidget("Viewport", parent), editorCamera_{},viewport_(new ViewportSurface(editorCamera_, this))
{
    setObjectName("ViewportPanel");
    viewport_->setAttribute(Qt::WA_NativeWindow);
    setWidget(viewport_);
    editorCamera_.SetPosition({0.0f, 0.0f, 0.0f});
    editorCamera_.LookAt({0.0f, 0.0f, 4.0f});

    connect(viewport_, &ViewportSurface::Resized, this, [this](int width, int height)
    {
        if (width > 0 && height > 0)
        {
            editorCamera_.SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
        }
    });
}
