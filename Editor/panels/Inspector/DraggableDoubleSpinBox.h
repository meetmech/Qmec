#pragma once

#include <QApplication>
#include <QCursor>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QLineEdit>
#include <QMouseEvent>

#include <algorithm>
#include <cmath>

class DraggableDoubleSpinBox final : public QDoubleSpinBox
{
public:
    explicit DraggableDoubleSpinBox(QWidget* parent = nullptr): QDoubleSpinBox(parent)
    {
        lineEdit()->installEventFilter(this);
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (watched != lineEdit())return QDoubleSpinBox::eventFilter(watched, event);

        if (event->type() == QEvent::MouseButtonPress)
        {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton)
            {
                dragCandidate_ = true;
                dragging_ = false;
                dragStartX_ = mouseEvent->globalPosition().x();
                dragStartValue_ = value();
                originalCursor_ = lineEdit()->cursor();
            }
        }
        else if (event->type() == QEvent::MouseMove && dragCandidate_)
        {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if ((mouseEvent->buttons() & Qt::LeftButton) == 0)
            {
                dragCandidate_ = false;
                return false;
            }

            const double horizontalDelta = mouseEvent->globalPosition().x() - dragStartX_;
            if (!dragging_ && std::abs(horizontalDelta) >= QApplication::startDragDistance())
            {
                dragging_ = true;
                dragModifier_ = (mouseEvent->modifiers() & Qt::ShiftModifier) != 0 ? 0.1
                    : ((mouseEvent->modifiers() & Qt::ControlModifier) != 0 ? 10.0 : 1.0);
                lineEdit()->deselect();
                lineEdit()->setCursor(Qt::SizeHorCursor);
            }

            if (dragging_)
            {
                constexpr double pixelsPerStep = 5.0;
                const double valueDelta = horizontalDelta / pixelsPerStep * singleStep() * dragModifier_;
                setValue(std::clamp(dragStartValue_ + valueDelta, minimum(), maximum()));
                return true;
            }
        }
        else if (event->type() == QEvent::MouseButtonRelease && dragCandidate_)
        {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton)
            {
                const bool consumed = dragging_;
                dragCandidate_ = false;
                dragging_ = false;
                if (consumed)
                {
                    lineEdit()->setCursor(originalCursor_);
                    return true;
                }
            }
        }

        return QDoubleSpinBox::eventFilter(watched, event);
    }

private:
    bool dragCandidate_{false};
    bool dragging_{false};
    double dragStartX_{0.0};
    double dragStartValue_{0.0};
    double dragModifier_{1.0};
    QCursor originalCursor_{};
};
