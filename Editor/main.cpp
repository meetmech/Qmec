#include <QApplication>
#include <QCoreApplication>
#include "Editor.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName("QMEC");
    QCoreApplication::setApplicationName("QMECEditor");
    qmec::editor::Editor editor;
    editor.show();

    return app.exec();
}
