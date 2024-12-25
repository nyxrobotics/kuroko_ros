#include "../include/kuroko_walking_gui/main_window.hpp"
#include <QApplication>
#include <QtGui>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  walking_gui::MainWindow w(argc, argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
