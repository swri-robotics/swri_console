#include <QtGui>
#include <QApplication>
#include <swri_console/console_master.h>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  swri_console::ConsoleMaster master(argc,argv);
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();
  return result;
}
