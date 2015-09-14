#include <QtGui>
#include <QApplication>
#include <swri_console/console_master.h>

#include <QStringList>
#include <QDirIterator>

void loadFonts()
{
  QStringList font_files;

  QDirIterator it(":/fonts", QDirIterator::Subdirectories);
  while (it.hasNext()) {
    it.next();
    
    if (!it.fileInfo().isFile()) {
      continue;
    }
    
    if (it.filePath().endsWith(".otf")) {
      font_files.append(it.filePath());
    }      

    if (it.filePath().endsWith(".ttf")) {
      font_files.append(it.filePath());
    }      
  }

  for (int i = 0; i < font_files.size(); i++) {
    
    int id = QFontDatabase::addApplicationFont(font_files[i]);
    if (id == -1) {
      qWarning() << "Failed to load font: " << font_files[i];
    } else {
      // qDebug() << "Loaded fonts from " << font_files[i] << ":";
      
      // QStringList families = QFontDatabase::applicationFontFamilies(id);
      // for (int j = 0; j < families.size(); j++) {
      //   qDebug() << "   " << families[j];
      // }
    }
  }
}

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  loadFonts();
  
  swri_console::ConsoleMaster master;
  master.createNewWindow();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();
  return result;
}
