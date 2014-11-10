#include "Arap.h"
#include <QTextCodec>

int main( int argc, char **argv )
{
	QApplication *app = new QApplication(argc, argv);
	QTextCodec::setCodecForTr(QTextCodec::codecForName("GB2312"));

	Arap *window = new Arap();
	window->show();
	return app->exec();
}