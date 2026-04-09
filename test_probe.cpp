#include <QApplication>
#include <QMediaPlayer>
#include <QVideoProbe>
#include <QDebug>
int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QMediaPlayer player;
    QVideoProbe probe;
    bool res = probe.setSource(&player);
    qDebug() << "Probe supported:" << res;
    return 0;
}
