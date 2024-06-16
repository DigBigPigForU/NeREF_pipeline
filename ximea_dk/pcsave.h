#ifndef PCSAVE_H
#define PCSAVE_H
#include "dk.h"


class pcSave : public QThread
{
    Q_OBJECT
public:
    explicit pcSave(QObject *parent = 0);
    ~pcSave();
protected:
    void run();
signals:
    void pcSaveIsOK();
public:
    dk *kinect;

};

#endif // PCSAVE_H
