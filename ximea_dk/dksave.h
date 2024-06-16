#ifndef DKSAVE_H
#define DKSAVE_H
#include "dk.h"


class dkSave : public QThread
{
    Q_OBJECT
public:
    explicit dkSave(QObject *parent = 0);
    ~dkSave();
protected:
    void run();
signals:
    void isOK();
public:
    dk *kinect;

};

#endif // DKSAVE_H
