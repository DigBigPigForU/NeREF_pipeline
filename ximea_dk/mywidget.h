#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QWidget>
#include "ximea.h"
#include "dk.h"
#include "dksave.h"
#include "pcsave.h"
#include "ur.h"
#include "pcproc.h"
#include "acthread.h"
#include <string>
#include <QFile>
#include <QFileDialog>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
using namespace cv;
using namespace std;

namespace Ui {
class myWidget;
}

class myWidget : public QWidget
{
    Q_OBJECT

public:
    explicit myWidget(QWidget *parent = 0);
    ~myWidget();

signals:
    void saveTifSignal();
    void saveRGBDSignal();
    void savePCDSignal();

private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void dealOK();
    void dealpcsOK();
    void dealPcDataSavingDown();
    void dealConnectStarted();
    void dealConnectStopped();
    void dealWriteDone();
    void dealReadDone();
    void dealEmptyData();
    void dealPosDone();
    void dealMoveDone();
    void dealMemoryPlus();
    void dealMemoryClear();
    void dealMemoryNotChosen();
    void policyMove();
    void update();
    void dealArrival();
    void on_pushButton_6_clicked();
    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_11_clicked();

    void on_pushButton_12_clicked();

    void on_pushButton_13_clicked();

    void on_pushButton_14_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_16_clicked();

    void on_pushButton_17_clicked();

    void on_pushButton_18_clicked();

    void on_pushButton_19_clicked();

    void on_pushButton_20_clicked();

    void on_pushButton_21_clicked();

    void on_pushButton_22_clicked();

    void on_pushButton_23_clicked();

    void on_pushButton_24_clicked();

    void on_pushButton_27_clicked();

    void on_pushButton_25_clicked();

    void on_pushButton_28_clicked();

    void on_pushButton_26_clicked();

    void on_pushButton_29_clicked();

    void on_pushButton_30_clicked();

    void on_pushButton_33_clicked();

    void on_pushButton_31_clicked();

    void on_pushButton_34_clicked();

    void on_pushButton_32_clicked();

    void on_pushButton_35_clicked();

    void on_pushButton_40_clicked();

    void on_pushButton_36_clicked();

    void on_pushButton_39_clicked();

    void on_pushButton_37_clicked();

    void on_pushButton_41_clicked();

    void on_pushButton_38_clicked();

    void on_pushButton_42_clicked();

    void on_comboBox_currentIndexChanged(const QString &arg1);

    void on_pushButton_44_clicked();

    void on_pushButton_43_clicked();

    void on_pushButton_45_clicked();

private:
    Ui::myWidget *ui;
    Ximea *ximea;
    dk *kinect;
    dkSave *kinectSave;
    pcSave *kpcSave;
    UR *ur;
    pcProc *kpcProcess;
    acThread *act;
    double score_all;
    double moveVec[6];
    double grad_thred;
    double len_gradient;
    vector<double> scoreVec;
    vector<double> gradient;
//    QString gradient_str;
    vector<vector<double>> scoreTable; //needKeeping ==0 时 获取1+12个数据  ==1时  只获取1个数据
    int pointNum;//计数器  统计已经过的有效路点数量
    int needKeeping;//0 1
    int isPausing;
};

#endif // MYWIDGET_H
