#ifndef UR_H
#define UR_H
#include <QObject>
#include <QtNetwork/QTcpSocket>
#include <QtCore>
#include <iostream>
#include <windows.h>
#include <stdlib.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

class UR : public QObject
{
    Q_OBJECT
public:
    explicit UR(QObject *parent = nullptr);
    friend class pcProc;
    ~UR();

    int datasum(QByteArray socketdata);
    QByteArray HexToBinary(QByteArray ba);
    double HexToDouble(QByteArray socketdata);
    int toint(char c);
    double vectorLength(double rx,double ry,double rz);

    void creatConnection();
    void stopConnection();
    void restartConnection();
    void readSocket();
    void showTcpPos();
    void writeSocket();
    void jointMove();   //single step
    void tcpMove(); //single step
    void jointMoveWithVec(int q1,int q2,int q3,int q4,int q5,int q6); //[1,-1,0,0-1,1]类似的向量
    void tcpMoveWithVec(int x,int y,int z,int rx,int ry,int rz);//[1,-1,0,0-1,1]类似的向量
    void tcpMoveDirectly(double x,double y,double z,double rx,double ry,double rz); //directly move to target tcp pos  //applied arrval judging
    void memorize();
    void resetMem();
    void retureToMemory();//applied arrval judging
    void setOrigin();
    void returnToOrigin();//applied arrval judging
    bool isArrived();// using this value to judge arrival  // not necessary to apply for single step

public:
    QTcpSocket *socket;
    QString ip;
    uint port;
    QByteArray script;
    QByteArray socketData;
    vector<double> tcp_pos;
    vector<double> joint_pos;
    vector<double> target_tcp_pos;
    vector<double> target_joint_pos;
    vector<double> memory_tcp_pos;
    vector<double> origin_tcp_pos;
    vector<double> origin_joint_pos;
    QString tcp_pos_str;
    QString joint_pos_str;
    int int_datalen;
    int joint_flag;//1-6 0
    int tcp_flag;//1-6 0
    int isForward;//1 or -1
    double double_time;
    double delta_rad;
    double delta_m;
    double precision_m;
    double precision_rad;
    Eigen::Isometry3d T_base2tcp;
    Eigen::Isometry3d T_tcp2cam;
    //保存专用变量 LIst或Vector
    vector<int> vec_int_datalen;
    vector<double> vec_double_time;
    vector<vector<double>> vec_tcp_pos;
    vector<vector<double>> vec_joint_pos;
    vector<vector<double>> vec_memory_tcp_pos;
    int count;//getData 次数
    int memoryNum;//Num of the memory point
    int memoryChosen;
    
signals:
    void connectStarted();
    void connectStopped();
    void disconnected();
    void readDone();
    void posDone();
    void writeDone();
    void jointMoveDone();
    void tcpMoveDone();
    void arrival(); //signal for arriving
    void arrival_for_adaptive(); //signal for adaptive
    void emptyData();
    void memoryPlus();
    void memoryClear();
    void memoryNotChosen();
public slots:
    
};

#endif // UR_H
