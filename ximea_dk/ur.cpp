#include "ur.h"

UR::UR(QObject *parent) : QObject(parent)
{
    socket = new QTcpSocket;
    socketData.clear();
    joint_pos.clear();
    joint_pos_str.clear();
    tcp_pos.clear();
    tcp_pos_str.clear();
    vec_int_datalen.clear();
    vec_double_time.clear();
    vec_tcp_pos.clear();
    vec_joint_pos.clear();
    count = 0;
    delta_m = 0.01; //1cm
    delta_rad = 0.02; //0.02 rad
    joint_flag = 0;//1-6 0
    tcp_flag = 0;//1-6 0
    isForward = 1;//1 or -1 
    precision_m = 0.001;
    precision_rad = 0.001;
    T_base2tcp = Eigen::Isometry3d::Identity();
    T_tcp2cam.matrix() << 0, 0.99955, 0.029996, -0.03459,
                          0, -0.03, 0.99955, 0.065924,
                          1, 0, 0, 0.12,
                          0, 0, 0, 1;
    memoryNum = 0;
    memoryChosen = 0;
}

UR::~UR()
{
    
}

void UR::creatConnection()
{
    socket->connectToHost(ip, port);
    if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
    emit connectStarted();
}

void UR::stopConnection()
{
    socket->disconnectFromHost();
    if (socket->state() == QAbstractSocket::UnconnectedState || socket->waitForDisconnected(1000))
    emit connectStopped();
}

void UR::restartConnection()
{
    stopConnection();
    creatConnection();
}

void UR::readSocket()
{
    restartConnection();
    for(int i=1;i<10;i++)   //读准为止  可以进行判断  但目前来看10次是够的
    {
        if (!((socketData).isEmpty())) { socketData.clear(); }
    //    if (socket->waitForReadyRead(-1)) { socketData = socket->read(1108); }//如果没有waitForReadyRead(-1) 会读不出数据//read(1108) 接收的数据不是实时的
        if (socket->waitForReadyRead(-1)) { socketData = socket->readAll(); }//如果没有waitForReadyRead(-1) 会读不出数据//read(1108) 接收的数据不是实时的
//        else {restartConnection();continue;}//if blocked, then restart the socket connection
    }

    if ((socketData).isEmpty())
    { 
        emit emptyData(); 
    }
	else 
    { 
        emit readDone();
    }
}

void UR::memorize()
{
    readSocket();
    memory_tcp_pos = tcp_pos;
    vec_memory_tcp_pos.push_back(memory_tcp_pos);
    memoryNum = vec_memory_tcp_pos.size();
    //UI change
    emit memoryPlus();//memory point has been added to the list
}

void UR::resetMem()
{
    memory_tcp_pos.clear();
    vec_memory_tcp_pos.clear();
    memoryNum = 0;
    memoryChosen = 0;
    //UI change
    emit memoryClear();
}

void UR::setOrigin()
{
    readSocket();
    origin_joint_pos = joint_pos;
    origin_tcp_pos = tcp_pos;
}

void UR::retureToMemory()
{
    if(memoryChosen)
    {
        if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
        {
            readSocket();// read current pos data
            target_tcp_pos.clear();
    //        target_tcp_pos = memory_tcp_pos;
            target_tcp_pos = vec_memory_tcp_pos[memoryChosen-1];
            script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(target_tcp_pos[0]).arg(target_tcp_pos[1]).arg(target_tcp_pos[2]).arg(target_tcp_pos[3]).arg(target_tcp_pos[4]).arg(target_tcp_pos[5]).toLatin1();
            //script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(memory_tcp_pos[0]).arg(memory_tcp_pos[1]).arg(memory_tcp_pos[2]).arg(memory_tcp_pos[3]).arg(memory_tcp_pos[4]).arg(memory_tcp_pos[5]).toLatin1();
            {
                socket->write(script+"\n");  //script末尾添加换行符
                while(!isArrived())
                {
                    _sleep(0.1*1000); //delay 1 s for next arrival detection
                }
                emit tcpMoveDone();
            }
        }
        else
        {
            emit disconnected();
        }
    }
    else
        emit memoryNotChosen();
}

void UR::returnToOrigin()
{
    if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
    {
        readSocket();// read current pos data
        target_tcp_pos.clear();
        target_tcp_pos = origin_tcp_pos;
        script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(target_tcp_pos[0]).arg(target_tcp_pos[1]).arg(target_tcp_pos[2]).arg(target_tcp_pos[3]).arg(target_tcp_pos[4]).arg(target_tcp_pos[5]).toLatin1();
//        script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(origin_tcp_pos[0]).arg(origin_tcp_pos[1]).arg(origin_tcp_pos[2]).arg(origin_tcp_pos[3]).arg(origin_tcp_pos[4]).arg(origin_tcp_pos[5]).toLatin1();
        {
            socket->write(script+"\n");  //script末尾添加换行符
            while(!isArrived())
            {
                _sleep(0.1*1000); //delay 1 s for next arrival detection
            }
            emit tcpMoveDone();
        }
    }
    else
    {
        emit disconnected();
    }
}

bool UR::isArrived()
{
    readSocket();// read current pos data, update the tcp pos data
    //judge the current pos & target pos & joint..
    if((fabs(target_tcp_pos[0]-tcp_pos[0]) <= precision_m &&
            fabs(target_tcp_pos[1]-tcp_pos[1]) <= precision_m &&
            fabs(target_tcp_pos[2]-tcp_pos[2]) <= precision_m &&
            fabs(target_tcp_pos[3]-tcp_pos[3]) <= precision_rad &&
            fabs(target_tcp_pos[4]-tcp_pos[4]) <= precision_rad &&
            fabs(target_tcp_pos[5]-tcp_pos[5]) <= precision_rad )
//            ||
//            (fabs(target_joint_pos[0]-joint_pos[0]) <= precision_rad &&
//             fabs(target_joint_pos[1]-joint_pos[1]) <= precision_rad &&
//             fabs(target_joint_pos[2]-joint_pos[2]) <= precision_rad &&
//             fabs(target_joint_pos[3]-joint_pos[3]) <= precision_rad &&
//             fabs(target_joint_pos[4]-joint_pos[4]) <= precision_rad &&
//             fabs(target_joint_pos[5]-joint_pos[5]) <= precision_rad )
            )
    {
        emit arrival();
        emit arrival_for_adaptive();
        //emit other signals
        return 1;
    }
    else
    {
        return 0;
    }

}

void UR::showTcpPos()
{
    QByteArray datalen = socketData.mid(0, 4).toHex();//数据包大小
    int_datalen = datasum(datalen);
    cout<<int_datalen<<endl;

    int begin = 4 + 0 * 8;
    QByteArray time = socketData.mid(begin, 8).toHex();//通电时间
    double_time = HexToDouble(time);
    cout<<double_time<<endl;

    joint_pos.clear();//每次获取数据前清零
    joint_pos_str.clear();

    begin = 4 + 31 * 8;
    for (int i = 0; i < 6; i++)
    {
        QByteArray ba = socketData.mid(begin + i * 8, 8).toHex();
        //qDebug() << ba;//prints six Actual joint positions TYPE:QByteArray
        joint_pos.push_back(HexToDouble(ba));
    }
    for (vector<double>::const_iterator iter = joint_pos.cbegin(); iter != joint_pos.cend(); iter++)//遍历tcp元素
    {
        //cout << "q_actual_pos  " << (*iter) << endl;
        joint_pos_str.append(QString::number(*iter, 10, 8));
        joint_pos_str.append("\t");
    }
    cout<<joint_pos_str.toStdString()<<endl;

    tcp_pos.clear();//每次获取数据前清零
    tcp_pos_str.clear();

    begin = 4 + 55 * 8;
    for (int i = 0; i < 6; i++)
    {
        QByteArray ba = socketData.mid(begin + i * 8, 8).toHex();
        //qDebug() << ba;//prints six Actual joint positions TYPE:QByteArray
        tcp_pos.push_back(HexToDouble(ba));
    }
    for (vector<double>::const_iterator iter = tcp_pos.cbegin(); iter != tcp_pos.cend(); iter++)//遍历tcp元素
    {
        //cout << "q_actual_pos  " << (*iter) << endl;
        tcp_pos_str.append(QString::number(*iter, 10, 8));
        tcp_pos_str.append("\t");
    }
    cout<<tcp_pos_str.toStdString()<<endl;

    //save for .csv
    vec_int_datalen.push_back(int_datalen);
    vec_double_time.push_back(double_time);
    vec_tcp_pos.push_back(tcp_pos);
    vec_joint_pos.push_back(joint_pos);

    //matrix transformation
    T_base2tcp = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rv_base2tcp(vectorLength(tcp_pos[3],tcp_pos[4],tcp_pos[5]), Eigen::Vector3d(tcp_pos[3]/vectorLength(tcp_pos[3],tcp_pos[4],tcp_pos[5]),tcp_pos[4]/vectorLength(tcp_pos[3],tcp_pos[4],tcp_pos[5]),tcp_pos[5]/vectorLength(tcp_pos[3],tcp_pos[4],tcp_pos[5])));
    T_base2tcp.rotate(rv_base2tcp);
    T_base2tcp.pretranslate(Eigen::Vector3d(tcp_pos[0],tcp_pos[1],tcp_pos[2]));
    emit posDone();
    count++;
}

void UR::writeSocket()
{
    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        socket->write(script+"\n");  //script末尾添加换行符
        emit writeDone();
    }
}

void UR::tcpMove()  //  TCP的RXRYRZ是成倍数的   角度不能这样设置  不像JOINT
{
    if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
    {
        readSocket();// read current pos data
        switch(tcp_flag)
        {
            case 1:
            script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(tcp_pos[0]+delta_m*isForward).arg(tcp_pos[1]).arg(tcp_pos[2]).arg(tcp_pos[3]).arg(tcp_pos[4]).arg(tcp_pos[5]).toLatin1();
            break;

            case 2:
            script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(tcp_pos[0]).arg(tcp_pos[1]+delta_m*isForward).arg(tcp_pos[2]).arg(tcp_pos[3]).arg(tcp_pos[4]).arg(tcp_pos[5]).toLatin1();
            break;

            case 3:
            script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(tcp_pos[0]).arg(tcp_pos[1]).arg(tcp_pos[2]+delta_m*isForward).arg(tcp_pos[3]).arg(tcp_pos[4]).arg(tcp_pos[5]).toLatin1();
            break;

            case 4:
            script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(tcp_pos[0]).arg(tcp_pos[1]).arg(tcp_pos[2]).arg(tcp_pos[3]+delta_rad*isForward).arg(tcp_pos[4]).arg(tcp_pos[5]).toLatin1();
            break;

            case 5:
            script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(tcp_pos[0]).arg(tcp_pos[1]).arg(tcp_pos[2]).arg(tcp_pos[3]).arg(tcp_pos[4]+delta_rad*isForward).arg(tcp_pos[5]).toLatin1();
            break;

            case 6:
            script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(tcp_pos[0]).arg(tcp_pos[1]).arg(tcp_pos[2]).arg(tcp_pos[3]).arg(tcp_pos[4]).arg(tcp_pos[5]+delta_rad*isForward).toLatin1();
            break;

        default:
            break;
        }
        //script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]).arg(joint_pos[1]).arg(joint_pos[2]).arg(joint_pos[3]).arg(joint_pos[4]).arg(joint_pos[5]).toLatin1();
        if (socket->state() == QAbstractSocket::ConnectedState)
            {
                socket->write(script+"\n");  //script末尾添加换行符
                emit tcpMoveDone();
            }
    }
    else
    {
        emit disconnected();
    }

}

void UR::tcpMoveWithVec(int x,int y,int z,int rx,int ry,int rz)    //-1,0,1 can be chosen
{
    if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
    {
        readSocket();// read current pos data
        script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(tcp_pos[0]+delta_m*x).arg(tcp_pos[1]+delta_m*y).arg(tcp_pos[2]+delta_m*z).arg(tcp_pos[3]+delta_rad*rx).arg(tcp_pos[4]+delta_rad*ry).arg(tcp_pos[5]+delta_rad*rz).toLatin1();
        {
            socket->write(script+"\n");  //script末尾添加换行符
            emit tcpMoveDone();
        }
    }
    else
    {
        emit disconnected();
    }

}

void UR::tcpMoveDirectly(double x,double y,double z,double rx,double ry,double rz) //directly move to target tcp pos
{
    if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
    {
        readSocket();// read current pos data
        target_tcp_pos.clear();
        target_tcp_pos.push_back(x); target_tcp_pos.push_back(y); target_tcp_pos.push_back(z); target_tcp_pos.push_back(rx); target_tcp_pos.push_back(ry); target_tcp_pos.push_back(rz);
        script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(target_tcp_pos[0]).arg(target_tcp_pos[1]).arg(target_tcp_pos[2]).arg(target_tcp_pos[3]).arg(target_tcp_pos[4]).arg(target_tcp_pos[5]).toLatin1();
        //        script = QString("movel(p[%1,%2,%3,%4,%5,%6],a=1.2,v=0.25,t=0,r=0)").arg(x).arg(y).arg(z).arg(rx).arg(ry).arg(rz).toLatin1();
        {
            socket->write(script+"\n");  //script末尾添加换行符
            while(!isArrived())
            {
                _sleep(0.1*1000); //delay 0.1 s for next arrival detection
            }
            emit tcpMoveDone();
        }
    }
    else
    {
        emit disconnected();
    }

}

void UR::jointMove()    //目前存在socket read data时不能立即更新的问题  存在粘包  导致当前的joint pos无法及时显示   解决方法  读10遍数据  读准为止
{
    if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
    {
        readSocket();// read current pos data
        switch(joint_flag)
        {
            case 1:
            script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]+delta_rad*isForward).arg(joint_pos[1]).arg(joint_pos[2]).arg(joint_pos[3]).arg(joint_pos[4]).arg(joint_pos[5]).toLatin1();
            break;

            case 2:
            script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]).arg(joint_pos[1]+delta_rad*isForward).arg(joint_pos[2]).arg(joint_pos[3]).arg(joint_pos[4]).arg(joint_pos[5]).toLatin1();
            break;

            case 3:
            script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]).arg(joint_pos[1]).arg(joint_pos[2]+delta_rad*isForward).arg(joint_pos[3]).arg(joint_pos[4]).arg(joint_pos[5]).toLatin1();
            break;

            case 4:
            script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]).arg(joint_pos[1]).arg(joint_pos[2]).arg(joint_pos[3]+delta_rad*isForward).arg(joint_pos[4]).arg(joint_pos[5]).toLatin1();
            break;

            case 5:
            script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]).arg(joint_pos[1]).arg(joint_pos[2]).arg(joint_pos[3]).arg(joint_pos[4]+delta_rad*isForward).arg(joint_pos[5]).toLatin1();
            break;

            case 6:
            script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]).arg(joint_pos[1]).arg(joint_pos[2]).arg(joint_pos[3]).arg(joint_pos[4]).arg(joint_pos[5]+delta_rad*isForward).toLatin1();
            break;

        default:
            break;
        }
        //script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]).arg(joint_pos[1]).arg(joint_pos[2]).arg(joint_pos[3]).arg(joint_pos[4]).arg(joint_pos[5]).toLatin1();
        if (socket->state() == QAbstractSocket::ConnectedState)
        {
            socket->write(script+"\n");  //script末尾添加换行符
            emit jointMoveDone();
        }
    }
    else
    {
        emit disconnected();
    }

}

void UR::jointMoveWithVec(int q1,int q2,int q3,int q4,int q5,int q6)    //-1,0,1 can be chosen
{
    if (socket->state() == QAbstractSocket::ConnectedState || socket->waitForConnected(1000))
    {
        readSocket();// read current pos data
        script = QString("movej([%1,%2,%3,%4,%5,%6],a=1,v=1,t=0,r=0)").arg(joint_pos[0]+delta_rad*q1).arg(joint_pos[1]+delta_rad*q2).arg(joint_pos[2]+delta_rad*q3).arg(joint_pos[3]+delta_rad*q4).arg(joint_pos[4]+delta_rad*q5).arg(joint_pos[5]+delta_rad*q6).toLatin1();
        if (socket->state() == QAbstractSocket::ConnectedState)
        {
            socket->write(script+"\n");  //script末尾添加换行符
            emit jointMoveDone();
        }
    }
    else
    {
        emit disconnected();
    }

}

int UR::datasum(QByteArray socketdata)
{
    double sum = 0;
    for (int i = 0; i < socketdata.size(); i++)
    {
        //qDebug() << socketdata.at(i);
        sum = sum * 16 + toint(socketdata.at(i));
    }
    //qDebug() << sum;
    return sum;
}

QByteArray UR::HexToBinary(QByteArray ba)
{
    QByteArray BinaryArray;
    for (int i = 0; i < ba.size(); i++)
    {
        //qDebug()<< socketdata.at(i);
        switch (ba.at(i))
        {
        case '0':BinaryArray.append("0000"); break;
        case '1':BinaryArray.append("0001"); break;
        case '2':BinaryArray.append("0010"); break;
        case '3':BinaryArray.append("0011"); break;
        case '4':BinaryArray.append("0100"); break;
        case '5':BinaryArray.append("0101"); break;
        case '6':BinaryArray.append("0110"); break;
        case '7':BinaryArray.append("0111"); break;
        case '8':BinaryArray.append("1000"); break;
        case '9':BinaryArray.append("1001"); break;
        case 'a':case 'A':BinaryArray.append("1010"); break;
        case 'b':case 'B':BinaryArray.append("1011"); break;
        case 'c':case 'C':BinaryArray.append("1100"); break;
        case 'd':case 'D':BinaryArray.append("1101"); break;
        case 'e':case 'E':BinaryArray.append("1110"); break;
        case 'f':case 'F':BinaryArray.append("1111"); break;
        }
    }
    //qDebug() << ba;
    //qDebug()<<BinaryArray;
    return BinaryArray;
}

double UR::HexToDouble(QByteArray socketdata)
{
    double sum_exponent = 0;//2的指数
    double sum_fraction = 0;
    QByteArray BinaryArray = HexToBinary(socketdata);
    //qDebug() << BinaryArray;
    QByteArray sign = BinaryArray.mid(0, 1);
    QByteArray exponent = BinaryArray.mid(1, 11);
    QByteArray fraction = BinaryArray.mid(12, 52);
    //qDebug() << sign << exponent << fraction;

    if (exponent == "00000000000") {
        sum_exponent = -1022;
    }
    else {
        bool ok;
        sum_exponent = exponent.toInt(&ok, 2) - 1023;
        sum_fraction++;
    }
    //deal with fraction
    for (int i = 1; i <= fraction.size(); i++)
    {
        //qDebug() <<fraction.at(i-1);
        if (fraction.mid(i - 1, 1) == "1")
        {
            //qDebug() <<fraction.at(i-1);
            sum_fraction = sum_fraction + pow(2, -i);
            //qDebug() << i;
        }
    }
    //qDebug() << sum_fraction << "sum_fraction";
    double return_value = pow(-1, sign.at(0) - '0')*pow(2, sum_exponent)*sum_fraction;
    return return_value;
}

int UR::toint(char c)
{
    if (c >= '0'&&c <= '9') { c = c - '0'; }
    if (c >= 'a'&&c <= 'z') { c = c - 'a'; }
    if (c >= 'A'&&c <= 'Z') { c = c - 'A'; }
    return c;
}

double UR::vectorLength(double rx, double ry, double rz)
{
    return (double)sqrt(rx*rx+ry*ry+rz*rz);
}
