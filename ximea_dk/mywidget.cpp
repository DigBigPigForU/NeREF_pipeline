#include "mywidget.h"
#include "ui_mywidget.h"

myWidget::myWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::myWidget)
{
    ui->setupUi(this);
    ui->pushButton_2->setEnabled(false);
    ui->pushButton_4->setEnabled(false);
    ui->pushButton_7->setEnabled(false);
    ui->pushButton_42->setEnabled(false);
    ximea = new Ximea(500000,this);
    kinect = new dk(this);
    kinectSave = new dkSave(this);
    kpcSave = new pcSave(this);
    ur = new UR(this);
    kpcProcess = new pcProc(this);
    act = new acThread(this);
    //Ptr value link
    kinectSave->kinect = kinect;
    kpcSave->kinect = kinect;
    kpcProcess->kinect = kinect;
    kpcProcess->ur = ur;
    act->kpcProcess = kpcProcess;
    //
    QTimer *timer = new QTimer(this);   //初始化一个定时器  用于刷新score 实时
    needKeeping = 0;//default 0 : need to calculate the table again
    isPausing = -1;//-1 1
    pointNum = 0;
    grad_thred = 0.001;
    gradient.push_back(0);gradient.push_back(0);gradient.push_back(0);gradient.push_back(0);gradient.push_back(0);gradient.push_back(0);
    ui->textBrowser->insertPlainText("Successfully opened RGBD camera!\n" );
    ui->textBrowser->insertPlainText("Successfully opened multispectral camera!\n");

//    connect(timer, SIGNAL(timeout()),
//            [=]()
//            {
//                ui->lineEdit_4->setText(QString::number(pointNum));
//                ui->lineEdit_5->setText(QString::number(ximea->xi_score));
//                ui->lineEdit_6->setText(QString::number(kinect->dk_score));
//                score_all = kinect->dk_score + ximea->xi_score;
//                ui->lineEdit_7->setText(QString::number(score_all));
//                ui->lineEdit_8->setText(QString("%1, %2, %3, %4, %5, %6").arg(gradient[0]).arg(gradient[1]).arg(gradient[2]).arg(gradient[3]).arg(gradient[4]).arg(gradient[5]));
//                ui->lineEdit_9->setText(QString::number(len_gradient));
//            }
//            );   //此写法没用  需要用 SIGNAL SLOT的写法

    connect(timer,SIGNAL(timeout()),this,SLOT(update()));  //必须这样写  其他写法没用
    timer->start(10);    //设置定时器刷新频率，即变量刷新频率。 ms

    connect(ximea,&Ximea::ximeaWindowsignal,
            [=]()
            {
                QImage qImage=QImage((unsigned char *)ximea->cv_mat_image.data,ximea->cv_mat_image.cols,ximea->cv_mat_image.rows,QImage::Format_Indexed8);
                ui->label->setPixmap(QPixmap::fromImage(qImage).scaled(512,277));
                ui->label->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);
            }
            );
    connect(kinect,&dk::dkWindowsignal,
            [=]()
            {
                QImage qImage2=QImage((unsigned char *)kinect->colorFrame.data,kinect->colorFrame.cols,kinect->colorFrame.rows,QImage::Format_ARGB32);
                ui->label_2->setPixmap(QPixmap::fromImage(qImage2).scaled(320,180));
                ui->label_2->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);

                QImage qImage3=QImage((unsigned char *)kinect->t_colorFrame.data,kinect->t_colorFrame.cols,kinect->t_colorFrame.rows,QImage::Format_ARGB32);
                ui->label_3->setPixmap(QPixmap::fromImage(qImage3).scaled(180,180));
                ui->label_3->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);

                QImage qImage4=QImage((unsigned char *)kinect->my_ct_zFloatFrame.data,kinect->my_ct_zFloatFrame.cols,kinect->my_ct_zFloatFrame.rows,QImage::Format_ARGB32);
                ui->label_4->setPixmap(QPixmap::fromImage(qImage4).scaled(180,180));
                ui->label_4->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);

                QImage qImage5=QImage((unsigned char *)kinect->my_dt_zFloatFrame.data,kinect->my_dt_zFloatFrame.cols,kinect->my_dt_zFloatFrame.rows,QImage::Format_ARGB32);
                ui->label_5->setPixmap(QPixmap::fromImage(qImage5).scaled(320,180));
                ui->label_5->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);
            }
            );
    connect(this,&myWidget::saveTifSignal,
            [=]()
            {
                std::cout << "Button click!" << std::endl;
                ui->textBrowser->insertPlainText("Button click!\n");
                ximea->count++;
                std::string fileStr;
                fileStr = to_string(ximea->count)  + ".tif";
                imwrite(fileStr, ximea->cv_mat_image);
                std::cout << "************************   " << "No." << ximea->count << " ximea data   ************************" << std::endl;
                std::cout << "Successfully saved all data to path!" << std::endl;
                ui->textBrowser->insertPlainText("*****   No.");
                ui->textBrowser->insertPlainText(QString::fromStdString(std::to_string(ximea->count)));
                ui->textBrowser->insertPlainText(" ximea data   *****\n");
                ui->textBrowser->insertPlainText("Successfully saved all data to path!\n");
                ui->pushButton_2->setEnabled(true);
            }
            );
    connect(this,&myWidget::saveRGBDSignal,
            [=]()
            {
                kinectSave->kinect = kinect;
                ui->textBrowser->insertPlainText("Button click!\n");
                ui->textBrowser->insertPlainText("This operation takes some time, please wait for a while...\n");
                kinectSave->start();
            }
            );
    connect(this,&myWidget::savePCDSignal,
            [=]()
            {
                kpcSave->kinect = kinect;
                ui->textBrowser->insertPlainText("Button click!\n");
                ui->textBrowser->insertPlainText("This operation takes some time, please wait for a while...\n");
                kpcSave->start();
            }
            );
    connect(this,&myWidget::destroyed,
            [=]()
            {
                ximea->terminate();
                kinect->terminate();
                kinectSave->terminate();
                kpcSave->terminate();
            }
            );
    connect(kinectSave,&dkSave::isOK,this,&myWidget::dealOK);
    connect(kpcSave,&pcSave::pcSaveIsOK,this,&myWidget::dealpcsOK);
    connect(kpcProcess,&pcProc::pcDataSavingDown,this,&myWidget::dealPcDataSavingDown);
    connect(ur,&UR::connectStarted,this,&myWidget::dealConnectStarted);
    connect(ur,&UR::connectStopped,this,&myWidget::dealConnectStopped);
    connect(ur,&UR::disconnected,this,&myWidget::dealConnectStopped);
    connect(ur,&UR::writeDone,this,&myWidget::dealWriteDone);
    connect(ur,&UR::jointMoveDone,this,&myWidget::dealMoveDone);
    connect(ur,&UR::tcpMoveDone,this,&myWidget::dealMoveDone);
    connect(ur,&UR::readDone,this,&myWidget::dealReadDone);
    connect(ur,&UR::emptyData,this,&myWidget::dealEmptyData);
    connect(ur,&UR::posDone,this,&myWidget::dealPosDone);
    connect(ur,&UR::memoryPlus,this,&myWidget::dealMemoryPlus);
    connect(ur,&UR::memoryClear,this,&myWidget::dealMemoryClear);
    connect(ur,&UR::memoryNotChosen,this,&myWidget::dealMemoryNotChosen);
    connect(ur,&UR::arrival,this,&myWidget::dealArrival);
    connect(ur,&UR::arrival_for_adaptive,kpcProcess,&pcProc::dealURArrival);
//    connect(ximea,SIGNAL(Ximea::ximeaScore(double)),SLOT(myWidget::addXimeaScore(double)));
//    connect(kinect,SIGNAL(dk::dkScore(double)),SLOT(myWidget::addDkScore(double)));
}

myWidget::~myWidget()
{
    delete ui;
}

void myWidget::update()
{
    ui->lineEdit_4->setText(QString::number(pointNum));
    ui->lineEdit_5->setText(QString::number(ximea->xi_score));
    ui->lineEdit_6->setText(QString::number(kinect->dk_score));
    score_all = kinect->dk_score + ximea->xi_score;
    ui->lineEdit_7->setText(QString::number(score_all));
    ui->lineEdit_8->setText(QString("%1, %2, %3, %4, %5, %6").arg(gradient[0]).arg(gradient[1]).arg(gradient[2]).arg(gradient[3]).arg(gradient[4]).arg(gradient[5]));
    ui->lineEdit_9->setText(QString::number(len_gradient));
}

void myWidget::policyMove()
{
    if(isPausing == -1)
    {
        switch(needKeeping)
        {
        case 0:
            scoreVec.clear();//single point clear
            for (int i=0;i<6;i++)
            {
                moveVec[i]=0;
            }
            ur->tcpMoveWithVec(moveVec[0],moveVec[1],moveVec[2],moveVec[3],moveVec[4],moveVec[5]);//原地踩一脚
            scoreVec.push_back(score_all);
            ur->memorize();//记录该点 后面可能还需要返回

            for(int j=0;j<6;j++)
            {
                for (int i=0;i<6;i++)
                    {
                        moveVec[i]=0;   //每一步都全部清零
                    }
                 moveVec[j]=1;   //+1
                 ur->tcpMoveWithVec(moveVec[0],moveVec[1],moveVec[2],moveVec[3],moveVec[4],moveVec[5]);
                 scoreVec.push_back(score_all);
                 ur->retureToMemory();//返回原始记录点
            }

            gradient.clear();
            len_gradient=0;
            for(int k=1;k<7;k++)
            {
                gradient.push_back((scoreVec[0]-scoreVec[k])/scoreVec[0]);
                len_gradient += pow(((scoreVec[0]-scoreVec[k])/scoreVec[0]),2);
            }
            len_gradient = pow((len_gradient),0.5);

            scoreTable.push_back(scoreVec);  //1+6
            pointNum++;

             ur->tcpMoveWithVec(gradient[0]/len_gradient,gradient[1]/len_gradient,gradient[2]/len_gradient,gradient[3]/len_gradient,gradient[4]/len_gradient,gradient[5]/len_gradient);
           /* //排序  moveVec  确定
            vector<double>::iterator bigi;
            bigi=max_element(scoreVec.begin(),scoreVec.end());
            vector<double>::iterator mini;
            mini=min_element(scoreVec.begin(),scoreVec.end());

            switch (distance(scoreVec.begin(),min))  //min score position
            {
            case 0:  //已经收敛了

                break;
            case 1: //1 最优
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
            default:
                break;
            }*/
            needKeeping = 1;
            break;
        case 1:
            scoreVec.clear();//single point clear
            for (int i=0;i<6;i++)
            {
                moveVec[i]=0;
            }
            ur->tcpMoveWithVec(moveVec[0],moveVec[1],moveVec[2],moveVec[3],moveVec[4],moveVec[5]);//原地踩一脚
            scoreVec.push_back(score_all);
            ur->memorize();//记录该点 后面可能还需要返回

            ur->tcpMoveWithVec(gradient[0]/len_gradient,gradient[1]/len_gradient,gradient[2]/len_gradient,gradient[3]/len_gradient,gradient[4]/len_gradient,gradient[5]/len_gradient);
            scoreVec.push_back(score_all);

            scoreTable.push_back(scoreVec);  //1+1
            pointNum++;
            if(scoreVec[1] > scoreVec[0])
            {
                ur->retureToMemory();
                needKeeping = 0;
            }
            break;
        default:
            break;
        }
    }
    else
    {
        //提示正在暂停
    }
}

void myWidget::on_pushButton_clicked()
{
    ximea->start();
    ui->pushButton->setEnabled(false);
    ui->pushButton_2->setEnabled(true);
    ui->textBrowser->insertPlainText("Create and run a new thread!\n");
    ui->textBrowser->insertPlainText("Start acquiring multispectral image streams!\n");
}

void myWidget::on_pushButton_2_clicked()
{
    ui->pushButton_2->setEnabled(false);
    emit myWidget::saveTifSignal();
}

void myWidget::on_pushButton_3_clicked()
{
    kinect->start();
    ui->pushButton_3->setEnabled(false);
    ui->pushButton_4->setEnabled(true);
    ui->pushButton_42->setEnabled(true);
    ui->textBrowser->insertPlainText("Create and run a new thread!\n");
    ui->textBrowser->insertPlainText("Start acquiring RGBD image streams!\n");
}

void myWidget::on_pushButton_4_clicked()
{
    ui->pushButton_4->setEnabled(false);
    ui->pushButton_5->setEnabled(false);
    emit myWidget::saveRGBDSignal();
}

void myWidget::on_pushButton_5_clicked() //可能两个之一会出错 因此需要保留单个保存功能
{
    ui->textBrowser->insertPlainText("Start saving both RGBD images and multispectral images!\n");
    ui->pushButton_5->setEnabled(false);
    ui->pushButton_2->setEnabled(false);
    ui->pushButton_4->setEnabled(false);
    ui->pushButton_42->setEnabled(false);
    emit myWidget::saveTifSignal();
    emit myWidget::saveRGBDSignal();
    emit myWidget::savePCDSignal();
}

void myWidget::dealOK()
{
    ui->textBrowser->insertPlainText("*****   No.");
    ui->textBrowser->insertPlainText(QString::fromStdString(std::to_string(kinectSave->kinect->i)));  //此处并非运行完毕 只是显示的需要  加一是因为打印比save更快，i来不及加一，因此提前加一
    ui->textBrowser->insertPlainText(" dk data   *****\n");
    ui->textBrowser->insertPlainText("Successfully saved all data to path!\n");
    ui->pushButton_2->setEnabled(true);
    ui->pushButton_4->setEnabled(true);
    ui->pushButton_5->setEnabled(true);
//    ui->pushButton_42->setEnabled(true);
}

void myWidget::dealpcsOK()
{
    ui->textBrowser->insertPlainText("*****   No.");
    ui->textBrowser->insertPlainText(QString::fromStdString(std::to_string(kpcSave->kinect->i_pc)));  //此处并非运行完毕 只是显示的需要  加一是因为打印比save更快，i来不及加一，因此提前加一
    ui->textBrowser->insertPlainText(" pc data   *****\n");
    ui->textBrowser->insertPlainText("Successfully saved all data to path!\n");
    ui->pushButton_2->setEnabled(true);
//    ui->pushButton_4->setEnabled(true);
    ui->pushButton_5->setEnabled(true);
    ui->pushButton_42->setEnabled(true);
}

void myWidget::dealPcDataSavingDown()
{
    ui->textBrowser->insertPlainText("Successfully saved all adaptively-acquired pc data to path!\n");
}

void myWidget::dealConnectStarted()
{
    ui->textBrowser_2->insertPlainText("UR socket connected!\n");
    ui->pushButton_6->setEnabled(false);
    ui->pushButton_7->setEnabled(true);
}

void myWidget::dealConnectStopped()
{
    ui->textBrowser_2->insertPlainText("UR socket disconnected!\n");
    ui->pushButton_6->setEnabled(true);
    ui->pushButton_7->setEnabled(false);
}

void myWidget::dealWriteDone()
{
    ui->textBrowser_2->insertPlainText(ui->lineEdit_3->text());
    ui->textBrowser_2->insertPlainText("\n");//显示末尾新添加换行符  换行显示
    ui->lineEdit_3->clear();
}

void myWidget::dealMoveDone()
{
    ui->textBrowser_2->insertPlainText(QString(ur->script));
    ui->textBrowser_2->insertPlainText("\n");//显示末尾新添加换行符  换行显示
    ui->lineEdit_3->clear();

    //等待相机画面恢复静止  再计算势能函数
//    score_all = kinect->dk_score + ximea->xi_score; //得分加总  需要去抖  考虑时滞  移动后更新  另外两个实时更新
}

void myWidget::dealArrival()
{
    ui->textBrowser_2->insertPlainText("aaaaaaaaaaaarrived!!!!!\n");
    std::cout<<"aaaaaaaaaaaarrived!!!!!"<<std::endl;
}

void myWidget::dealEmptyData()
{
    ui->textBrowser_2->insertPlainText("Socket data is empty!\n");
}

void myWidget::dealReadDone()
{
    ui->textBrowser_2->insertPlainText("Socket data has been read!\n");
    //ui->textBrowser_2->insertPlainText(ur->...);    //打印一些解析后的数据 例如tcpPos、time等
    ur->showTcpPos();
}

void myWidget::dealPosDone()
{
    ui->textBrowser_2->insertPlainText("*data size: ");
    ui->textBrowser_2->insertPlainText(QString::number(ur->int_datalen));
    ui->textBrowser_2->insertPlainText("\n");

    ui->textBrowser_2->insertPlainText("*power-on time: ");
    ui->textBrowser_2->insertPlainText(QString::number(ur->double_time));
    ui->textBrowser_2->insertPlainText("\n");

    ui->textBrowser_2->insertPlainText("*joint_pos: ");
    ui->textBrowser_2->insertPlainText((ur->joint_pos_str));
    ui->textBrowser_2->insertPlainText("\n");

    ui->textBrowser_2->insertPlainText("*tcp_pos: ");
    ui->textBrowser_2->insertPlainText((ur->tcp_pos_str));
    ui->textBrowser_2->insertPlainText("\n");

//    ui->textBrowser_2->insertPlainText("*tcp_pos_matrix: ");
//    ui->textBrowser_2->insertPlainText(QString::fromStdString(ur->T_base2tcp.matrix()));
//    ui->textBrowser_2->insertPlainText("\n");
    std::cout << ur->T_base2tcp.matrix() << std::endl;
}

void myWidget::dealMemoryClear()
{
    ui->comboBox->clear();
}

void myWidget::dealMemoryPlus()
{
    ui->comboBox->addItem(QString::number(ur->memoryNum));
}

void myWidget::dealMemoryNotChosen()
{
    ui->textBrowser_2->insertPlainText("The target memory pos has not been chosen!");
    ui->textBrowser_2->insertPlainText("\n");
}

void myWidget::on_pushButton_6_clicked()
{
//    ur->ip = QString::fromStdString(ui->lineEdit->text().toStdString());
    ur->ip = ui->lineEdit->text();
    ur->port = ui->lineEdit_2->text().toUInt();
    ur->creatConnection();
}

void myWidget::on_pushButton_7_clicked()
{
    ur->stopConnection();
}

void myWidget::on_pushButton_8_clicked()
{
    ur->script = (ui->lineEdit_3->text()).toUtf8();
    ur->writeSocket();
}

void myWidget::on_pushButton_10_clicked()
{
    QByteArray stop_str = "stopj(a=1)";
    ui->lineEdit_3->setText(stop_str);
    ur->script =(ui->lineEdit_3->text()).toUtf8();
    ur->writeSocket();
}

void myWidget::on_pushButton_9_clicked()
{
    ur->readSocket();
}

void myWidget::on_pushButton_11_clicked()
{

    QString csvFile = QFileDialog::getExistingDirectory(this);
    if(csvFile.isEmpty())
       return;

    //2.文件名采用系统时间戳生成唯一的文件
    QDateTime current_date_time =QDateTime::currentDateTime();
    QString current_date =current_date_time.toString("yyyy_MM_dd_hh_mm_ss");
    csvFile += tr("/urData_%2.csv").arg(current_date);

    //3.用QFile打开.csv文件 如果不存在则会自动新建一个新的文件
    QFile file(csvFile);
    if ( file.exists())
    {
        //如果文件存在执行的操作，此处为空，因为文件不可能存在
    }
    file.open( QIODevice::ReadWrite | QIODevice::Text );
    std::cout<<"exporting data..."<<std::endl;
    ui->textBrowser_2->insertPlainText("exporting data...\n");
    QTextStream out(&file);

    //4.获取数据 创建第一行
    out<<tr("dataSize,")<<tr("powerOnTime,")<<tr("joint1,")<<tr("joint2,")<<tr("joint3,")<<tr("joint4,")<<tr("joint5,")<<tr("joint6,")<<tr("x,")<<tr("y,")<<tr("z,")<<tr("rx,")<<tr("ry,")<<tr("rz,\n");//表头
    //其他数据可按照这种方式进行添加即可
    for(int c = 0;c<ur->count;c++)
    {
        out<<tr(to_string(ur->vec_int_datalen[c]).data());
        out<<tr(",");
        out<<tr(to_string(ur->vec_double_time[c]).data());
        out<<tr(",");
        for (int i=0;i<6;i++)
        {
            out<<tr(to_string(ur->vec_joint_pos[c][i]).data());
            out<<tr(",");
        }
        for (int i=0;i<6;i++)
        {
            out<<tr(to_string(ur->vec_tcp_pos[c][i]).data());
            out<<tr(",");
        }
        out<<tr("\n");
    }

    //5.写完数据需要关闭文件
    std::cout<<"done!"<<std::endl;
    ui->textBrowser_2->insertPlainText("done!\n");
    file.close();
}

void myWidget::on_pushButton_12_clicked()
{
    ur->isForward = 1;
    ur->tcp_flag = 1;
    ur->tcpMove();
}

void myWidget::on_pushButton_13_clicked()
{
    ur->isForward = -1;
    ur->tcp_flag = 1;
    ur->tcpMove();
}

void myWidget::on_pushButton_14_clicked()
{
    ur->isForward = 1;
    ur->tcp_flag = 2;
    ur->tcpMove();
}

void myWidget::on_pushButton_15_clicked()
{
    ur->isForward = -1;
    ur->tcp_flag = 2;
    ur->tcpMove();
}

void myWidget::on_pushButton_16_clicked()
{
    ur->isForward = 1;
    ur->tcp_flag = 3;
    ur->tcpMove();
}

void myWidget::on_pushButton_17_clicked()
{
    ur->isForward = -1;
    ur->tcp_flag = 3;
    ur->tcpMove();
}

void myWidget::on_pushButton_18_clicked()
{
    ur->isForward = 1;
    ur->tcp_flag = 4;
    ur->tcpMove();
}

void myWidget::on_pushButton_19_clicked()
{
    ur->isForward = -1;
    ur->tcp_flag = 4;
    ur->tcpMove();
}

void myWidget::on_pushButton_20_clicked()
{
    ur->isForward = 1;
    ur->tcp_flag = 5;
    ur->tcpMove();
}

void myWidget::on_pushButton_21_clicked()
{
    ur->isForward = -1;
    ur->tcp_flag = 5;
    ur->tcpMove();
}

void myWidget::on_pushButton_22_clicked()
{
    ur->isForward = 1;
    ur->tcp_flag = 6;
    ur->tcpMove();
}

void myWidget::on_pushButton_23_clicked()
{
    ur->isForward = -1;
    ur->tcp_flag = 6;
    ur->tcpMove();
}

void myWidget::on_pushButton_24_clicked()
{
    ur->isForward = 1;
    ur->joint_flag = 1;
    ur->jointMove();
}

void myWidget::on_pushButton_27_clicked()
{
    ur->isForward = -1;
    ur->joint_flag = 1;
    ur->jointMove();
}

void myWidget::on_pushButton_25_clicked()
{
    ur->isForward = 1;
    ur->joint_flag = 2;
    ur->jointMove();
}

void myWidget::on_pushButton_28_clicked()
{
    ur->isForward = -1;
    ur->joint_flag = 2;
    ur->jointMove();
}

void myWidget::on_pushButton_26_clicked()
{
    ur->isForward = 1;
    ur->joint_flag = 3;
    ur->jointMove();
}

void myWidget::on_pushButton_29_clicked()
{
    ur->isForward = -1;
    ur->joint_flag = 3;
    ur->jointMove();
}

void myWidget::on_pushButton_30_clicked()
{
    ur->isForward = 1;
    ur->joint_flag = 4;
    ur->jointMove();
}

void myWidget::on_pushButton_33_clicked()
{
    ur->isForward = -1;
    ur->joint_flag = 4;
    ur->jointMove();
}

void myWidget::on_pushButton_31_clicked()
{
    ur->isForward = 1;
    ur->joint_flag = 5;
    ur->jointMove();
}

void myWidget::on_pushButton_34_clicked()
{
    ur->isForward = -1;
    ur->joint_flag = 5;
    ur->jointMove();
}

void myWidget::on_pushButton_32_clicked()
{
    ur->isForward = 1;
    ur->joint_flag = 6;
    ur->jointMove();
}

void myWidget::on_pushButton_35_clicked()
{
    ur->isForward = -1;
    ur->joint_flag = 6;
    ur->jointMove();
}

void myWidget::on_pushButton_40_clicked()
{
    ur->setOrigin();
}

void myWidget::on_pushButton_36_clicked()
{
    ur->memorize();
}

void myWidget::on_pushButton_39_clicked()
{
    ur->returnToOrigin();
}

void myWidget::on_pushButton_37_clicked()
{
    ur->retureToMemory();
}

void myWidget::on_pushButton_41_clicked()
{
    isPausing = isPausing * (-1);
}

void myWidget::on_pushButton_38_clicked()
{
    //初始化所有policy move相关的参数
    isPausing = -1;
    pointNum = 0;
    needKeeping = 0;
    scoreVec.clear();
    gradient.clear();
    scoreTable.clear();
    //开始循环执行policy move  step
    while(len_gradient > grad_thred)
    {
        policyMove();
    }
    //提示已经完成自适应路径规划
    ui->textBrowser_2->insertPlainText("Policy Move Finished !!!");
    ui->textBrowser_2->insertPlainText("\n");
    //可以添加自动采集的code
}

void myWidget::on_pushButton_42_clicked()
{
    ui->pushButton_42->setEnabled(false);
    ui->pushButton_5->setEnabled(false);
    emit myWidget::savePCDSignal();
}

void myWidget::on_comboBox_currentIndexChanged(const QString &arg1)
{
    if(!arg1.isEmpty())
    {
        ur->memoryChosen = arg1.toInt();
    }
    else
        ur->memoryChosen = 0;
}

void myWidget::on_pushButton_44_clicked() //adaptive capturing with the robotic arm
{
    //control the robotic arm to move towards the memory pos 1 2 3 ...
    kpcProcess->adaptiveCapturing();
    act->start();    //the ur socket could not be executed in thread...
}

void myWidget::on_pushButton_43_clicked()
{
    ur->resetMem();
}

void myWidget::on_pushButton_45_clicked()
{
    kpcProcess->autoSavingMemPc();
}
