#include "RobotNavigation.h"
#include <chrono>


using namespace std;
using namespace Eigen;

//robot size in mm
#define BODY_LENGTH 310
#define BODY_WIDTH 140

//globle
#define TIMESTEP 0.01
#define N 5

//#define LONG_RANGE
//#define HIGH_SPEED
//#define HIGH_ACCURACY

//varible local

//functions local

//member varible init
RobotNavigation::RobotNavigation():atodf(0,0x48),atodt(0,0x49)
{
    //start message
    cout << "Robot Navigation start ..." << endl;

    //varible initialize
    mnstate = 0;
    mcntSM = 0;

    //globle
    mdDX = 0;
    mdDY = 0;
    mdYaw = 0;
    mdVx = 0;
    mdVy = 0;
    mdW = 0;
    mdAx = 0;
    mdAy = 0;
    mdAlpha = 0;
    mdKdx = 0;
    mdKdy = 0;
    mdKw = 0;
    mdKvx = 0;
    mdKvy = 0;
    mdKw = 0;

    //Sensor
    mdDistSensor[8] = {0};
    mdTouchSensor[2] = {0};
    mdForceSensor[4] = {0};
    
    //Robot
    M = 3;
    I = 1.0 / 12 * M * (BODY_LENGTH*BODY_LENGTH + BODY_WIDTH*BODY_WIDTH);
}

RobotNavigation::~RobotNavigation()
{

}

void RobotNavigation::TFInitialize()
{   
    //for debug
    cout << "Touch & Force sensor ADS init..." << endl;

    //ADS1115 instantiate
    char *i2c_filename = (char*)"/dev/i2c-1";
    const unsigned char A_TO_D_ADDRESS_01 = 0x48;
    const unsigned char A_TO_D_ADDRESS_02 = 0x49;
    // AToD AtoDForce(i2c_filename, A_TO_D_ADDRESS_01);
    // AToD AtoDTouch(i2c_filename, A_TO_D_ADDRESS_02);

    ///-----------------------------------------------workable sulotion
    //*AtoDForce = AToD(i2c_filename, A_TO_D_ADDRESS_01);
    //*AtoDTouch = AToD(i2c_filename, A_TO_D_ADDRESS_02);
    atodf = AToD(i2c_filename, A_TO_D_ADDRESS_01);
    atodt = AToD(i2c_filename, A_TO_D_ADDRESS_02);
}

void RobotNavigation::LsInitialize()
{       
    //for debug
    cout << "Laser sensor & TCA init..." << endl;

    //TCA9548a instantiate
    tca.init(0x70);

    //VL53L0X instantiate
    for (int i=0; i<6; i++)
    {
        //set channel to corresponding tof
        tca.set_channel(i);

        //init tof
        tof[i].init();
        tof[i].setTimeout(500);
        tof[i].startContinuous();
  
    //     #if defined LONG_RANGE
    //     // lower the return signal rate limit (default is 0.25 MCPS)
    //     tof[i].setSignalRateLimit(0.1);
    //     // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    //     tof[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    //     tof[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    //     #endif

    //     #if defined HIGH_SPEED
    //     // reduce timing budget to 20 ms (default is about 33 ms)
    //     tof[i].setMeasurementTimingBudget(20000);
    //     #elif defined HIGH_ACCURACY
    //     // increase timing budget to 200 ms
    //     tof[i].setMeasurementTimingBudget(200000);
    //     #endif
    }

    tca.no_channel();
}

void RobotNavigation::TFGetValue()
{   
    //for debug
    cout << "Touch & Force sensor value..." << endl;

    //record time elapsedTime
    auto beginTime = std::chrono::high_resolution_clock::now();

    //ADS1115 value
    //force sensors & touch sensors
    double channelVoltagesForce[4] = {0.0,0.0,0.0,0.0};
    double channelVoltagesTouch[4] = {0.0,0.0,0.0,0.0};
    for (int i=0; i<4; i++) 
    {   
        ///-----------------------------------------------workable sulotion
	    // AtoDForce->GetMeasurement(i+1,0,1.0,channelVoltagesForce[i]);
        // AtoDTouch->GetMeasurement(i+1,0,1.0,channelVoltagesTouch[i]);

        atodf.GetMeasurement(i+1,0,1.0,channelVoltagesForce[i]);
        atodt.GetMeasurement(i+1,0,1.0,channelVoltagesTouch[i]);
    }

    //raw data transfer
    mdForceSensor[0] = channelVoltagesForce[0];
    mdForceSensor[1] = channelVoltagesForce[1];
    mdForceSensor[2] = channelVoltagesForce[2];
    mdForceSensor[3] = channelVoltagesForce[3];

    mdTouchSensor[0] = channelVoltagesTouch[0];
    mdTouchSensor[1] = channelVoltagesTouch[1];

    //data process
    cout << "mdForceSensor0  " << mdForceSensor[0] << endl;
    cout << "mdForceSensor1  " << mdForceSensor[1] << endl;
    cout << "mdForceSensor2  " << mdForceSensor[2] << endl;
    cout << "mdForceSensor3  " << mdForceSensor[3] << endl;

    cout << "mdTouchSensor0  " << mdTouchSensor[0] << endl;
    cout << "mdTouchSensor1  " << mdTouchSensor[1] << endl;
    
    
}

void RobotNavigation::LsGetValue()
{       
    //for debug
    cout << "Laser sensor value..." << endl;

    //record time elapsedTime
    auto beginTime = std::chrono::high_resolution_clock::now();
    
    //VL53l0x value
    for (int j=0; j<6; j++)
    {
        //set channel to corresponding tof
        tca.set_channel(j);
        //cout << "--- 4 ---" << endl;

        //read tof
        //mdDistSensor[j] = tof[j].readRangeSingleMillimeters();
        mdDistSensor[j] = tof[j].readRangeContinuousMillimeters();
        //cout << "--- 5 ---" << endl;
        //tof[j].stopContinuous();
    }

    // tca.no_channel();
    
    // //data process
    cout << "mdDistSensor0  " << mdDistSensor[0] << "mm" << endl;
    cout << "mdDistSensor1  " << mdDistSensor[1] << "mm" << endl;
    cout << "mdDistSensor2  " << mdDistSensor[2] << "mm" << endl;
    cout << "mdDistSensor3  " << mdDistSensor[3] << "mm" << endl;
    cout << "mdDistSensor4  " << mdDistSensor[4] << "mm" << endl;
    cout << "mdDistSensor5  " << mdDistSensor[5] << "mm" << endl;

    // time caculation
    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-beginTime);
    double programTimes = (double) elapsedTime.count();
    cout << "TIME :: getvalue take  " << programTimes << "ms" << endl;
}

void RobotNavigation::Run()
{   
    //actions on every state
    switch (mnstate)
    {   
        //init state
        case 0:
            mdVx = 0;
            mdVy = 0;
            mdW = 0;
            break;

        //直行
        case 1:
            cout << "case1 1号舱道直行" << endl;
            mdVx = 40;
            LSideMSD(150);
            break;
        

    }
    

}

void RobotNavigation::Update()
{
    switch (mnstate)
    {
        //transfer condition
        case 0:
            mcntSM++;
            if (mcntSM > 500)
            {
                mcntSM = 0;
                mnstate = 1;
            }
            break;

        case 1:
            //
            if (mdDistSensor[0] > 400)
            {
                mcntSM++;
            }

            if (mcntSM > 300)
            {
                mcntSM = 0;
                mnstate = 2;
            }
            break;
        
        
    }
}

void RobotNavigation::SetValue()
{
    // velx = vx;
    // vely = vy;
    // velw = w;
}

void RobotNavigation::RSideMSD(double threshold)
{      
    //apply MSD
    //Y
    mdDY = ((mdDistSensor[4] + mdDistSensor[5])/2 - threshold);
    mdDYPre = mdDY;

    mdAy = 1/M * (-mdKdy*mdDY-mdKvy*mdVy);
    mdVy = mdVy + mdAy * TIMESTEP;

    //Yaw
    mdYaw = atan2(mdDistSensor[4] - mdDistSensor[5], BODY_LENGTH);
    mdYawPre = mdYaw;
   
    mdAlpha = (-mdKyaw*mdYaw-mdKw*mdW)/I;
    mdW = mdW + mdAlpha * TIMESTEP;
}

void RobotNavigation::LSideMSD(double threshold)
{       
    //apply MSD
    //Y
    mdDY = -((mdDistSensor[0] + mdDistSensor[1])/2 - threshold);
    mdDYPre = mdDY;

    mdAy = 1/M * (-mdKdy*mdDY-mdKvy*mdVy);
    mdVy = mdVy + mdAy * TIMESTEP;

    //Yaw
    mdYaw = atan2(mdDistSensor[1] - mdDistSensor[0], BODY_LENGTH);
    mdYawPre = mdYaw;
   
    mdAlpha = (-mdKyaw*mdYaw-mdKw*mdW)/I;
    mdW = mdW + mdAlpha * TIMESTEP;
}

void RobotNavigation::RearMSD(double threshold)
{   
    //apply MSD
    //X
    mdDX = ((mdDistSensor[2] + mdDistSensor[3])/2 - threshold);
    mdDXPre = mdDX;
    mdAx = 1/M * (-mdKdx*mdDX-mdKvx*mdVx);
    mdVx = mdVx + mdAx * TIMESTEP;

    //Yaw
    mdYaw = atan2(mdDistSensor[3] - mdDistSensor[2], BODY_WIDTH);
    mdYawPre = mdYaw;
   
    mdAlpha = (-mdKyaw*mdYaw-mdKw*mdW)/I;
    mdW = mdW + mdAlpha * TIMESTEP;
}

void RobotNavigation::SideMPC(float thresholdYaw, float thresholdY, int side)
{
    //define
    Matrix<float, 4, 4> A_mpc;
    Matrix<float, 4, 2> B_mpc;
    Matrix<float, 4, 4> Adt;
    Matrix<float, 4, 2> Bdt;
    Matrix<float, N*4, 4> Aqp;
    Matrix<float, N*4, N*2> Bqp;
    Matrix<float, 6, 6> temp_AB_mpc,expmm;
    Matrix<float, N*4, N*4> L;
    Matrix<float, N*2, N*2> K;

    Vector<float, 4> X_ref1;
    Vector<float, N*4> X_ref;
    Vector<float, 4> X_cur;

    //double force,torque;
    //int n = 5;

    //input
    //left -1 right 1
    mdDY = side * ((mdDistSensor[0] + mdDistSensor[1])/2-thresholdY);
    mdYaw = -side * atan2(mdDistSensor[1] - mdDistSensor[0], BODY_WIDTH);

    //function
    X_cur << mdYaw, mdDY, mdW, mdVy;
    X_ref1 << 0.0, 0.0, 0.0, 0.0;

    A_mpc = MatrixXf::Zero(4, 4);
    B_mpc = MatrixXf::Zero(4, 2);
    A_mpc.block(0, 2, 2, 2) = MatrixXf::Identity(2, 2);
    
    B_mpc(2, 0) = 1.0/I;
    B_mpc(3, 1) = 1.0/M;

    temp_AB_mpc = MatrixXf::Zero(6, 6);
    expmm = MatrixXf::Zero(6, 6);

    temp_AB_mpc.block(0, 0, 4, 4) = A_mpc;
    temp_AB_mpc.block(0, 4, 4, 2) = B_mpc;
    temp_AB_mpc.block(4, 0, 2, 6) = MatrixXf::Zero(2, 6);

    temp_AB_mpc =  TIMESTEP * temp_AB_mpc;
    expmm = temp_AB_mpc.exp();

    Adt = expmm.block(0, 0, 4, 4);
    Bdt = expmm.block(0, 4, 4, 2);

    Aqp = MatrixXf::Zero(N*4,4);
    Bqp = MatrixXf::Zero(N*4,N*2);

    //get Adt exp
    // Matrix<float, 6, 6> powerMats[7];
    // powerMats[0].block(0, 0, 6, 6) = MatrixXf::Identity(6, 6);
    // for (int m = 1; m < 6; m++)
    // {
    //     powerMats[m] = Adt * powerMats[m-1];
    // }

    //get Aqp & Bqp
    for (int i = 0; i < N; i++)
    {
        X_ref.segment(4 * i, 4) = X_ref1;
        Aqp.block(4 * i, 0, 4, 4) = Adt.pow(i + 1);//powerMats[i + 1];
        for (int j = 0; j < i + 1; j++)
        {
            Bqp.block(4 * i, 2 * j, 4, 2) = Adt.pow(i - j) * Bdt;//powerMats[i - j]
        }
    }

    //get L & K
    L = MatrixXf::Identity(N*4, N*4);
    for(int i = 0; i < N; i++)
    {
        L(0 + 4 * i, 0 + 4 * i) = 5;//theta
        L(1 + 4 * i, 1 + 4 * i) = 25;//y 25  zhen dang xiao 10
        L(2 + 4 * i, 2 + 4 * i) = 1;//w
        L(3 + 4 * i, 3 + 4 * i) = 1;//vy 1
    }

    K = 0.00001 * MatrixXf::Identity(N*2, N*2);//0.001 shouxian kuai 0.00001

    Matrix<float, N*2, N*2> temp_H;
    Vector<float, N*2> temp_g;

    temp_H = 2 * (Bqp.transpose() * L * Bqp + K);
    temp_g = 2 * Bqp.transpose() * L * (Aqp * X_cur - X_ref);

    // Matrix<float, 8, 6> temp_uA;
    // Matrix<float, 40, 30> temp_tA;
    //Vector<float, 10> temp_Forcesum;
    Vector<float, 2> Force;

    //using namespace qpOASES
    USING_NAMESPACE_QPOASES

    
    real_t H[N*2*N*2];
    real_t g[N*2];
    
    for (int i = 0; i < N*2; i++)
    {
        for (int j = 0; j < N*2; j++)
        {
             H[i * N*2 + j] = temp_H(i, j);
        }
        g[i] = temp_g(i);
    }

	//real_t fA[40*30];
	real_t lb[N*2];
	real_t ub[N*2];
	// real_t lbA[40];
	// real_t ubA[40];
    //float u = 4.0;
    // temp_uA <<u, 0, 1, 0, 0, 0,
    //           -u, 0, 1, 0, 0, 0,
    //           0, u, 1, 0, 0, 0,
    //           0, -u, 1, 0, 0, 0,
    //           0, 0, 0, u, 0, 1,
    //           0, 0, 0, -u, 0, 1,
    //           0, 0, 0, 0, u, 1,
    //           0, 0, 0, 0, -u, 1;
    // for (int i = 0; i< n; i++)
    // {
    //     temp_tA.block(8 * i, 6 * i, 8, 6) = temp_uA;
    // }
    //temp_tA = MatrixXf::Zero(40,30);
    // for (int i = 0; i < 40; i++)
    // {
    //     for (int j = 0; j < 30; j++)
    //     {
    //         fA[i * 40 + j] = temp_tA(i, j);
    //     }
    // }
    for (int i = 0; i < N*2; i++)
    {
        lb[i] = -0.05;
        ub[i] = 0.05;
    }
    // for (int i = 0; i < 40; i++)
    // {
    //     lbA[i] = 0;
    //     ubA[i] = 1000000;
    // }
	QProblem eg(N*2,0);
	int_t nWSR = 60;
	//example.init( H,g,fA,lb,ub,lbA,ubA,nWSR);
    eg.init(H,g,NULL,lb,ub,NULL,NULL,nWSR);
	real_t xOpt[N*2];
	eg.getPrimalSolution(xOpt);
    // for (int i = 0; i < 10; i++)
    // {
    //     temp_Forcesum(i) = xOpt[i];
    // }
    Force << xOpt[0], xOpt[1];
    // XX = Aqp * X_cur + Bqp * temp_Forcesum;

    //output
    mdAy = 1.0/M * Force(1);
    mdVy = mdVy + mdAy * TIMESTEP;
    mdVy = mdVy * cos(mdYaw);
   
    mdAlpha = 1.0/I * Force(0);
    mdW = mdW + mdAlpha * TIMESTEP;
}
