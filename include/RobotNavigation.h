#include <iostream>
#include <cmath>
#include <string>
#include <stdio.h>
#include <stddef.h>	//#define NULL ...
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <linux/i2c-dev.h>
#include "AToD.h"
#include "TCA9548a.h"
#include "VL53L0X.h"

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/MatrixFunctions>

//qp
#include <qpOASES.hpp>

using namespace std;

class RobotNavigation
{
    public:
        //-Const set
        //const M = 3;

        //-Varible Set-
        //---State Machine---
        //State Mark
        int mnstate;
        //Counter
        int mcntSM;

        double M,I;


        //---Laser Distance Sensor---
        //Position
        double mdDX,mdDY,mdYaw;
        double mdDXPre,mdDYPre,mdYawPre;
        //Velocity
        double mdVx,mdVy,mdW;
        //acc
        double mdAx,mdAy,mdAlpha;
        //Coeffcient
        double mdKdx,mdKdy,mdKyaw;
        double mdKvx,mdKvy,mdKw;
        //Senor Relate
        double mdDistSensor[8];

        //---Touch Sensor---
        //Sensor Relate
        double mdTouchSensor[2];

        //---Force Sensor---
        double mdForceSensor[4];

        //ADS1115
        ///-----------------------------------------------workable sulotion
        //AToD *AtoDForce;
        //AToD *AtoDTouch;


        // char *i2c_filename = (char*)"/dev/i2c-1";
        // const unsigned char A_TO_D_ADDRESS_01 = 0x48;
        // const unsigned char A_TO_D_ADDRESS_02 = 0x49;
        // AToD AtoDForce(char, const unsigned char);
        // AToD AtoDTouch(char, const unsigned char);

        AToD atodf;
        AToD atodt;
       

        //TCA9548a
        TCA9548a tca;

        //VL53L0x
        VL53L0X tof[6];


        //-Function-
        RobotNavigation();
        ~RobotNavigation();//destructor
        void TFInitialize();
        void LsInitialize();
        void TFGetValue();
        void LsGetValue();
        void Run();
        void Update();
        void SetValue();
        void RSideMSD(double threshold);
        void LSideMSD(double threshold);
        void RearMSD(double threshold);
        void SideMPC(float thresholdYaw, float thresholdY, int side);

};