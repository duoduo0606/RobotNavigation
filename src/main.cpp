#include "RobotNavigation.h"

using namespace std;

int main()
{
    //
    RobotNavigation rn;

    //
    //rn.LsInitialize();
    rn.TFInitialize();

    //
    while (1)
    {
        //rn.LsGetValue();
        rn.TFGetValue();
        //rn.Run();
        //rn.Update();
    }

    return 1;
}