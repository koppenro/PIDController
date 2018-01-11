#include "PIDController.h"

int main(int argc,char** argv)
{
  PIDController pid = PIDController();
  pid.readConfigFile("../TempControl.cfg");
  pid.initTB();
  if( pid.initGC(1) ) {
    pid.setOutput(true);
  }
  pid.run();
}
