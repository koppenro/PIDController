#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <math.h>
#include <thread>
#include <fstream>

#include "KITTempBoard.h"
#include "GenesysControl.h"

using namespace std::chrono;

class PIDController
{
  private:
    // class members read from config file
    double fT_Set;
    int fContChannel0;
    int fContChannel1;
    double fPWeight;
    double fIWeight;
    double fDWeight;
    std::string fTBPort;
    std::string fGCPort;

    // additional class variables
    high_resolution_clock::time_point fTimerPrevious;
    double fEtPrevious;
    float fT_SetPrevious;
    double fMeanPrevious;
    bool fFirstPIDStep;
    double fNewVoltage;
    double fOldVoltage;
    bool fRunning;
    bool fGCRunning;
    double fIntOfEtPrevious;
    double fTotalTime;
    bool fDewPointLimited;

    // device classes
    KITTempBoard * fTB;
    GenesysControl * fGC;

    // class members
    double PIDStep(double pT_Is, double pT_Set);
    void startControlLoop();
    void userInteraction();
    int signum(double pValue);
    double DewPointWatch();
    double calcMeanTControl(std::vector<double> pCurTemps, std::vector<double> pCurTemps2);

  public:

    PIDController();
    ~PIDController();
    bool initTB();
    bool initGC(int pAddress = 6);
    bool readConfigFile(std::string pPathToCfgFile);
    void run();

    void setTSet(double cTSet);
    void setOutput(bool pValue);
    void resetIValue();

};

#endif // PIDCONTROLLER_H
