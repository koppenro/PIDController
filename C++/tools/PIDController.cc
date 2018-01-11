#include "PIDController.h"

// Geraetemanager und PID Steuerung
// Klasse kommuniziert mit Temperaturausleseplatine und TDK Genesys

PIDController::PIDController() {
  fT_Set = 20;
  fContChannel0 = 1;    // Index of first control channel
  fContChannel1 = 2;    // Index of second control channel
  fNewVoltage = 0;      // Voltage to set (result of PIDStep)
  fOldVoltage = 0;      // Storage for voltage of previous PID step
  fDewPointLimited = false;

  // device management
  fTBPort = "";
  fGCPort = "";
  fTB = new KITTempBoard();
  fGC = new GenesysControl();
  fRangeLow = -25;
  fRangeUp = 30;

  // PID controller settings
  fPWeight = 0;
  fIWeight = 0;
  fDWeight = 0;
  fIntOfEtPrevious = 0; // I-Value during PID controlling

  // additional class variables
  fFirstPIDStep = true;
  fMeanPrevious = 0;
  fRunning = false;
  fTimerPrevious = high_resolution_clock::now();
  fEtPrevious = 0;
  fT_SetPrevious = 0;
  fGCRunning = false;
  double fTotalTime = 0;

}

PIDController::~PIDController() {

}

// ###############################################################################################
// ##                                    Device manager                                         ##
// ###############################################################################################

bool PIDController::initTB() {   // Temperature Board control
  // Init Temperature Board
  bool cTBinit = fTB->init(fTBPort);
  if( cTBinit ) {
    fTB->switchDigitalPin(false);
    std::cout << "INFO: Connection with Temperature Board succesfully established" << std::endl;
    fRunning = true;
  }
  else {
    std::cout << "ERROR: Failed to open connection to Temperature Board!" << std::endl;
    fRunning = false;
  }
  return cTBinit;
}

bool PIDController::initGC(int pAddress) {   // Genesys Control
  // Init Genesys low power supply
  bool cGCinit = fGC->init(fGCPort, pAddress);
  if ( cGCinit ) {
    fGC->setVoltage(0);
    fGC->setOutput(false);
    std::cout << "INFO: Connection with Genesys Low power supply succesfully established" << std::endl;
  }
  else { std::cout << "ERROR: Failed to open connection to Genesys power supply" << std::endl; }
  fGCRunning = cGCinit;
  return cGCinit;
}

double PIDController::DewPointWatch() {
  double cDewPoint = 4.;
  std::ifstream humidity;
  humidity.open("/mnt/1wire/honeywell/humidity");
  std::string cLine;
  if (humidity.is_open()) {
    getline(humidity, cLine);
    cDewPoint = std::stod(cLine);
  }

  if ( cDewPoint > fT_Set ) {
      std::cout << "INFO: Temperature is limited to be higher than " << cDewPoint-1 << "(Dew point)" << std::endl;
    fDewPointLimited = true;
    return cDewPoint-1;
  }
  if ( fDewPointLimited ) {std::cout << "INFO: Temperature is not any more limited due to the dew point" << std::endl;}
  fDewPointLimited = false;
  return fT_Set;
}

bool PIDController::readConfigFile(std::string pPathToCfgFile) {
  std::ifstream configfile;
  configfile.open(pPathToCfgFile);
  std::string line;
  bool opened = configfile.is_open();
  if (opened) {
    while( getline(configfile, line)) {
      if (line[0] == '[') continue;
      int posEqual=line.find('=');
      if (line.substr(0,posEqual-1) == "TSet") { fT_Set = std::stod(line.substr(posEqual+1,line.size())); }
      if (line.substr(0,posEqual-1) == "ControlChannel0") { fContChannel0 = std::stoi(line.substr(posEqual+1,line.size())); }
      if (line.substr(0,posEqual-1) == "ControlChannel1") { fContChannel1 = std::stoi(line.substr(posEqual+1,line.size())); }
      if (line.substr(0,posEqual-1) == "P") { fPWeight = std::stod(line.substr(posEqual+1,line.size())); }
      if (line.substr(0,posEqual-1) == "I") { fIWeight = std::stod(line.substr(posEqual+1,line.size())); }
      if (line.substr(0,posEqual-1) == "P") { fDWeight = std::stod(line.substr(posEqual+1,line.size())); }
      if (line.substr(0,posEqual-1) == "TBPort") { fTBPort = line.substr(posEqual+2,line.size()); }
      if (line.substr(0,posEqual-1) == "GCPort") {
        fGCPort = line.substr(posEqual+2,line.size());
        if (fGCPort == "-") {
          fGCPort = "";
        }
      }
      if (line.substr(0, posEqual-1) == "RangeLow") { fRangeLow = std::stod(line.substr(posEqual+1,line.size())); }
      if (line.substr(0, posEqual-1) == "RangeUp") { fRangeUp = std::stod(line.substr(posEqual+1,line.size())); }
    }
    std::cout << "INFO: Config file parameters read" << std::endl;
    std::cout << fT_Set << "\t" << fContChannel0 << "\t" << fContChannel1 << "\t" << fPWeight << "\t" << fIWeight << "\t" << fDWeight << "\t" << fTBPort << "\t" << fGCPort << "\t" << fRangeLow << "\t" << fRangeUp << std::endl;
  }
  else {
    std::cout << "ERROR: Could not open config file! Program will start with default hard coded parameters!" << std::endl;
  }
  configfile.close();
  return opened;
}


// ###############################################################################################
// ##                                    PID implementation                                     ##
// ###############################################################################################

double PIDController::PIDStep(double pT_Is, double pT_Set) {

  // proportional–integral–derivative controller
  // error value e(t) = desired setpoint SP - measured process variable PV
  // u(t) = Kp*e(t) + Ki*INT_0^t(e(t)dt) + Kd*de(t)/dt

  //Check for large changes in T_Set
  if (fabs(fT_SetPrevious - pT_Set) > 5) {
    fIntOfEtPrevious = 0; //Reset value of PID integral
    fT_SetPrevious = pT_Set;
  }

  // Calculation of dt
  high_resolution_clock::time_point cTimerCurrent = high_resolution_clock::now();
  if ( fFirstPIDStep ) {
    fTimerPrevious = cTimerCurrent;
    fFirstPIDStep = false;
  }
  duration<double, std::milli> time_span = cTimerCurrent - fTimerPrevious;
  double cTimerDifference = time_span.count()/1000.;
  fTotalTime = fTotalTime + cTimerDifference;

  double cEt = pT_Is - pT_Set;   // error value (DeltaT)

  // proportional part
  double cPP = cEt * fPWeight;

  // integral part
  float cMaxDeltaT = 2;
  double cEtForI = 0;
  // if (cEt < 0) {cEtForI = -cMaxDeltaT;}
  if (cEt > 2) {cEtForI = cMaxDeltaT;}           // Only when DeltaT is smaller than 2°C start controlling system with current DeltaT,
  else if (cEt < -2) {cEtForI = -cMaxDeltaT;}    // before use maximum DeltaT to reach desired temperature as fast as possible
  else {cEtForI = cEt;}
  double cIP = fIntOfEtPrevious + cTimerDifference * cEtForI * fIWeight;

  // derivative part
  double cDP = 0;
  if ( cTimerDifference != 0 ) {
    cDP = fDWeight * (cEt - fEtPrevious)/cTimerDifference;
  }

  double cSetVoltage = cPP + cIP + cDP;
  double cMaxVoltage = 12;

  // Actualize previous values
  fTimerPrevious = cTimerCurrent;
  fIntOfEtPrevious = cIP;
  fEtPrevious = cEt;

  if (cSetVoltage > cMaxVoltage) {
    return(cMaxVoltage);
  }
  else if (cSetVoltage < -cMaxVoltage) {
    return(-cMaxVoltage);
  }
  else {return cSetVoltage;}
}

double PIDController::calcMeanTControl(std::vector<double> pCurTemps, std::vector<double> pCurTemps2) {

  std::vector<double> cTempValues = {};
  cTempValues.push_back(pCurTemps[fContChannel0]);
  cTempValues.push_back(pCurTemps2[fContChannel0]);
  cTempValues.push_back(pCurTemps[fContChannel1]);
  cTempValues.push_back(pCurTemps2[fContChannel1]);

  // Calculate mean value
  double cMean = 0;
  for(std::vector<double>::iterator it = cTempValues.begin(); it != cTempValues.end(); ++it) { cMean = cMean + *it; }
  cMean = cMean/cTempValues.size();

  //Calculate standard deviation of mean value
  double s = 0;
  for(std::vector<double>::iterator it = cTempValues.begin(); it != cTempValues.end(); ++it) { s = s + pow((*it - cMean),2); }
  s = sqrt(s/(cTempValues.size()-1));

  //Reject values which are more than three standard deviation away from previous mean value
  double lowerLimit = fMeanPrevious - 3*s;
  double upperLimit = fMeanPrevious + 3*s;
  cMean = cMean*cTempValues.size();
  int counter = 0;
  for(std::vector<double>::iterator it = cTempValues.begin(); it != cTempValues.end(); ++it) {
    if ( *it > upperLimit || *it < lowerLimit ) {
      std::cout << "Mean before rejection " << cMean/cTempValues.size() << std::endl;
      cMean = cMean - *it;
      counter++;
      std::cout << "Rejected " << *it << std::endl;
    }
  }
  cMean = cMean/(cTempValues.size() - counter);
  cMean = fMeanPrevious;

  return cMean;
}



// ###############################################################################################
// ##                               Measurement Control                                         ##
// ###############################################################################################


void PIDController::run() {

  std::thread first(&PIDController::startControlLoop,this);
  std::thread second(&PIDController::userInteraction,this);
  first.join();
  second.join();
  if(fGCRunning) {
    fGC->setOutput(false);
    fGC->setVoltage(0);
  }
}

void PIDController::startControlLoop() {
  std::ofstream outputfile;
  outputfile.open("Test.txt");
  outputfile << "// Temperature Control\n";
  outputfile << "// P = " << fPWeight << ", I = " << fIWeight << ", D = " << fDWeight << "\n";
  outputfile << "// Time\tTSet\tTIs\tV\tT0\tT1\tT2\tT3\tI\n";
  std::cout << "Time\tTSet\tTIs\tV\tT0\tT1\tT2\tT3\tI" << std::endl;
  while (fRunning) {
    std::vector<double> cCurTemps = fTB->getTemperature();
    //usleep(500000);
    std::vector<double> cCurTemps2 = fTB->getTemperature();
    if(cCurTemps.size() != 4 || cCurTemps2.size() != 4) {
      // Stop control loop due to failure during reading of temperatures
      if (fGCRunning) {
        fGC->setVoltage(0);
        fGC->setOutput(0);
      }
      std::cout << "ERROR: Failure during reading of temperatures" << std::endl;
      fRunning = false;
    }
/*    // Test if all temperatures are inside the allowed range
    for(std::vector<double>::iterator it = cCurTemps.begin(); it != cCurTemps.end(); ++it) {
      if ( *it < fRangeLow or *it > fRangeUp) {
        fRunning = false;
        std::cout << "ERROR: Some of the temperature values are not in the allowed range!" << std::endl;
      }
    }
    for(std::vector<double>::iterator it = cCurTemps2.begin(); it != cCurTemps2.end(); ++it) {
      if ( *it < fRangeLow or *it > fRangeUp) {
        fRunning = false;
        std::cout << "ERROR: Some of the temperature values are not in the allowed range!" << std::endl;
      }
    }
*/

    if (fFirstPIDStep) { fMeanPrevious = (cCurTemps[fContChannel0] + cCurTemps[fContChannel1] + cCurTemps2[fContChannel0] + cCurTemps2[fContChannel1])/4.; }
    if (fRunning) {
      double cTIs = PIDController::calcMeanTControl(cCurTemps, cCurTemps2);
      double cTSet = PIDController::DewPointWatch();
      fNewVoltage = PIDController::PIDStep(cTIs, cTSet);

      //Print out for saving
      outputfile << fTotalTime << "\t" << cTSet << "\t" << cTIs << "\t" << fNewVoltage << "\t" << cCurTemps[0] << "\t" << cCurTemps[1] << "\t" << cCurTemps[2] << "\t" << cCurTemps[3] << "\t" << fIntOfEtPrevious << "\n";
      std::cout << fTotalTime << "\t" << cTSet << "\t" << cTIs << "\t" << fNewVoltage << "\t" << cCurTemps[0] << "\t" << cCurTemps[1] << "\t" << cCurTemps[2] << "\t" << cCurTemps[3] << "\t" << fIntOfEtPrevious << std::endl;

      if ( PIDController::signum(fOldVoltage)*PIDController::signum(fNewVoltage) == -1 ) {
        if (fGCRunning) {
          fGC->setVoltage(0);
          while(fGC->getMeasuredOutputVoltage() > 00.1) {
            usleep(200000);
          }
        }
        if ( PIDController::signum(fNewVoltage) == -1 ) {
          //std::cout << "Switch Digital Pin true" << std::endl;
          fTB->switchDigitalPin(true);
        }
        else {
          //std::cout << "Switch Digital Pin false" << std::endl;
          fTB->switchDigitalPin(false);
        }
      }
      if (fGCRunning) { fGC->setVoltage(abs(fNewVoltage)); }
      fOldVoltage = fNewVoltage;
    }
    usleep(100000);
  }
  if (fGCRunning) {
    fGC->setVoltage(0);
    fGC->setOutput(false);
  }
  outputfile.close();
  std::cout << "Control loop ended!" << std::endl;
}

void PIDController::userInteraction() {
  std::string cInput = "";
  while( fRunning ) {
    std::cin >> cInput;
    if ( cInput == "exit" ) {
      fRunning = false;
    }
    if ( cInput == "resI") {
      PIDController::resetIValue();
    }
    std::size_t cFound = cInput.find("setT=");
    if ( cFound != std::string::npos ) {
      PIDController::setTSet(std::stod(cInput.substr(cFound+5,cInput.size())));
    }
  }
}

int PIDController::signum(double pValue) {
  if (pValue >= 0) {return 1;}
  else {return -1;}
}

// ###############################################################################################
// ##                                  set functions                                            ##
// ###############################################################################################

void PIDController::setTSet(double cTSet) {
  fT_Set = cTSet;
}

void PIDController::setOutput(bool pState) {
  if (fGCRunning) {
    fGC->setOutput(pState);
  }
  std::cout << "INFO: Set Output to " << pState << std::endl;
}

void PIDController::resetIValue() {
  fIntOfEtPrevious = 0;
}
