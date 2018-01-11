#include "GenesysControl.h"

GenesysControl::GenesysControl() {

  // Define settings of serial communication
  memset(&tio,0,sizeof(tio));
  tio.c_iflag = 0;
  tio.c_oflag = 0;  //Raw output
  tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1
  tio.c_lflag = 0;
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 0.5;  //.5 seconds read timeout
  cfsetispeed(&tio,B9600);            // 9600 baud input
  cfsetospeed(&tio,B9600);            // 9600 baud output
}

GenesysControl::~GenesysControl() {

}

bool GenesysControl::init(std::string pPortName, int pRemAddress) {
  if (pPortName == "") {
    std::cout << "WARNING: No port for Genesys power supply given" << std::endl;
    return false;
  }
  tty_fd=open(pPortName.c_str(), O_RDWR | O_NOCTTY);        // O_NONBLOCK might override VMIN and VTIME
  tcflush(tty_fd, TCIFLUSH);
  tcsetattr(tty_fd,TCSANOW,&tio);
  fcntl(tty_fd, F_SETFL, O_NONBLOCK);       // make the reads non-blocking (return immediately even if no byte is answered)

  std::string cCommand = "";
  char * cTemp = new char[20];
  sprintf(cTemp, "ADR %2i\r", pRemAddress);
  cCommand = cTemp;
  GenesysControl::SerialWrite(cCommand);
  std::string cAnswer = GenesysControl::SerialRead();
  if (cAnswer.compare("OK\r") != 0) {
    sprintf(cTemp, "Could not contact Genesys with address %2i. Please make sure that the address of the instrument corresponds with the software address!", pRemAddress);
    std::cout << cTemp << std::endl;
    return false;
  }
  GenesysControl::SerialWrite("IDN?\r");
  cAnswer = GenesysControl::SerialRead();
  if (cAnswer.find("LAMBDA,GEN") == std::string::npos) {
    std::cout << "Instrument is not a TDK-Lambda Genesys!" << std::endl;
    return false;
  }
  std::cout << "Port to Genesys succesfully opened!" << std::endl;

  //Clear status and reset
  GenesysControl::SerialWrite("CLS\r");
  GenesysControl::SerialRead();
  //GenesysControl::SerialWrite("RST\r");
  //GenesysControl::SerialRead();
  usleep(1000000); // sleep for 1 second
  return true;
}

bool GenesysControl::SerialWrite(std::string pCommand) {

  int cWritten = 0, cSpot = 0;
  do {
    cWritten = write( tty_fd, &pCommand[cSpot], 1 );
    cSpot += cWritten;
  } while(pCommand[cSpot-1] != '\r' && cWritten > 0);

  //cWritten = write( tty_fd, &pCommand, pCommand.length());

    return true;
}

std::string GenesysControl::SerialRead() {

  int n = 0, cSpot = 0;
  char buf = '\0';
  std::string cAnswer;

  do {
    n = read( tty_fd, &buf, 1);
    if (n > 0) {
      cAnswer.append(&buf);
    }
  } while( buf != '\r');

  if (n < 0) {
    std::cout << "Error reading: " << strerror(errno) << std::endl;
  }
  else if (n == 0) {
    std::cout << "Read nothing!" << std::endl;
  }
  return cAnswer;
}

double GenesysControl::setVoltage(double pVoltage) {
  if (pVoltage > 20 or pVoltage < 0) {
    std::cout << "ERROR: Voltage value higher than allowed maximum (20 V) or smaller than zero!" << std::endl;
    return -2;
  }
  std::string cCommand = "";
  char * cTemp = new char[20];
  sprintf(cTemp, "PV %.2f\r", pVoltage);
  cCommand = cTemp;
  GenesysControl::SerialWrite(cCommand);
  std::string cAnswer = GenesysControl::SerialRead();
  if (cAnswer.compare("OK\r") != 0) {
    std::cout << "ERROR: Setting of output voltage value did not work!" << std::endl;
    return -1;
  }
  GenesysControl::SerialWrite("PV?\r");
  cAnswer = GenesysControl::SerialRead();
  if (cAnswer.compare(cCommand.substr(3,cCommand.length()))) {
    std::cout << "ERROR: Unexpected answer of Genesys during read back of voltage value settings!" << std::endl;
    return -1;
  }
  else {
    double cDAnswer = std::stod(cAnswer);
    //printf("Voltage set to %.2f V.\n",cDAnswer);
    return cDAnswer;
  }
}

double GenesysControl::getMeasuredOutputVoltage() {
  GenesysControl::SerialWrite("MV?\r");
  std::string cAnswer = GenesysControl::SerialRead();
  return std::stod(cAnswer);
}

int GenesysControl::setCurrent(double pCurrent) {
  if (pCurrent > 38 or pCurrent < 0) {
    std::cout << "ERROR: current value higher than allowed maximum (38 A) or smaller than zero!" << std::endl;
    return -2;
  }
  std::string cCommand = "";
  char * cTemp = new char[20];
  sprintf(cTemp, "PC %.2f\r", pCurrent);
  cCommand = cTemp;
  GenesysControl::SerialWrite(cCommand);
  std::string cAnswer = GenesysControl::SerialRead();
  if (cAnswer.compare("OK\r") != 0) {
    std::cout << "ERROR: Setting of output current value did not work!" << std::endl;
    return -1;
  }
  GenesysControl::SerialWrite("PC?\r");
  cAnswer = GenesysControl::SerialRead();
  if (cAnswer.compare(cCommand.substr(3,cCommand.length()))) {
    std::cout << "ERROR: Unexpected answer of Genesys during read back of current value settings!" << std::endl;
    return -1;
  }
  else {
    double cDAnswer = std::stod(cAnswer);
    printf("Current set to %.2f V.\n",cDAnswer);
    return cDAnswer;
  }
}

double GenesysControl::getMeasuredOutputCurrent() {
  GenesysControl::SerialWrite("MC?\r");
  std::string cAnswer = GenesysControl::SerialRead();
  return std::stod(cAnswer);
}

int GenesysControl::setFoldbackProtection(bool pStatus, double pCurrent) {
  if (pStatus) {
    std::cout << "Setting current for foldback protection" << std::endl;
    if (GenesysControl::setCurrent(pCurrent) > 0) {
      GenesysControl::SerialWrite("FLD ON\r");
      std::string cAnswer = GenesysControl::SerialRead();
      if (cAnswer.compare("OK\r") != 0) {
        std::cout << "ERROR while setting foldback protection!" << std::endl;
        return -1;
      }
      else {
        std::cout << "Foldback protection is activated." << std::endl;
        return 1;
      }
    }
    else  {
      std::cout << "ERROR while setting current for foldback protection!" << std::endl;
      return -1;
    }
  }
  else {
    GenesysControl::SerialWrite("FLD OFF\r");
    std::string cAnswer = GenesysControl::SerialRead();
    if (cAnswer.compare("OK\r") != 0) {
      std::cout << "ERROR while disabling foldback protection!" << std::endl;
      return -1;
    }
    else {
      std::cout << "Foldback protection is disabled" << std::endl;
      return 0;
    }
  }

}

int GenesysControl::getOutputStatus() {
  GenesysControl::SerialWrite("OUT?\r");
  std::string cStatus = GenesysControl::SerialRead();
  if (cStatus.compare("OFF\r") == 0) {return 0;}
  else if (cStatus.compare("ON\r") == 0) {return 1;}
  else {
    std::cout << "ERROR: unexpected answer of Genesys!" << std::endl;
    return -1;
  }
}

bool GenesysControl::setOutput(bool cStatus) {
  if (cStatus) {
    GenesysControl::SerialWrite("OUT ON\r");
    GenesysControl::SerialRead();
  }
  else {
    GenesysControl::SerialWrite("OUT OFF\r");
    GenesysControl::SerialRead();
  }
  //usleep(1000000); //sleep for 1 second to allow voltage and current to ramp
  return cStatus;
}

void GenesysControl::SerialClose() {

  GenesysControl::SerialWrite("CLS\r");
  GenesysControl::SerialRead();
  GenesysControl::SerialWrite("RST\r");
  GenesysControl::SerialRead();
  close(tty_fd);
}
