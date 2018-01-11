#include "KITTempBoard.h"

KITTempBoard::KITTempBoard() {

  //class variables
  fCurrentTemp = {-266.,-266.,-266.,-266.};

  // Define settings of serial communication
  memset(&tio,0,sizeof(tio));
  tio.c_iflag = 0;
  tio.c_oflag = 0;  //Raw output
  tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1
  tio.c_lflag &= ~ICANON; // Set non-canonical mode (before = 0)
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 50;  // 5 seconds read timeout
  cfsetispeed(&tio,B19200);            // 19200 baud input
  cfsetospeed(&tio,B19200);            // 19200 baud output

}

KITTempBoard::~KITTempBoard() {

}

bool KITTempBoard::init(std::string pPortName) {
  if (pPortName == "") {
    std::cout << "WARNING: No port for KIT Temperature Board given" << std::endl;
    return false;
  }
  tty_fd=open(pPortName.c_str(), O_RDWR);        // O_NONBLOCK might override VMIN and VTIME
  if ( tty_fd < 0 ) {
    std::cout << "ERROR: Could not open serial connection to Temperature Board via '" << pPortName << "''" << std::endl;
    return false;
  }
  tcflush(tty_fd, TCIFLUSH);
  tcsetattr(tty_fd,TCSANOW,&tio);
  fcntl(tty_fd, F_SETFL, O_NONBLOCK);       // make the reads non-blocking (return immediately even if no byte is answered)

  KITTempBoard::SerialWrite("N");
  KITTempBoard::SerialWrite("O");
  KITTempBoard::SerialWrite("Z");
  std::cout << "Test 1" << std::endl;
  std::string cAnswer = KITTempBoard::SerialRead();
  std::cout << "Test" << std::endl;
  if (cAnswer.substr(0,18).compare("Sensor Settings: ")!= 1 ) {   // card is after power cycle in continuous readout mode
    cAnswer = KITTempBoard::SerialRead();
  }
  std::cout << cAnswer << std::endl;
  std::string cAnswer2 = KITTempBoard::SerialRead();
  std::cout << cAnswer2 << std::endl;
  if (cAnswer.substr(0,18).compare("Sensor Settings: ") != 1) {
    std::cout << "FALSE" << std::endl;
    return false;
  }
  return true;
}

bool KITTempBoard::SerialWrite(std::string pCommand) {

  int cWritten = 0;
  cWritten = write( tty_fd, pCommand.c_str(), pCommand.length());

  return true;
}

std::string KITTempBoard::SerialRead() {

  int n = 0, cSpot = 0;
  char buf = '\0';
  std::string cAnswer;

  do {
    n = read( tty_fd, &buf, 1);
    if (n > 0) {
      cAnswer.append(&buf);
    }
  } while( buf != '\n');
  cAnswer.erase(cAnswer.length()-1, cAnswer.length());  //delete '\n' from answer

  if (n < 0) {
    std::cout << "Error reading: " << strerror(errno) << std::endl;
  }
  else if (n == 0) {
    std::cout << "Read nothing!" << std::endl;
  }
  return cAnswer;
}

void KITTempBoard::setPTConfig(std::string hexDec) {

  //Settings in hex:
  // PT100: 0 | PT1000: 1
  // PT1 | PT2 | PT3 | PT4
  //  1     0     1     0
  // reversed order: 0 1 0 1 = 5
  std::string cCommand = "C";
  cCommand.append(hexDec);
  std::cout << cCommand << std::endl;
  KITTempBoard::SerialWrite(cCommand);
  KITTempBoard::SerialWrite("O");   // set request mode
  usleep(5000000);
  KITTempBoard::SerialRead();
  //KITTempBoard::SerialRead();
  //KITTempBoard::SerialRead();
  usleep(1000000);

}

void KITTempBoard::getStatus() {

  KITTempBoard::SerialWrite("Z");
  std::cout << KITTempBoard::SerialRead() << std::endl;
  std::cout << KITTempBoard::SerialRead() << std::endl;
}

std::vector<double> KITTempBoard::getTemperature() {
  fCurrentTemp.erase(fCurrentTemp.begin(),fCurrentTemp.end());
  KITTempBoard::SerialWrite("R");
  std::string cAnswer = KITTempBoard::SerialRead();
  //std::cout << cAnswer << std::endl;
  int cPosition = cAnswer.find("\t");
  std::string substr = "";
  //int counter = 0;
  while(cPosition != std::string::npos) {
    fCurrentTemp.push_back(stod(cAnswer.substr(0, cPosition)));
    cAnswer.erase(0,cPosition+1);
    cPosition = cAnswer.find("\t");
    //counter++;
  }
  return fCurrentTemp;
}

void KITTempBoard::switchDigitalPin(bool pStatus) {

  if (pStatus == true) {
    KITTempBoard::SerialWrite("S");   // switch digital pin on
  }
  else {
    KITTempBoard::SerialWrite("T");   // switch digital pin off
  }
}

void KITTempBoard::internalCalibration() {

  KITTempBoard::SerialWrite("X");
  std::string cAnswer = KITTempBoard::SerialRead();
  std::cout << cAnswer << std::endl;
}
