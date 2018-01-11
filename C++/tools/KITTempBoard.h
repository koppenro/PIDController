#ifndef KITTEMPBOARD_H
#define KITTEMPBOARD_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <iostream>
#include <vector>

class KITTempBoard
{
  private:
    struct termios tio;
    int tty_fd;
    bool SerialWrite(std::string pCommand);
    std::string SerialRead();

  public:
    std::vector<double> fCurrentTemp;
    KITTempBoard();
    ~KITTempBoard();
    bool init(std::string pPortName = "/dev/ttyUSB0");
    void setPTConfig(std::string hexDec);
    void getStatus();
    std::vector<double> getTemperature();
    void switchDigitalPin(bool pStatus);
    void internalCalibration();
};

#endif // KITTEMPBOARD_H
