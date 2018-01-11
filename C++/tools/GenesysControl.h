#ifndef GENESYSCONTROL_H
#define GENESYSCONTROL_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> // needed for memset
#include <iostream>


class GenesysControl
{
  private:
    struct termios tio;
    int tty_fd;
    bool SerialWrite(std::string pCommand);
    std::string SerialRead();

  public:
    GenesysControl();
    ~GenesysControl();
    bool init(std::string pPortName = "/dev/ttyUSB0", int pRemAddress = 6);
    double setVoltage(double pVoltage);
    double getMeasuredOutputVoltage();
    int setCurrent(double pCurrent);
    double getMeasuredOutputCurrent();
    int setFoldbackProtection(bool pStatus, double pCurrent);
    int getOutputStatus();
    bool setOutput(bool cStatus);
    void SerialClose();

};

#endif // GENESYSCONTROL_H
