#ifndef INITSERIAL_H
#define INITSERIAL_H
#include <serial/serial.h>
#include <memory>

std::vector<std::shared_ptr<serial::Serial>>  initSerial();
bool isLinkTrackUWBDevice(const serial::PortInfo &device);
#endif // INITSERIAL_H
