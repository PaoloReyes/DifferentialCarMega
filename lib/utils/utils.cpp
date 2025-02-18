#include "utils.h"

RS485::RS485(HardwareSerial* serial, uint16_t read_write_enabler) {
    this->read_write_enabler = read_write_enabler;
    this->serial = serial;

    pinMode(read_write_enabler, OUTPUT);
}

String RS485::read() {
    digitalWrite(this->read_write_enabler, LOW);

    if (this->serial->available() > 0) {
        return this->serial->readString();
    }
    
    return "";
}

void RS485::write(const char* data) {
    digitalWrite(this->read_write_enabler, HIGH);

    this->serial->write(data);
    
    digitalWrite(this->read_write_enabler, LOW);
}