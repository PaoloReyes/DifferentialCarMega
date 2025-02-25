#include "utils.h"

RS485::RS485(HardwareSerial* serial, uint32_t baudrate, uint16_t read_write_enabler) {
    this->read_write_enabler = read_write_enabler;
    this->serial = serial;
    this->serial->begin(baudrate);

    pinMode(read_write_enabler, OUTPUT);
    digitalWrite(read_write_enabler, LOW);
}

String RS485::read() {
    digitalWrite(this->read_write_enabler, LOW);

    while (!this->serial->available()) {
        delay(1);
    }

    char buffer[10];
    this->serial->readBytes(buffer, this->serial->available());
    return String(buffer);
}

void RS485::write(const char* data) {
    digitalWrite(this->read_write_enabler, HIGH);

    this->serial->write(data);
    this->serial->flush();  

    digitalWrite(this->read_write_enabler, LOW);
}