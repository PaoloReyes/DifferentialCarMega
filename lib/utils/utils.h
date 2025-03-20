#ifndef UTILS_H
    #define UTILS_H

    #include <Arduino.h>

    class RS485 {
        public:
            RS485(HardwareSerial* serial, uint32_t baudrate, uint16_t read_write_enabler);
            String read();
            void write(const char* data);
        private:
            uint16_t read_write_enabler;
            HardwareSerial* serial;
    };

    typedef struct {
        double x, y;
    } container_position_t;
#endif