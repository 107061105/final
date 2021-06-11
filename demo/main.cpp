#include "mbed.h"
#include "math.h"
#include "bbcar.h"
#include "bbcar_rpc.h"
#include "string.h"

Ticker servo_ticker;
PwmOut pin5(D5), pin6(D6);
BufferedSerial pc(USBTX,USBRX); //tx,rx
BufferedSerial uart(D1,D0); //tx,rx
//DigitalInOut ping(D12);
BufferedSerial xbee(D10, D9);
BBCar car(pin5, pin6, servo_ticker);

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t1;
Timer t;

/*void ping_val(void) {
    float value;
    char buffer[10];
    xbee.set_baud(9600);
    while(1) {
        ping.output();
        ping = 0; wait_us(200);
        ping = 1; wait_us(5);
        ping = 0; wait_us(5);

        ping.input();
        while(ping.read() == 0);
        t.start();
        while(ping.read() == 1);
        value = t.read();
        value = value * 17700.4f;
        sprintf(buffer, "%f\r\n", value);
        xbee.write(buffer, sizeof(buffer));
        //printf("%s\r\n", buffer);
        t.stop();
        t.reset();

        ThisThread::sleep_for(1s);
    }
}*/

int main(){
    uart.set_baud(9600);
    xbee.set_baud(9600);

    //t1.start(callback(&queue, &EventQueue::dispatch_forever));

    xbee.set_blocking(false);

    //xbee.sigio(mbed_event_queue()->event(ping_val));

    while(1){
        if(uart.readable()) {
            double d1 = 0;
            double d2 = 0;
            int order = 0;
            int sign = 0;
            double dx = 0;
            double dz = 0;
            double degree = 0;
            double error_dx = 0;
            char buffer[20];

            char recv[1];

            uart.read(recv, sizeof(recv));
            if (recv[0] == 'd') {
                do {
                    uart.read(recv, sizeof(recv));
                    if (recv[0] == '-') {
                        sign = 1;
                    } else if (recv[0] != '.') {
                        d1 = 10 * d1 + (((int)(recv[0])) - ((int)('0')));
                    }
                } while (recv[0] != '.');
                do {
                    uart.read(recv, sizeof(recv));
                    if (recv[0] != '\r') {
                        order = order + 1;
                        double k = ((int)(recv[0])) - ((int)('0'));
                        for (int i = 0; i < order; i++) k = 0.1 * k;
                        d2 = d2 + k;
                    }
                } while (recv[0] != '\r');
                dx = d1 + d2;
                if (sign) dx = dx * -1;
                d1 = d2 = sign = order = 0;

                do {
                    uart.read(recv, sizeof(recv));
                    if (recv[0] == '-') {
                        sign = 1;
                    } else if (recv[0] != '.') {
                        d1 = 10 * d1 + (((int)(recv[0])) - ((int)('0')));
                    }
                } while (recv[0] != '.');
                do {
                    uart.read(recv, sizeof(recv));
                    if (recv[0] != '\r') {
                        order = order + 1;
                        double k = ((int)(recv[0])) - ((int)('0'));
                        for (int i = 0; i < order; i++) k = 0.1 * k;
                        d2 = d2 + k;
                    }
                } while (recv[0] != '\r');
                dz = d1 + d2;
                dz = dz * -1;
                d1 = d2 = sign = order = 0;

                do {
                    uart.read(recv, sizeof(recv));
                    if (recv[0] == '-') {
                        sign = 1;
                    } else if (recv[0] != '.') {
                        d1 = 10 * d1 + (((int)(recv[0])) - ((int)('0')));
                    }
                } while (recv[0] != '.');
                do {
                    uart.read(recv, sizeof(recv));
                    if (recv[0] != '\r') {
                        order = order + 1;
                        double k = ((int)(recv[0])) - ((int)('0'));
                        for (int i = 0; i < order; i++) k = 0.1 * k;
                        d2 = d2 + k;
                    }
                } while (recv[0] != '\r');
                degree = d1 + d2;
                if (degree >= 270 && degree <= 360) degree = degree - 360;

                sprintf(buffer, "%.2f, %.2f, %.2f\r\n", dx, dz, degree);
                xbee.write(buffer, sizeof(buffer));

                if (abs(dz) <= 4) {
                    if (abs(degree - 10 * dx) <= 8 && degree >= 0) {
                        car.stop();
                        break;
                    } else if (abs(degree + 10 * dx) <= 8 && degree < 0) {
                        car.stop();
                        break;
                    } else if (degree >= 0) {
                        car.follow(-8, -8);
                    } else {
                        car.follow(8, 8);
                    }
                } else {
                    if (abs(dx) >= 0.6 && dz <= 10) error_dx = 5 * (abs(dx) - 0.8);
                    if (10 * abs(dx) >= (abs(degree))) {
                        if (dx < 0) {
                            double speed = 10 * (abs(dx) - abs(degree) / 4) + 10 / (dz - 5) + error_dx;
                            car.follow(30, -30 - speed);
                        } else {
                            double speed = 10 * (abs(dx) - abs(degree) / 4) + 10 / (dz - 5) + error_dx;
                            car.follow(30 + speed, -30);
                        }
                    } else {
                        car.goStraight(30);
                    }
                }
            }
        }
    }
}