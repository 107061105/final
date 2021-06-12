#include "mbed.h"
#include "math.h"
#include "mbed.h"
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

void park(Arguments *in, Reply *out);
RPCFunction rpcpark(&park, "park");

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t1;
Timer t;

int flag = 0;

/*void ping_val(void) {
    float value;
    char buffer[20];
    // xbee.set_baud(9600);
    while(flag == 1) {
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
        sprintf(buffer, "Ping : %f\r\n", value);
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

    char buf_rpc[256], outbuf_rpc[256], buffer[20];
    FILE *devin = fdopen(&xbee, "r");
    FILE *devout = fdopen(&xbee, "w");
    flag = 0; 
    while (flag == 0) {
        memset(buf_rpc, 0, 256);
        for( int i = 0; ; i++ ) {
            char recv = fgetc(devin);
            if(recv == '\n') {
                printf("\r\n");
                break;
            }
            buf_rpc[i] = fputc(recv, devout);
        }
        RPC::call(buf_rpc, outbuf_rpc);
    }

    sprintf(buffer, "START TAG DETECT\r\n");
    xbee.write(buffer, sizeof(buffer));

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

                if (abs(dz) <= 4) {
                    if (degree <= 0 && ((abs(dx + dz / 4) <= 0.4) || abs(degree + 10 * dx) <= 8)) {
                        car.stop();
                        break;
                    } else if (degree > 0 && ((abs(dx - dz / 4) <= 0.4) || abs(degree - 10 * dx) <= 8)) {
                        car.stop();
                        break;
                    }
                    /*} else if (abs(degree - 10 * dx) <= 8 && degree >= 0) {
                        car.stop();
                        break;
                    } else if (abs(degree + 10 * dx) <= 8 && degree < 0) {
                        car.stop();
                        break;
                    }*/ else if (degree >= 0) {
                        car.follow(-7, -7);
                    } else {
                        car.follow(7, 7);
                    }
                } else {
                    if (abs(degree) <= 15) {
                        if (dx < -abs(degree / 10) || dx < -1) {
                            double speed = -1 * (30 + 1.5 * abs(dz));
                            car.follow(30, speed);
                        } else if (dx > abs(degree / 10) || dx > 1) {
                            double speed = 30 + 1.5 * abs(dz);
                            car.follow(speed, -30);
                        } else {
                            car.goStraight(30);
                        }
                    } else if (degree < 0) {
                        if (abs(dx) >= abs(dz / 2)) {
                            if (dx < 0) {
                                double speed = -1 * (30 + abs(dz));
                                car.follow(30, speed);
                            } else {
                                double speed = 30 + abs(dz);
                                car.follow(speed, -30);
                            }
                        } else car.goStraight(30);
                    } else {
                        if (abs(dx) >= abs(dz / 2)) {
                            if (dx < 0) {
                                double speed = -1 * (30 + abs(dz));
                                car.follow(30, speed);
                            } else {
                                double speed = 30 + abs(dz);
                                car.follow(speed, -30);
                            }
                        } else car.goStraight(30);
                    }
                }
                /*if (abs(dz) <= 4) {
                    if (abs(degree - 10 * dx) <= 8 && degree >= 0) {
                        car.stop();
                        break;
                    } else if (abs(degree + 10 * dx) <= 8 && degree < 0) {
                        car.stop();
                        break;
                    } else if (degree >= 0) {
                        car.follow(-10, -10);
                    } else {
                        car.follow(10, 10);
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
                }*/
            } else {
                do {
                    uart.read(recv, sizeof(recv));
                } while (recv[0] != '\r');
            }
        }
    }

    flag = 2; 

    ThisThread::sleep_for(2000ms);
    sprintf(buffer, "START LINE DETECT\r\n");
    xbee.write(buffer, sizeof(buffer));
    car.follow(20, 20);

    while(flag != 0){
        if(uart.readable()) {
            int value = 0;
            int sign = 0;
            char recv[1];
        
            uart.read(recv, sizeof(recv));
            if (recv[0] == 's' && flag == 3) {
                car.stop();
                do {
                    uart.read(recv, sizeof(recv));
                } while (recv[0] != '\r');
                sprintf(buffer, "LINE DETECT OVER\r\n");
                xbee.write(buffer, sizeof(buffer));
                break;
            } else if (recv[0] == 'f') {
                car.follow(20, 20);
                do {
                    uart.read(recv, sizeof(recv));
                } while (recv[0] != '\r');
            } else if (recv[0] == 'r') {
                flag = 3;
                do {
                    uart.read(recv, sizeof(recv));
                    if (recv[0] == '-') {
                        sign = 1;
                    } else if (recv[0] != '\r') {
                        value = 10 * value + (((int)(recv[0])) - ((int)('0')));
                    }
                } while (recv[0] != '\r');
                if (value > 10) {
                    if (sign) value = value * -1;
                    car.follow(30.0 + value / 7, -(30.0 - value / 7));
                } else {
                    if (sign) value = value * -1;
                    car.follow(35.0 + value, -(35.0 - value));
                }
            } else {
                do {
                    uart.read(recv, sizeof(recv));
                } while (recv[0] != '\r');
            }

        }
    }
}

void park(Arguments *in, Reply *out) {
    int d1 = in->getArg<double>();
    int d2 = in->getArg<double>();
    const char *direction = in->getArg<const char*>();
    char buffer[20];
    sprintf(buffer, "\r\nSTART PARKING\r\n");
    xbee.write(buffer, sizeof(buffer));

    double timed1 = (d1 + 10) / 16.5 * 1000;
    double timed2 = (d2 + 2) / 16.5 * 1000;

    if (strcmp(direction, "west") == 0) {
        car.goStraight(-100);
        ThisThread::sleep_for(timed1);
        car.stop();
        ThisThread::sleep_for(1000ms);
        car.turn(100,-0.01);
        ThisThread::sleep_for(1000ms);
        car.stop();
        ThisThread::sleep_for(1000ms);
        car.goStraight(-100);
        ThisThread::sleep_for(timed2);
        car.stop();
        ThisThread::sleep_for(2000ms);
        flag = 1;
        return;
    } else if (strcmp(direction, "east") == 0) {
        car.goStraight(-100);
        ThisThread::sleep_for(timed1);
        car.stop();
        ThisThread::sleep_for(1000ms);
        car.turn(100,0.01);
        ThisThread::sleep_for(970ms);
        car.stop();
        ThisThread::sleep_for(1000ms);
        car.goStraight(-100);
        ThisThread::sleep_for(timed2);
        car.stop();
        ThisThread::sleep_for(2000ms);
        flag = 1;
        return;
    }
    
}