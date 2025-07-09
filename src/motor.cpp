#include "../lib/motor.h"
#include <iostream>

Motor::Motor() : voltage(0) {
    std::cout << "Motor created.\n";
}

Motor::~Motor() {
    std::cout << "Motor destroyed.\n";
}

void Motor::setSpeed() {

}

void Motor::start() {
    
}

void Motor::stop() {

}
