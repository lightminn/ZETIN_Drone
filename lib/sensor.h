#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <string>
#include <vector>
#include <memory>

class SensorData {
public:
    SensorData(int id, const std::string& type, const std::string& location)
        : id_(id), type_(type), location_(location) {}

    int getId() const { return id_; }
    std::string getType() const { return type_; }
    std::string getLocation() const { return location_; }

private:
    int id_;
    std::string type_;
    std::string location_;
};

class SensorDataFactory {
public:
    static std::unique_ptr<SensorData> createSensorData(int id, const std::string& type, const std::string& location) {
        return std::make_unique<SensorData>(id, type, location);
    }
};

#endif // SENSOR_H