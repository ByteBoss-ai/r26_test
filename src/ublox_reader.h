#ifndef UBLOX_READER_H
#define UBLOX_READER_H

#include <string>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>

struct GPS {
    double lat;
    double lon;
    double height;
};

struct classId {
    uint32_t iTOW;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
};

int decodeUBX(uint8_t *buffer, classId *gps);

GPS gpsFromData(const classId &gps);

std::vector<uint8_t> hexToBytes(const std::string &rawHex);

std::pair<GPS, GPS> readUbloxFile(const std::string &filename);

#endif // UBLOX_READER_H
