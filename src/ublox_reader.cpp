#include "ublox_reader.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

//This function extracts the numbers from the UBX message payload.
//memcpy means copy raw bytes from the buffer into your GPS struct.
//Offsets (+0, +4, +8...) are used because each value is 4 bytes long.
static int NAV_POSLLH(uint8_t *buffer, classId *gps) {
  memcpy(&gps->iTOW,   buffer + 0, 4);
  memcpy(&gps->lon,    buffer + 4, 4);
  memcpy(&gps->lat,    buffer + 8, 4);
  memcpy(&gps->height, buffer + 12, 4);
  memcpy(&gps->hMSL,   buffer + 16, 4);
  memcpy(&gps->hAcc,   buffer + 20, 4);
  memcpy(&gps->vAcc,   buffer + 24, 4);
  return 0;
}

//This function takes a line of text like: B5 62 01 02 1C 00 A1 07 3B 00 ...
//and turns it into real binary bytes [0xB5, 0x62, 0x01, 0x02, ...].
//Because UBX works in binary, not plain text.
static vector<uint8_t> hexToBytes(const string &rawHex) {
  vector<uint8_t> bytes;
  stringstream ss(rawHex);
  string token;
  while (ss >> token) {
    bytes.push_back(static_cast<uint8_t>(stoul(token, nullptr, 16)));
  }
  return bytes;
}

//This checks what kind of UBX message we got.
//buffer[2] = class → 0x01 means NAV message.
//buffer[3] = ID → 0x02 means POSLLH (Position in Lat/Lon/Height).
//If it matches, we call NAV_POSLLH with the payload part.
// Corrected UBX Class/ID check + payload offset
int decodeUBX(uint8_t *buffer, classId *gps) {
  if (buffer[2] == 0x01 && buffer[3] == 0x02) { 
    return NAV_POSLLH(buffer + 6, gps);         
  }
  return 1;
}

//Converts the raw GPS values into real-world units:
//Latitude and Longitude come in 1e-7 degrees → multiply by 1e-7 to get degrees.
//Height comes in millimeters → divide by 1000 to get meters.
GPS gpsFromData(const classId &gps) {
  GPS out;
  out.lat = gps.lat * 1e-7;
  out.lon = gps.lon * 1e-7;
  out.height = gps.height / 1000.0;
  return out;
}

//Opens the UBX hex file.
//If it can’t, it prints an error and returns two dummy {0.0, 0.0} GPS points.
pair<GPS, GPS> readUbloxFile(const string &filename) {
  ifstream file(filename);
  if (!file.is_open()) {
    cerr << "Error: cannot open file " << filename << endl;
    return {{0.0, 0.0}, {0.0, 0.0}};
  }

  // Read two UBX hexstring lines: start and goal UBX messages
  string rawStart, rawGoal;
  getline(file, rawStart);
  getline(file, rawGoal);

  //Prints out the raw hex lines to debug
  cout << "Raw UBX Start: " << rawStart << endl;
  cout << "Raw UBX Goal : " << rawGoal << endl;

  //Converts those hex text lines into real binary bytes.
  vector<uint8_t> startBytes = hexToBytes(rawStart);
  vector<uint8_t> goalBytes = hexToBytes(rawGoal);

  //Calls your decoder on each UBX message. decodeUBX checks if it’s a NAV-POSLLH message.
  // If yes, it extracts lat, lon, height into the struct classId.
  classId gpsStartData, gpsGoalData;
  decodeUBX(startBytes.data(), &gpsStartData);
  decodeUBX(goalBytes.data(), &gpsGoalData);

  //Converts the raw GPS values into human-readable units:
  //lat and lon from 1e-7 degrees → real degrees.
  //height from millimeters → meters.
  GPS startGPS = gpsFromData(gpsStartData);
  GPS goalGPS = gpsFromData(gpsGoalData);

  //Prints out the final GPS coordinates to debug
  file.close();

  return {startGPS, goalGPS};
}

