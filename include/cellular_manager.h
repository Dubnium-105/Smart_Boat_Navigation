#ifndef CELLULAR_MANAGER_H
#define CELLULAR_MANAGER_H

#include <Arduino.h>

struct CellularStatus {
  bool moduleReady;
  bool registered;
  bool dataAttached;
  int rssi;
  String operatorName;
};

void setupCellular();
bool connectCellular();
bool cellularAutoReconnect();
bool isCellularConnected();
CellularStatus getCellularStatus();
void setCellularLogEnabled(bool enabled);

#endif  // CELLULAR_MANAGER_H