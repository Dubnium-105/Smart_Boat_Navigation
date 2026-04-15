#ifndef CELLULAR_PARSER_H
#define CELLULAR_PARSER_H

#include <string>

namespace cellular {

enum class CeregStat {
  Unknown = -1,
  NotRegistered = 0,
  HomeRegistered = 1,
  Searching = 2,
  RegistrationDenied = 3,
  UnknownState = 4,
  RoamingRegistered = 5,
};

enum class CgattStat {
  Unknown = -1,
  Detached = 0,
  Attached = 1,
};

CeregStat parseCeregStat(const std::string& line);
bool isRegistered(CeregStat stat);

int parseSignalQuality(const std::string& line);
CgattStat parseCgattStat(const std::string& line);
std::string parseOperatorName(const std::string& line);

}  // namespace cellular

#endif  // CELLULAR_PARSER_H