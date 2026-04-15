#include "cellular_parser.h"

#include <cctype>

namespace cellular {

namespace {

bool parseLastInt(const std::string& line, int* outValue) {
  if (outValue == nullptr || line.empty()) {
    return false;
  }

  int end = static_cast<int>(line.size()) - 1;
  while (end >= 0 && !std::isdigit(static_cast<unsigned char>(line[end]))) {
    end--;
  }
  if (end < 0) {
    return false;
  }

  int start = end;
  while (start >= 0 && std::isdigit(static_cast<unsigned char>(line[start]))) {
    start--;
  }
  start++;

  int value = 0;
  for (int i = start; i <= end; ++i) {
    value = value * 10 + (line[i] - '0');
  }
  *outValue = value;
  return true;
}

}  // namespace

CeregStat parseCeregStat(const std::string& line) {
  int stat = -1;
  if (!parseLastInt(line, &stat)) {
    return CeregStat::Unknown;
  }

  switch (stat) {
    case 0:
      return CeregStat::NotRegistered;
    case 1:
      return CeregStat::HomeRegistered;
    case 2:
      return CeregStat::Searching;
    case 3:
      return CeregStat::RegistrationDenied;
    case 4:
      return CeregStat::UnknownState;
    case 5:
      return CeregStat::RoamingRegistered;
    default:
      return CeregStat::Unknown;
  }
}

bool isRegistered(CeregStat stat) {
  return stat == CeregStat::HomeRegistered || stat == CeregStat::RoamingRegistered;
}

int parseSignalQuality(const std::string& line) {
  // +CSQ: <rssi>,<ber>
  std::size_t colon = line.find(':');
  if (colon == std::string::npos) {
    return -1;
  }

  std::size_t firstDigit = line.find_first_of("0123456789", colon + 1);
  if (firstDigit == std::string::npos) {
    return -1;
  }

  std::size_t pos = firstDigit;
  int rssi = 0;
  while (pos < line.size() && std::isdigit(static_cast<unsigned char>(line[pos]))) {
    rssi = rssi * 10 + (line[pos] - '0');
    pos++;
  }

  if (rssi == 99 || rssi < 0 || rssi > 31) {
    return -1;
  }
  return rssi;
}

CgattStat parseCgattStat(const std::string& line) {
  int stat = -1;
  if (!parseLastInt(line, &stat)) {
    return CgattStat::Unknown;
  }

  if (stat == 0) {
    return CgattStat::Detached;
  }
  if (stat == 1) {
    return CgattStat::Attached;
  }
  return CgattStat::Unknown;
}

std::string parseOperatorName(const std::string& line) {
  // +COPS: <mode>,<format>,"<oper>",<act>
  std::size_t firstQuote = line.find('"');
  if (firstQuote == std::string::npos) {
    return "";
  }

  std::size_t secondQuote = line.find('"', firstQuote + 1);
  if (secondQuote == std::string::npos || secondQuote <= firstQuote + 1) {
    return "";
  }

  return line.substr(firstQuote + 1, secondQuote - firstQuote - 1);
}

}  // namespace cellular