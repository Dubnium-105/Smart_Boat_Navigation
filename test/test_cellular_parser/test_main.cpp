#include <unity.h>

#include "cellular_parser.h"

void test_parse_cereg_home_registered() {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(cellular::CeregStat::HomeRegistered),
                        static_cast<int>(cellular::parseCeregStat("+CEREG: 0,1")));
}

void test_parse_cereg_roaming_registered() {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(cellular::CeregStat::RoamingRegistered),
                        static_cast<int>(cellular::parseCeregStat("+CEREG: 2,5,\"1234\"")));
}

void test_parse_cereg_unknown_for_error() {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(cellular::CeregStat::Unknown),
                        static_cast<int>(cellular::parseCeregStat("ERROR")));
}

void test_is_registered() {
  TEST_ASSERT_TRUE(cellular::isRegistered(cellular::CeregStat::HomeRegistered));
  TEST_ASSERT_TRUE(cellular::isRegistered(cellular::CeregStat::RoamingRegistered));
  TEST_ASSERT_FALSE(cellular::isRegistered(cellular::CeregStat::NotRegistered));
}

void test_parse_signal_quality() {
  TEST_ASSERT_EQUAL_INT(15, cellular::parseSignalQuality("+CSQ: 15,99"));
  TEST_ASSERT_EQUAL_INT(-1, cellular::parseSignalQuality("+CSQ: 99,99"));
  TEST_ASSERT_EQUAL_INT(-1, cellular::parseSignalQuality("malformed"));
}

void test_parse_cgatt() {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(cellular::CgattStat::Attached),
                        static_cast<int>(cellular::parseCgattStat("+CGATT: 1")));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(cellular::CgattStat::Detached),
                        static_cast<int>(cellular::parseCgattStat("+CGATT: 0")));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(cellular::CgattStat::Unknown),
                        static_cast<int>(cellular::parseCgattStat("ERROR")));
}

void test_parse_operator_name() {
  TEST_ASSERT_EQUAL_STRING("CMCC",
                           cellular::parseOperatorName("+COPS: 0,0,\"CMCC\",7").c_str());
  TEST_ASSERT_EQUAL_STRING("", cellular::parseOperatorName("+COPS: 0").c_str());
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_parse_cereg_home_registered);
  RUN_TEST(test_parse_cereg_roaming_registered);
  RUN_TEST(test_parse_cereg_unknown_for_error);
  RUN_TEST(test_is_registered);
  RUN_TEST(test_parse_signal_quality);
  RUN_TEST(test_parse_cgatt);
  RUN_TEST(test_parse_operator_name);

  return UNITY_END();
}
