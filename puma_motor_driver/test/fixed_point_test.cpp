#include <gtest/gtest.h>
#include <iostream>
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/message.h"


class TestFixedPoint8x8 : public ::testing::Test
{
public:
  static const int16_t input[];
  static const float output[];
  static const int num_cases;
};

const float TestFixedPoint8x8::output[] = {
  0.0,
  1.0,
  -1.0,
  -0.00001,
  127.001,
  -127.002,
  123.42678
};

const int16_t TestFixedPoint8x8::input[] = {
  int16_t(0.0 * float(1<<8)),
  int16_t(1.0 * float(1<<8)),
  int16_t(-1.0 * float(1<<8)),
  int16_t(-0.00001 * float(1<<8)),
  int16_t(127.001 * float(1<<8)),
  int16_t(-127.002 * float(1<<8)),
  int16_t(123.42678 * float(1<<8))
};

const int TestFixedPoint8x8::num_cases = sizeof(TestFixedPoint8x8::output) / sizeof(float);
const static float epsilon = 1.0 / float(1<<8);

TEST(TestFixedPoint16x16, test16x16interpret)
{
  double cases[7] = {
    0.0,
    1.0,
    -1.0,
    -0.00001,
    5000.00001,
    32000.000025,
    -32000.000025
  };

  for (int i = 0; i < 7; i++)
  {
    puma_motor_driver::Driver::StatusField status;
    int32_t input = cases[i] * double(1<<16);
    memcpy(&status.data, &input, 4);
    EXPECT_NEAR(cases[i], status.interpretFixed16x16(), 1.0 / double(1<<16));
  }
}

TEST(TestFixedPoint8x8, test8x8interpret)
{
  float cases[7] = {
    0.0,
    1.0,
    -1.0,
    -0.00001,
    127.001,
    -127.002,
    123.42678
  };

  for (int i = 0; i < 7; i++)
  {
    puma_motor_driver::Driver::StatusField status;
    int16_t input = cases[i] * float(1<<8);
    memcpy(&status.data, &input, 2);
    EXPECT_NEAR(cases[i], status.interpretFixed8x8(), 1.0 / float(1<<8));
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
