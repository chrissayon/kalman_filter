// 010-TestCase.cpp

// Let Catch provide main():
#define CATCH_CONFIG_MAIN
// #define CATCH_CONFIG_ALL_PARTS
#include "catch.hpp"
#include "kalman_filter.h"

TEST_CASE("Pass Tests")
{
    kalman_filter kalmanFilter;
    kalmanFilter.predict(1, 10);
    // kalmanFilter.covariance_matrix

    REQUIRE(8.25 == kalmanFilter.covariance_matrix(0,0));
    REQUIRE(0.25 == kalmanFilter.process_noise_matrix(0,0));
}

// Expected compact output (all assertions):
// prompt> *.exe --reporter compact --success