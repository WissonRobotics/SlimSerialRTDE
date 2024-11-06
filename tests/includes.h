#pragma once

#if defined(__GNUC__) && __GNUC__ == 12
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"  // Workaround for GCC 12
#endif
#include <catch2/catch_all.hpp>
#if defined(__GNUC__) && __GNUC__ == 12
    #pragma GCC diagnostic pop
#endif

#include "utils.h"
#include <chrono>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <stdlib.h>
 