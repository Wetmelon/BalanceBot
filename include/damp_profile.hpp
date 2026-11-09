#pragma once

// Damp backend for every file in the build, firmware and SIL alike:
// ETL containers, and Damp's own float math (damp/math/trig.hpp) at run time.
#define DAMP_BACKEND_ETL
#define DAMP_MATH_BACKEND_DAMP
