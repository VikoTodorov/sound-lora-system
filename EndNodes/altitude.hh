#ifndef ALTITUDE_HH
#define ALTITUDE_HH

float altitude(const int32_t press, const float seaLevel = 1013.25) {
  static float Altitude;
  Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return Altitude;
}

#endif
