#pragma once

#include <Arduino.h>

#define GRID_HEIGHT 1000
#define GRID_WIDTH 1000
#define CELL_SIZE 250


namespace mtrn3100 {
  class EncoderOdometry {
    public:
      EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

      void update(float leftValue,float rightValue) {
        float tL = leftValue - lastLPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 
        float tR = -(rightValue - lastRPos); // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 

        float dS = (R * tL + R * tR) / 2.0; // Average displacement of both wheels
        float dH = (-R * tL + R * tR) / (L); // Change in heading 

        x += dS * cos(h + (dH / 2));
        y += dS * sin(h + (dH / 2));
        h += dH;

        if (h > M_PI) h -= (2 * M_PI);
        else if (h < M_PI) h+= (2 * M_PI);

        lastLPos = leftValue;
        lastRPos = rightValue;
      }

      float getX() const { return x; }
      float getY() const { return y; }
      float getH() const { return h; }

    private:
        float x, y, h;
        const float R, L;
        float lastLPos, lastRPos;
  };

}