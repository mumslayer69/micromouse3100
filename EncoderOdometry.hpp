#pragma once

#include <Arduino.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {
        float tL = leftValue - lastLPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 
        float tR = -(rightValue - lastRPos); // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 

        float dS = (R * tL + R * tR) / 2.0;
        float dH = (-R * tL + R * tR) / (L);

        x += dS * cos(h + dH / 2);
        y += dS * sin(h + dH / 2);
        h += dH;

        lastLPos = leftValue;
        lastRPos = rightValue;

        // float tL = leftValue - lastLPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 
        // float tR = rightValue - lastRPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 

        // x += 0;
        // y += 0;
        // h += 0;
        
        // lastLPos = 0; 
        // lastRPos = 0;
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
