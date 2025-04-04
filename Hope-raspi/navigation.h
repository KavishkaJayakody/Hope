#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "encoders.h"

class navigation
{
private:
    /* data */
public:
    navigation(/* args */);
    encoders enc;
    // ~navigation();
    void move_straight(float dist_mm); // distance in mm, forward +, backward -
    void turn(float angle_deg); // angle degree, anticlkwise +
    void moveTillWall();
    bool moveTillWallTask2(float currentDist); // distance in mm, forward +, backward -
    void moveTillLine();
};

bool navigation::moveTillWallTask2(float task2Dist)
{   
    // update this from time to time and check the condition.
    float temp = enc.robotDistance(); // Get the distance from the encoders
    task2Dist = task2Dist + enc.robotDistance() - temp; // Calculate the distance travelled
    if (task2Dist>1200){
        return true;
    } else {
        return false;
    }
}

#endif // NAVIGATION_H