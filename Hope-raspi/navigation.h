#ifndef NAVIGATION_H
#define NAVIGATION_H

class navigation
{
private:
    /* data */
public:
    navigation(/* args */);
    // ~navigation();
    void move_straight(float dist_mm); // distance in mm, forward +, backward -
    void turn(float angle_deg); // angle degree, anticlkwise +
    bool move_straight_till_wall_or_line();
    bool moveTillWall(float dist_mm); // distance in mm, forward +, backward -
};

bool navigation::moveTillWall(float task2Dist)
{
    if (task2Dist>1200){
        return true;
    } else {
        return false;
    }
}

#endif // NAVIGATION_H