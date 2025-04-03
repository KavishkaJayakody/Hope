#ifndef TASKS_H
#define TASKS_H

// #include <Arduino.h>
#include "navigation.h"
#include "encoders.h"

class tasks
{
public:
    navigation nav; // Navigation object
    encoders enc; // Encoders object
    tasks(); // Constructor with navigation object;
    bool task1();
    bool task2();
    bool task3();
    bool task4();
    bool task5();
    bool task6();

private:
    // task 2
    bool task2_done = false; // Flag to indicate if task 2 is done
};

void tasks::task1()
{
    // Task 1 implementation
    
}

bool tasks::task2()
{
    bool wallRoadDone = false;
    bool foundWall = false;
    float startDist = enc.robotDistance(); // Get the initial distance from the encoders
    float turnDist = 0; // Initialize the end distance
    float tempDist; // Initialize the temporary distance
    float task2Dist = 0; // Initialize the task 2 distance

    nav.move_straight(150); // Move straight for 150 mm
    nav.turn(-90); // Turn 90 degrees clockwise

    // WITH DISTANCE MEASUREMENT BUT CANNOT CHECK TASK 2 END =(. NOW CAN. YAY.
    while (!wallRoadDone) {
        wallRoadDone = nav.moveTillWall(turnDist); // Move straight until wall or line is found
        if (wallRoadDone) {
            break;
        }
        task2Dist = enc.robotDistance() - startDist - turnDist; // Calculate the distance travelled
        if (found_wall) {
            // turn left
            nav.turn(90); // Turn 90 degrees anticlockwise 
            
            //  move forward
            tempDist = enc.robotDistance();
            nav.move_straight(300); // Move straight for 150 mm
            turnDist = turnDist + (enc.robotDistance() - tempDist);
            
            //turn right to original direction
            nav.turn(-90); // Turn 90 degrees clockwise
            
            // move again till left wall found
            wallRoadDone = nav.moveTillWall(turnDist); // Move straight until wall or line is found
            if (wallRoadDone) {
                break; // Exit the loop if task 2 is done
            }

            // turn right
            nav.turn(-90); // Turn 90 degrees clockwise
            
            // move forward
            tempDist = enc.robotDistance();
            nav.move_straight(300); // Move straight for 150 mm
            turnDist = turnDist + enc.robotDistance() - tempDist;
            
            // turn left to original direction
            nav.turn(90); // Turn 90 degrees clockwise
        }
    }

    // while (!task2_done) {
    //     // CANNOT FIND TASK 2 DONEEEEEEEEEEEEEEEEEEEEEE
    //     found_wall = nav.moveTillWall(); // Move straight until wall or line is found
    //     if (found_wall) {
    //         // turn left
    //         nav.turn(90); // Turn 90 degrees anticlockwise 
    //         //  move forward
    //         nav.move_straight(300); // Move straight for 150 mm
    //         //turn right to original direction
    //         nav.turn(-90); // Turn 90 degrees clockwise
    //         // move again till left wall found
    //         nav.moveTillWall();
    //         // turn right
    //         nav.turn(-90); // Turn 90 degrees clockwise
    //         // move forward
    //         nav.move_straight(300); // Move straight for 150 mm
    //         // turn left to original direction
    //         nav.turn(90); // Turn 90 degrees clockwise
    //     }
    // }
    return task2_done; // Return the task 2 status
}


void tasks::task3()
{
    // Task 3 implementation
}

void tasks::task4()
{
    // Task 4 implementation
}

void tasks::task5()
{
    // Task 5 implementation
}

void tasks::task6()
{
    // Task 6 implementation
}


#endif // TASKS_H


