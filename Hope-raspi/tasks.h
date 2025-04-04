#ifndef TASKS_H
#define TASKS_H

// #include <Arduino.h>
#include "navigation.h"
#include "encoders.h"

# define GOOD 1
# define BAD 0

class tasks
{
public:
    navigation nav; // Navigation object
    encoders enc; // Encoders object
    tasks(); // Constructor with navigation object;
    bool task1(); // IMPLEMENTED
    bool task2(); // IMPLEMENTED
    bool task3(); // IMPLEMENTED
    bool task4();
    bool task5();
    bool task6();

    bool task2nodist(); // just in case...

private:
    // task 2
    bool task2_done = false; // Flag to indicate if task 2 is done
};

void tasks::task1()
{   
    int potatoJuncs = 0;
    bool potatoFound = false;

    // iterate through all 5 rows
    for (int junc = 0; junc < 5; junc++) {
        // reset values
        potatoJuncs = 0;
        potatoFound = false;

        // move to the next row
        nav.moveTillJunction(); // Move until a junction is found
        nav.turn(-90); // Turn 90 degrees clockwise
        
        // move to the potato
        while(!potatoFound || (potatoJuncs < 3)) { // Continue moving until a potato is found or 2 junctions are crossed
            // move till potatoFound 
            potatoFound = nav.moveTillPotato(); // Move until a potato is detected. false if junction.
            if (!potatoFound) {
                potatoJuncs++; // Increment the junction count if a junction is found
            }
        } 

        // take the potato
        raspi.takePotato(); // Ask the Raspberry Pi to take the potato
        
        // go to the next junction and face
        nav.turn(180); // Turn 90 degrees anticlockwise
        for (int i = 0; i < potatoJuncs+1; i++) {
            nav.moveTillJunction(); // Move until a junction is found
        }
        nav.turn(-90); // Move straight for 150 mm
    }
    
    // go to the start of task 2
    nav.moveTillJunction(); // Move until a junction is found
}

bool tasks::task2()
{
    bool wallDone = false;
    float side = 1; // right line (should turn left)
    
    float startDist = enc.robotDistance(); // Get the initial distance from the encoders
    float turnDist = 0; // Initialize the end distance
    float tempDist; // Initialize the temporary distance
    float task2Dist = 0; // Initialize the task 2 distance

    nav.move_straight(150); // Move straight for 150 mm
    nav.turn(-90); // Turn 90 degrees clockwise

    // WITH DISTANCE MEASUREMENT BUT CANNOT CHECK TASK 2 END =(. NOW CAN. YAY.
    while (!wallDone) {
        
        // MOVE TILL THE RIGHT WALL
        task2Dist = enc.robotDistance() - startDist - turnDist; // Calculate the distance travelled

        wallDone = nav.moveTillWallTask2(task2Dist); // Move straight until wall or line is found
        if (wallDone) {
            break;
        }
        


        // CHANGE TO LEFT SIDE

        // if (foundWall) {
        // turn left
        nav.turn(90); // Turn 90 degrees anticlockwise 
        
        //  move forward
        tempDist = enc.robotDistance();
        nav.move_straight(300); // Move straight for 150 mm
        turnDist = turnDist + (enc.robotDistance() - tempDist);
        
        //turn right to original direction
        nav.turn(-90); // Turn 90 degrees clockwise
        

        

        // MOVE TILL THE LEFT WALL
        task2Dist = enc.robotDistance() - startDist - turnDist; // Calculate the distance travelled

        wallDone = nav.moveTillWallTask2(task2Dist); // Move straight until wall or line is found
        if (wallDone) {
            side = -1; // Set wall to true
            break; // Exit the loop if task 2 is done
        }



        // CHANGE TO RIGHT SIDE
        // turn right
        nav.turn(-90); // Turn 90 degrees clockwise
        
        // move forward
        tempDist = enc.robotDistance();
        nav.move_straight(300); // Move straight for 150 mm
        turnDist = turnDist + enc.robotDistance() - tempDist;
        
        // turn left to original direction
        nav.turn(90); // Turn 90 degrees clockwise
        // }
    }

    // LETS GO RAMP!!!!

    // turn right if on left line.
    turn((float)90*side); // Turn 90 degrees clockwise if on left line
    nav.move_straight(150); // Move straight for 150 mm
    turn((float)(-90)*side);
    // nav.moveTillLine(); 
    nav.moveTillWalll(); // Move straight until wall is found
    task2_done = true; // Set task 2 done to true

    return task2_done; // Return the task 2 status
}


bool tasks::task2nodist()
{
    // Task 2 implementation without distance measurement
    nav.move_straight(150); // Move straight for 150 mm
    nav.turn(-90); // Turn 90 degrees clockwise
    bool foundLine = false; // Initialize foundLine to false

    while (!task2_done) {
        // CANNOT FIND TASK 2 DONEEEEEEEEEEEEEEEEEEEEEE
        foundLine = nav.moveTillWallorLine(); // Move straight until wall or line is found
        if (!foundLine) {
            // turn left
            nav.turn(90); // Turn 90 degrees anticlockwise 
            //  move forward
            nav.move_straight(300); // Move straight for 150 mm
            //turn right to original direction
            nav.turn(-90); // Turn 90 degrees clockwise
            // move again till left wall found
            foundLine = nav.moveTillWallorLine();
            if (!foundLine) {
                task2_done = true; // Set task 2 done to true
                break; // Exit the loop if task 2 is done
            }
            // turn right
            nav.turn(-90); // Turn 90 degrees clockwise
            // move forward
            nav.move_straight(300); // Move straight for 150 mm
            // turn left to original direction
            nav.turn(90); // Turn 90 degrees clockwise
        }
        if (!foundLine) {
            task2_done = true; // Set task 2 done to true
            break; // Exit the loop if task 2 is done
        }
    }
    return task2_done; // Return the task 2 status
}


void tasks::task3()
{   
    // READ APRILTAG
   
    nav.moveTillWall();
    nav.turn(-90); // Turn 90 degrees clockwise
    nav.moveTillLine();
    nav.turn(90);
    nav.moveTillJunction(); // align also
    nav.move_straight(-150);
    nav.turn(-90);
    bool goodRed = raspi.isRedGood(); // ask raspberry to find tag

    // BASKETING POTATOES 

    // go to the basket and read tag
    nav.move_straight(-100); // reverse a bit
    nav.turn(-90);
    nav.moveTillLine(); // align also
    // nav.move_straight(300);
    nav.move_straight(-150);
    nav.turn(90); // turn towards the basket to read
    bool redBox = raspi.findBoxColour(); // ask raspberry to find tag

    // turn rear to put the potatoes
    nav.turn(180);
    nav.move_straight(-50);
    
    bool openGood = false;

    if ((goodRed && redBox)||(!goodRed && !redBox)) {
        raspi.openGate(GOOD); //ASK OSHANI ABOUT SERVO CONTROLLING
        openGood = true;
    } else {
        // goodRed && !redBox || !goodRed && redBox
        raspi.openGate(BAD);
    }

    // go to the next basket
    nav.move_straight(50);
    nav.turn(-90);
    nav.moveTillLine(); // and align

    nav.move_straight(-150); // move towards the box
    nav.turn(90); // turn rear towards the box

    // put the other set of potatoes
    if (openGood) {
        raspi.openGate(BAD);
    } else {
        raspi.openGate(GOOD);
    }
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
    int dry = 0; // Initialize dry to false
    float startDist;
    float tempDist;
    int drypot;

    // iterate through all 3 boxes
    for (int pot = 0; pot < 3; pot++) {
        nav.moveTillPotato(); // move untill a box is found
        nav.turn(90); // turn to the left
        startDist = enc.robotDistance(); // Get the initial distance from the encoders
        nav.moveTillLine();
        tempDist = enc.robotDistance() - startDist; // Calculate the distance travelled
        dry = raspi.detectDryPot();
        if (dry) {
            raspi.ledOn(); // turn on the LED
            int drypot = pot; // Set the drypot to the current potato number
            // break; // if using break, dont use drypot
        }
        nav.turn(180);
        nav.move(tempDist); // move back to the box
        nav.turn(90); // turn to the left
    }

    // go to the well and take water
    nav.turn(180);
    nav.moveTillPotato(); // move forward untill the water box
    nav.turn(90);
    nav.moveTillLine(); // move forward untill the line
    raspi.takeWater(); // ask raspberry to take the water
    nav.move_straight(-50); // move back a bit
    nav.turn(180);

    // water the dry potatoes
    nav.moveTillLine(); // go infront of the middle potato
    nav.move_straight(-50); // move back a bit
    nav.turn((float)(1-drypot)*90); //turn to the line of the drypot
    nav.moveTillLine();
    nav.turn((float)(drypot-1)*90); // turn to the drypot
    nav.moveTillLine(); // move forward untill the line
    raspi.waterPot(); // ask raspberry to water the potato

    // now tasks are finished
    raspi.ledOn(); 
    raspi.playStarman();

    // YAYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY!!!!!!!!!!!!!!!!!!!!
}


#endif // TASKS_H


