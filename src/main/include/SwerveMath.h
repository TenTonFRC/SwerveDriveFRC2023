#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

using namespace std;

const float rotationVectorMultiplier = 1;   // controls how much of the vector addition is dedicated to rotation vs field movement
// 0 < x < float limit idk
const float speedLimit = 1;   // limit motor speed output
// 0 < x <= 1
const float baseWidth = 14.5;   // inches
const float baseLength = 23.25;

float relativeX = baseWidth / 2;
float relativeY = baseLength / 2;

// ***KATRINA THIS IS WHERE YOU ACTUALLY CHANGE THINGS
float xLeft = 0; //controller left joystick X
float yLeft = 0; //controller left joystick Y
float xRight = 0; //controller right joystick X (rotation)
float gyro = 0; //robot gyro reading (rad)
// ***KATRINA THIS IS WHERE YOU ACTUALLY CHANGE THINGS

float magnitude(float x, float y) // magnitude of vector
{
    return sqrt(x*x + y*y);
}

float findMax(float arr[]) // edward was mad at c++ libraries
{
    float sus = max(arr[0],arr[1]);
    for(int i=2; i<4; i++){
        sus = max(sus,arr[i]);
    }
    return sus;
}

int getDegree(float x) // Edward can't read radians for troubleshooting
{
    float a = x*180/3.141593;
    int b = static_cast<int>(a);
    return b;
}

int main()
{
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    float rotationScalar = rotationVectorMultiplier*xRight/magnitude(relativeX,relativeY);
    float posVect[2];
    if(magnitude(xLeft, yLeft)){
        posVect[0] = magnitude(xLeft,yLeft)*sin(atan2(xLeft,yLeft)-gyro);
        posVect[1] = magnitude(xLeft,yLeft)*cos(atan2(xLeft,yLeft)-gyro); // math notation of vector
    }
    else{
        posVect[0] = 0;
        posVect[1] = 0;
    }
    float frontLeft[2] = {(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    float frontRight[2] = {(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]};
    float backLeft[2] = {-(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    float backRight[2] = {-(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]}; // declare rotation vector directions and add positional

    float physFL[2] = {magnitude(frontLeft[0], frontLeft[1]), atan2(frontLeft[0], frontLeft[1])}; // converts to physics notation
    float physFR[2] = {magnitude(frontRight[0], frontRight[1]), atan2(frontRight[0], frontRight[1])};
    float physBL[2] = {magnitude(backLeft[0], backLeft[1]), atan2(backLeft[0], backLeft[1])};
    float physBR[2] = {magnitude(backRight[0], backRight[1]), atan2(backRight[0], backRight[1])};

    float magnitudes[4] = {physFL[0],physFR[0],physBL[0],physBR[0]};
    float MAX = findMax(magnitudes);
    if(MAX>1){
        physFL[0] = physFL[0]*speedLimit/MAX;
        physFR[0] = physFR[0]*speedLimit/MAX;
        physBL[0] = physBL[0]*speedLimit/MAX;
        physBR[0] = physBR[0]*speedLimit/MAX;
    }

    //*** phys[F/B][R/L] [0] = motor strength; [1] = direction relative to motor front [-pi, pi]

    // TEST IF WORK:

    printf("Front Left Mag %lf\n",physFL[0]);
    printf("Front Right Mag %lf\n",physFR[0]);
    printf("Back Left Mag %lf\n",physBL[0]);
    printf("Back Right Mag %lf\n",physBR[0]);
    printf("\n");
    printf("Front Left Arg degree: %d\n",getDegree(physFL[1]));
    printf("Front Right Arg degree: %d\n",getDegree(physFR[1]));
    printf("Back Left Arg degree: %d\n",getDegree(physBL[1]));
    printf("Back Right Arg degree: %d\n",getDegree(physBR[1]));
    return 0;
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
}