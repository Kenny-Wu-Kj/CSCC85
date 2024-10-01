/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 10 m/s at touchdown
	  * Maximum landing angle should be less than 15 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>

#include "Lander_Control.h"

#include <stdio.h>

#define THRUSTER_TYPE_FAILURE -5.0;

enum ThrusterType {
    main_thruster,
    left_thruster,
    right_thruster
};

double MAX_HORIZON_ACCEL = sqrt(pow(35.00, 2) - pow(10.00, 2));
double horizontal_acc = 0;
double vertical_acc = G_ACCEL;
//double T_angle = 0;

double T_angle(ThrusterType thruster){
  switch (thruster) {
    case main_thruster:
    // printf("atan2(%f, %f)=%f\n", vertical_acc, horizontal_acc, atan2(vertical_acc, horizontal_acc));
      return (M_PI / 2) - atan2(vertical_acc, horizontal_acc);
    case left_thruster:
      return - atan2(vertical_acc, horizontal_acc);
    case right_thruster:
      printf("T_angle: %f", (M_PI - atan2(vertical_acc, horizontal_acc)) * 180 / M_PI);
      printf("atan2(vertical_acc, horizontal_acc): %f", atan2(vertical_acc, horizontal_acc));
      return M_PI - atan2(vertical_acc, horizontal_acc);
    default:
      fprintf(stderr,"Thruster type DNE!\n");
      return THRUSTER_TYPE_FAILURE;
  }
}

double T_power(ThrusterType thruster){
    switch (thruster) {
    case main_thruster:
      return fmin(1.0, sqrt(pow(horizontal_acc, 2) + pow(vertical_acc, 2))/ 35.00);
    case left_thruster:
      return fmin(1.0, sqrt(pow(horizontal_acc, 2) + pow(vertical_acc, 2))/ 25.00);
    case right_thruster:
      return fmin(1.0, sqrt(pow(horizontal_acc, 2) + pow(vertical_acc, 2))/ 25.00);
    default:
      fprintf(stderr,"Thruster type DNE!\n");
      return THRUSTER_TYPE_FAILURE;
  }
}

// Function to check the main thruster status
void checkMainThruster(int MT_OK) {
    if (MT_OK == 1) {
        printf("Main thruster is working properly.\n");
    } else {
        printf("Main thruster is not working properly.\n");
    }
}

// Function to check the right thruster status
void checkRightThruster(int RT_OK) {
    if (RT_OK == 1) {
        printf("Right thruster is working properly.\n");
    } else {
        printf("Right thruster is not working properly.\n");
    }
}

// Function to check the left thruster status
void checkLeftThruster(int LT_OK) {
    if (LT_OK == 1) {
        printf("Left thruster is working properly.\n");
    } else {
        printf("Left thruster is not working properly.\n");
    }
}

// Function to log the position of the landing platform
void logPlatformPosition(double PLAT_X, double PLAT_Y) {
    printf("Landing platform position - X: %.2f, Y: %.2f\n", PLAT_X, PLAT_Y);
}

double angle_normalizer(double angle){
    angle = fmod(angle, 360.0);  // Modulo 360 to wrap the angle
    if (angle < 0) {
        angle += 360.0;  // Ensure the angle is positive
    }
    return angle;
}

// Exclusive Control
void Thruster_Control(ThrusterType thruster, double power){
  switch (thruster)
  {
  case main_thruster:
    Main_Thruster(power);
    Right_Thruster(0);
    Left_Thruster(0);
    break;
  case left_thruster:
    Main_Thruster(0);
    Right_Thruster(power);
    Left_Thruster(0);
    break;
  case right_thruster:
    Main_Thruster(0);
    Right_Thruster(power);
    Left_Thruster(0);
    break;
  default:
    fprintf(stderr,"Thruster type DNE!\n");
    break;
  }
}

void Rotate_to_Angle(double theta) {
    double degree = 180 * theta / M_PI;

    printf("NEXT Rotate_to_Angle=%f\n", degree);
    
    if (Angle() >= 0 && Angle() < degree)
  {
    Rotate(degree -Angle());
    
  } else if (Angle()>degree && Angle()<360){
    if(Angle() > 180 + degree){
      Rotate(360-Angle() + degree);
    }else{
      Rotate(degree-Angle());
    }
  }
}

void Lander_Control(void)
{
 /*
   This is the main control function for the lander. It attempts
   to bring the ship to the location of the landing platform
   keeping landing parameters within the acceptable limits.

   How it works:

   - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
   - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
   - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

   As noted above, this function assumes everything is working
   fine.
*/

/*************************************************
 TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
**************************************************/

 double VXlim;
 double VYlim;
 double Vx;
 double Vy;
 //Assumptions:  1. Control of rotation always reliable (Not selectable/ No substitution)
 //              2. If all three thrusters are not OK, then the module is done.
 //Control using one thruster
 //if any thruster fail

  // double MAX_HORIZON_ACCEL = sqrt(pow(35.00, 2) - pow(10.00, 2));
  // double horizontal_acc = 0;
  // double vertical_acc = 8.87;
  // double T_angle = (M_PI / 2) - atan2(vertical_acc, horizontal_acc);

  // checkMainThruster(MT_OK);
  // checkRightThruster(RT_OK);
  // checkLeftThruster(LT_OK);
  // logPlatformPosition(PLAT_X, PLAT_Y);
  
  // fprintf(stderr,"Hiii!\n");

 if (MT_OK && RT_OK && LT_OK) {
  fprintf(stderr,"Hiii!\n");
  // Set velocity limits depending on distance to platform.
  // If the module is far from the platform allow it to
  // move faster, decrease speed limits as the module
  // approaches landing. You may need to be more conservative
  // with velocity limits when things fail.
  if (fabs(Position_X()-PLAT_X)>200) VXlim=25;
  else if (fabs(Position_X()-PLAT_X)>100) VXlim=15;
  else if (fabs(Position_X()-PLAT_X)>35) VXlim=5;
  else VXlim=2;

  if (PLAT_Y-Position_Y()>200) VYlim=-20;
  else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
  else VYlim=-4;				       // limit descent velocity

  // Ensure we will be OVER the platform when we land
  if (fabs(PLAT_X-Position_X())/fabs(Velocity_X())>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y())) VYlim=0;

  // IMPORTANT NOTE: The code below assumes all components working
  // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
  // fail. More likely, you will need a set of case-based code
  // chunks, each of which works under particular failure conditions.

  // Check for rotation away from zero degrees - Rotate first,
  // use thrusters only when not rotating to avoid adding
  // velocity components along the rotation directions
  // Note that only the latest Rotate() command has any
  // effect, i.e. the rotation angle does not accumulate
  // for successive calls.

  if (Angle()>1&&Angle()<359)
  {
    if (Angle()>=180) Rotate(360-Angle());
    else Rotate(-Angle());
    return;
  }

  // Module is oriented properly, check for horizontal position
  // and set thrusters appropriately.
  if (Position_X()>PLAT_X)
  {
    // Lander is to the LEFT of the landing platform, use Right thrusters to move
    // lander to the left.
    Left_Thruster(0);	// Make sure we're not fighting ourselves here!
    if (Velocity_X()>(-VXlim)) Right_Thruster((VXlim+fmin(0,Velocity_X()))/VXlim);
    else
    {
    // Exceeded velocity limit, brake
    Right_Thruster(0);
    Left_Thruster(fabs(VXlim-Velocity_X()));
    }
  }
  else
  {
    // Lander is to the RIGHT of the landing platform, opposite from above
    Right_Thruster(0);
    if (Velocity_X()<VXlim) Left_Thruster((VXlim-fmax(0,Velocity_X()))/VXlim);
    else
    {
    Left_Thruster(0);
    Right_Thruster(fabs(VXlim-Velocity_X()));
    }
  }

  // Vertical adjustments. Basically, keep the module below the limit for
  // vertical velocity and allow for continuous descent. We trust
  // Safety_Override() to save us from crashing with the ground.
  if (Velocity_Y()<VYlim) Main_Thruster(1.0);
  else Main_Thruster(0);

 } else if (MT_OK) {

    // Left_Thruster(0);
    // Right_Thruster(0);

    // // fprintf(stderr,"Using main truster only!\n");

    // // Failure detected
    // //printf("T_angle(main_thruster)=%f,T_power(main_thruster)=%f\n", T_angle(main_thruster), T_power(main_thruster));
    // // Rotate_to_Angle(T_angle(main_thruster));
    // // Main_Thruster(T_power(main_thruster));

    // if (fabs(Position_X()-PLAT_X)>200) VXlim=15;
    // else if (fabs(Position_X()-PLAT_X)>100) VXlim=10;
    // else if (fabs(Position_X()-PLAT_X)>35) VXlim=5;
    // else VXlim=3;

    // if (PLAT_Y-Position_Y()>200) VYlim=-20;
    // else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
    // else VYlim=-4;				       // limit descent velocity

    // // Ensure we will be OVER the platform when we land
    // if (fabs(PLAT_X-Position_X())/fabs(Velocity_X())>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y())) VYlim=-1;

    // // Calculated the general 
    // if (Position_X()>PLAT_X)
    // {
    //   // lander to the left.
    //   //printf("fabs(PLAT_Y-Position_Y()) = %f\n", fabs(PLAT_Y-Position_Y()));
    //   //printf("fmax(1.0, (35/fabs(PLAT_Y-Position_Y()))) = %f\n", fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)));
    //   //printf("(VXlim+fmin(0,Velocity_X())) = %f\n", (VXlim+fmin(0,Velocity_X())));
    //   if (Velocity_X()>(-VXlim)) horizontal_acc = - fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim+fmin(0,Velocity_X()))/4;
    //   // Exceeded velocity limit, brake
    //   else horizontal_acc = 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));

    // }
    // else
    // {
    //   // Lander is to the RIGHT of the landing platform, opposite from above
    //   //printf("fabs(PLAT_Y-Position_Y()) = %f\n", fabs(PLAT_Y-Position_Y()));
    //   //printf("fmax(1.0, (35/fabs(PLAT_Y-Position_Y()))) = %f\n", fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)));
    //   //printf("(VXlim-fmax(0,Velocity_X())) = %f\n",  (VXlim-fmax(0,Velocity_X())));
    //   if (Velocity_X()<VXlim) horizontal_acc = fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim-fmax(0,Velocity_X()))/4;
    //   // Exceeded velocity limit, brake
    //   else horizontal_acc = - 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));
    // }

    // printf("fabs(PLAT_X-Position_X()) = %f\n", fabs(PLAT_X-Position_X()));
    // if (Velocity_Y()<VYlim) vertical_acc = G_ACCEL + fmax(0,VYlim-Velocity_Y());
    // else vertical_acc = G_ACCEL + fmin(0,VYlim-Velocity_Y());

    // if(fabs(PLAT_X-Position_X())<=10.0 && fabs(PLAT_Y-Position_Y())<=30.0){
    //     horizontal_acc = 0;
    //     vertical_acc = 2;
    // }

    // //printf("NEXT horizontal_acc=%f,vertical_acc=%f\n", horizontal_acc, vertical_acc);
    
    // Rotate_to_Angle(T_angle(main_thruster));
    // Main_Thruster(T_power(main_thruster));
    // //printf("After angle=%f\n", Angle());

    ThrusterType thruster;
    double power;
    double angle;

    if(MT_OK) thruster = main_thruster;
    else if(RT_OK) thruster = right_thruster;
    else if(LT_OK) thruster = left_thruster;
    else{ //Error Handler, no way to save the lander.
      Main_Thruster(1);
      Left_Thruster(1);
      Right_Thruster(1);
      fprintf(stderr,"We are going to crush!\n");
    }
    // Right_Thruster(0);
    // Left_Thruster(0);

    if (fabs(Position_X()-PLAT_X)>200) VXlim=15;
    else if (fabs(Position_X()-PLAT_X)>100) VXlim=10;
    else if (fabs(Position_X()-PLAT_X)>35) VXlim=5;
    else if (fabs(Position_X()-PLAT_X)>10) VXlim=3;
    else VXlim=0;

    if (PLAT_Y-Position_Y()>200) VYlim=-20;
    else if (PLAT_Y-Position_Y()>100) VYlim=-10;
    else if (fabs(Position_X()-PLAT_X)>60) VXlim=-4;  // These are negative because they
    else VYlim=-2;				       // limit descent velocity

    double y_d = fabs(PLAT_Y-Position_Y());
    double x_d = fabs(PLAT_X-Position_X());
    printf("fabs(PLAT_Y-Position_Y()) = %f\n", y_d);
    printf("fabs(PLAT_X-Position_X()) = %f\n", x_d);

    if(x_d<=22.0 && y_d<=30.0){
      printf("going to landing\n");
      // Main_Thruster(0);
      // Rotate_to_Angle(0);
      power = 0;
      angle = 0;

    }else{
      // Ensure we will be OVER the platform when we land
      if (fabs(PLAT_X-Position_X())/(1 + fabs(Velocity_X()))>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y()))
      {printf("we got here\n");VYlim=-1;}

      // Calculated the general 

      // Adjust VXlim
      if (Position_X()>PLAT_X)
      {
        // lander to the left.
        if (Velocity_X()>(-VXlim)) horizontal_acc = - fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim+fmin(0,Velocity_X()))/4;
        // Exceeded velocity limit, brake
        else horizontal_acc = 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));
      }
      else
      {
        // Lander is to the RIGHT of the landing platform, opposite from above
        if (Velocity_X()<VXlim) horizontal_acc = fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim-fmax(0,Velocity_X()))/4;
        // Exceeded velocity limit, brake
        else horizontal_acc = - 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));
      }

      // Adjust VYlim
      if (Velocity_Y()<VYlim) vertical_acc = G_ACCEL + fmax(0,VYlim-Velocity_Y());
      else{
        printf("winthin the VYlim\n");
        double time = (x_d)/(1 + fabs(Velocity_X()));
        double distance = y_d - 30;
        double acc = 2 * (distance - Velocity_Y() * time) / pow(time, 2);
        printf("time:%f; distance:%f; acc:%f\n", time, distance, acc);
        vertical_acc = fmax(0, (G_ACCEL - fmax(0, acc)-1));
      } 

      printf("NEXT horizontal_acc=%f,vertical_acc=%f\n", horizontal_acc, vertical_acc);

      angle = T_angle(thruster); //Rotation diff calculated
      power = T_power(thruster); //Max power diff calculated

    }

    //Thrust the result;
    Rotate_to_Angle(angle);
    Thruster_Control(thruster, power);

 } else if (RT_OK) {
    Main_Thruster(0);
    Left_Thruster(0);

    if (fabs(Position_X()-PLAT_X)>200) VXlim=15;
    else if (fabs(Position_X()-PLAT_X)>100) VXlim=10;
    else if (fabs(Position_X()-PLAT_X)>35) VXlim=5;
    else if (fabs(Position_X()-PLAT_X)>10) VXlim=3;
    else VXlim=0;

    if (PLAT_Y-Position_Y()>200) VYlim=-20;
    else if (PLAT_Y-Position_Y()>100) VYlim=-10;
    else if (fabs(Position_X()-PLAT_X)>60) VXlim=-4;  // These are negative because they
    else VYlim=-2;				       // limit descent velocity

    double y_d = fabs(PLAT_Y-Position_Y());
    double x_d = fabs(PLAT_X-Position_X());
    printf("fabs(PLAT_Y-Position_Y()) = %f\n", y_d);
    printf("fabs(PLAT_X-Position_X()) = %f\n", x_d);

    if(x_d<=22.0 && y_d<=30.0){
        printf("going to landing\n");
        Right_Thruster(0);
        Rotate_to_Angle(0);
      }else{
      // Ensure we will be OVER the platform when we land
      if (fabs(PLAT_X-Position_X())/(1 + fabs(Velocity_X()))>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y()))
      {printf("we got here\n");VYlim=-1;}

      // Calculated the general 
      if (Position_X()>PLAT_X)
      {
        // lander to the left.
        if (Velocity_X()>(-VXlim)) horizontal_acc = - fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim+fmin(0,Velocity_X()))/4;
        // Exceeded velocity limit, brake
        else horizontal_acc = 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));
      }
      else
      {
        // Lander is to the RIGHT of the landing platform, opposite from above
        if (Velocity_X()<VXlim) horizontal_acc = fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim-fmax(0,Velocity_X()))/4;
        // Exceeded velocity limit, brake
        else horizontal_acc = - 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));
      }

      if (Velocity_Y()<VYlim) vertical_acc = G_ACCEL + fmax(0,VYlim-Velocity_Y());
      else{
        printf("winthin the VYlim\n");
        double time = (x_d)/(1 + fabs(Velocity_X()));
        double distance = y_d - 30;
        double acc = 2 * (distance - Velocity_Y() * time) / pow(time, 2);
        vertical_acc = fmax(0, (G_ACCEL - fmax(0, acc)-1));
      } 

      printf("NEXT horizontal_acc=%f,vertical_acc=%f\n", horizontal_acc, vertical_acc);
    
      Rotate_to_Angle(T_angle(right_thruster));
      Right_Thruster(T_power(right_thruster));
    }
    //printf("After angle=%f\n", Angle());


 } else if (LT_OK) {
  Main_Thruster(0);
  Right_Thruster(0);

  if (fabs(Position_X()-PLAT_X)>200) VXlim=15;
  else if (fabs(Position_X()-PLAT_X)>100) VXlim=10;
  else if (fabs(Position_X()-PLAT_X)>35) VXlim=5;
  else if (fabs(Position_X()-PLAT_X)>10) VXlim=3;
  else VXlim=0;

  if (PLAT_Y-Position_Y()>200) VYlim=-20;
  else if (PLAT_Y-Position_Y()>100) VYlim=-10;
  else if (fabs(Position_X()-PLAT_X)>60) VXlim=-4;  // These are negative because they
  else VYlim=-2;				       // limit descent velocity

  double y_d = fabs(PLAT_Y-Position_Y());
  double x_d = fabs(PLAT_X-Position_X());
  printf("fabs(PLAT_Y-Position_Y()) = %f\n", y_d);
  printf("fabs(PLAT_X-Position_X()) = %f\n", x_d);

  if(x_d<=22.0 && y_d<=30.0){
      printf("going to landing\n");
      Left_Thruster(0);
      Rotate_to_Angle(0);
    }else{
    // Ensure we will be OVER the platform when we land
    if (fabs(PLAT_X-Position_X())/(1 + fabs(Velocity_X()))>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y()))
    {printf("we got here\n");VYlim=-1;}

    // Calculated the general 
    if (Position_X()>PLAT_X)
    {
      // lander to the left.
      if (Velocity_X()>(-VXlim)) 
        horizontal_acc = - fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim+fmin(0,Velocity_X()))/4;
      // Exceeded velocity limit, brake
      else horizontal_acc = 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));
    }
    else
    {
      // Lander is to the RIGHT of the landing platform, opposite from above
      if (Velocity_X()<VXlim) 
        horizontal_acc = fmax(1.0, pow((150/fabs(PLAT_Y-Position_Y())), 1.5)) * (VXlim-fmax(0,Velocity_X()))/4;
      // Exceeded velocity limit, brake
      else horizontal_acc = - 2 * fabs(VXlim-Velocity_X()) * fmax(1.0, sqrt(VXlim)) * fmax(1.0, (100/fabs(PLAT_Y-Position_Y()))) * fmax(1.0, (35/fabs(PLAT_X-Position_X())));
    }

    if (Velocity_Y()<VYlim) vertical_acc = G_ACCEL + fmax(0,VYlim-Velocity_Y());
    else{
      printf("winthin the VYlim\n");
      double time = (x_d)/(1 + fabs(Velocity_X()));
      double distance = y_d - 30;
      double acc = 2 * (distance - Velocity_Y() * time) / pow(time, 2);
      vertical_acc = fmax(0, (G_ACCEL - fmax(0, acc)-1));
    } 

    printf("NEXT horizontal_acc=%f,vertical_acc=%f\n", horizontal_acc, vertical_acc);
  
    Rotate_to_Angle(T_angle(left_thruster));
    Left_Thruster(T_power(left_thruster));
  }

 } else {
  Main_Thruster(1);
  Left_Thruster(1);
  Right_Thruster(1);
  fprintf(stderr,"We are going to crush!\n");
 }
}

void Safety_Override(void)
{
 /*
   This function is intended to keep the lander from
   crashing. It checks the sonar distance array,
   if the distance to nearby solid surfaces and
   uses thrusters to maintain a safe distance from
   the ground unless the ground happens to be the
   landing platform.

   Additionally, it enforces a maximum speed limit
   which when breached triggers an emergency brake
   operation.
 */

/**************************************************
 TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
**************************************************/

/**************************************************
  How this works:
  Check the sonar readings, for each sonar
  reading that is below a minimum safety threshold
  AND in the general direction of motion AND
  not corresponding to the landing platform,
  carry out speed corrections using the thrusters
**************************************************/
if ((MT_OK && RT_OK && LT_OK)) {
 double DistLimit;
 double Vmag;
 double dmin;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=Velocity_X()*Velocity_X();
 Vmag+=Velocity_Y()*Velocity_Y();

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 if (Velocity_X()>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?
 if (dmin<DistLimit*fmax(.25,fmin(fabs(Velocity_X())/5.0,1)))
 { // Too close to a surface in the horizontal direction
  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }

  if (Velocity_X()>0){
   Right_Thruster(1.0);
   Left_Thruster(0.0);
  }
  else
  {
   Left_Thruster(1.0);
   Right_Thruster(0.0);
  }
 }

 // Vertical direction
 dmin=1000000;
 if (Velocity_Y()>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if (Angle()>1||Angle()>359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }
  if (Velocity_Y()>2.0){
   Main_Thruster(0.0);
  }
  else
  {
   Main_Thruster(1.0);
  }
 }
 }
}
