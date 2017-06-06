#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="CommonCode.h" />
///
/// <summary>
///     This module contains code common to both Autonomous and TeleOp modes.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

//
// Program switches.
//

// Common switches.
//#define _COMPETITION_ENABLED
#ifdef _TEAM_6541
#define _ENABLE_CENTERGOAL
#endif

// TeleOp switches.

// Debug switches.
#ifndef _COMPETITION_ENABLED
#define _ENABLE_TESTMODES
#define _DEBUG_ELEVATOR
//#define _DEBUG_PICKUP
//#define _DEBUG_DRIVE
#endif

//
// Include libraries.
//

// Include RobotC drivers.
#include "..\RobotCDrivers\drivers\hitechnic-sensormux.h"
//#include "..\RobotCDrivers\drivers\lego-ultrasound.h"

// Include FTC library.
#include "..\ftclib\batt.h"
#include "..\ftclib\sm.h"
#include "..\ftclib\servo.h"
#include "..\ftclib\touch.h"
#include "..\ftclib\drive.h"
#include "..\ftclib\pidctrl.h"
#include "..\ftclib\pidmotor.h"

// Include local files.
#include "RobotInfo.h"

//
// Constants.
//

//
// Global data.
//

// Drive susbsystem.
DRIVE           g_drive;

// Elevator subsystem.
TOUCH           g_elevatorLimitSW;
PIDCTRL         g_elevatorPidCtrl;
PIDMOTOR        g_elevatorMotor;
// Goal Capture subsystem.
SERVO           g_goalCaptureServo;
#ifdef _TEAM_3543
//Goal Grabber subsystem
SERVO           g_goalGrabberServo;
#endif
// DrawBridge Subsystem.
SERVO           g_drawBridgeServo;
// Miscellaneous.
BATT            g_batt;

//
// Callback functions.
//
float PIDCtrlModeGetInput(PIDCTRL &pidCtrl);
float PIDCtrlGetInput(PIDCTRL &pidCtrl)
{
    float inputValue = 0.0;

    if (IsSameReference(pidCtrl, g_elevatorPidCtrl))
    {
        inputValue = nMotorEncoder[elevatorMotor]/ELEVATOR_CLICKS_PER_INCH;
    }
    else
    {
        inputValue = PIDCtrlModeGetInput(pidCtrl);
    }

    return inputValue;
}   //PIDCtrlGetInput

void TouchEvent(TOUCH &touch, bool fActive)
{
    if (IsSameReference(touch, g_elevatorLimitSW))
    {
        if (fActive)
        {
            //
            // We reached the bottom, stop the motor.
            //
            PIDMotorReset(g_elevatorMotor);
            PlayImmediateTone(440, 15);
        }
        else
        {
            PlayImmediateTone(880, 15);
        }
        //
        // Reset the encoder when engaging or disengaging the touch sensor.
        // This will compensate the small amount of free play on the chain
        // where the motor is winding or unwinding the chain while the elevator
        // is not yet moving.
        //
        nMotorEncoder[elevatorMotor] = 0;
    }
}   //TouchEvent

//
// Main functions.
//

/**
 *  This function is called once globally to initialize the robot. Typically,
 *  this function initializes all the sensors and robot subsystems as well as
 *  global data that are common to both Autonomous and TeleOp modes.
 */
void
CommonInit()
{
    //
    // Initialize Drive subsystem.
    //

    DriveInit(g_drive,
              frontLeftMotor, frontRightMotor,
              rearLeftMotor, rearRightMotor,
              DRIVEO_FRONT_ENCODERS
             );
    //
    // Initialize Elevator subsystem.
    //
    TouchInit(g_elevatorLimitSW, elevatorLimitSW);
    PIDCtrlInit(g_elevatorPidCtrl,
                ELEVATOR_KP,
                ELEVATOR_KI,
                ELEVATOR_KD,
                ELEVATOR_TOLERANCE,
                ELEVATOR_SETTLING,
                PIDCTRLO_ABS_SETPT);
    PIDMotorInit(g_elevatorMotor,
                 elevatorMotor,
                 g_elevatorPidCtrl,
                 0,
                 &g_elevatorLimitSW);
    //Goal Capture initialization
    ServoInit(g_goalCaptureServo,
              goalCaptureServo
              );
#ifdef _TEAM_3543
    //Goal Grabber initialization
    ServoInit(g_goalGrabberServo,
              goalGrabberServo);
#endif
    // DrawBridge initialization
    ServoInit(g_drawBridgeServo,
              drawBridgeServo
              );

    //
    // Initialize miscellaneous.
    //
    BattInit(g_batt, 5, true);
}   //CommonInit

//
// Other periodic tasks.
//

/**
 *  This function is called periodically to perform tasks common to both
 *  Autonomous and TeleOp modes that require high resolution (e.g. gyro
 *  integration).
 */
void CommonHiFreqTasks()
{
}   //CommonHiFreqTasks

/**
 *  This function is called periodically to perform tasks related to sensors
 *  and inputs that are common to both Autonomous and TeleOp modes.
 */
void CommonInputTasks()
{
    BattTask(g_batt);
    TouchTask(g_elevatorLimitSW);
}   //CommonInputTasks

/**
 *  This function is called periodically to perform tasks related to motors
 *  and other outputs that are common to both Autonomous and TeleOp modes.
 */
void CommonOutputTasks()
{
    PIDMotorTask(g_elevatorMotor);
    ServoTask(g_drawBridgeServo);
#ifdef _TEAM_3543
    ServoTask(g_goalGrabberServo);
#endif
    ServoTask(g_goalCaptureServo);
    DriveTask(g_drive);
}   //CommonOutputTasks
