#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TeleOpCode.h" />
///
/// <summary>
///     This module contains the TeleOp-only functions.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

//
// Include libraries.
//

// Include RobotC drivers.
#include "..\RobotCDrivers\drivers\lego-ultrasound.h"

// Include FTC library.
#include "..\ftclib\joystick.h"

//
// Global data.
//

// Input subsystem.
JOYSTICK    g_Joystick1;
JOYSTICK    g_Joystick2;

// Elevator subsystem.
float       g_PrevElevatorTarget = 0.0;
bool        g_elevatorOverride = false;
bool        g_slowDriveOverride = false;
bool        g_slowBrushOverride = false;
bool        g_reverseDrive = false;

// Miscellaneous.

//
// Callback functions.
//
float PIDCtrlModeGetInput(PIDCTRL &pidCtrl)
{
    float inputValue = 0.0;
    return inputValue;
}   //PIDCtrlModeGetInput

/**
 *  This function handles the joystick button notification events.
 *
 *  @param joystickID Specifies the joystick the event was from.
 *  @param eventType Specifies the event type.
 *  @param eventID Specifies the event ID.
 *  @param fPressed Specifies the event is a press or a release.
 */
void
JoyBtnEvent(
    int  joystickID,
    int  eventType,
    int  eventID,
    bool fPressed
    )
{
    if (joystickID == 1)
    {
        if (eventType == JOYEVENT_BUTTON)
        {
            switch (eventID)
            {
                case Logitech_Btn1:
                    break;

                case Logitech_LB7:
                    g_reverseDrive = fPressed;
                    break;

                case Logitech_RB8:
                    g_slowDriveOverride = fPressed;
                    break;

                case Logitech_Btn9:
                    break;

                case Logitech_Btn10:
                    break;

                default:
                    break;
            }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
            {
                case TopHat_Up:
                    break;

                case TopHat_Down:
                    break;

                default:
                    break;
            }
        }
    }
    else if (joystickID == 2)
    {
        if (eventType == JOYEVENT_BUTTON)
        {
            switch (eventID)
            {
                case Logitech_Btn1:
#ifdef _TEAM_6541
                    if (fPressed)
                    {
                        PIDMotorSetTarget(g_elevatorMotor,
                                          ELEVATOR_CENTERGOAL_HEIGHT,
                                          false);
                        ServoSetAngle(g_drawBridgeServo,
                                      DRAWBRIDGE_CAPTURE);
                    }
#endif
                    break;

                case Logitech_Btn2:
                    if (fPressed)
                    {
                        PIDMotorSetTarget(g_elevatorMotor,
                                          ELEVATOR_LOW_GOAL,
                                          false);
                        ServoSetAngle(g_drawBridgeServo,
                                      DRAWBRIDGE_CAPTURE);
                    }
                    break;

                case Logitech_Btn3:
                    if (fPressed)
                    {
                        PIDMotorSetTarget(g_elevatorMotor,
                                          ELEVATOR_MID_GOAL,
                                          false);
                        ServoSetAngle(g_drawBridgeServo,
                                      DRAWBRIDGE_CAPTURE);
                    }

                    break;

                case Logitech_Btn4:
                    if (fPressed)
                    {
                        PIDMotorSetTarget(g_elevatorMotor,
                                          ELEVATOR_HIGH_GOAL,
                                          false);
                        ServoSetAngle(g_drawBridgeServo,
                                      DRAWBRIDGE_CAPTURE);

                    }
                    break;

                case Logitech_LB5:
#ifdef _TEAM_3543
                    if (fPressed)
                    {
                        ServoSetAngle(g_goalGrabberServo,
                                      GOALGRABBER_DOWN);
                    }
                    else
                    {
                        ServoSetAngle(g_goalGrabberServo,
                                      GOALGRABBER_UP);
                    }
#endif
                    break;

                case Logitech_LB7:
                    g_slowBrushOverride = fPressed;
                    break;

                case Logitech_RB8:
                    g_elevatorOverride = fPressed;
                    break;

                case Logitech_Btn9:
                    break;

                case Logitech_Btn10:
                    ServoSetAngle(g_drawBridgeServo,
                                  DRAWBRIDGE_CAPTURE);
                    PIDMotorZeroCalibrate(g_elevatorMotor,
                                          ELEVATOR_CAL_POWER,
                                          ELEVATOR_STALL_MINPOWER,
                                          ELEVATOR_STALL_TIMEOUT,
                                          false);
                    break;

                default:
                    break;
            }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
            {
                case TopHat_Up:
                    ServoSetAngle(
                        g_goalCaptureServo,
                        GOALCAPTURE_UP);
                    break;

                case TopHat_Down:
                    ServoSetAngle(
                        g_goalCaptureServo,
                        GOALCAPTURE_DOWN);
                    break;

                case TopHat_Left:
#ifdef _TEAM_3543
                    ServoSetAngle(
                        g_drawBridgeServo,
                        DRAWBRIDGE_RELEASE);
#else
                    ServoSetAngle(
                        g_drawBridgeServo,
                        (PIDCtrlGetInput(g_elevatorPidCtrl) < ELEVATOR_LOW_GOAL + 2)?
                            DRAWBRIDGE_RELEASE_LOW:
                            DRAWBRIDGE_RELEASE);
#endif
                    break;


                case TopHat_Right:
                    ServoSetAngle(
                        g_drawBridgeServo,
                        DRAWBRIDGE_CAPTURE);
                    break;

                default:
                    break;

            }
        }
    }
}   //JoyBtnEvent

//
// Main functions.
//

/**
 *  This function is called once globally to do TeleOp specific
 *  initializations.
 */
void TeleOpInit()
{
    //
    // Initialize Input subsystem.
    //
    JoystickInit(g_Joystick1, 1);
    JoystickInit(g_Joystick2, 2);
}   //TeleOpInit

/**
 *  This function is called before TeleOp mode starts.
 */
void
TeleOpStart()
{
    ServoSetAngle(g_drawBridgeServo,
                  DRAWBRIDGE_CAPTURE);
#ifdef _TEAM_3543
    ServoSetAngle(g_goalCaptureServo,
                  GOALCAPTURE_UP);
#endif
}   //TeleOpStart

/**
 *  This function is called after TeleOp mode ends.
 */
void
TeleOpStop()
{
}   //TeleOpStop

/**
 *  This function is called periodically to perform the TeleOp tasks.
 *
 *  @param time Specifies the running time since TeleOp started.
 */
void
TeleOpTasks(
    float time
    )
{
    nxtDisplayTextLine(0, "TeleOp [%5.1f]", time);

    //
    // Process Drive subsystem.
    //
    int leftPower, rightPower;
    leftPower = JoystickGetY1WithDeadband(g_Joystick1, true);
    rightPower = JoystickGetY2WithDeadband(g_Joystick1, true);
    if (g_slowDriveOverride)
    {
        leftPower /= 4;
        rightPower /= 4;
    }
    if (g_reverseDrive)
    {
        int tmp = leftPower;
        leftPower = -rightPower;
        rightPower = -tmp;
    }
    DriveTank(g_drive, leftPower, rightPower);
#ifdef _DEBUG_DRIVE
    nxtDisplayTextLine(1, "FL=%d,FR=%d",
                       nMotorEncoder[frontLeftMotor],
                       nMotorEncoder[frontRightMotor]);
    nxtDisplayTextLine(2, "YPos=%6.1f",
                        ((nMotorEncoder[frontLeftMotor] + nMotorEncoder[frontRightMotor]) / 2) / CLICKS_PER_INCH);
                       //DriveGetYPos(g_drive));
    nxtDisplayTextLine(3, "Rot=%6.1f", DriveGetRotPos(g_drive));
#endif

    //
    // Process Elevator subsystem.
    //
    int elevatorPower = JoystickGetY2WithDeadband(g_Joystick2, true);
    if (TouchGetState(g_elevatorLimitSW) && (elevatorPower < 0))
    {
        //
        // We are completely retracted, so don't retract anymore.
        //
        elevatorPower = 0;
    }
    if (g_elevatorOverride)
    {
        PIDMotorSetPower(g_elevatorMotor,
                         elevatorPower,
                         ELEVATOR_STALL_MINPOWER,
                         ELEVATOR_STALL_TIMEOUT);
#ifdef _DEBUG_ELEVATOR
        nxtDisplayTextLine(3, "EP=%4d", elevatorPower);
#endif
    }
    else
    {
        //
        // Increase deadband to make sure the elevator motor doesn't get
        // stalled when joystick is not returned to zero.
        //
        float currTarget = (elevatorPower > 0)? ELEVATOR_UPPER_LIMIT:
                           (elevatorPower < 0)? ELEVATOR_LOWER_LIMIT: -1.0;
#ifdef _DEBUG_ELEVATOR
        nxtDisplayTextLine(3, "pT=%5.1f,cT=%5.1f", g_PrevElevatorTarget, currTarget);
#endif
        if (currTarget != g_PrevElevatorTarget)
        {
            if (elevatorPower == 0)
            {
                PIDMotorReset(g_elevatorMotor);
            }
            else
            {
                PIDMotorSetTarget(g_elevatorMotor, currTarget, false);
            }
            //nxtDisplayTextLine(4, "cT=%d, p=%d", currTarget, slidePower);
            g_PrevElevatorTarget = currTarget;
        }
    }
#ifdef _DEBUG_ELEVATOR
    nxtDisplayTextLine(1, "L=%5.1f,E=%d",
                       PIDCtrlGetInput(g_elevatorPidCtrl),
                       nMotorEncoder[elevatorMotor]);
    nxtDisplayTextLine(2, "JP=%4d", elevatorPower);
#endif
    //process brush subsystem
    int brushPower = JoystickGetY1WithDeadband(g_Joystick2, true);
    if (g_slowBrushOverride)
    {
        brushPower /= 3;
    }
    motor[brushMotor] = brushPower;
//    nxtDisplayTextLine(4, "LIR=%d,RIR=%d", HTIRS2readACDir(leftIR), HTIRS2readACDir(rightIR));
//    nxtDisplayTextLine(4, "Sonar=%5.1f", USreadDist(backSonar)*INCHES_PER_CM);
}   //TeleOpTasks

//
// Other periodic tasks.
//

/**
 *  This function is called periodically to perform tasks specific to
 *  TeleOp mode that require high resolution (e.g. gyro integration).
 */
void TeleOpHiFreqTasks()
{
}   //TeleOpHiFreqTasks

/**
 *  This function is called periodically to perform tasks related to sensors
 *  and inputs that are specific to TeleOp mode.
 */
void TeleOpInputTasks()
{
    JoystickTask(g_Joystick1);
    JoystickTask(g_Joystick2);
}   //TeleOpInputTasks

/**
 *  This function is called periodically to perform tasks related to motors
 *  and other outputs that are specific to TeleOp mode.
 */
void TeleOpOutputTasks()
{
}   //TeleOpOutputTasks
