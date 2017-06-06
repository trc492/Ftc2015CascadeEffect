#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="AutoCode.h" />
///
/// <summary>
///     This module contains the Autonomous-only functions.
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
#include "..\ftclib\kalman.h"
#include "..\ftclib\gyro.h"
#include "..\ftclib\irseeker.h"
#include "..\ftclib\sensor.h"
#include "..\ftclib\timer.h"
#include "..\ftclib\menu.h"
#include "..\ftclib\piddrive.h"

//
// Constants.
//

// Event types.
#define EVTTYPE_TIMER           (EVTTYPE_NONE + 1)
#define EVTTYPE_PIDDRIVE        (EVTTYPE_NONE + 2)
#define EVTTYPE_PIDELEVATOR     (EVTTYPE_NONE + 3)

//
// Global data.
//

// Inputs
GYRO            g_gyro;
IRSEEKER        g_leftIR;
IRSEEKER        g_rightIR;
#ifdef _TEAM_6541
SENSOR          g_sideSonar;
SENSOR          g_backSonar;
#endif

// PID controls
PIDCTRL         g_encoderDrivePidCtrl;
PIDCTRL         g_gyroTurnPidCtrl;

#ifdef _ENABLE_CENTERGOAL
PIDCTRL         g_irPidCtrl;
PIDCTRL         g_sonarPidCtrl;
#endif

PIDDRIVE        g_encoderPidDrive;
#ifdef _ENABLE_CENTERGOAL
PIDDRIVE        g_sonarIrPidDrive;
PIDDRIVE        g_sonarPidDrive;
#endif

// Miscellaneous
TIMER           g_timer;
SM              g_autoSM;

// Menus
MENU            g_strategyMenu;
MENU            g_autoDelayMenu;
MENU            g_defenseDistMenu;
MENU            g_parkOppoDefenseMenu;
MENU            g_parkFarEndMenu;
#ifdef _ENABLE_CENTERGOAL
MENU            g_kickStandOptionMenu;
#endif

int             g_strategy =  STRATEGY_NOAUTO;
unsigned long   g_autoDelay = 0;
float           g_defenseDist = 0.0;
int             g_parkOppoDefense = PARK_OPPODEFENSE_NONE;
int             g_parkFarEnd = PARKFAREND_NONE;
float           g_centerGoalPos = 0;
#ifdef _ENABLE_CENTERGOAL
int             g_kickStandOption = KICKSTAND_NONE;
#endif

//
// Include autonomous strategies.
//
#include "Defense.h"
#ifdef _TEAM_3543
#include "ParkGoal3543.h"
#include "ScoreOnly3543.h"
#else
#include "ParkGoal6541.h"
#include "ScoreOnly6541.h"
#ifdef _ENABLE_CENTERGOAL
#include "CenterGoal.h"
#endif
#endif
#include "KickStand.h"
#include "TestMode.h"

//
// Callback functions.
//
float PIDCtrlModeGetInput(PIDCTRL &pidCtrl)
{
    float inputValue = 0.0;

    if (IsSameReference(pidCtrl, g_encoderDrivePidCtrl))
    {
        inputValue = DriveGetYPos(g_drive)/CLICKS_PER_INCH;
    }
    else if (IsSameReference(pidCtrl, g_gyroTurnPidCtrl))
    {
        inputValue = GyroGetHeading(g_gyro);
    }
#ifdef _ENABLE_CENTERGOAL
    else if (IsSameReference(pidCtrl, g_sonarPidCtrl))
    {
        inputValue = USreadDist(backSonar)*INCHES_PER_CM;
    }
    else if (IsSameReference(pidCtrl, g_irPidCtrl))
    {
        inputValue = IRSeekerGetACDir(g_leftIR) +
                     IRSeekerGetACDir(g_rightIR);
    }
#endif

    return inputValue;
}   //PIDCtrlModeGetInput

float SensorGetValue(SENSOR &sensor)
{
    float value = 0.0;

#ifdef _TEAM_6541
    if (IsSameReference(sensor, g_sideSonar))
    {
        value = USreadDist(sideSonar)*INCHES_PER_CM;
    }
    else if (IsSameReference(sensor, g_backSonar))
    {
        value = USreadDist(backSonar)*INCHES_PER_CM;
    }
#endif

    return value;
}   //SensorGetValue

void SensorEvent(SENSOR &sensor, int zone)
{
#ifdef _TEAM_6541
    if (IsSameReference(sensor, g_sideSonar))
    {
        if (zone == SENSORZONE_LOW)
        {
            if (PIDDriveIsEnabled(g_encoderPidDrive))
            {
                PIDDriveAbort(g_encoderPidDrive);
                nxtDisplayTextLine(4, "Sonar=%5.1f",
                                   USreadDist(sideSonar)*INCHES_PER_CM);
            }
        }
    }
    else if (IsSameReference(sensor, g_backSonar))
    {
        if (zone == SENSORZONE_LOW)
        {
            if (PIDDriveIsEnabled(g_encoderPidDrive))
            {
                PIDDriveAbort(g_encoderPidDrive);
                nxtDisplayTextLine(4, "Sonar=%5.1f",
                                   USreadDist(backSonar)*INCHES_PER_CM);
            }
        }
    }
#endif
}   //SensorEvent

//
// Main functions.
//

/**
 *  This function is called once globally to do Autonomous specific
 *  initializations.
 */
void AutoInit()
{
    // Inputs
    GyroInit(g_gyro, gyroSensor);
#ifdef _TEAM_6541
    SensorInit(g_sideSonar, sideSonar, SONAR_PARK_TARGET, SONAR_PARK_TARGET);
    SensorInit(g_backSonar, sideSonar, SONAR_GOAL_TARGET, SONAR_GOAL_TARGET);
#endif
    IRSeekerInit(g_leftIR, leftIR);
    IRSeekerInit(g_rightIR, rightIR);
    //
    // Perform elevator or scissor jack zero calibration:
    // To prevent the motor from being fried because of the touch sensor
    // failed to engage, the following code will stop the motor as soon
    // as the touch sensor is engaged or if the elevator is stuck for
    // more than the timeout period. There are a number of reasons the
    // touch sensor failed to engage. It could be a mechanical problem
    // or the touch sensor is on a MUX and we forgot to turn the MUX power
    // ON.
    //

    PIDMotorZeroCalibrate(g_elevatorMotor,
                          ELEVATOR_CAL_POWER,
                          ELEVATOR_STALL_MINPOWER,
                          ELEVATOR_STALL_TIMEOUT);


    //Drive subsystem
    PIDCtrlInit(g_encoderDrivePidCtrl,
                ENC_KP, ENC_KI, ENC_KD,
                ENC_TOLERANCE, ENC_SETTLING);
    PIDCtrlInit(g_gyroTurnPidCtrl,
                GYROTURN_KP, GYROTURN_KI, GYROTURN_KD,
                GYROTURN_TOLERANCE, GYROTURN_SETTLING);
#ifdef _ENABLE_CENTERGOAL
    PIDCtrlInit(g_irPidCtrl,
                IR_KP, IR_KI, IR_KD,
                IR_TOLERANCE, IR_SETTLING,
                PIDCTRLO_ABS_SETPT | PIDCTRLO_INVERSE_INPUT);
    PIDCtrlInit(g_sonarPidCtrl,
                SONAR_KP, SONAR_KI, SONAR_KD,
                SONAR_TOLERANCE, SONAR_SETTLING,
                (PIDCTRLO_ABS_SETPT | PIDCTRLO_INVERSE_INPUT |
                 PIDCTRLO_INVERSE_OUTPUT));
#endif

    PIDDriveInit(g_encoderPidDrive,
                 g_drive,
                 NULL,
                 &g_encoderDrivePidCtrl,
                 &g_gyroTurnPidCtrl);
#ifdef _ENABLE_CENTERGOAL
    PIDDriveInit(g_sonarIrPidDrive,
                 g_drive,
                 NULL,
                 &g_encoderDrivePidCtrl,//&g_sonarPidCtrl,
                 &g_irPidCtrl);
    PIDDriveInit(g_sonarPidDrive,
                 g_drive,
                 NULL,
                 &g_sonarPidCtrl,
                 &g_gyroTurnPidCtrl);
#endif

    ServoSetAngle(g_goalCaptureServo, GOALCAPTURE_UP);
    ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_CAPTURE);

    TimerInit(g_timer);
    SMInit(g_autoSM);

    //
    // Initialize menus.
    //

    // Strategy menu.
    MenuInit(g_strategyMenu, "Strategies:");
    MenuAddChoice(g_strategyMenu, "No Auto", (float)STRATEGY_NOAUTO);
    MenuAddChoice(g_strategyMenu, "Defense", (float)STRATEGY_DEFENSE);
    MenuAddChoice(g_strategyMenu, "Park Goal", (float)STRATEGY_PARK_GOAL);
    MenuAddChoice(g_strategyMenu, "Score Only", (float)STRATEGY_SCORE_ONLY);
    MenuAddChoice(g_strategyMenu, "Kick Stand", (float)STRATEGY_KICKSTAND);
#ifdef _ENABLE_CENTERGOAL
    MenuAddChoice(g_strategyMenu, "Center Goal", (float)STRATEGY_CENTER_GOAL);
#endif
#ifdef _ENABLE_TESTMODES
    MenuAddChoice(g_strategyMenu, "Test Mode", (float)STRATEGY_TEST_MODE);
#endif

    // Autonomous delay menu.
    MenuInit(g_autoDelayMenu, "Autonomous Delay:");
    MenuAddChoice(g_autoDelayMenu, "None", 0.0);
    MenuAddChoice(g_autoDelayMenu, "1 sec", 1000.0);
    MenuAddChoice(g_autoDelayMenu, "2 sec", 2000.0);
    MenuAddChoice(g_autoDelayMenu, "4 sec", 4000.0);
    MenuAddChoice(g_autoDelayMenu, "6 sec", 6000.0);
    MenuAddChoice(g_autoDelayMenu, "8 sec", 8000.0);
    MenuAddChoice(g_autoDelayMenu, "10 sec", 10000.0);
    MenuAddChoice(g_autoDelayMenu, "12 sec", 12000.0);

    // Defense distance menu.
    MenuInit(g_defenseDistMenu, "Defense Distance:");
    MenuAddChoice(g_defenseDistMenu, "3 ft", 36.0);
    MenuAddChoice(g_defenseDistMenu, "4 ft", 48.0);
    MenuAddChoice(g_defenseDistMenu, "6 ft", 72.0);
    MenuAddChoice(g_defenseDistMenu, "8 ft", 96.0);

    // Park Opponent Defense Menu
    MenuInit(g_parkOppoDefenseMenu, "Opponent Defense:");
    MenuAddChoice(g_parkOppoDefenseMenu, "None", PARK_OPPODEFENSE_NONE);
    MenuAddChoice(g_parkOppoDefenseMenu, "Yes", PARK_OPPODEFENSE_YES);

    // Park Far End Menu
    MenuInit(g_parkFarEndMenu, "Park Far End:");
    MenuAddChoice(g_parkFarEndMenu, "None", PARKFAREND_NONE);
    MenuAddChoice(g_parkFarEndMenu, "Return far end", PARKFAREND_YES);
#ifdef _TEAM_6541
    MenuAddChoice(g_parkFarEndMenu, "Grab tall goal", PARKFAREND_GRAB_TALLGOAL);
#endif

#ifdef _ENABLE_CENTERGOAL
    // KickStandOption Menu
    MenuInit(g_kickStandOptionMenu, "Kickstand option:");
    MenuAddChoice(g_kickStandOptionMenu, "None", KICKSTAND_NONE);
    MenuAddChoice(g_kickStandOptionMenu, "Go for it", KICKSTAND_YES);
#endif

    g_strategy  = (int)MenuGetChoice(g_strategyMenu);

#ifdef _ENABLE_TESTMODES
    if (g_strategy == STRATEGY_TEST_MODE)
    {
        TestModeInit();
    }
    else
#endif
    if (g_strategy != STRATEGY_NOAUTO)
    {
        g_autoDelay = (unsigned long)MenuGetChoice(g_autoDelayMenu);
        if (g_strategy == STRATEGY_DEFENSE)
        {
            g_defenseDist = MenuGetChoice(g_defenseDistMenu);
        }
        else if (g_strategy == STRATEGY_PARK_GOAL)
        {
            g_parkOppoDefense = MenuGetChoice(g_parkOppoDefenseMenu);
            g_parkFarEnd = MenuGetChoice(g_parkFarEndMenu);
        }
#ifdef _ENABLE_CENTERGOAL
        else if (g_strategy == STRATEGY_CENTER_GOAL)
        {
            g_kickStandOption = MenuGetChoice(g_kickStandOptionMenu);
        }
#endif
    }

    nxtDisplayCenteredTextLine(0, "%s",
#ifdef _ENABLE_TESTMODES
                               (g_strategy == STRATEGY_TEST_MODE)?
                                    MenuGetChoiceText(m_testModeMenu):
                                    MenuGetChoiceText(g_strategyMenu));
#else
                               MenuGetChoiceText(g_strategyMenu));
#endif
    nxtDisplayCenteredTextLine(3, "Wait to Start...");
}   //AutoInit


/**
 *  This function is called periodically to perform tasks specific to
 *  Autonomous mode that require high resolution (e.g. gyro integration).
 */
void AutoHiFreqTasks()
{
    GyroTask(g_gyro);
    TimerTask(g_timer);
#ifdef _TEAM_6541
    SensorTask(g_sideSonar);
    SensorTask(g_backSonar);
#endif
}   //AutoHiFreqTasks

/**
 *  This function is called periodically to perform tasks related to sensors
 *  and inputs that are specific to Autonomous mode.
 */
void AutoInputTasks()
{
    IRSeekerTask(g_leftIR);
    IRSeekerTask(g_rightIR);
}   //AutoInputTasks

/**
 *  This function is called periodically to perform tasks related to motors
 *  and other outputs that are specific to Autonomous mode.
 */
void AutoOutputTasks()
{
#ifdef _ENABLE_CENTERGOAL
    PIDDriveTask(g_sonarIrPidDrive);
    PIDDriveTask(g_sonarPidDrive);
#endif
    PIDDriveTask(g_encoderPidDrive);
}   //AutoOutputTasks

/**
 *  This function is called before Autonomous mode starts.
 */
void
AutoStart()
{
    float irDir = IRSeekerGetACDir(g_leftIR);
#ifdef _TEAM_3543
    g_centerGoalPos = irDir < 1.5? 0:
                      irDir < 2.5? 1: 2;
#else
    g_centerGoalPos = irDir < 7.5? 0:
                      irDir < 8.5? 1: 2;
//    writeDebugStreamLine("ir=%4.1f, Pos=%d", irDir, g_centerGoalPos);
#endif
    SMStart(g_autoSM);
}   //AutoStart

/**
 *  This function is called after Autonomous mode ends.
 */
void
AutoStop()
{
    SMStop(g_autoSM);
}   //AutoStop

/**
 *  This function is called periodically to perform the Autonomous tasks.
 *
 *  @param time Specifies the running time since Autonomous started.
 */
void AutoTasks(
    float time
    )
{
    static float prevTime = 0.0;
    nxtDisplayTextLine(0, "Auto: %d [%5.1f]",
                       SMGetState(g_autoSM) - SMSTATE_STARTED, time);
    if (prevTime < 30.0 && time >= 30.0)
    {
        PlaySound(soundBeepBeep);
    }
    prevTime = time;

    switch (g_strategy)
    {
        case STRATEGY_NOAUTO:
            SMStop(g_autoSM);
            break;

        case STRATEGY_DEFENSE:
            DoDefense(g_autoSM, g_autoDelay, g_defenseDist);
            break;

        case STRATEGY_PARK_GOAL:
            DoParkGoal(g_autoSM, g_autoDelay, g_parkOppoDefense, g_parkFarEnd);
            break;

        case STRATEGY_SCORE_ONLY:
            DoScoreOnly(g_autoSM, g_autoDelay);
            break;

        case STRATEGY_KICKSTAND:
            DoKickStand(g_autoSM, g_autoDelay);
            break;

#ifdef _ENABLE_CENTERGOAL
        case STRATEGY_CENTER_GOAL:
            DoCenterGoal(g_autoSM, g_autoDelay, g_kickStandOption);
            break;
#endif

#ifdef _ENABLE_TESTMODES
        case STRATEGY_TEST_MODE:
            DoTestMode();
            break;
#endif

        default:
            TErr(("You shall not pass %d!", g_strategy));
            break;
    }
}   //AutoTasks
