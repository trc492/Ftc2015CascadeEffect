#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TestMode.h" />
///
/// <summary>
///     This module contains the test mode code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifdef _ENABLE_TESTMODES

//
// Constants.
//
#define STRATEGY_TEST_MODE      100

#define TESTMODE_TIMED_DRIVE    0
#define TESTMODE_DRIVE_4FT      1
#define TESTMODE_TURN_360       2
#define TESTMODE_TEST_IR        5
/*
#define TESTMODE_TEST_IRDRIVE   6
#define TESTMODE_TEST_SONAR     7
*/

//
// Local data.
//
TIMER   m_testModeTimer;
SM      m_testModeSM;

MENU    m_testModeMenu;
int     m_testMode = TESTMODE_TIMED_DRIVE;

/**
 *  This function initializes the Test Mode menu.
 */
void
TestModeInit(
    void
    )
{
    TimerInit(m_testModeTimer);
    SMInit(m_testModeSM);

    //
    // Initialize TestMode menu.
    //
    MenuInit(m_testModeMenu, "Test modes:");
    MenuAddChoice(m_testModeMenu, "Timed Drive", (float)TESTMODE_TIMED_DRIVE);
    MenuAddChoice(m_testModeMenu, "Drive 4ft", (float)TESTMODE_DRIVE_4FT);
    MenuAddChoice(m_testModeMenu, "Turn 360", (float)TESTMODE_TURN_360);
    MenuAddChoice(m_testModeMenu, "Test IR Sensors", (float)TESTMODE_TEST_IR);
    /*
    MenuAddChoice(m_testModeMenu, "Test IR Drive", (float)TESTMODE_TEST_IRDRIVE);
    MenuAddChoice(m_testModeMenu, "Test Sonar", (float)TESTMODE_TEST_SONAR);
    MenuAddChoice(m_testModeMenu, "Test Color", (float)TESTMODE_TEST_COLOR);
#ifdef _USE_RADAR
    MenuAddChoice(m_testModeMenu, "Test Radar", (float)TESTMODE_TEST_RADAR);
#endif
    */

    m_testMode = (int)MenuGetChoice(m_testModeMenu);
    SMStart(m_testModeSM);
}   //TestModeInit

/**
 *  This function drives the robot forward for a fixed period of time.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTimedDrive(
    SM &sm
    )
{
    nxtDisplayTextLine(1, "FL=%d,FR=%d",
                       nMotorEncoder[frontLeftMotor], nMotorEncoder[frontRightMotor]);
    nxtDisplayTextLine(2, "Enc=%f",
                       (nMotorEncoder[frontLeftMotor] +
                        nMotorEncoder[frontRightMotor])/2.0);
    nxtDisplayTextLine(3, "yPos=%f", DriveGetYPos(g_drive));
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Drive forward slowly for 3 seconds.
                //
                nMotorEncoder[frontLeftMotor] = 0;
                nMotorEncoder[frontRightMotor] = 0;
                DriveTank(g_drive, 20, 20);
                TimerSet(m_testModeTimer, 3000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                DriveTank(g_drive, 0, 0);
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_encoderDrivePidCtrl);
        PIDCtrlDebugInfo(g_gyroTurnPidCtrl);
    }
}   //DoTimedDrive

/**
 *  This function drives the robot forward for 4 feet.
 *
 *  @param sm Specifies the state machine.
 */
void
DoDrive4ft(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_encoderDrivePidCtrl);
    PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

    	switch (currState)
    	{
    	    case SMSTATE_STARTED:
    	        //
    	        // Drive forward for 4 ft.
    	        //
    	        PIDDriveSetTarget(g_encoderPidDrive,
    	                          48.0, 0.0,
    	                          false,
    	                          &sm,
    	                          EVTTYPE_PIDDRIVE);
    	        SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
    	        SMWaitEvents(sm, currState + 1);
    	        break;

    	    default:
    	        SMStop(sm);
    	        PlayTone(440, 50);
    	        break;
    	}
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_encoderDrivePidCtrl);
        PIDCtrlDebugInfo(g_gyroTurnPidCtrl);
    }
}   //DoDrive4ft

/**
 *  This function turn the robot clockwise for 360 degrees.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTurn360(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Turn clockwise 90 degrees.
                //
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0, 360.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_gyroTurnPidCtrl);
    }
}   //DoTurn360

/**
 *  This function tests the IR sensors.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestIRSensors(
    SM &sm
    )
{
//    PIDCtrlDisplayInfo(1, g_sonarPidCtrl);
#ifdef _TEAM_6541
    nxtDisplayTextLine(1, "sonar=%5.1f", PIDCtrlModeGetInput(g_sonarPidCtrl));
    PIDCtrlDisplayInfo(3, g_irPidCtrl);
#endif
    nxtDisplayTextLine(2, "irL=%4.1f,irR=%4.1f",
                       IRSeekerGetACDir(g_leftIR),
                       IRSeekerGetACDir(g_rightIR));
//    nxtDisplayTextLine(1, "TestIR[%d]", SMGetState(sm));
//    nxtDisplayTextLine(2, "Elev=%5.1f", PIDCtrlGetInput(g_elevatorPidCtrl));
//    nxtDisplayTextLine(3, "L=%4.1f,R=%4.1f",
//                       IRSeekerGetACDir(g_leftIR),
//                       IRSeekerGetACDir(g_rightIR));
#ifdef _TEAM_6541
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                PIDCtrlSetPowerLimits(g_sonarPidCtrl, -30, 30);
                PIDDriveSetTarget(g_sonarIrPidDrive,
                                  12.0,//SONAR_GOAL_TARGET,
                                  IR_TARGET,
                                  true);
#if 0
                                  &sm,
                                  EVTTYPE_PIDDRIVE);
#endif
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
#endif
}   //DoTestIRSensors

#if 0
/**
 *  This function tests the IR drive.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestIRDrive(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_irPidCtrl);
    PIDCtrlDisplayInfo(3, g_sonarPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                PIDMotorSetTarget(g_armMotor,
                                  60.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDARM);
                SMAddWaitEvent(sm, EVTTYPE_PIDARM);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                PIDDriveSetTarget(g_irPidDrive,
                                  IR_TARGET,
                                  0.0,
                                  0.0,
                                  0,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }

    if (SMIsEnabled(sm))
    {
        PIDCtrlDebugInfo(g_irPidCtrl);
        PIDCtrlDebugInfo(g_sonarPidCtrl);
    }
}   //DoTestIRDrive

/**
 *  This function tests the sonar drive.
 *
 *  @param sm Specifies the state machine.
 */
void
DoTestSonar(
    SM &sm
    )
{
    PIDCtrlDisplayInfo(1, g_sonarPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                PIDMotorSetTarget(g_armMotor,
                                  60.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDARM);
                SMAddWaitEvent(sm, EVTTYPE_PIDARM);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                PIDDriveSetTarget(g_sonarPidDrive,
                                  0.0,
                                  24.0,
                                  0.0,
                                  0,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoTestSonar
#endif

/**
 *  This function executes the test mode routine.
 */
void
DoTestMode(
    void
    )
{
    switch (m_testMode)
    {
        case TESTMODE_TIMED_DRIVE:
            DoTimedDrive(m_testModeSM);
            break;

        case TESTMODE_DRIVE_4FT:
            DoDrive4ft(m_testModeSM);
            break;

        case TESTMODE_TURN_360:
            DoTurn360(m_testModeSM);
            break;

        case TESTMODE_TEST_IR:
            DoTestIRSensors(m_testModeSM);
            break;

 #if 0
        case TESTMODE_TEST_IRDRIVE:
            DoTestIRDrive(m_testModeSM);
            break;

        case TESTMODE_TEST_SONAR:
            DoTestSonar(m_testModeSM);
            break;
#endif

        default:
            TErr(("Invalid test mode %d!", m_TestMode));
            break;
    }

    TimerTask(m_testModeTimer);
}   //DoTestMode

#endif  //ifdef _ENABLE_TESTMODES
