#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="ScoreOnly.h" />
///
/// <summary>
///     This module contains the implementation of the Park Goal strategy.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

/**
 *  This function drives the robot down the ramp and pulls the goal into the parking zone
 *  autonomously.
 *
 *  @param sm Specifies the state machine.
 *  @param delay Specifies the delay if any.
 *  @param option Specifies the parking option.
 */
void DoScoreOnly(SM &sm, unsigned long delay)
{
    PIDCtrlDisplayInfo(1, g_encoderDrivePidCtrl);
    PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_MID_GOAL,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDELEVATOR);
                ServoSetAngle(g_goalCaptureServo, GOALCAPTURE_UP);
                if (delay != 0)
                {
                    TimerSet(g_timer, delay, &sm, EVTTYPE_TIMER);
                    SMAddWaitEvent(sm, EVTTYPE_TIMER);
                    SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    SMSetState(sm, currState + 1);
                }
                break;

            case SMSTATE_STARTED + 1:
                //
                // Drive backward down the ramp.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -30, 30);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -65.0,//-52.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 2:
                //
                // Drive slowly towards the goal.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -15, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -40.0,//-30.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 3:
                //
                // Clamp down on goal wait for the elevator.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl,
                                      DRIVE_MIN_VALUE, DRIVE_MAX_VALUE);
                ServoSetAngle(g_goalCaptureServo,GOALCAPTURE_DOWN);
                TimerSet(g_timer, 3000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_RELEASE);
                TimerSet(g_timer, 3000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 5:
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_CAPTURE);
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_DOWN_POS,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDELEVATOR,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDELEVATOR);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoScoreOnly
