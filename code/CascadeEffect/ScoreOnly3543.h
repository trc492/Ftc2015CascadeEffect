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
                // Raise Elevator before going down the ramp
                //
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_RAMP_POS,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDELEVATOR,
                                  500);
                SMAddWaitEvent(sm, EVTTYPE_PIDELEVATOR);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 2:
                //
                // Drive backward down the ramp.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -40, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -50.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 3:
                //
                //Drive backwards more
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -15, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -24.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2500);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                //
                // Clamp down on goal with grabber.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, DRIVE_MIN_VALUE, DRIVE_MAX_VALUE);
                ServoSetAngle(g_goalGrabberServo,GOALGRABBER_DOWN );
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_MID_GOAL,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDELEVATOR,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDELEVATOR);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 5:
                //
                // Drive forward
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -30, 30);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  26.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 6:
                //
                // Release grabber and Drive backwards a bit.
                //
                ServoSetAngle(g_goalGrabberServo,GOALGRABBER_UP);
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -10, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -12.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 7:
                //
                // Clamp down with goal capture.
                //
                ServoSetAngle(g_goalCaptureServo, GOALCAPTURE_DOWN);
                TimerSet(g_timer, 1000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 8:
                //
                // Lower Drawbridge
                //
                ServoSetAngle(g_drawBridgeServo,
                              DRAWBRIDGE_RELEASE);
                TimerSet(g_timer, 2000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 9:
                //
                //Complete final turn.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, DRIVE_MIN_VALUE, DRIVE_MAX_VALUE);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  0.0,
                                  45.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  1000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 10:
                //
                //Wait to see if the ball falls in
                //
                TimerSet(g_timer, 2000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 11:
                //
                //Lower Elevator for TeleOp.
                //
                ServoSetAngle(g_drawBridgeServo,
                              DRAWBRIDGE_CAPTURE);
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_DOWN_POS,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDELEVATOR,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDELEVATOR);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoScoreOnly
