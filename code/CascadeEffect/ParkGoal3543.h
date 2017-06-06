#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="ParkGoal.h" />
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
 *  @param parkOppoDefense Specifies whether to anticipate opponent defense.
 *  @param parkFarEnd Specifies whether to go back to the far end of the field.
 */
void DoParkGoal(SM &sm, unsigned long delay, int parkOppoDefense, int parkFarEnd)
{
    static float startHeading = 0.0;
    static float startPos     = 0.0;
    PIDCtrlDisplayInfo(1, g_encoderDrivePidCtrl);
    //PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                startHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
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
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -40, 40);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -52.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                if (parkOppoDefense == PARK_OPPODEFENSE_YES)
                {
                    SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    SMWaitEvents(sm, currState + 2);
                }
                break;

            case SMSTATE_STARTED + 3:
                //
                // Turn towards side of field.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -25, 25);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  10.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  1000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                //
                // Drive backward slowly towards to goal.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -15, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -25.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2500);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 5:
                //
                // Clamp down on goal with grabber and raise elevator
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

            case SMSTATE_STARTED + 6:
                //
                // Drive forward
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -30, 30);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  26.0,
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
                // Release grabber and Drive backwards a bit.
                //
                ServoSetAngle(g_goalGrabberServo,GOALGRABBER_UP);
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -15, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -15.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 8:
                //
                // Clamp down with goal capture.
                //
                ServoSetAngle(g_goalCaptureServo, GOALCAPTURE_DOWN);
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, DRIVE_MIN_VALUE, DRIVE_MAX_VALUE);
                TimerSet(g_timer, 1000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 9:
                //
                // Lower Drawbridge
                //
                ServoSetAngle(g_drawBridgeServo,
                              DRAWBRIDGE_RELEASE);
                TimerSet(g_timer, 2000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 10:
                //
                // Turn towards the parking zone.
                //
                float angle = 40.0 -
                              (PIDCtrlModeGetInput(g_gyroTurnPidCtrl) -
                               startHeading);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  angle,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  1000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 11:
                //
                // Drive forward to the parking zone.
                //
                startPos = DriveGetYPos(g_drive);
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -40, 40);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  60.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  4000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 12:
                //
                // Lower Elevator
                //
                ServoSetAngle(g_drawBridgeServo,
                              DRAWBRIDGE_CAPTURE);
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_DOWN_POS,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDELEVATOR,
                                  1500);
                //
                // Turn 180.
                //
                float turnAngle = 190.0 -
                              (PIDCtrlModeGetInput(g_gyroTurnPidCtrl) -
                               startHeading);
                //nxtDisplayTextLine(4, "%5.1f %5.1f", PIDCtrlModeGetInput(g_gyroTurnPidCtrl), turnAngle);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  turnAngle,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 13:
                //
                //Drive to parking goal
                //
                float distTravel = DriveGetYPos(g_drive) - startPos;
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -15, 15);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -(26.0 - distTravel),//-(90.0 + 14.0 - distTravel),
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                if (parkFarEnd == PARKFAREND_YES)
                {
                    SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    SMWaitEvents(sm, 1000);
                }
                break;

            case SMSTATE_STARTED + 14:
                //
                // Drive forward 12 inches
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, DRIVE_MIN_VALUE, DRIVE_MAX_VALUE);
                ServoSetAngle(g_goalCaptureServo, GOALCAPTURE_UP);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  12.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  6000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 15:
                //
                // Turn a little to aim towards the tubes
                //
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  10.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 16:
                //
                // Drive forwards to the tubes
                //
                ServoSetAngle(g_goalCaptureServo, GOALCAPTURE_UP);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  78.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  6000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 17:
                //
                // Turn around
                //
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -160.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoParkGoal
