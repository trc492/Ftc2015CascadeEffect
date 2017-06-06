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
 *  This function drives the robot down the ramp and pulls the goal into
 *  the parking zone autonomously.
 *
 *  @param sm Specifies the state machine.
 *  @param delay Specifies the delay if any.
 *  @param parkOppoDefense Specifies whether to anticipate opponent defense.
 *  @param parkFarEnd Specifies whether to go back to the far end of the field.
 */
void DoParkGoal(SM &sm, unsigned long delay, int parkOppoDefense, int parkFarEnd)
{
    static float startHeading = 0.0;
    static float startPos = 0.0;
//    PIDCtrlDebugInfo(g_elevatorPidCtrl);
//    PIDCtrlDisplayInfo(1, g_encoderDrivePidCtrl);
//    nxtDisplayTextLine(3, "lIR=%4.1f,rIR=%4.1f",
//                       IRSeekerGetACDir(g_leftIR),
//                       IRSeekerGetACDir(g_rightIR));
//    PIDCtrlDisplayInfo(3, g_gyroTurnPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);
        float angle;
        float distTravel;

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Raise the elevator.
                //
                startHeading = PIDCtrlModeGetInput(g_gyroTurnPidCtrl);
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
                // Drive backward down the ramp (first segment).
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -20, 20);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -65.0,
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
                // Drive slowly towards the goal (second segment).
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -15, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -24.0,
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
                // Clamp down on mid-goal.
                //
                ServoSetAngle(g_goalCaptureServo,GOALCAPTURE_DOWN);
                TimerSet(g_timer, 300, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                //
                // Turn towards the parking zone for mid-goal.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl,
                                      DRIVE_MIN_VALUE, DRIVE_MAX_VALUE);
                angle = 35.0 -
                        (PIDCtrlModeGetInput(g_gyroTurnPidCtrl) -
                         startHeading);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  angle,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  2000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 5:
                //
                // Drive forward to the parking zone for mid-goal.
                //
                startPos = DriveGetYPos(g_drive);
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -40, 40);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  70.0,
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
                // Turn around to push mid-goal in.
                //
                angle = 190.0 -
                        (PIDCtrlModeGetInput(g_gyroTurnPidCtrl) -
                         startHeading);
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_RELEASE);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  angle,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 7:
                //
                // Drive towards the parking zone with mid-goal.
                //
                distTravel = DriveGetYPos(g_drive) - startPos;
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -25, 25);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -(20.0 - distTravel),
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
                // Mid-Goal release
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl,
                                      DRIVE_MIN_VALUE, DRIVE_MAX_VALUE);
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_CAPTURE);
                ServoSetAngle(g_goalCaptureServo, GOALCAPTURE_UP);
                TimerSet(g_timer, 200, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 9:
                //
                // Bring elevator to tall goal or down.
                //
                if (parkFarEnd == PARKFAREND_GRAB_TALLGOAL)
                {
                    PIDMotorSetTarget(g_elevatorMotor,
                                      ELEVATOR_HIGH_GOAL,
                                      false);
                }
                else
                {
                    PIDMotorSetTarget(g_elevatorMotor,
                                      ELEVATOR_DOWN_POS,
                                      false);
                }
                PIDDriveSetTarget(g_encoderPidDrive,
                                  12.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  1000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 10:
                //
                // Turn toward tall goal.
                //
                angle = 34.0 -      //36.0
                        (PIDCtrlModeGetInput(g_gyroTurnPidCtrl) -
                         startHeading);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  angle,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                if (parkFarEnd == PARKFAREND_YES ||
                    parkFarEnd == PARKFAREND_GRAB_TALLGOAL)
                {
                    SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    SMWaitEvents(sm, 1000);
                }
                break;

            case SMSTATE_STARTED + 11:
                //
                // Drive toward tall goal.
                //
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -80.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                if (parkFarEnd == PARKFAREND_GRAB_TALLGOAL)
                {
                    SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    SMWaitEvents(sm, 1000);
                }
                break;

            case SMSTATE_STARTED + 12:
                //
                // Drive slowly towards tall goal.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -30, 0);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -16.0, //-16.0 -18.0
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  1500);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 13:
                //
                // Clamp down on tall goal.
                //
                ServoSetAngle(g_goalCaptureServo,GOALCAPTURE_DOWN);
                TimerSet(g_timer, 200, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 14:
                //
                // Drive forward to the parking zone with tall goal.
                //
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_RELEASE);
                startPos = DriveGetYPos(g_drive);
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  115.0, //115.0, 112.0
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
                // Turn around with tall goal.
                //
                angle = 180.0 -
                        (PIDCtrlModeGetInput(g_gyroTurnPidCtrl) -
                         startHeading);
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_RELEASE);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  angle,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 16:
                //
                // Bring the elevator back down.
                //
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_CAPTURE);
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_DOWN_POS,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDELEVATOR);
                SMAddWaitEvent(sm, EVTTYPE_PIDELEVATOR);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoParkGoal
