void DoCenterGoal(SM &sm, unsigned long delay, int kickStandOption)
{
    static float distTable1[] = {-63.0, -29.0, -18.0};
    static float angleTable1[] = {-20.0, -45.0, 45.0};
    static float irAngleTable[] = {9.8, 9.5, 9.5, 9.5};
    static float sonarDistTable[] = {7.0, 8.5, 8.0};

    PIDCtrlDisplayInfo(1, g_sonarPidCtrl);
    PIDCtrlDisplayInfo(3, g_irPidCtrl);
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                //
                // Raise the elevator and optionally wait delay.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -30, 30);
                PIDCtrlSetPowerLimits(g_sonarPidCtrl, -20, 20);
                if (g_centerGoalPos == 0)
                {
                    PIDCtrlSetPowerLimits(g_gyroTurnPidCtrl, -70, 70);
                }
                PIDCtrlSetPowerLimits(g_irPidCtrl, -50, 50);
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_CENTERGOAL_HEIGHT,
                                  false);
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
                PIDDriveSetTarget(g_encoderPidDrive,
                                  distTable1[g_centerGoalPos],
                                  angleTable1[g_centerGoalPos],
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  6000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 2:
                if (g_centerGoalPos == 0)
                {
	                PIDDriveSetTarget(g_encoderPidDrive,
	                                  0.0,
	                                  110.0,
	                                  false,
	                                  &sm,
	                                  EVTTYPE_PIDDRIVE,
	                                  3000);
	                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
	                SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    SMSetState(sm, currState + 1);
                }
                break;

            case SMSTATE_STARTED + 3:
                PIDDriveSetTarget(g_sonarIrPidDrive,
                                  0.0,
                                  irAngleTable[g_centerGoalPos],
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 4:
                PIDDriveSetTarget(g_sonarPidDrive,
                                  sonarDistTable[g_centerGoalPos],
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  5000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 5:
                TimerSet(g_timer, 1000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 6:
                //
                // Dump the ball.
                //
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_RELEASE);
                TimerSet(g_timer, 3000, &sm, EVTTYPE_TIMER);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, currState + 1);
                break;

           case SMSTATE_STARTED + 7:
                //
                // Raise Drawbridge, lower elevator and back up a bit.
                //
                ServoSetAngle(g_drawBridgeServo, DRAWBRIDGE_CAPTURE);
                PIDMotorSetTarget(g_elevatorMotor,
                                  ELEVATOR_DOWN_POS,
                                  false);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  12.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

           case SMSTATE_STARTED + 8:
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  45.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                if (kickStandOption == KICKSTAND_YES)
                {
                    //
                    // Go knock down kickstand.
                    //
                    SMWaitEvents(sm, currState + 1);
                }
                else
                {
                    //
                    // We are done.
                    //
                    SMWaitEvents(sm, 1000);
                }
                break;

           case SMSTATE_STARTED + 9:
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -18.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

           case SMSTATE_STARTED + 10:
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  -45.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

           case SMSTATE_STARTED + 11:
                PIDDriveSetTarget(g_encoderPidDrive,
                                  -30.0,
                                  0.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

           case SMSTATE_STARTED + 12:
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  45.0,
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  3000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

           default:
                //
                // We are done, stop!
                //
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoCenterGoal
