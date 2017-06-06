#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="KickStand.h" />
///
/// <summary>
///     This module contains the implementation of the Kick Stand strategy.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

/**
 *  This function determines the position of the pole using the IR Seekers and knocks it down
 *  autonomously.
 *
 *  @param sm Specifies the state machine.
 *  @param delay Specifies the delay if any.
 */
void DoKickStand(SM &sm, unsigned long delay)
{
#ifdef _TEAM_3543
    static float distTable1[3]={-44.0,
                                -55.0,
                                -45.0};
    static float angleTable1[3]={17.0,
                                 23.0,
                                 35.0};
    static float distTable2[3]={0.0,
                                0.0,
                                -36.0};
    static float angleTable2[3]={90.0,
                                 -45.0,
                                 -45.0};
#else
    static float distTable1[3]={-44.0,
                                -55.0,
                                -45.0};
    static float angleTable1[3]={17.0,
                                 23.0,
                                 36.0};
    static float distTable2[3]={0.0,
                                0.0,
                                -36.0};
    static float angleTable2[3]={90.0,
                                 45.0,
                                 -30.0};
#endif

    nxtDisplayTextLine(1, "irL=%4.1f,irR=%4.1f",
                       IRSeekerGetACDir(g_leftIR),
                       IRSeekerGetACDir(g_rightIR));
    if (SMIsReady(sm))
    {
        int currState = SMGetState(sm);

        switch (currState)
        {
            case SMSTATE_STARTED:
                nxtDisplayTextLine(2,"pos=%d", g_centerGoalPos + 1);
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
                // Turn and drive according to angleTable and distTable.
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -50, 50);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  distTable1[g_centerGoalPos],
                                  angleTable1[g_centerGoalPos],
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  7000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            case SMSTATE_STARTED + 2:
                //
                // Final turn or drive according to angleTabe2 and distTable2.
                //      index 1: turn 90
                //      index 2: turn -45
                //      index 3: turn -45 and drive backwards 36inch
                //
                PIDCtrlSetPowerLimits(g_encoderDrivePidCtrl, -70, 70);
                PIDDriveSetTarget(g_encoderPidDrive,
                                  0.0,
                                  distTable2[g_centerGoalPos],
                                  angleTable2[g_centerGoalPos],
                                  false,
                                  &sm,
                                  EVTTYPE_PIDDRIVE,
                                  4000);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
                SMWaitEvents(sm, currState + 1);
                break;

            default:
                SMStop(sm);
                PlayTone(440, 50);
                break;
        }
    }
}   //DoScoreOnly
