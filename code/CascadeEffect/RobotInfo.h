#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="RobotInfo.h" />
///
/// <summary>
///     This module contains the Robot Info constants.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ROBOTINFO_H
#define _ROBOTINFO_H

//
// Drive subsystem info.
//
#define DRIVE_MIN_VALUE                 -100
#define DRIVE_MAX_VALUE                 100
#define TURN_MIN_VALUE                  -100
#define TURN_MAX_VALUE                  100

#ifdef _TEAM_3543
  #define ENC_KP                        3.3     //3.0
  #define ENC_KI                        0.0     //0.0
  #define ENC_KD                        0.0     //0.0
  #define ENC_TOLERANCE                 1.0
  #define ENC_SETTLING                  200
  #define CLICKS_PER_INCH               78.0    //78.0

  #define GYROTURN_KP                   4.0     //4.0
  #define GYROTURN_KI                   0.0     //0.0
  #define GYROTURN_KD                   0.0     //0.0
  #define GYROTURN_TOLERANCE            2.0
  #define GYROTURN_SETTLING             200

  #define IR_KP                         10.0    //10.0
  #define IR_KI                         0.0
  #define IR_KD                         3.0
  #define IR_TOLERANCE                  1.0
  #define IR_SETTLING                   200
  #define IR_TARGET                     10.0

  #define SONAR_KP                      2.6     //2.6
  #define SONAR_KI                      0.0
  #define SONAR_KD                      0.0
  #define SONAR_TOLERANCE               1.0
  #define SONAR_SETTLING                200
  #define SONAR_PARK_TARGET             12.0    //inches
  #define SONAR_GOAL_TARGET             12.0    //inches
#else
  #define ENC_KP                        3.6     //3.5
  #define ENC_KI                        0.0     //0.0
  #define ENC_KD                        0.0     //0.0
  #define ENC_TOLERANCE                 1.0
  #define ENC_SETTLING                  200
  #define CLICKS_PER_INCH               76.0   //118.0

  #define GYROTURN_KP                   5.0     //5.0
  #define GYROTURN_KI                   0.0     //0.0
  #define GYROTURN_KD                   0.0     //0.0
  #define GYROTURN_TOLERANCE            2.0
  #define GYROTURN_SETTLING             200

  #define IR_KP                         20.0    //10.0
  #define IR_KI                         0.0
  #define IR_KD                         3.0
  #define IR_TOLERANCE                  0.2
  #define IR_SETTLING                   200
  #define IR_TARGET                     10.0

  #define SONAR_KP                      2.0     //2.6
  #define SONAR_KI                      0.0
  #define SONAR_KD                      0.0
  #define SONAR_TOLERANCE               0.2
  #define SONAR_SETTLING                200
  #define SONAR_PARK_TARGET             12.0    //inches
  #define SONAR_GOAL_TARGET             5.0     //inches
#endif

//
// Elevator subsystem info.
//
#ifdef _TEAM_3543
  #define ELEVATOR_KP                   550.0   //500.0
  #define ELEVATOR_KI                   0.0
  #define ELEVATOR_KD                   0.0
  #define ELEVATOR_TOLERANCE            1.0     //inches
  #define ELEVATOR_SETTLING             200     //msec
  #define ELEVATOR_CLICKS_PER_INCH      371.2
  #define ELEVATOR_UPPER_LIMIT          45.0    //inches
  #define ELEVATOR_LOWER_LIMIT          0.0     //0.0
  #define ELEVATOR_CENTERGOAL_HEIGHT    48.0    //inches
  #define ELEVATOR_DOWN_POS             0.0     //inch
  #define ELEVATOR_RAMP_POS             2.0     //inches
  #define ELEVATOR_LOW_GOAL             12.25   //inches??? 12.5???
  #define ELEVATOR_MID_GOAL             23.5
  #define ELEVATOR_HIGH_GOAL            36.0
#else
  #define ELEVATOR_KP                   350.0   //500.0
  #define ELEVATOR_KI                   0.0
  #define ELEVATOR_KD                   0.0
  #define ELEVATOR_TOLERANCE            0.2     //inches
  #define ELEVATOR_SETTLING             200     //msec
  #define ELEVATOR_CLICKS_PER_INCH      1000.0
  #define ELEVATOR_UPPER_LIMIT          35.0    //inches
  #define ELEVATOR_LOWER_LIMIT          0.0     //0.0
  #define ELEVATOR_CENTERGOAL_HEIGHT    34.5    //inches
  #define ELEVATOR_DOWN_POS             0.0     //inch
  #define ELEVATOR_RAMP_POS             2.0     //inches
  #define ELEVATOR_LOW_GOAL             19.2    //inches
  #define ELEVATOR_MID_GOAL             25.35
  #define ELEVATOR_HIGH_GOAL            29.5
#endif

#define ELEVATOR_CAL_POWER              -50
#define ELEVATOR_STALL_MINPOWER         30
#define ELEVATOR_STALL_TIMEOUT          100     //msec

//
// GoalCapture subsystem info.
//
#ifdef _TEAM_3543
  #define GOALCAPTURE_UP                180.0
  #define GOALCAPTURE_DOWN              124.0
#else
  #define GOALCAPTURE_UP                10.0
  #define GOALCAPTURE_DOWN              110.0
#endif

#ifdef _TEAM_3543
//
// GoalGrabber subsystem info.
//
  #define GOALGRABBER_UP                0.0
  #define GOALGRABBER_DOWN              100.0
#endif

//
// DrawBridge subsystem info.
//
#ifdef _TEAM_3543
  #define DRAWBRIDGE_RELEASE            160.0
  #define DRAWBRIDGE_CAPTURE            0.0
  #define DRAWBRIDGE_DUMP_DELAY         500     //msec
#else
  #define DRAWBRIDGE_RELEASE_LOW        15.0
  #define DRAWBRIDGE_RELEASE            23.0
  #define DRAWBRIDGE_CAPTURE            100.0
  #define DRAWBRIDGE_DUMP_DELAY         500     //msec
#endif

//
// Autonomous definitions.
//
#define AUTO_INIT_DELAY                 0

// Strategies menu.
#define STRATEGY_NOAUTO                 0
#define STRATEGY_DEFENSE                1
#define STRATEGY_PARK_GOAL              2
#define STRATEGY_CENTER_GOAL            3
#define STRATEGY_SCORE_ONLY             4
#define STRATEGY_KICKSTAND              5

// ParkOption menu.
#define PARKOPTION_NONE                 0
#define PARKOPTION_FAR_END              1

// Park goal Options
#define PARK_OPPODEFENSE_NONE           0
#define PARK_OPPODEFENSE_YES            1

// Park Far End Option
#define PARKFAREND_NONE                 0
#define PARKFAREND_YES                  1
#define PARKFAREND_GRAB_TALLGOAL        2

// Kick Stand Option
#define KICKSTAND_NONE                  0
#define KICKSTAND_YES                   1

#endif  //ifndef _ROBOTINFO_H
