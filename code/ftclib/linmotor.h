#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="LinMotor.h" />
///
/// <summary>
///     This module contains the library functions for the linearized motor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _LINMOTOR_H
#define _LINMOTOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_LINMOTOR

//
// Constants.
//
#define NUM_MAP_ENTRIES         10

//
// Type definitions.
//
typedef struct
{
    tMotor          motorID;
    int             motorPowers[NUM_MAP_ENTRIES*2 + 1];
    float           motorSpeeds[NUM_MAP_ENTRIES*2 + 1];
    int             idx;
    int             power;
    float           speed;
    int             prevEncoder;
    unsigned long   prevTime;
} LINMOTOR;

/**
 *  This function initializes the linearized motor.
 *
 *  @param linMotor Points to the LINMOTOR structure.
 *  @param motorID Specifies the motor ID.
 *  @param maxSpeed Specifies the maximum motor speed.
 */
void
LinMotorInit(
    LINMOTOR &linMotor,
    tMotor motorID,
    float maxSpeed
    )
{
    TFuncName("LinMotorInit");
    TLevel(INIT);
    TEnter();

    int power;

    linMotor.motorID = motorID;
    for (int i = 0; i < NUM_MAP_ENTRIES*2 + 1; i++)
    {
        linMotor.motorPowers[i] = MOTOR_MAX_VALUE*(i - NUM_MAP_ENTRIES)/
                                  NUM_MAP_ENTRIES;
        linMotor.motorSpeeds[i] = maxSpeed*(i - NUM_MAP_ENTREIS)/
                                  NUM_MAP_ENTRIES;
    }
    motor[linMotor.motorID] = 0;
    nMotorEncoder[linMotor.motorID] = 0;
    linMotor.idx = NUM_MAP_ENTRIES;
    linMotor.power = 0;
    linMotor.speed = 0.0;
    linMotor.prevEncoder = 0;
    linMotor.prevTime = GetMsecTime();

    TExit();
    return;
}   //LinMotorInit

/**
 *  This function sets the motor power.
 *
 *  @param linMotor Points to the LINMOTOR structure.
 *  @param power Specifies the power level to set the motor.
 */
void
LinMotorSetPower(
    LINMOTOR &linMotor,
    int power
    )
{
    TFuncName("LinMotorSetPower");
    TLevel(API);
    TEnterMsg(("power=%d", power));

    power = BOUND(power, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    linMotor.idx = LinMotorFindPower(linMotor, power);
    linMotor.power = power;

    TExit()
    return;
}   //LinMotorSetPower

/**
 *  This function sets the linearized motor speed.
 *
 *  @param linMotor Points to the LINMOTOR structure.
 *  @param speed Specifies the speed to set the motor.
 */
void
LinMotorSetSpeed(
    LINMOTOR &linMotor,
    float speed
    )
{
    TFuncName("LinMotorSetSpeed");
    TLevel(API);
    TEnterMsg(("speed=%f", speed));

    linMotor.idx = LinMotorFindSpeed(linMotor, speed);
    linMotor.power = LinMotorCalcPower(linMotor, linMotor.idx, speed);

    TExit()
    return;
}   //LinMotorSetSpeed

/**
 *  This function performs the LinMotor task.
 *
 *  @param linMotor Points to the LINMOTOR structure.
 */
void
LinMotorTask(
    LINMOTOR &linMotor
    )
{
    TFuncName("LinMotorTask");
    TLevel(TASK);
    TEnter();

    unsigned long currTime = GetMsecTime();
    float dt = (currTime - linMotor.prevTime)/1000.0;
    int currEncoder = nMotorEncoder[linMotor.motorID];
    float speed = (currEncoder - linMotor.prevEncoder)/dt;

    if (abs(speed - linMotor.speed) > 10000.0)
    {
#ifdef _DEBUG_LINMOTOR
        TPrintfLIne("Bogus speed: %6.1f", speed);
#endif
        speed = linMotor.speed;
        currEncoder = (long)(speed*dt) + linMotor.prevEncoder;
    }
    else
    {
        linMotor.speed = speed;
    }
    LinMotorUpdateMap(linMotor, power, speed);

    motor[linMotor.motorID] = linMotor.power;
    linMotor.prevEncoder = currEncoder;
    linMotor.prevTime = currTime;

    TExit();
    return;
}   //LinMotorTask

#endif  //ifndef _LINMOTOR_H
