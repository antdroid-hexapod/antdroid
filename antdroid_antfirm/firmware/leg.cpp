/* leg.cpp: definition of leg class, methos and attributes for hexapods
 *
 * Copyright (C) 2014 Alexander Gil and Javier Román
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * Library to create and control a leg object. The leg objects can be used 
 * alone or throw the hexapod library, which is in charge of create and
 * control the six of them. 
 *
 * The methods are:
 *
 * Leg: constructor. Needs as initial attributes the min and max angles
 *      for each servo and the number of the new leg. 
 *
 * Attach(byte , byte , byte ): attaches the three servos of the leg.
 * TryCalDefaultPosition(short , short ): moves the leg to def. position
 * TryCalculatePosition(short ): calculates the new servo angles using IK
 * TryCalRelativePosition(short ): 
 * TryCalTransferTrajectory(short , short ): 
 * TryCalRotationPosition(short ): 
 * CalculateTimeMove(uint8_t ): calculates the needed time for the next move
 *                     taking into account the current and the next position
 *                     and the maximum allowed speed.
 * TryUpdatePosition(uint16_t ): moves the servos to its new angles.
 * TryMoveServos(short,short,short,uint16_t ): moves the servos to its
 *                                          new location.   
 * SaveCalibrationMovePosition(uint8_t, uint8_t ): allows us to calibrate the three servos 
 *                      of the leg and moves them to their locations.
 */

/*
 * Axis system:
 *     X    forwards
 *     Y    left
 *     Z    up
 *
 * Default angle values:
 *    Degree    One decimal. Example: 1800 = 180.0 º
 *    Radians   Four decimal. Example: 31416 = 3,1416 rad
 */
 
#include "leg.h"

#include <avr/eeprom.h>
#include <ServoEx.h>


Leg::Leg(short CoxaDefaultAngle, short coxa_min, short coxa_max, 
    short femur_min, short femur_max, short tibia_min, short tibia_max,
    uint8_t legNumber)
    : _coxaDefaultAngle(CoxaDefaultAngle),
    _coxaMin(coxa_min),
    _coxaMax(coxa_max),
    _femurMin (femur_min),
    _femurMax(femur_max),
    _tibiaMin (tibia_min),
    _tibiaMax(tibia_max),
    _legNumber(legNumber),
    _coxaCalibrationAngle(eeprom_read_byte((unsigned char *)(legNumber*3+Coxa))),
    _femurCalibrationAngle(eeprom_read_byte((unsigned char *)(legNumber*3+Femur))),
    _tibiaCalibrationAngle(eeprom_read_byte((unsigned char *)(legNumber*3+Tibia)))
{
    log("In Leg::Leg", Debug);

    log("New leg object", _legNumber, Info);
}

void Leg::Attach(byte pinServoCoxa, byte pinServoFemur, byte pinServoTibia)
{
    log("In Leg::Attach", Debug);

    _servoCoxa.attach(pinServoCoxa);
    _servoFemur.attach(pinServoFemur);
    _servoTibia.attach(pinServoTibia);

    log("New LEG object attached", Info);
}

bool Leg::TryCalDefaultPosition(const short footDistance, const short footHeight)
{
    log("In Leg::TryCalDefaultPosition", Debug);

    short default_position[3];

    default_position[0] = ((long)footDistance * GetCos(_coxaDefaultAngle))
        /Shift4Decimal;
    default_position[1] = ((long)footDistance * GetSin(_coxaDefaultAngle))
        /Shift4Decimal;
    default_position[2] = footHeight;

    if(!TryCalculatePosition(default_position))
        return false;

    return true;
}

bool Leg::WriteStartPosition(void)
{
    if(_alarm)
    {
        log("_alarm == true", Error);
        return false;
    }

    const short coxaAngle = _coxaAngle/Shift1Decimal + _coxaCalibrationAngle;
    const short femurAngle = _femurAngle/Shift1Decimal + _femurCalibrationAngle; 
    const short tibiaAngle = _tibiaAngle/Shift1Decimal + _tibiaCalibrationAngle;

    if((coxaAngle <= 0) || (femurAngle <= 0) || (tibiaAngle <= 0)
        || (coxaAngle >= 180) || (tibiaAngle >= 180) || (femurAngle >= 180))
    {
        log("Angles are not in range", Error);
        log("Leg:", _legNumber, Debug);
        log("Coxa", coxaAngle, Debug);
        log("Femur", femurAngle, Debug);
        log("Tibia", tibiaAngle, Debug);
        
        return false;
    }

    currentX = _foot_position[0];
    currentY = _foot_position[1]; 
    
    
    _servoCoxa.write(coxaAngle);
    _servoFemur.write(femurAngle);
    _servoTibia.write(tibiaAngle);

    _coxaCurrentAngle = (coxaAngle - _coxaCalibrationAngle)*Shift1Decimal;
    _femurCurrentAngle = (femurAngle - _femurCalibrationAngle)*Shift1Decimal;
    _tibiaCurrentAngle = (tibiaAngle - _tibiaCalibrationAngle)*Shift1Decimal;

    return true;

}

bool Leg::TryCalculatePosition(short foot_position[3])
{
    log("In Leg::TryCalculatePosition", Debug);

    for(byte i=0;i<3;i++)
        _foot_position[i] = foot_position[i];


    if(!TryCalculateInverseKinematic())
        return false;
    _updateXY = true;

    return true;
}

bool Leg::TryCalRelativePosition(short foot_position[3])
{
    log("In Leg::TryCalRelativePosition", Debug);

    _foot_position[0] = foot_position[0] + currentX;
    _foot_position[1] = foot_position[1] + currentY;
    _foot_position[2] = foot_position[2];
    
    if(!TryCalculateInverseKinematic())
        return false; 
    _updateXY = false;

    return true;
}

bool Leg::TryCalTransferTrajectory(short foot_position[3], const short gap)
{
    log("In Leg::TryCalTransferTrajectory", Debug);

    _foot_position[0] = currentX;
    _foot_position[1] = currentY;
    _foot_position[2] = foot_position[2] + gap ;


    if(!TryCalculateInverseKinematic())
    {
        log("Trying to calculate new position", Warn);

        if (_coxaDefaultAngle > 0)
            _foot_position[1] = foot_position[1]/2 + currentY + 20;
        else
            _foot_position[1] = foot_position[1]/2 + currentY - 20;

        if(!TryCalculateInverseKinematic())
            return false;
    }  
    _updateXY = false;

    return true;
}

bool Leg::TryCalRotationPosition(short foot_position[3])
{
    log("In Leg::TryCalRotationPosition", Debug);

    for(byte i=0;i<3;i++)
        _foot_position[i] = foot_position[i];

    if(!TryCalculateInverseKinematic())
        return false;
    _updateXY = false;

    return true;
}

bool Leg::TryCalculateInverseKinematic(void)
{
    log("In Leg::TryCalculateInverseKinematic", Debug);

	long XYhyp;
	long aux[3];
	short hip_foot;

    log("Leg:", _legNumber, Debug);
    log("_foot_position[0]", _foot_position[0], Debug);
    log("_foot_position[1]", _foot_position[1], Debug);
    log("_foot_position[2]", _foot_position[2], Debug);

    if(!tryCalibrationCompleted())
    {
        return false;
    }

    XYhyp = Hypotenuse (_foot_position[0], _foot_position[1]);
	_coxaAngle = Rad2Deg(GetArcTan(_foot_position[1], _foot_position[0], XYhyp)) 
	           - _coxaDefaultAngle;
    
    _coxaAngle = - _coxaAngle;

    log("_CoxaAngle: ", _coxaAngle, Debug);

    if ((_coxaAngle < _coxaMin) || (_coxaAngle > _coxaMax) )
    {
        log("Coxa angle not in range", Error);
        _alarm = true;
        return false;
    }

	XYhyp = XYhyp - CoxaLength;

	hip_foot = Hypotenuse(_foot_position[2], XYhyp);
    log("hip_foot: ", hip_foot, Debug);

    if (hip_foot > (FemurLength + TibiaLength) )
    {
        log("Leg is too short", Error);
        _alarm = true;
        return false;
    }

	aux[2] = (GetArcTan( XYhyp, -_foot_position[2], hip_foot));

	//Law of cosines to get Femur-hip_foot Angle
	aux[0] = (((long)FemurLength * FemurLength) - ((long)TibiaLength * 
		TibiaLength)) + ((long)hip_foot * hip_foot); 
	aux[1] = ((long) 2 * FemurLength * hip_foot);
	aux[0] = GetArcCos((aux[0] * Shift4Decimal) /aux[1]);//Femur-hip_foot Angle
	_femurAngle = - Rad2Deg((aux[2] - Pi/2) + aux[0]);
    
    if(_coxaDefaultAngle < 0)
        _femurAngle = - _femurAngle;

    log("_femurAngle", _femurAngle, Debug);
    
    if ((_femurAngle < _femurMin)|| (_femurAngle > _femurMax) )
    {
        log("Femur angle not in range", Error);
        _alarm = true;
        return false;
    }

	//Law of cosines to get Femur-Tibia Angle
	aux[0] = (((long)FemurLength * FemurLength) - ((long)hip_foot * hip_foot)) 
	+ ((long)TibiaLength * TibiaLength);
	aux[1] = 2 * FemurLength * TibiaLength;

	//Femur-Tibia Angle
	aux[0] = GetArcCos((aux[0] * Shift4Decimal) / aux[1]);
	_tibiaAngle = Rad2Deg(aux[0] - Pi/2);
    if(_coxaDefaultAngle > 0)
        _tibiaAngle = - _tibiaAngle;

    log("_tibiaAngle", _tibiaAngle, Debug);

    if ((_tibiaAngle < _tibiaMin)|| (_tibiaAngle > _tibiaMax) )
    {
        log("Tibia angle not in range", Error);
        _alarm = true;
        return false;
    }

    _alarm = false;
	return true;
}

bool Leg::TryUpdatePosition(const uint16_t timeMove)
{
    log("In Leg::TryUpdatePosition", Debug);

    if(_alarm)
    {
        log("_alarm == true", Error);
        return false;
    }

    if(!TryMoveServos(_coxaAngle/Shift1Decimal + _coxaCalibrationAngle, 
        _femurAngle/Shift1Decimal + _femurCalibrationAngle, 
        _tibiaAngle/Shift1Decimal + _tibiaCalibrationAngle,timeMove))
        return false;


    return true;
}

uint16_t Leg::CalculateTimeMove(const uint8_t speed)
{
    log("In Leg::CalculateTimeMove", Debug);

    uint16_t maxDif = 0;
    uint16_t timeMove;
    uint16_t realSpeed;
    short difAngle[3];

    difAngle[0] = _coxaCurrentAngle - _coxaAngle;
    difAngle[1] = _femurCurrentAngle - _femurAngle;
    difAngle[2] = _tibiaCurrentAngle - _tibiaAngle;

    for(byte i=0; i<3; i++)
    {
        if (difAngle[i] < 0)
            difAngle[i] = -difAngle[i];

        if(difAngle[i]>maxDif)
            maxDif=difAngle[i];
    }

    maxDif = maxDif / Shift1Decimal;
    realSpeed = ((long)speed * MaxRealSpeed)/MaxSpeed;
    timeMove = ((uint32_t)(maxDif*Shift4Decimal))/realSpeed; // miliseconds

    log("realSpeed", realSpeed, Debug);
    log("timeMove", timeMove, Debug);

    return timeMove; 
}

bool Leg::TryMoveServos(const short coxaAngle, const short femurAngle,
    const short tibiaAngle, const unsigned int mTime)
{

    log("In Leg::TryMoveServos", Debug);

    if((coxaAngle <= 0) || (femurAngle <= 0) || (tibiaAngle <= 0)
        || (coxaAngle >= 180) || (tibiaAngle >= 180) || (femurAngle >= 180))
    {
        log("Angles are not in range", Error);
        log("Leg:", _legNumber, Debug);
        log("Coxa", coxaAngle, Debug);
        log("Femur", femurAngle, Debug);
        log("Tibia", tibiaAngle, Debug);

        return false;
    }

    if(_updateXY)
    {
        currentX = _foot_position[0];
        currentY = _foot_position[1]; 
    }
    
    _servoCoxa.move(coxaAngle, mTime);
    _servoFemur.move(femurAngle, mTime);
    _servoTibia.move(tibiaAngle, mTime);

    _coxaCurrentAngle = (coxaAngle - _coxaCalibrationAngle)*Shift1Decimal;
    _femurCurrentAngle = (femurAngle - _femurCalibrationAngle)*Shift1Decimal;
    _tibiaCurrentAngle = (tibiaAngle - _tibiaCalibrationAngle)*Shift1Decimal;

    log("Leg:", _legNumber, Debug);
    log("Time move:", mTime, Debug);
    log("_coxaCurrentAngle", _coxaCurrentAngle, Debug);
    log("_femurCurrentAngle", _femurCurrentAngle, Debug);
    log("_tibiaCurrentAngle", _tibiaCurrentAngle, Debug);

    return true;
}

void Leg::WaitUntilStop(void){
    while(IsMoving());
}

bool Leg::IsMoving(){

	if (_servoCoxa.moving() || _servoFemur.moving() || _servoTibia.moving())
		return true;

	return false;
}

void Leg::SaveCalibrationMovePosition(uint8_t member, uint8_t angle)
{
    if(!TrySaveValueCalibration(member, angle))
        return;

    log("Moving servo to default position", Info);

    if(member == Coxa)
        _servoCoxa.move(_coxaCalibrationAngle, DefaultTime);
   
    else if (member == Femur)
        _servoFemur.move(_femurCalibrationAngle, DefaultTime);
    
    else
        _servoTibia.move(_tibiaCalibrationAngle, DefaultTime);    
}

bool Leg::TrySaveValueCalibration(uint8_t member, uint8_t angle)
{
    log("In Leg::TrySaveValueCalibration", Debug);

    if(angle > 180 || angle < 0 || member > 2 || member < 0)
    {
        log("Calibrate values not in range", Error);
        return false;
    }

    if(eeprom_read_byte((unsigned char *)(_legNumber*3+member)) != angle)
    {
        log("Writing to EEPROM memory. Do not turn off the power source", Warn);
        eeprom_write_byte((unsigned char *)(_legNumber*3+member), angle);
        log("Save completed.", Warn);
        _coxaCalibrationAngle = eeprom_read_byte((unsigned char *)(_legNumber*3+Coxa)); 
        _femurCalibrationAngle = eeprom_read_byte((unsigned char *)(_legNumber*3+Femur));
        _tibiaCalibrationAngle = eeprom_read_byte((unsigned char *)(_legNumber*3+Tibia));
    }

    return true;
}


void Leg::SaveDefaultCalibration(void)
{
    log("Saving default calibration", Info);
    uint8_t defaultAngle;
    defaultAngle = CalculateCalibrationPosition(_coxaMin, _coxaMax);
    SaveCalibrationMovePosition(Coxa, defaultAngle);
    defaultAngle = CalculateCalibrationPosition(_femurMin, _femurMax);
    SaveCalibrationMovePosition(Femur, defaultAngle);
    defaultAngle = CalculateCalibrationPosition(_tibiaMin, _tibiaMax);
    SaveCalibrationMovePosition(Tibia, defaultAngle);
}

uint8_t Leg::CalculateCalibrationPosition(short minAngle, short maxAngle)
{
    uint8_t defaultAngle = 0;

    defaultAngle = (900 -(maxAngle - minAngle) /2 - minAngle)/10;

    log("Default angle = ", defaultAngle, Info);
    return defaultAngle;
}

void Leg::ServosToCalibrationAngles(void)
{
    _servoCoxa.write(_coxaCalibrationAngle);
    _servoFemur.write(_femurCalibrationAngle);
    _servoTibia.write(_tibiaCalibrationAngle);
}