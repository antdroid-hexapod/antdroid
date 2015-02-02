/* leg.h: definition of leg class, methos and attributes for hexapods
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

#ifndef leg_h
#define leg_h


#include <ServoEx.h>
#include <avr/pgmspace.h>

#include "Arduino.h"
#include "trig.h"
#include "Configuration.h"
#include "calibration.h"
#include "log.h"
#include "uncopyable.h"

#define MaxSpeed 255

#define Coxa    0
#define Femur   1
#define Tibia   2

class Leg: private Uncopyable
{
	public:

    short currentX;
    short currentY;

	Leg(short CoxaDefaultAngle, short coxa_min, short coxa_max, 
        short femur_min, short femur_max, short tibia_min, short tibia_max,
        uint8_t legNumber);
	void Attach(byte pinServoCoxa, byte pinServoFemur, byte pinServoTibia);
	bool TryCalDefaultPosition(const short footDistance, const short footHeight);
    bool WriteStartPosition(void);    
	bool TryCalculatePosition(short foot_position[]);
	bool TryCalRelativePosition(short foot_position[]);
	bool TryCalTransferTrajectory(short foot_position[], const short gap);
    bool TryCalRotationPosition(short foot_position[3]);
    bool TryUpdatePosition(const uint16_t timeMove);
    uint16_t CalculateTimeMove(const uint8_t speed);
    bool TryMoveServos(const short newCoxaAngle, const short newFemurAngle, 
        const short newTibiaAngle, const unsigned int moveTime);
    void WaitUntilStop(void);
    void SaveCalibrationMovePosition(uint8_t member, uint8_t angle);
    bool TrySaveValueCalibration(uint8_t member, uint8_t angle);
    void SaveDefaultCalibration(void);
    void ServosToCalibrationAngles(void);
    
	private:
    const uint8_t _legNumber;

	const short _coxaDefaultAngle;
    const short _coxaMin;
    const short _coxaMax;
    const short _femurMin;
    const short _femurMax;
    const short _tibiaMin;
    const short _tibiaMax;

    short _coxaCurrentAngle;
    short _femurCurrentAngle;
    short _tibiaCurrentAngle;

	short _coxaAngle;
	short _femurAngle;
	short _tibiaAngle;

	short _foot_position[3];

    bool _alarm;

    bool _updateXY;

	ServoEx _servoCoxa;
    ServoEx _servoFemur;
    ServoEx _servoTibia;

    bool IsMoving(void);
	bool TryCalculateInverseKinematic(void);
    uint8_t CalculateCalibrationPosition(short minAngle, short maxAngle);

};

#endif