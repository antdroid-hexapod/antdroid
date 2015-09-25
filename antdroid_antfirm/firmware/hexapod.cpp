/* hexapod.cpp: definiton of hexapod class and its methos and attributes.
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
  * Main library to control the hexapod. It supports serial control by
  * using the serial communication port or you can attach the hexapod 
  * object as a node of the ROS system and controll it using that OS.
  *
  */


#include "hexapod.h"

const byte Pin[] PROGMEM = {
	LeftFrontCoxaPin, LeftFrontFemurPin, LeftFrontTibiaPin, 
	RightFrontCoxaPin, RightFrontFemurPin, RightFrontTibiaPin, 
	LeftMiddleCoxaPin, LeftMiddleFemurPin, LeftMiddleTibiaPin, 
	RightMiddleCoxaPin, RightMiddleFemurPin, RightMiddleTibiaPin,
	LeftRearCoxaPin, LeftRearFemurPin, LeftRearTibiaPin, 
	RightRearCoxaPin, RightRearFemurPin, RightRearTibiaPin
};


const short CoxaOffset[] PROGMEM = {
	CoxaOffsetX, CoxaOffsetY, CoxaOffsetX, -CoxaOffsetY,
	0, CoxaOffsetY, 0, -CoxaOffsetY,
	-CoxaOffsetX, CoxaOffsetY, -CoxaOffsetX, -CoxaOffsetY
};


Hexapod::Hexapod(void)
:	_floorHeight(FootHeight),
	_rotate(false),
	_speed(DefaultSpeed),
	_footDistance(FootDistance),
	_footDistanceStep(FootDistanceStep),
	_speedStep(SpeedStep),
	_floorHeightStep(FloorHeightStep),
	_voltage(20 + BATTERY_CUTOFF_VOLTAGE )
{
	log("In Hexapod::Hexapod", Debug);

	_legs = new Leg*[6];

	_legs[0] = new Leg(LeftFrontCoxaDefaultAngle, LeftFrontCoxaMin, 
		LeftFrontCoxaMax, LeftFemurMin, LeftFemurMax, LeftTibiaMin,
		LeftTibiaMax, 0);
	_legs[1] = new Leg(RightFrontCoxaDefaultAngle, RightFrontCoxaMin,
		RightFrontCoxaMax,	RightFemurMin, RightFemurMax, RightTibiaMin,
		RightibiaMax, 1); 
	_legs[2] = new Leg(LeftMiddleCoxaDefaultAngle, LeftMiddleCoxaMin,
		LeftMiddleCoxaMax, LeftFemurMin, LeftFemurMax, LeftTibiaMin,
		 LeftTibiaMax, 2); 
	_legs[3] = new Leg(RightMiddleCoxaDefaultAngle, RightMiddleCoxaMin,
	 	RightMiddleCoxaMax, RightFemurMin, RightFemurMax, RightTibiaMin, 
	 	RightibiaMax, 3);
	_legs[4] = new Leg(LeftRearCoxaDefaultAngle, LeftRearCoxaMin,
		LeftRearCoxaMax, LeftFemurMin, LeftFemurMax, LeftTibiaMin, 
		LeftTibiaMax, 4); 
	_legs[5] = new Leg(RightRearCoxaDefaultAngle, RightRearCoxaMin, 
		RightRearCoxaMax, RightFemurMin, RightFemurMax, RightTibiaMin, 
		RightibiaMax, 5);
}

void Hexapod::Start(void)
{
	log("In Hexapod::Start", Debug);

	PowerOffServos();

	GoStartingPostion();

	for(byte i = 0;i < 6; i++)
	{
		_legs[i]->Attach(pgm_read_byte(&Pin[3*i]), pgm_read_byte(&Pin[3*i+1]),
			pgm_read_byte(&Pin[3*i+2]));
	}

	CalculateMaxFloorHeight();

    if(!tryCalibrationCompleted())
    {
    	for (byte i = 0; i < 6; i++)
    	{
    		_legs[i]->SaveDefaultCalibration();
    	}
    }

	PowerOnServos();
	
	EnableDefaultGait();

}

void Hexapod::GoStartingPostion(void)
{
	for(byte i = 0;i < 6; i++)
	{
		if(!_legs[i]->TryCalDefaultPosition(_footDistance , _floorHeight))
			{
				if(tryCalibrationCompleted())
					return;
			}
	}

	for(byte i = 0;i < 6; i++)
	{
		if(!_legs[i]->WriteStartPosition())
			return;
	}
}

void Hexapod::EnableDefaultGait(void)
{
	EnableTripodGait();	
}

void Hexapod::EnableTripodGait( void)
{
	const uint8_t sequence[6] = {0, 1, 1, 0, 0, 1};

	for (byte i = 0; i < 6; i++)
	{
		_sequence[i] = sequence[i];
	}

	log("Tripod Gait Enabled", Info);

	GoDefaultPosition();
}

void Hexapod::EnableRippleGait(void)
{
	const uint8_t sequence[6] = {0, 2, 3, 1, 2, 0};

	for (byte i = 0; i < 6; i++)
	{
		_sequence[i] = sequence[i];
	}

	log("Ripple Gait Enabled", Info);

	GoDefaultPosition();
}


void Hexapod::GoDefaultPosition()
{
	uint16_t timeMove = 0;

	for(byte i = 0;i < 6; i++)
	{
		if(!_legs[i]->TryCalDefaultPosition(_footDistance , _floorHeight))
			{
				if(tryCalibrationCompleted())
					return;
			}
	}

	for(byte i = 0;i < 6; i++)
	{
		if (timeMove < _legs[i]->CalculateTimeMove(_speed))
			timeMove = _legs[i]->CalculateTimeMove(_speed);
	}

	for(byte i = 0;i < 6; i++)
	{
		if(!_legs[i]->TryUpdatePosition(timeMove))
			return;
	}
}

void Hexapod::CalculateMaxFloorHeight()
{ 
	log("In Hexapod::CalculateMaxFloorHeight", Debug);

	_MaxFloorHeight = - (TibiaLength + 
		((long)GetSin(LeftFemurMax) * FemurLength) / Shift4Decimal);	

	log("_MaxFloorHeight", _MaxFloorHeight, Debug);
}



void Hexapod::Balance(const short pitch, const short roll, const short yaw)
{
	log("In Hexapod::Balance", Debug);

	uint16_t timeMove = 0;
	uint8_t balanceSpeed = _speed * BalanceSpeedProportion;

	const short sinRoll = GetSin(roll);
	const short cosRoll = GetCos(roll);

	const short sinPitch = GetSin(pitch);
	const short cosPitch = GetCos(pitch);

	const short sinYaw = GetSin(yaw);
	const short cosYaw = GetCos(yaw);

	short position[6][3] = {0};

	short aux[3];

	short rotationMatrix[3][3] = {
		{((long)cosPitch * cosYaw)/Shift4Decimal,
		(-((long)cosPitch * sinYaw)/Shift4Decimal),
		sinPitch} ,
		{((long)cosRoll*sinYaw)/ Shift4Decimal + 
			((((long)sinRoll*sinPitch)/ Shift4Decimal)*cosYaw)/ Shift4Decimal,
		(((long)cosRoll*cosYaw)/ Shift4Decimal) -
			((((long)sinRoll*sinPitch)/ Shift4Decimal)* sinYaw)/ Shift4Decimal,
		 -((long)sinRoll*cosPitch)/ Shift4Decimal} ,
		{(((long)sinRoll*sinYaw)/ Shift4Decimal) -
			((((long)cosRoll*sinPitch)/ Shift4Decimal)* cosYaw)/ Shift4Decimal, 
		(((long)sinRoll*cosYaw)/ Shift4Decimal) +
			((((long)cosRoll*sinPitch)/ Shift4Decimal)* sinYaw)/ Shift4Decimal, 
		((long)cosRoll*cosPitch)/ Shift4Decimal}
	};

	// Rotate in absolute coordinates
	for(byte f = 0; f < 6; f++)
	{
		for(byte i = 0; i < 3; i++)
		{
			aux[0] = ((long)rotationMatrix[i][0] * 
			((short)pgm_read_word(&CoxaOffset[2*f]) + _legs[f]->currentX)) 
			/ Shift4Decimal;
			aux[1] = ((long)rotationMatrix[i][1] * 
			((short)pgm_read_word(&CoxaOffset[2*f+1]) + _legs[f]->currentY))
			/ Shift4Decimal;
			aux[2] = ((long)rotationMatrix[i][2] * _floorHeight) 
			/ Shift4Decimal;
			position[f][i] = aux[0] + aux[1] + aux[2];
		}	
	}

	// Relative coordinates
	for(byte f = 0; f < 6; f++)
	{
		position[f][0] = position[f][0]-(short)pgm_read_word(&CoxaOffset[2*f]);
		position[f][1] = position[f][1]-(short)pgm_read_word(&CoxaOffset[2*f+1]);
	}

	for(byte f = 0; f < 6; f++)
	{
		if(!_legs[f]->TryCalRotationPosition(position[f]))
			return;
	}

	for(byte f = 0; f < 6; f++)
	{
		if(_legs[f]->CalculateTimeMove(balanceSpeed) > timeMove)
			timeMove = _legs[f]->CalculateTimeMove(balanceSpeed);
	}

	for(byte f = 0; f < 6; f++)
	{
		if(!_legs[f]->TryUpdatePosition(timeMove))
			return;
	}
}

void Hexapod::Rotate(const short yawAngle)
{
	log("In Hexapod::Rotate", Debug);

	const short sinYaw = GetSin(yawAngle);
	const short cosYaw = GetCos(yawAngle);

	short aux[2];

	short rotationMatrix[2][2]={
		{cosYaw, -sinYaw},
		{sinYaw, cosYaw}
	};

	for(byte f = 0; f < 6;f++)
	{
		for (byte i = 0; i < 2; i++)
		{
			aux[0] = ((long)rotationMatrix[i][0] * _legs[f]->currentX) 
			/ Shift4Decimal;
			aux[1] = ((long)rotationMatrix[i][1] * _legs[f]->currentY)
			/ Shift4Decimal;
			if(i == 0)
				_position[f][i] = aux[0] + aux[1] - _legs[f]->currentX;
			else 
				_position[f][i] = aux[0] + aux[1] - _legs[f]->currentY;
		}
		_position[f][2] = _floorHeight;	
	}

	for (byte f = 0; f < 2; f++)
	{
		for (byte i = 0; i < 2; i++)
		{
			if (f == i)
				rotationMatrix[f][i]=GetCos(-yawAngle);
			else if (i == 0)
				rotationMatrix[f][i]=GetSin(-yawAngle);
			else 
				rotationMatrix[f][i]=-GetSin(-yawAngle);
		}
	}

	for (byte f = 0; f < 6; f++)
	{
		for(byte i = 0; i < 2; i++)
		{
			aux[0] = ((long)rotationMatrix[i][0] * _legs[f]->currentX) 
			/ Shift4Decimal;
			aux[1] = ((long)rotationMatrix[i][1] * _legs[f]->currentY)
			/ Shift4Decimal;
			if(i == 0)
				_negativePosition[f][i] = aux[0] + aux[1] - _legs[f]->currentX;
			else
				_negativePosition[f][i] = aux[0] + aux[1] - _legs[f]->currentY;
		}
		_negativePosition[f][2] = _floorHeight; 	
	}

	_rotate = true;

	Walk(0,0);
}

void Hexapod::Walk(const short x, const short y)
{
	log("In Hexapod::Walk", Debug);

	uint16_t timeMoveTransfer = 0;
	uint16_t timeMove = 0;

	const short gap = Gap;

	InitPosition( x, y);

	if(IsCollising())
		return;

	const byte steps = ReturnSteps();

	const uint8_t speed_body = _speed / (steps -1);

	for (byte current_step = 0; current_step < steps; current_step++)
	{
		byte previous = (current_step == 0) ? steps -1 : current_step -1;

		for(byte i = 0; i < 6; i++)
		{
			if(_sequence[i] == current_step)
			{
				if(!_legs[i]->TryCalTransferTrajectory(_position[i], gap))
					return;
			}
			if(_sequence[i] == previous)
				if(!_legs[i]->TryCalRelativePosition(_negativePosition[i]))
					return;
		}
		for(byte i=0;i<6;i++)
		{
			if((_sequence[i] == current_step) || (_sequence[i]== previous) )
				_legs[i]->WaitUntilStop();
		}

		for(byte i = 0; i < 6; i++)
		{
			if(_sequence[i] == current_step)
			{	
				if(timeMoveTransfer < _legs[i]->CalculateTimeMove(_speed))
					timeMoveTransfer = _legs[i]->CalculateTimeMove(_speed);
			}
				if(_sequence[i] == previous && current_step == 0)
					if( timeMove < _legs[i]->CalculateTimeMove(speed_body))
					timeMove = _legs[i]->CalculateTimeMove(speed_body);
		}
		
		timeMove = (timeMove > timeMoveTransfer * 2 * (steps -1)) ? 
			timeMove : timeMoveTransfer * 2 * (steps -1) ;

		for(byte i = 0; i < 6; i++)
		{
			if(_sequence[i] == current_step)
			{
				if(!_legs[i]->TryUpdatePosition(timeMoveTransfer))
					return;
			}
			
			if(_sequence[i] == previous)
				if(!_legs[i]->TryUpdatePosition(timeMove))
					return;
		}

		for(byte i=0;i<6;i++)
		{
			if(_sequence[i] == current_step)
				if(!_legs[i]->TryCalRelativePosition(_position[i]))
					return;
		}
		for(byte i=0;i<6;i++)
		{
			if(_sequence[i] == current_step)
				_legs[i]->WaitUntilStop();

		}
		for(byte i = 0; i < 6; i++)
		{
			if(_sequence[i] == current_step)
				if(timeMoveTransfer < _legs[i]->CalculateTimeMove(_speed))
					timeMoveTransfer = _legs[i]->CalculateTimeMove(_speed);
		}

		for(byte i = 0; i < 6; i++)
		{
			if(_sequence[i] == current_step)
				if(!_legs[i]->TryUpdatePosition(timeMoveTransfer))
					return;
		}
	}
}

void Hexapod::InitPosition(const short x, const short y)
{
	log("In Hexapod::InitPosition", Debug);

	if(!_rotate)
	{
		for(byte f = 0; f < 6; f++)
		{	
			_position[f][0] = x;
			_position[f][1] = y;
			_position[f][2] = _floorHeight;			

			for (byte i=0;i<2;i++)
			{
				_negativePosition[f][i] = - _position[f][i];
			}
			_negativePosition[f][2] = _position[f][2];
		}
	}

	_rotate = false;
}

byte Hexapod::ReturnSteps(void)
{
	log("In Hexapod::ReturnSteps", Debug);

	byte steps = 0;

	for(byte i = 0; i < 6; i++)
	{
		if(_sequence[i] > steps)
			steps = _sequence[i];
	}
	return ++steps;
}

bool Hexapod::IsCollising(void)
{
	log("In Hexapod::IsCollising", Debug);

	const short max_distance = (_position[0][0] > 0) ? _position[0][0]
		: - _position[0][0];

	if(max_distance >= (CoxaOffsetX +
		FootDistance * GetCos(LeftFrontCoxaDefaultAngle))/2)
	{
		log("Hexapod::IsCollising == true", Error);		
		return true;
	}
	
	return false;
}

void Hexapod::RiseSpeed(void)
{
	log("In Hexapod::RiseSpeed", Debug);

	if ( ((short)_speed + _speedStep) < 255)
		_speed += _speedStep;

	log("_speed", _speed, Debug);
}

void Hexapod::DecreaseSpeed(void)
{
	log("In Hexapod::DecreaseSpeed", Debug);

	if ( ((short)_speed - _speedStep) > 0)
		_speed -= _speedStep;

	log("_speed", _speed, Debug);
}

void Hexapod::ChangeSpeedStep(uint8_t speedStep)
{
	log("In Hexapod::ChangeSpeedStep", Debug);
	_speedStep = speedStep;
}

void Hexapod::ChangeSpeed(uint8_t speed)
{
	log("In Hexapod::ChangeSpeed", Debug);
	
	if(speed == 1)
		RiseSpeed();
	else if(speed == 0 )
		DecreaseSpeed();
	else if((speed >1) && (speed <= 255))
		_speed = speed;

	log("\% Speed", ((float)_speed*100)/255, Info);
}

void Hexapod::RiseHeight(void)
{
	log("In Hexapod::RiseHeight", Debug);
	if ( _floorHeight + _floorHeightStep > _MaxFloorHeight)
		_floorHeight -= _floorHeightStep;

}

void Hexapod::DecreaseHeight(void)
{
	log("In Hexapod::DecreaseHeight", Debug);
	if ( _floorHeight - _floorHeightStep < 0)
		_floorHeight += _floorHeightStep;

}

void Hexapod::ChangeHeightStep(uint8_t heightStep)
{
	log("In Hexapod::ChangeHeightStep", Debug);
	_floorHeightStep = heightStep;
}

void Hexapod::ChangeHeight(short height)
{
	log("In Hexapod::ChangeHeight", Debug);
	
	if(height == 1)
		RiseHeight();

	else if(height == 0)
		DecreaseHeight();

	else if((height < -1) && (height < _MaxFloorHeight ))
		_floorHeight = -height;

	log("Height distance:", -_floorHeight , Info);
}

void Hexapod::RiseFootDistance(void)
{
	log("In Hexapod::RiseFootDistance", Debug);
	if (_footDistance < CoxaLength + FemurLength + TibiaLength - 30)
		_footDistance += _footDistanceStep;
}

void Hexapod::DecreaseFootDistance(void)
{
	log("In Hexapod::DecreaseFootDistance", Debug);
	if (_footDistance > CoxaLength - 20)
		_footDistance -= _footDistanceStep;
}

void Hexapod::ChangeFootDistanceStep(uint8_t footDistanceStep)
{
	log("In Hexapod::ChangeFootDistanceStep", Debug);
	_footDistanceStep = footDistanceStep;
}

void Hexapod::ChangeFootDistance(short footDistance)
{
	log("In Hexapod::ChangeFootDistance", Debug);

	if(footDistance == 1)
		RiseFootDistance();
	else if(footDistance == 0)
		DecreaseFootDistance();
	else if((footDistance >1) && (footDistance < CoxaLength + FemurLength
		+ TibiaLength - 30) )
		_footDistance = footDistance;

	log("FootDistance:",_footDistance , Info);
}

void Hexapod::CalibrateLeg(const byte legNumber, const uint8_t member,
	const uint8_t angle)
{
	log("In Hexapod::CalibrateLeg", Debug);

	log("Ang [255]: validates FIRST calibration.", Info);
	log("Ang [254]: shows current calibration.", Info);
	log("Ang [253]: servos will move to the calibration value", Info);
	log("Ang [252]: servos won't move to the calibration value", Info);

	static uint8_t moveLegs = 0;

	if(legNumber > 5 || legNumber < 0)
	{
		log("LegNumber range [0-5]", Error);
		return;
	}

	if(angle == 252)
	{
		moveLegs = 0;
		log("Move and calibrate OFF", Warn);
		return;
	}

	if(angle == 253)
	{
		moveLegs = 1;
		LegsToCalibrationAngles();
		log("Move and calibrate ON", Warn);
		return;
	}

	if(angle == 255)
	{
		writeCalibrationCompleted();
		return;
	}

	if(angle == 254)
	{
		currentCalibration();
		return;
	}
	if(!moveLegs)
	{
		_legs[legNumber]->TrySaveValueCalibration(member, angle);
	}
	else
	{
		_legs[legNumber]->SaveCalibrationMovePosition(member, angle);
	}
	
}


void Hexapod::ChangeGait(uint8_t type, const uint8_t sequence[6])
{
	if(type ==1)
	{
		EnableTripodGait();
		return;
	}

	if(type == 2)
	{
		EnableRippleGait();
		return;
	}

	if(type == 0)
	{
		EnableCustomGait(sequence);
		return;
	}

	log("Gait type error",Error);

}

void Hexapod::EnableCustomGait(const uint8_t sequence[6])
{
	for (byte i = 0; i < 6; i++)
	{
		_sequence[i] = sequence[i];
	}

	GoDefaultPosition();
}

void Hexapod::MoveLeg(const byte legNumber, const uint16_t x,
	const uint16_t y, const uint16_t z)
{
	short foot_position[3];
	foot_position[0]= x;
	foot_position[1]= y;
	foot_position[2]= z;

	if(legNumber >5)
	{
		log("Leg number must be into 0 and 5",Error);
		return;
	}

	if(!_legs[legNumber]->TryCalculatePosition(foot_position))
		return;

	uint16_t timeMove = _legs[legNumber]->CalculateTimeMove(_speed);

	if(!_legs[legNumber]->TryUpdatePosition(timeMove))
		return;
}

void Hexapod::LegsToCalibrationAngles()
{
	for(byte i = 0;i < 6; i++)
	{
		_legs[i]->ServosToCalibrationAngles();
	}
}

void Hexapod::ReadSensors()
{
    #ifdef VoltageInPin

	_voltage = readVoltage();

	if(_voltage < BATTERY_CUTOFF_VOLTAGE)
		PowerOffServos();
    
    #endif
}

void Hexapod::PowerOnServos()
{
	#ifdef ServoPowerPin
	digitalWrite(ServoPowerPin, HIGH);
	#endif
}

void Hexapod::PowerOffServos()
{
	#ifdef ServoPowerPin
	digitalWrite(ServoPowerPin, LOW);
	#endif
}

Hexapod::~Hexapod()
{
	delete _legs;
}