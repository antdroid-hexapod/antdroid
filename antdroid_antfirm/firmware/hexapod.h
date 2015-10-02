/* hexapod.h: definiton of hexapod class and its methos and attributes.
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

#ifndef hexapod_h
#define hexapod_h

#include "leg.h"
#include "trig.h"
#include "Configuration.h"
#include "log.h"
#include "uncopyable.h"
#include "calibration.h"


class Hexapod: private Uncopyable
{
	public:
		Hexapod(void);
		
		void Start(void);

		void Balance(const short pitch, const short roll, const short yaw);

		void EnableTripodGait(void);
		void EnableRippleGait(void);
		
		void Rotate(const short yawAngle);
		void Walk(const short x, const short y);
		
		void RiseSpeed(void);
		void DecreaseSpeed(void);
		void ChangeSpeedStep(uint8_t speedStep);
		void ChangeSpeed(uint8_t speed);

		void RiseHeight(void);
		void DecreaseHeight(void);
		void ChangeHeightStep(uint8_t heightStep);
		void ChangeHeight(short height);

		void RiseMode(void);
		void DecreaseMode(void);
		void ChangeMode(uint8_t mode);

		void RiseFootDistance(void);
		void DecreaseFootDistance(void);
		void ChangeFootDistanceStep(uint8_t footDistanceStep);
		void ChangeFootDistance(short footDistance);
		
		void CalibrateLeg(const byte legNumber, const uint8_t member,
			const uint8_t angle);
		
		void ChangeGait(uint8_t type, const uint8_t sequence[]);

		void MoveLeg(const byte legNumber, const uint16_t x,
			const uint16_t y, const uint16_t z);

		void LegsToCalibrationAngles(void);

		~Hexapod();

	private:
		Leg** _legs;
	
		short _floorHeight;
		short _floorHeightStep;
		short _MaxFloorHeight;
		short _MinFloorHeight;

		uint8_t _sequence[6];
		uint8_t _speed;
		uint8_t _speedStep;

		short _footDistance;
		short _footDistanceStep;

		short _position[6][3];
		short _negativePosition[6][3];

		bool _rotate;

		uint8_t _mode;

		void InitPosition(const short x, const short y);

		byte ReturnSteps(void);

		void GoStartingPostion(void);

		void EnableDefaultGait(void);

		void GoDefaultPosition(void);

		void CalculateMaxFloorHeight(void);

		bool IsCollising(void);

		void EnableCustomGait(const uint8_t sequence[]);
};



#endif