/* trig.cpp: Trigonometric functions optimized for Inverse Kinematic in hexapods
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

#include "trig.h"

//*****************************************************************************
// Tables
// ArcCosinus Table
// Table build in to 3 part to get higher accuracy near cos = 1. 
// The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad 
// = 0.521 deg.
//  -    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//  -    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//  -    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
// Since the tables are overlapping the full range of 127+127+64 is not
// necessary. Total bytes: 277

static const byte GetACos[] PROGMEM = {    
  255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,
  231,229,228,227,225,224,223,221,220,219,217,216,215,214,212,211,210,208,207,
  206,204,203,201,200,199,197,196,195,193,192,190,189,188,186,185,183,182,181,
  179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,157,155,154,152,
  150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,
  117,115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,
  72,72,72,71,71,71,70,70,70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,
  64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,59,59,58,58,58,57,57,57,56,56,
  55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,46,46,45,
  45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,
  31,30,29,28,28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,
  19,19,19,18,18,18,17,17,17,17,16,16,16,15,15,15,14,14,13,13,13,12,12,11,11,
  10,10,9,9,8,7,6,6,5,3,0 };//

// Sin table 90 deg, persision 0.5 deg [180 values]
static const word Get_Sin[] PROGMEM = {
  0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218,
  1305, 1391, 1478, 1564, 1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249,
  2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007, 3090, 3173, 3255,
  3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226,
  4305, 4383, 4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150,
  5224, 5299, 5372, 5446, 5519, 5591, 5664, 5735, 5807, 5877, 5948, 6018,
  6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819,
  6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547,
  7604, 7660, 7716, 7771, 7826, 7880, 7933, 7986, 8038, 8090, 8141, 8191,
  8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660, 8703, 8746,
  8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205,
  9238, 9271, 9304, 9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563,
  9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 9762, 9781, 9799, 9816,
  9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961,
  9969,9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000 };

//*****************************************************************************
// GetCos: Get cosinus from the angle +/- multiple circles
// Input: Angle in degree with one decimal.
// Return: Cosine of angle input with four decimals.
//*****************************************************************************

short GetCos(short AngleDeg1)
{
  short cos4;
  short ABSAngleDeg1;    //Absolute value of the Angle in Degrees, decimals = 1
  //Get the absolute value of AngleDeg
  if (AngleDeg1 < 0)
  {
  	ABSAngleDeg1 = AngleDeg1 * -1;
  }
  else
  {
  	ABSAngleDeg1 = AngleDeg1;
  }
  //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
  if (AngleDeg1 < 0)    //Negative values
  	AngleDeg1 = 3600-(ABSAngleDeg1-(3600*(ABSAngleDeg1/3600)));
  else                //Positive values
  	AngleDeg1 = ABSAngleDeg1-(3600*(ABSAngleDeg1/3600));

  if (AngleDeg1>=0 && AngleDeg1<=900)     // 0 to 90 deg
  {
  	cos4 = pgm_read_word(&Get_Sin[(900-(AngleDeg1))/5]);
  	return cos4;
  }     

  else if (AngleDeg1>900 && AngleDeg1<=1800)     // 90 to 180 deg
  {
  	cos4 = -pgm_read_word(&Get_Sin[(AngleDeg1-900)/5]);            
  	return cos4;
  }    
  else if (AngleDeg1>1800 && AngleDeg1<=2700) // 180 to 270 deg
  {
  	cos4 = -pgm_read_word(&Get_Sin[(2700-AngleDeg1)/5]);
  	return cos4;
  }    

  else if(AngleDeg1>2700 && AngleDeg1<=3600) // 270 to 360 deg
  {
  	cos4 = pgm_read_word(&Get_Sin[(AngleDeg1-2700)/5]);
  	return cos4;  
  }
}    

//*****************************************************************************
// GetSin: Get sinus from the angle +/- multiple circles
// Input: Angle in degree with one decimal.
// Return: Sinus of angle input with four decimals.
//*****************************************************************************

short GetSin(short AngleDeg1)
{
  short sen4;
  sen4 = GetCos(900-AngleDeg1);
  return sen4;
}

//*****************************************************************************
// GetArcCos: Get arc cosine from the cosine with 4 decimals
// Input: Cosine with four decimals.
// Return: Angle in radians with four decimals.
//*****************************************************************************

short GetArcCos(short cos4)
{
	short AngleRad4; 
  bool NegativeValue;    

	if (cos4 < 0)
	{
		cos4 = -cos4;
		NegativeValue = true;
	}
	else
		NegativeValue = false;

  //Limit cos4 to his maximal value
	cos4 = min(cos4,Shift4Decimal);

	if ((cos4>=0) && (cos4<9000))
	{
		AngleRad4 = (byte)pgm_read_byte(&GetACos[cos4/79]);
    AngleRad4 = ((long)AngleRad4 * 616/Shift1Decimal); //616=((Pi/2)*Shift1Decimal)/255)
    }    
	else if ((cos4>=9000) && (cos4<9900))
	{
	  AngleRad4 = (byte)pgm_read_byte(&GetACos[(cos4-9000)/8+114]);
    AngleRad4 = (long)((long)AngleRad4 * 616/Shift1Decimal);
  	}
	else if ((cos4>=9900) && (cos4<=10000))
	{
	  AngleRad4 = (byte)pgm_read_byte(&GetACos[(cos4-9900)/2+227]);
    AngleRad4 = (long)((long)AngleRad4 * 616/Shift1Decimal);
  }

  //Add negative sign
	if (NegativeValue)
		AngleRad4 = 31416 - AngleRad4;

return AngleRad4;
}    

//*****************************************************************************
// GetArcTan: Get simplyfied arc tangent function based on fixed point arc cos
// GetArcTan(X,Y,hip)= ArcTan(X/Y) 
// Input: X and Y.
// Return: Angle in radians with four decimals.
//*****************************************************************************

short GetArcTan (short x, short y, long XYhyp)
{
	short Atan4;
	short AngleRad4;

	AngleRad4 = GetArcCos (((long)y * Shift4Decimal) / XYhyp);
  
  if (x < 0)    // removed overhead... Atan4 = AngleRad4 * (y / abs(y));  
    Atan4 = -AngleRad4;
  else
  	Atan4 = AngleRad4;
  return Atan4;
}    

//*****************************************************************************
// Hypotenuse: Calculate hypotenuse
// Input: X and Y.
// Return: Hypotenuse.
//*****************************************************************************
short Hypotenuse (short x, short y)
{
  long hyp;

  hyp = Isqrt(((long)x * x ) + ((long)y * y));
  return hyp;
}

//*****************************************************************************
// Isqrt: Calculate squared.
// Input: Number.
// Return: Square.
//*****************************************************************************
unsigned long Isqrt (unsigned long n) 
{
	unsigned long root;
	unsigned long remainder;
	unsigned long  place;

	root = 0;
	remainder = n;
  place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

  while (place > remainder)
  	place = place >> 2;
  while (place)
  {
  	if (remainder >= root + place)
  	{
  		remainder = remainder - root - place;
  		root = root + (place << 1);
  	}
  	root = root >> 1;
  	place = place >> 2;
  }
  return root;
}

//*****************************************************************************
// Rad2Deg: Transform ange in radians to angle in degree.
// Input: Angle in radians with four decimals.
// Return: Angle in degree with one decimal.
//*****************************************************************************

short Rad2Deg(short AngleRad4)
{
  short AngleDeg1;

  AngleDeg1 = ((long)AngleRad4 * 180)/3141;

  return AngleDeg1;
}
