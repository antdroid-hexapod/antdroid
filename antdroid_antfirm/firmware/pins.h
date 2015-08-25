/* pins.h: defined pins of different boards
 *
 * Copyright (C) 2015 Alexander Gil and Javier Román
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

// In arduino Mega using 12 to 23 motors will disable PWM on pins 11 and 12.

#ifndef pins_h
#define pins_h

#if MB(ANTDROID)
    #define LeftFrontCoxaPin        22
    #define LeftFrontFemurPin       23
    #define LeftFrontTibiaPin       24

    #define RightFrontCoxaPin       53
    #define RightFrontFemurPin      52
    #define RightFrontTibiaPin      51

    #define LeftMiddleCoxaPin       25
    #define LeftMiddleFemurPin      26
    #define LeftMiddleTibiaPin      27

    #define RightMiddleCoxaPin      50
    #define RightMiddleFemurPin     49
    #define RightMiddleTibiaPin     48

    #define LeftRearCoxaPin         28
    #define LeftRearFemurPin        29
    #define LeftRearTibiaPin        30

    #define RightRearCoxaPin        47
    #define RightRearFemurPin       46
    #define RightRearTibiaPin       45

    #define ServoAuxPin             34

    #define VoltageInPin            "A0"
    #define ServoOnPin              13

#endif

#if MB(PROTOTIPE)
    #define LeftFrontCoxaPin        52
    #define LeftFrontFemurPin       53
    #define LeftFrontTibiaPin       51

    #define RightFrontCoxaPin       49
    #define RightFrontFemurPin      48
    #define RightFrontTibiaPin      47

    #define LeftMiddleCoxaPin       46
    #define LeftMiddleFemurPin      45
    #define LeftMiddleTibiaPin      44

    #define RightMiddleCoxaPin      43
    #define RightMiddleFemurPin     42
    #define RightMiddleTibiaPin     41

    #define LeftRearCoxaPin         40
    #define LeftRearFemurPin        39
    #define LeftRearTibiaPin        38

    #define RightRearCoxaPin        37
    #define RightRearFemurPin       36
    #define RightRearTibiaPin       35

#endif

#endif