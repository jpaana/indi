/*
    USB_Focus v3 Focuser
    Copyright (C) 2016 Jarno Paananen (jarno.paananen@gmail.com)

    Essentially a search & replace job based on:

    Moonlite Focuser
    Copyright (C) 2013 Jasem Mutlaq (mutlaqja@ikarustech.com)

    and X2 driver from https://github.com/ando--io/X2USBFocus
    and https://github.com/ando--io/FocusControl

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef USBFOCUSV3_H
#define USBFOCUSV3_H

#include "indibase/indifocuser.h"

class USBFocus : public INDI::Focuser
{
public:
    USBFocus();
    ~USBFocus();

    typedef enum { FOCUS_HALF_STEP, FOCUS_FULL_STEP } FocusStepMode;
    typedef enum { ROTATION_CW, ROTATION_CCW } MotorRotation;

    virtual bool Connect();
    virtual bool Disconnect();
    const char * getDefaultName();
    virtual bool initProperties();
    virtual bool updateProperties();
    virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
    virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
    virtual IPState MoveAbsFocuser(uint32_t ticks);
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks);
    virtual bool SetFocuserSpeed(int speed);
    virtual bool AbortFocuser();
    virtual void TimerHit();

private:

    int PortFD;
    double targetPos, lastPos, lastTemperature;
    unsigned int currentSpeed;
    bool moveInProgress;
    unsigned int targetPosition;

    void GetFocusParams();
    bool reset();
    bool updateConfiguration();
    bool updateTemperature();
    bool updatePosition();
    bool isMoving();
    bool Ack();

    bool MoveFocuser(unsigned int position);
    bool setStepMode(FocusStepMode mode);
    bool setRotation(MotorRotation rotation);
    bool setSpeed(unsigned short speed);
    bool setMaxPos(unsigned int maxPos);
    bool setTemperatureMinMove(unsigned int minimum);
    bool setTemperatureCoefficient(unsigned int coefficient);
    bool setTemperatureCompensation(bool enable);

    INumber TemperatureN[1];
    INumberVectorProperty TemperatureNP;

    ISwitch StepModeS[2];
    ISwitchVectorProperty StepModeSP;

    ISwitch RotationS[2];
    ISwitchVectorProperty RotationSP;

    INumber MaxTravelN[1];
    INumberVectorProperty MaxTravelNP;

    INumber TemperatureSettingN[2];
    INumberVectorProperty TemperatureSettingNP;

    ISwitch TemperatureCompensateS[2];
    ISwitchVectorProperty TemperatureCompensateSP;

    ISwitch ResetS[1];
    ISwitchVectorProperty ResetSP;

    INumber FirmwareVersionN[1];
    INumberVectorProperty FirmwareVersionNP;
};

#endif // USBFOCUS_H
