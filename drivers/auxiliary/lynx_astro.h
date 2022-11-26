/*
    Lynx Astro Dew Controller INDI driver
    Copyright (C) 2022 Jarno Paananen

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

#pragma once

#include <defaultdevice.h>

#include "indipropertynumber.h"
#include "indipropertyswitch.h"
#include "indipropertytext.h"

/***************************** Lynx Astro Commands **************************/

#define LA_CMD_LEN 32
#define LA_RES_LEN 64

#define LA_VERSION_CMD ":GV#"
#define LA_VERSION_RES ":GV%8s#"

#define LA_GET_DEVICE_CMD ":GD#"
#define LA_GET_DEVICE_RES ":GD%d#"

#define LA_GET_SERIAL_CMD ":GS#"
#define LA_GET_SERIAL_RES ":GS%8s#"

#define LA_GET_FREQUENCY_CMD ":GF#"
#define LA_GET_FREQUENCY_RES ":GF%i#"

#define LA_SET_FREQUENCY_CMD ":SF%d#"
#define LA_SET_FREQUENCY_RES ":SF1#"

#define LA_GET_OUTPUT_STATE_CMD ":GA#"
#define LA_GET_OUTPUT_STATE_1_CHANNEL_RES ":GAA%d-#"
#define LA_GET_OUTPUT_STATE_4_CHANNEL_RES ":GAA%d-B%d-C%d-D%d#"

#define LA_SET_CHANNEL_CMD ":SC%d%04d#"
#define LA_SET_CHANNEL_RES ":SC1#"


/**************************** LynxAstro Constants **************************/

namespace Connection
{
class Serial;
};

class LynxAstro : public INDI::DefaultDevice
{
    public:
        LynxAstro();
        virtual ~LynxAstro() = default;

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual void TimerHit() override;

    private:
        bool sendCommand(const char *cmd, char *response);

        bool Handshake();
        bool Ack();

        bool reset();

        bool readStatus();
        
        bool getOutputPower(char* response);
        bool getPWMFrequency(char* response);

        bool setOutputPower(int channel, int power);
        bool setPWMFrequency(int setting);

        Connection::Serial *serialConnection{ nullptr };
        int PortFD{ -1 };

        int numChannels;

        INDI::PropertyNumber OutputNP{0}; // Set in runtime
        INDI::PropertySwitch PWMFrequencySP {4};

        INDI::PropertyText SerialNumberTP {1};
        INDI::PropertyText FWversionTP {1};
};
