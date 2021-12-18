/*
    Arduheater INDI driver
    Copyright (C) 2021 Jarno Paananen

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

/***************************** Arduheater Commands **************************/

#define AH_CMD_LEN 32
#define AH_RES_LEN 64
#define AH_NUM_CHAN 4

#define AH_VERSION_CMD ":V#"
#define AH_VERSION_RES ":V%15s#"

#define AH_ENABLE_CMD ":+%i" // channel
#define AH_DISABLE_CMD ":-%i" // channel

#define AH_GET_AMBIENT_STATE_CMD ":A#"
#define AH_GET_AMBIENT_STATE_RES ":A%i,%i,%i#" // temperature, humidity, dewpoint

#define AH_GET_OUTPUT_STATE_CMD ":B%i#" // channel
#define AH_GET_OUTPUT_STATE_RES ":B%i,%i,%i,%i#" // channel, temperature, set_point, output_value

#define AH_GET_PID_CMD ":C%i#" // channel, Not implemented in current firmware
#define AH_GET_PID_RES ":C%i,%i,%i,%i,%i#" // channel, P term, I term, D term, u(?)

#define AH_GET_OUTPUT_CFG_CMD ":D%i#" // channel
#define AH_GET_OUTPUT_CFG_RES ":D%i,%i,%i,%i,%i,%i,%i,%i,%i#" 
// channel
// min_output, max_output, is_autostart,
// temp_offset, setpoint_offset),
// kp, ki, kd
#define AH_SET_OUTPUT_CFG_CMD ":D%i,%i,%i,%i,%i,%i,%i,%i,%i#" 

#define AH_GET_STATE_CMD ":?#"
#define AH_GET_STATE_RES ":?%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i#" // 
//        output[0].is_connected(), output[1].is_connected(), output[2].is_connected(), output[3].is_connected(),
//        output[0].is_ready(),     output[1].is_ready(),     output[2].is_ready(),     output[3].is_ready(),
//        output[0].is_enabled(),   output[1].is_enabled(),   output[2].is_enabled(),   output[3].is_enabled()"

#define AH_GET_SENSOR_CFG_CMD ":F%i#" // channel
// Notice the response below really has D even though the command has F
#define AH_GET_SENSOR_CFG_RES ":D%i,%i,%i,%i,%i#" // channel, nominal_temp, resistor_value, bcoefficient, nominal_value
#define AH_SET_SENSOR_CFG_CMD ":F%i,%i,%i,%i,%i#" // channel, nominal_temp, resistor_value, bcoefficient, nominal_value

#define AH_GET_AMBIENT_CFG_CMD ":G#"
#define AH_GET_AMBIENT_CFG_RES ":G%i,%i#" // temp_offset, rh_offset
#define AH_SET_AMBIENT_CFG_CMD ":G%i,%i#" // temp_offset, rh_offset


/**************************** Arduheater Constants **************************/

namespace Connection
{
class Serial;
};

class Arduheater : public INDI::DefaultDevice
{
    public:
        Arduheater();
        virtual ~Arduheater() = default;

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
        bool readSettings();

        bool getOutputState(char* response);
        bool getAmbientState(char* response);
        bool getOutputData(int channel, char* response);

        bool getOutputConfig(int channel, char* response);
        bool getSensorConfig(int channel, char* response);
        bool getAmbientConfig(char* response);

        bool setOutputEnable(int channel);
        bool setOutputConfig(int channel);
        bool setSensorConfig(int channel);
        bool setAmbientConfig();

        Connection::Serial *serialConnection{ nullptr };
        int PortFD{ -1 };

        INDI::PropertyText FWversionTP{1};

        // Status fields
        INDI::PropertyNumber TemperatureNP{AH_NUM_CHAN};
        INDI::PropertyNumber SetpointNP{AH_NUM_CHAN};
        INDI::PropertyNumber OutputNP{AH_NUM_CHAN};

        INDI::PropertySwitch ConnectedSP{AH_NUM_CHAN};
        INDI::PropertySwitch ReadySP{AH_NUM_CHAN};
        INDI::PropertySwitch EnabledSP{AH_NUM_CHAN};

        INDI::PropertyNumber AmbientNP{1};
        INDI::PropertyNumber HumidityNP{1};
        INDI::PropertyNumber DewpointNP{1};

        // Configuration fields
        INDI::PropertyNumber MinOutputNP{AH_NUM_CHAN};
        INDI::PropertyNumber MaxOutputNP{AH_NUM_CHAN};
        INDI::PropertySwitch AutostartSP{AH_NUM_CHAN};
        INDI::PropertyNumber TemperatureOffsetNP{AH_NUM_CHAN};
        INDI::PropertyNumber SetpointOffsetNP{AH_NUM_CHAN};

        INDI::PropertyNumber KpNP{AH_NUM_CHAN};
        INDI::PropertyNumber KiNP{AH_NUM_CHAN};
        INDI::PropertyNumber KdNP{AH_NUM_CHAN};

        INDI::PropertyNumber NominalTempNP{AH_NUM_CHAN};
        INDI::PropertyNumber ResistorValueNP{AH_NUM_CHAN};
        INDI::PropertyNumber BcoefficientNP{AH_NUM_CHAN};
        INDI::PropertyNumber NominalValueNP{AH_NUM_CHAN};

        INDI::PropertyNumber AmbientOffsetNP{1};
        INDI::PropertyNumber HumidityOffsetNP{1};
};
