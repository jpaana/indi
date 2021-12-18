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
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
   USA

*/

#include "arduheater.h"
#include "connectionplugins/connectionserial.h"
#include "indicom.h"

#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

#define ARDUHEATER_TIMEOUT 3

std::unique_ptr<Arduheater> _Arduheater(new Arduheater());

Arduheater::Arduheater()
{
    setVersion(1, 0);
}

bool Arduheater::initProperties()
{
    DefaultDevice::initProperties();

    for(int i = 0; i < AH_NUM_CHAN; ++i)
    {
        char name[32];
        char label[32];

        snprintf(name, sizeof(name), "CHANNEL%d", i);
        snprintf(label, sizeof(label), "Channel %d", i);

        TemperatureNP[i].fill(name, label, "%3.2f", -50., 70., 0., 0.);
        SetpointNP[i].fill(name, label, "%3.2f", -50., 70., 0., 0.);
        OutputNP[i].fill(name, label, "%3.0f", 0., 100., 0., 0.);

        ConnectedSP[i].fill(name, label, ISS_OFF);
        ReadySP[i].fill(name, label, ISS_OFF);
        EnabledSP[i].fill(name, label, ISS_OFF);

        MinOutputNP[i].fill(name, label, "%3.2f", 0., 100., 0., 0.);
        MaxOutputNP[i].fill(name, label, "%3.2f", 0., 100., 0., 0.);
        AutostartSP[i].fill(name, label, ISS_OFF);
        TemperatureOffsetNP[i].fill(name, label, "%3.2f", -50., 70., 0., 0.);
        SetpointOffsetNP[i].fill(name, label, "%3.2f", -50., 70., 0., 0.);

        KpNP[i].fill(name, label, "%3.2f", -100., 100., 0., 0.);
        KiNP[i].fill(name, label, "%3.2f", -100., 100., 0., 0.);
        KdNP[i].fill(name, label, "%3.2f", -100., 100., 0., 0.);

        NominalTempNP[i].fill(name, label, "%3.2f", -50., 70., 0., 0.);
        ResistorValueNP[i].fill(name, label, "%3.2f", 0., 10000., 0., 0.);
        BcoefficientNP[i].fill(name, label, "%3.2f", -50., 70., 0., 0.);
        NominalValueNP[i].fill(name, label, "%3.2f", -50., 70., 0., 0.);
    }
    TemperatureNP.fill(getDeviceName(), "TEMPERATURES", "Temperatures", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);
    SetpointNP.fill(getDeviceName(), "SETPOINT", "Set points", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);
    OutputNP.fill(getDeviceName(), "OUTPUT", "Outputs", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    ConnectedSP.fill(getDeviceName(), "CONNECTED", "Connected", MAIN_CONTROL_TAB, IP_RO, ISR_NOFMANY, 0, IPS_IDLE);
    ReadySP.fill(getDeviceName(), "READY", "Ready", MAIN_CONTROL_TAB, IP_RO, ISR_NOFMANY, 0, IPS_IDLE);
    EnabledSP.fill(getDeviceName(), "ENABLED", "Enabled", MAIN_CONTROL_TAB, IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    MinOutputNP.fill(getDeviceName(), "MIN_OUTPUT", "Min.Outputs", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    MaxOutputNP.fill(getDeviceName(), "MAX_OUTPUT", "Max.Outputs", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    AutostartSP.fill(getDeviceName(), "AUTOSTART", "Autostart", OPTIONS_TAB, IP_RW, ISR_NOFMANY, 0, IPS_IDLE);
    TemperatureOffsetNP.fill(getDeviceName(), "TEMPERATURE_OFFSET", "Temperature offsets", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    SetpointOffsetNP.fill(getDeviceName(), "SETPOINT_OFFSET", "Setpoint offsets", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    KpNP.fill(getDeviceName(), "PID_P", "PID P", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    KiNP.fill(getDeviceName(), "PID_I", "PID I", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    KdNP.fill(getDeviceName(), "PID_D", "PID D", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    NominalTempNP.fill(getDeviceName(), "NOMINAL_TEMPERATURE", "Nominal temperatures", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    ResistorValueNP.fill(getDeviceName(), "RESISTOR_VALUE", "Resistor values", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    BcoefficientNP.fill(getDeviceName(), "B_COEFFICIENT", "B coeffifient", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    NominalValueNP.fill(getDeviceName(), "NOMINAL_VALUE", "Nominal values", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    AmbientNP[0].fill("AMBIENT", "Ambient", "%3.2f", -50., 70., 0., 0.);
    AmbientNP.fill(getDeviceName(), "TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);
    HumidityNP[0].fill("AMBIENT", "Ambient", "%3.2f", 0., 100., 0., 0.);
    HumidityNP.fill(getDeviceName(), "HUMIDITY", "Humidity", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);
    DewpointNP[0].fill("AMBIENT", "Ambient", "%3.2f", -50., 70., 0., 0.);
    DewpointNP.fill(getDeviceName(), "DEWPOINT", "Dew point", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    AmbientOffsetNP[0].fill("OFFSET", "Offset", "%3.2f", -50., 70., 0., 0.);
    AmbientOffsetNP.fill(getDeviceName(), "TEMPERATURE", "Temperature", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    HumidityOffsetNP[0].fill("OFFSET", "Offset", "%3.2f", -100., 100., 0., 0.);
    HumidityOffsetNP.fill(getDeviceName(), "HUMIDITY", "Humidity", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    /* Firmware version */
    FWversionTP[0].fill("FIRMWARE", "Firmware Version", "");
    FWversionTP.fill(getDeviceName(), "FW_VERSION", "Firmware", OPTIONS_TAB, IP_RO, 0, IPS_IDLE);

    setDriverInterface(AUX_INTERFACE);

    //    addDebugControl();
    //    addConfigurationControl();
    addAuxControls();
    setDefaultPollingPeriod(10000);
    addPollPeriodControl();

    // No simulation control for now

    serialConnection = new Connection::Serial(this);
    serialConnection->registerHandshake([&]()
    {
        return Handshake();
    });
    registerConnection(serialConnection);
    return true;
}

bool Arduheater::updateProperties()
{
    DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&TemperatureNP);
        defineProperty(&SetpointNP);
        defineProperty(&OutputNP);

        defineProperty(&ConnectedSP);
        defineProperty(&ReadySP);
        defineProperty(&EnabledSP);

        defineProperty(&AmbientNP);
        defineProperty(&HumidityNP);
        defineProperty(&DewpointNP);

        defineProperty(&MinOutputNP);
        defineProperty(&MaxOutputNP);
        defineProperty(&AutostartSP);
        defineProperty(&TemperatureOffsetNP);
        defineProperty(&SetpointOffsetNP);

        defineProperty(&KpNP);
        defineProperty(&KiNP);
        defineProperty(&KdNP);

        defineProperty(&NominalTempNP);
        defineProperty(&ResistorValueNP);
        defineProperty(&BcoefficientNP);
        defineProperty(&NominalValueNP);

        defineProperty(&AmbientOffsetNP);
        defineProperty(&HumidityOffsetNP);

        defineProperty(&FWversionTP);

        loadConfig(true);
        readSettings();
        readStatus();
        LOG_INFO("Arduheater parameters updated, device ready for use.");
        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(TemperatureNP.getName());
        deleteProperty(SetpointNP.getName());
        deleteProperty(OutputNP.getName());

        deleteProperty(ConnectedSP.getName());
        deleteProperty(ReadySP.getName());
        deleteProperty(EnabledSP.getName());

        deleteProperty(AmbientNP.getName());
        deleteProperty(HumidityNP.getName());
        deleteProperty(DewpointNP.getName());

        deleteProperty(MinOutputNP.getName());
        deleteProperty(MaxOutputNP.getName());
        deleteProperty(AutostartSP.getName());
        deleteProperty(TemperatureOffsetNP.getName());
        deleteProperty(SetpointOffsetNP.getName());

        deleteProperty(KpNP.getName());
        deleteProperty(KiNP.getName());
        deleteProperty(KdNP.getName());

        deleteProperty(NominalTempNP.getName());
        deleteProperty(ResistorValueNP.getName());
        deleteProperty(BcoefficientNP.getName());
        deleteProperty(NominalValueNP.getName());

        deleteProperty(AmbientOffsetNP.getName());
        deleteProperty(HumidityOffsetNP.getName());

        deleteProperty(FWversionTP.getName());
    }

    return true;
}

const char *Arduheater::getDefaultName()
{
    return "Arduheater";
}

bool Arduheater::sendCommand(const char *cmd, char *resp)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;
    char errstr[MAXRBUF];
    LOGF_DEBUG("CMD: %s.", cmd);

    tcflush(PortFD, TCIOFLUSH);
    if ((rc = tty_write(PortFD, cmd, strlen(cmd), &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Error writing command %s: %s.", cmd, errstr);
        return false;
    }

    if (resp)
    {
        // All commands and responses have format :something#
        static char readbuffer[128];
        if ((rc = tty_nread_section(PortFD, readbuffer, sizeof(readbuffer), '#', ARDUHEATER_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(rc, errstr, MAXRBUF);
            LOGF_ERROR("Error reading response for command %s: %s.", cmd, errstr);
            return false;
        }

        if (nbytes_read < 2)
        {
            LOGF_ERROR("Invalid response for command %s: %s.", cmd, resp);
            return false;
        }
        // remove extra characters before : and after # inclusively
        char* start = strchr(readbuffer, ':');
        char* end = strrchr(readbuffer, '#');
        ptrdiff_t cmdLen = end - start + 1;
        memcpy(resp, start, cmdLen);
        resp[cmdLen] = '\0';
        LOGF_DEBUG("RES: %s.", resp);
    }
    return true;
}


bool Arduheater::Handshake()
{
    PortFD = serialConnection->getPortFD();

    if (Ack())
    {
        LOG_INFO("Arduheater is online. Getting device parameters...");
        return true;
    }
    LOG_ERROR("Error retrieving data from Arduheater, please ensure controller "
              "is powered and the port is correct.");
    return false;
}

bool Arduheater::Ack()
{
    return true;
    char resp[AH_RES_LEN] = {};
    tcflush(PortFD, TCIOFLUSH);

    if (!sendCommand(AH_VERSION_CMD, resp))
        return false;

    char firmware[16];

    int ok = sscanf(resp, AH_VERSION_RES, firmware);

    if (ok != 1)
    {
        LOGF_ERROR("Arduheater not properly identified! Answer was: %s.", resp);
        return false;
    }

    FWversionTP[0].setText(firmware);
    FWversionTP.setState(IPS_OK);
    FWversionTP.apply();
    return true;
}

bool Arduheater::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    if (EnabledSP.isNameMatch(name))
    {
        EnabledSP.update(states, names, n);
        EnabledSP.setState(IPS_BUSY);
        EnabledSP.apply();
        for(int channel = 0; channel < AH_NUM_CHAN; ++channel)
        {
            setOutputEnable(channel);
        }
        return true;
    }

    if (AutostartSP.isNameMatch(name))
    {
        AutostartSP.update(states, names, n);
        AutostartSP.setState(IPS_BUSY);
        AutostartSP.apply();
        for(int channel = 0; channel < AH_NUM_CHAN; ++channel)
        {
            setOutputConfig(channel);
        }
        return true;
    }
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool Arduheater::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    for ( auto prop :
            {
                &AmbientOffsetNP, &HumidityOffsetNP
            })
    {
        if(prop->isNameMatch(name))
        {
            prop->update(values, names, n);
            prop->setState(IPS_BUSY);
            prop->apply();

            setAmbientConfig();
            readSettings();
            return true;
        }
    }

    for ( auto prop :
            {
                &MinOutputNP, &MaxOutputNP, &TemperatureOffsetNP, &SetpointOffsetNP, &KpNP, &KiNP, &KdNP
            })
    {
        if(prop->isNameMatch(name))
        {
            prop->update(values, names, n);
            prop->setState(IPS_BUSY);
            prop->apply();

            for(int channel = 0; channel < AH_NUM_CHAN; ++channel)
            {
                setOutputConfig(channel);
            }
            return true;
        }
    }

    for ( auto prop :
            {
                &NominalTempNP, &ResistorValueNP, &BcoefficientNP, &NominalValueNP
            })
    {
        if(prop->isNameMatch(name))
        {
            prop->update(values, names, n);
            prop->setState(IPS_BUSY);
            prop->apply();

            for(int channel = 0; channel < AH_NUM_CHAN; ++channel)
            {
                setSensorConfig(channel);
            }
            return true;
        }
    }
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool Arduheater::readStatus()
{
    char resp[AH_RES_LEN];

    if(!getOutputState(resp))
        return false;

    int is_connected[AH_NUM_CHAN];
    int is_ready[AH_NUM_CHAN];
    int is_enabled[AH_NUM_CHAN];

    int num = sscanf(resp, AH_GET_STATE_RES,
                     &is_connected[0], &is_connected[1], &is_connected[2], &is_connected[3],
                     &is_ready[0], &is_ready[1], &is_ready[2], &is_ready[3],
                     &is_enabled[0], &is_enabled[1], &is_enabled[2], &is_enabled[3]);
    if (num != 12)
        return false;

    for(int channel = 0; channel < AH_NUM_CHAN; ++channel)
    {
        ConnectedSP[channel].setState(is_connected[channel] ? ISS_ON : ISS_OFF);
        ReadySP[channel].setState(is_ready[channel] ? ISS_ON : ISS_OFF);
        EnabledSP[channel].setState(is_enabled[channel] ? ISS_ON : ISS_OFF);
    }
    for ( auto prop :
            {
                &ConnectedSP, &ReadySP, &EnabledSP
            })
    {
        prop->setState(IPS_OK);
        prop->apply();
    }

    if (!getAmbientState(resp))
        return false;

    int temperature, humidity, dewpoint;
    num = sscanf(resp, AH_GET_AMBIENT_STATE_RES, &temperature, &humidity, &dewpoint);
    if(num != 3)
        return false;

    AmbientNP[0].setValue(temperature);
    HumidityNP[0].setValue(humidity);
    DewpointNP[0].setValue(dewpoint);

    for ( auto prop :
            {
                &AmbientNP, &HumidityNP, &DewpointNP
            })
    {
        prop->setState(IPS_OK);
        prop->apply();
    }

    for (int channel = 0; channel < AH_NUM_CHAN; ++channel)
    {
        if (!getOutputData(channel, resp))
            return false;

        int temperature;
        int setpoint;
        int output;
        int chan;
        num = sscanf(resp, AH_GET_OUTPUT_STATE_RES, &chan, &temperature, &setpoint, &output);
        if (num != 4)
            return false;

        TemperatureNP[channel].setValue(temperature);
        SetpointNP[channel].setValue(setpoint);
        OutputNP[channel].setValue(output);
    }
    for ( auto prop :
            {
                &TemperatureNP, &SetpointNP, &OutputNP
            })
    {
        prop->setState(IPS_OK);
        prop->apply();
    }
    return true;
}

bool Arduheater::readSettings()
{
    char resp[AH_RES_LEN];

    for (int channel = 0; channel < AH_NUM_CHAN; ++channel)
    {
        if(!getOutputConfig(channel, resp))
            return false;

        int chan, min_output, max_output, autostart, temp_offset, setpoint_offset;
        int kp, ki, kd;

        int num = sscanf(resp, AH_GET_OUTPUT_CFG_RES, &chan, &min_output,
                         &max_output, &autostart, &temp_offset, &setpoint_offset,
                         &kp, &ki, &kd);
        if (num != 9)
            return false;

        MinOutputNP[channel].setValue(min_output);
        MaxOutputNP[channel].setValue(max_output);
        AutostartSP[channel].setState(autostart ? ISS_ON : ISS_OFF);
        TemperatureOffsetNP[channel].setValue(temp_offset);
        SetpointOffsetNP[channel].setValue(setpoint_offset);
        KpNP[channel].setValue(kp);
        KiNP[channel].setValue(ki);
        KdNP[channel].setValue(kd);

        if(!getSensorConfig(channel, resp))
            return false;

        int nominal_temp, resistor_value, bcoefficient, nominal_value;
        num = sscanf(resp, AH_GET_SENSOR_CFG_RES, &chan, &nominal_temp,
                     &resistor_value, &bcoefficient, &nominal_value);
        if (num != 5)
            return false;

        NominalTempNP[channel].setValue(nominal_temp);
        ResistorValueNP[channel].setValue(resistor_value);
        BcoefficientNP[channel].setValue(bcoefficient);
        NominalValueNP[channel].setValue(nominal_value);
    }

    for ( auto prop :
            {
                &MinOutputNP, &MaxOutputNP, &TemperatureOffsetNP, &SetpointOffsetNP, &KpNP, &KiNP, &KpNP, &NominalValueNP, &ResistorValueNP,
                &BcoefficientNP, &NominalValueNP
            })
    {
        prop->setState(IPS_OK);
        prop->apply();
    }

    AutostartSP.setState(IPS_OK);
    AutostartSP.apply();

    if(!getAmbientConfig(resp))
        return false;

    int temp_offset, humidity_offset;
    int num = sscanf(resp, AH_GET_AMBIENT_CFG_RES, &temp_offset, &humidity_offset);
    if (num != 2)
        return false;

    AmbientOffsetNP[0].setValue(temp_offset);
    HumidityOffsetNP[0].setValue(humidity_offset);
    for ( auto prop :
            {
                &AmbientOffsetNP, &HumidityOffsetNP
            })
    {
        prop->setState(IPS_OK);
        prop->apply();
    }
    return true;
}

bool Arduheater::getOutputState(char* resp)
{
    return sendCommand(AH_GET_STATE_CMD, resp);
}

bool Arduheater::getAmbientState(char* resp)
{
    return sendCommand(AH_GET_AMBIENT_STATE_CMD, resp);
}

bool Arduheater::getOutputData(int channel, char* resp)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd) - 1, AH_GET_OUTPUT_STATE_CMD, channel);
    return sendCommand(cmd, resp);
}

bool Arduheater::getOutputConfig(int channel, char* resp)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd) - 1, AH_GET_OUTPUT_CFG_CMD, channel);
    return sendCommand(cmd, resp);
}

bool Arduheater::getSensorConfig(int channel, char* resp)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd) - 1, AH_GET_SENSOR_CFG_CMD, channel);
    return sendCommand(cmd, resp);
}

bool Arduheater::getAmbientConfig(char* resp)
{
    return sendCommand(AH_GET_AMBIENT_CFG_CMD, resp);
}

bool Arduheater::setOutputEnable(int channel)
{
    char resp[16];
    char cmd[16];
    if(EnabledSP[channel].getState() == ISS_ON)
    {
        snprintf(cmd, sizeof(cmd) - 1, AH_ENABLE_CMD, channel);
    }
    else
    {
        snprintf(cmd, sizeof(cmd) - 1, AH_DISABLE_CMD, channel);
    }
    return sendCommand(cmd, resp);
}

bool Arduheater::setOutputConfig(int channel)
{
    char resp[64];
    char cmd[64];
    snprintf(cmd, sizeof(cmd) - 1, AH_SET_OUTPUT_CFG_CMD, channel,
             static_cast<int16_t>(MinOutputNP[channel].getValue()),
             static_cast<int16_t>(MaxOutputNP[channel].getValue()),
             AutostartSP[channel].getState() == ISS_ON ? 1 : 0,
             static_cast<int16_t>(TemperatureOffsetNP[channel].getValue()),
             static_cast<int16_t>(SetpointOffsetNP[channel].getValue()),
             static_cast<int16_t>(KpNP[channel].getValue()),
             static_cast<int16_t>(KiNP[channel].getValue()),
             static_cast<int16_t>(KdNP[channel].getValue()));
    return sendCommand(cmd, resp);
}

bool Arduheater::setSensorConfig(int channel)
{
    char resp[64];
    char cmd[64];
    snprintf(cmd, sizeof(cmd) - 1, AH_SET_SENSOR_CFG_CMD, channel,
             static_cast<int16_t>(NominalTempNP[channel].getValue()),
             static_cast<int16_t>(ResistorValueNP[channel].getValue()),
             static_cast<int16_t>(BcoefficientNP[channel].getValue()),
             static_cast<int16_t>(NominalValueNP[channel].getValue()));
    return sendCommand(cmd, resp);
}

bool Arduheater::setAmbientConfig()
{
    char cmd[16];
    char resp[16];
    int16_t temperatureOffset = static_cast<int16_t>(AmbientOffsetNP[0].getValue());
    int16_t humidityOffset = static_cast<int16_t>(HumidityOffsetNP[0].getValue());

    snprintf(cmd, sizeof(cmd) - 1, AH_SET_AMBIENT_CFG_CMD, temperatureOffset, humidityOffset);
    return sendCommand(cmd, resp);
}

void Arduheater::TimerHit()
{
    if (!isConnected())
        return;

    // Get temperatures etc.
    readStatus();
    SetTimer(getCurrentPollingPeriod());
}
