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
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
   USA

*/

#include "lynx_astro.h"
#include "connectionplugins/connectionserial.h"
#include "indicom.h"

#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

#define LYNXASTRO_TIMEOUT 3

std::unique_ptr<LynxAstro> _LynxAstro(new LynxAstro());

LynxAstro::LynxAstro()
{
    setVersion(1, 0);
}

bool LynxAstro::initProperties()
{
    DefaultDevice::initProperties();

    PWMFrequencySP[0].fill("FREQ_732HZ", "732 Hz");
    PWMFrequencySP[1].fill("FREQ_2930HZ", "2.93 kHz");
    PWMFrequencySP[2].fill("FREQ_11700HZ", "11.7 kHz");
    PWMFrequencySP[3].fill("FREQ_47000HZ", "47 kHz");
    PWMFrequencySP.fill(getDeviceName(), "PWM_FREQUENCY", "PWM Frequency", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /* Serial number */
    SerialNumberTP[0].fill("NUMBER", "Number", "");
    SerialNumberTP.fill(getDeviceName(), "SERIAL", "Serial", OPTIONS_TAB, IP_RO, 0, IPS_IDLE);

    /* Firmware version */
    FWversionTP[0].fill("VERSION", "Firmware Version", "");
    FWversionTP.fill(getDeviceName(), "FIRMWARE", "Firmware", OPTIONS_TAB, IP_RO, 0, IPS_IDLE);

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

bool LynxAstro::updateProperties()
{
    DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(OutputNP);
        defineProperty(PWMFrequencySP);
        defineProperty(SerialNumberTP);
        defineProperty(FWversionTP);

        loadConfig(true);
        readStatus();
        LOG_INFO("Lynx Astro parameters updated, device ready for use.");
        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(OutputNP);
        deleteProperty(PWMFrequencySP);
        deleteProperty(SerialNumberTP);
        deleteProperty(FWversionTP);
    }

    return true;
}

const char *LynxAstro::getDefaultName()
{
    return "LynxAstro";
}

bool LynxAstro::sendCommand(const char *cmd, char *resp)
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
        if ((rc = tty_nread_section(PortFD, readbuffer, sizeof(readbuffer), '#', LYNXASTRO_TIMEOUT, &nbytes_read)) != TTY_OK)
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


bool LynxAstro::Handshake()
{
    PortFD = serialConnection->getPortFD();

    if (Ack())
    {
        LOG_INFO("Lynx Astro Dew Controller is online. Getting device parameters...");
        return true;
    }
    LOG_ERROR("Error retrieving data from Lynx Astro Dew Controller, please ensure controller "
              "is powered and the port is correct.");
    return false;
}

bool LynxAstro::Ack()
{
    char resp[LA_RES_LEN] = {};
    tcflush(PortFD, TCIOFLUSH);

    if (!sendCommand(LA_VERSION_CMD, resp))
        return false;

    char firmware[16];

    int ok = sscanf(resp, LA_VERSION_RES, firmware);

    if (ok != 1)
    {
        LOGF_ERROR("Lynx Astro not properly identified! Answer was: %s.", resp);
        return false;
    }

    FWversionTP[0].setText(firmware);
    FWversionTP.setState(IPS_OK);
    FWversionTP.apply();

    // Read device type for number of channels
    if(!sendCommand(LA_GET_DEVICE_CMD, resp))
        return false;

    ok = sscanf(resp, LA_GET_DEVICE_RES, &numChannels);
    if(ok != 1)
        return false;

    OutputNP.resize(numChannels);
    for(int i = 0; i < numChannels; ++i)
    {
        char name[32];
        char label[32];

        snprintf(name, sizeof(name), "CHANNEL%d", i+1);
        snprintf(label, sizeof(label), "Channel %d", i+1);

        OutputNP[i].fill(name, label, "%4.0f", 0., 1023., 1., 0.);
    }
    OutputNP.fill(getDeviceName(), "OUTPUT", "Outputs", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    if(!getPWMFrequency(resp))
        return false;

    int frequency;
    ok = sscanf(resp, LA_GET_FREQUENCY_RES, &frequency);
    if(ok != 1)
        return false;

    for(int i = 0; i < 4; ++i)
    {
        PWMFrequencySP[i].setState(i + 1 == frequency? ISS_ON: ISS_OFF);
    }
    PWMFrequencySP.setState(IPS_OK);
    PWMFrequencySP.apply();
    return true;
}

bool LynxAstro::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    if (PWMFrequencySP.isNameMatch(name))
    {
        PWMFrequencySP.update(states, names, n);
        PWMFrequencySP.setState(IPS_BUSY);
        PWMFrequencySP.apply();

        bool res = setPWMFrequency(PWMFrequencySP.findOnSwitchIndex());
        if (res)
        {
            PWMFrequencySP.setState(IPS_OK);
        }
        else
        {
            PWMFrequencySP.setState(IPS_ALERT);
        }
        PWMFrequencySP.apply();
        return true;
    }
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool LynxAstro::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    if(OutputNP->isNameMatch(name))
    {
        OutputNP->update(values, names, n);
        OutputNP->setState(IPS_BUSY);
        OutputNP->apply();

        for(int channel = 0; channel < numChannels; ++channel)
        {
            setOutputPower(channel, static_cast<int>(OutputNP[channel].getValue()));
        }
        return true;
    }
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool LynxAstro::readStatus()
{
    char resp[LA_RES_LEN];

    if(!getOutputPower(resp))
        return false;

    int power[4];
    if(numChannels == 1)
    {
        int num = sscanf(resp, LA_GET_OUTPUT_STATE_1_CHANNEL_RES, &power[0]);
        if(num != 1)
            return false;
    }
    else
    {
        int num = sscanf(resp, LA_GET_OUTPUT_STATE_4_CHANNEL_RES, &power[0], &power[1], &power[2], &power[3]);
        if(num != 4)
            return false;
    }
    for (int channel = 0; channel < numChannels; ++channel)
    {
        OutputNP[channel].setValue(power[channel]);
    }
    OutputNP.setState(IPS_OK);
    OutputNP.apply();
    return true;
}

bool LynxAstro::getOutputPower(char* resp)
{
    return sendCommand(LA_GET_OUTPUT_STATE_CMD, resp);
}

bool LynxAstro::setOutputPower(int channel, int power)
{
    char resp[16];
    char cmd[16];
    snprintf(cmd, sizeof(cmd) - 1, LA_SET_CHANNEL_CMD, channel, power);
    return sendCommand(cmd, resp);
}

bool LynxAstro::getPWMFrequency(char* resp)
{
    return sendCommand(LA_GET_FREQUENCY_CMD, resp);
}

bool LynxAstro::setPWMFrequency(int frequency)
{
    char resp[16];
    char cmd[16];
    snprintf(cmd, sizeof(cmd) - 1, LA_SET_FREQUENCY_CMD, frequency);
    return sendCommand(cmd, resp);
}

void LynxAstro::TimerHit()
{
    if (!isConnected())
        return;

    readStatus();
    SetTimer(getCurrentPollingPeriod());
}
