/*
    USB_Focus v3 Focuser
    Copyright (C) 2016 Jarno Paananen (jarno.paananen@gmail.com)

    Essentially a search & replace job based on:

    Moonlite Focuser
    Copyright (C) 2013 Jasem Mutlaq (mutlaqja@ikarustech.com)

    and X2 driver from https://github.com/ando--io/X2USBFocus
    and FocusControl from https://github.com/ando--io/FocusControl

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

#include "usbfocusv3.h"
#include "indicom.h"

#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <memory>

#define USBFOCUS_TIMEOUT   3

#define POLLMS  3000

std::unique_ptr<USBFocus> usbFocus(new USBFocus());

void ISGetProperties(const char *dev)
{
    usbFocus->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    usbFocus->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
    usbFocus->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    usbFocus->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
   INDI_UNUSED(dev);
   INDI_UNUSED(name);
   INDI_UNUSED(sizes);
   INDI_UNUSED(blobsizes);
   INDI_UNUSED(blobs);
   INDI_UNUSED(formats);
   INDI_UNUSED(names);
   INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
    usbFocus->ISSnoopDevice(root);
}

#define GET_POSITION_CMD "FPOSRO"
#define GET_TEMPERATURE_CMD "FTMPRO"
#define GET_SETTINGS_CMD "SGETAL"
#define GET_SIGN_CMD "FTAXXA"

#define SET_STEPMODE_H_CMD "SMSTPD"
#define SET_STEPMODE_F_CMD "SMSTPF"

#define SET_ROTATION_CW_CMD "SMROTH"
#define SET_ROTATION_ACW_CMD "SMROTT"

#define SET_SPEED_CMD "SMO%03d"
#define SET_MAXPOS_CMD "M%05d"

#define MOVE_IN_CMD "I%05d"
#define MOVE_OUT_CMD "O%05d"

#define SET_TMPCOEF_CMD "FLA%03d"
#define SET_TMPCOEFSIGN_CMD "FZAXX%01d"
#define SET_TMPCOEFTHR_CMD "SMA%03d"

#define STOP_CMD "FQUITx"
#define RESET_CMD "FMMODE" // ? Windows driver seems to use this for something like reset
#define RESET_EEPROM_CMD "SEERAZ"

USBFocus::USBFocus()
{
    // Can move in Absolute & Relative motions, can AbortFocuser motion, and has variable speed.
    SetFocuserCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_HAS_VARIABLE_SPEED);

    lastPos = 0;
    lastTemperature = 0;
    moveInProgress = false;
}

USBFocus::~USBFocus()
{

}

bool USBFocus::initProperties()
{
    INDI::Focuser::initProperties();

    FocusSpeedN[0].min = 1;
    FocusSpeedN[0].max = 5;
    FocusSpeedN[0].value = 1;

    /* Port */
    IUFillText(&PortT[0], "PORT", "Port", "/dev/focuser");
    IUFillTextVector(&PortTP, PortT, 1, getDeviceName(), "DEVICE_PORT", "Ports", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    /* Step Mode */
    IUFillSwitch(&StepModeS[0], "Half Step", "", ISS_OFF);
    IUFillSwitch(&StepModeS[1], "Full Step", "", ISS_ON);
    IUFillSwitchVector(&StepModeSP, StepModeS, 2, getDeviceName(), "Step Mode", "", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /* Rotation */
    IUFillSwitch(&RotationS[0], "Clockwise", "", ISS_OFF);
    IUFillSwitch(&RotationS[1], "Counter clockwise", "", ISS_ON);
    IUFillSwitchVector(&RotationSP, RotationS, 2, getDeviceName(), "Rotation", "", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /* Focuser temperature */
    IUFillNumber(&TemperatureN[0], "TEMPERATURE", "Celsius", "%6.2f", -50, 70., 0., 0.);
    IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Maximum Travel
    IUFillNumber(&MaxTravelN[0], "MAXTRAVEL", "Maximum travel", "%6.0f", 1., 60000., 0., 10000.);
    IUFillNumberVector(&MaxTravelNP, MaxTravelN, 1, getDeviceName(), "FOCUS_MAXTRAVEL", "Max. travel", OPTIONS_TAB, IP_RW, 0, IPS_IDLE );

    // Temperature Settings
    IUFillNumber(&TemperatureSettingN[0], "Coefficient", "", "%3.0f", -999., 999., 1., 15.);
    IUFillNumber(&TemperatureSettingN[1], "Minimum move", "", "%3.0f", 0., 999., 1., 15.);
    IUFillNumberVector(&TemperatureSettingNP, TemperatureSettingN, 2, getDeviceName(), "Temperature Settings", "", OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Compensate for temperature
    IUFillSwitch(&TemperatureCompensateS[0], "Enable", "", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateS[1], "Disable", "", ISS_ON);
    IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "Temperature Compensate", "", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Reset
    IUFillSwitch(&ResetS[0], "Zero", "", ISS_OFF);
    IUFillSwitchVector(&ResetSP, ResetS, 1, getDeviceName(), "Reset", "", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Firmware version
    IUFillNumber(&FirmwareVersionN[0], "Version", "", "%4.0f", 0., 9999., 1., 0.);
    IUFillNumberVector(&FirmwareVersionNP, FirmwareVersionN, 1, getDeviceName(), "FIRMWARE_VERSION", "Firmware", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    /* Relative and absolute movement */
    FocusRelPosN[0].min = 0.;
    FocusRelPosN[0].max = 30000.;
    FocusRelPosN[0].value = 0;
    FocusRelPosN[0].step = 1000;

    FocusAbsPosN[0].min = 0.;
    FocusAbsPosN[0].max = 60000.;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step = 1000;

    addDebugControl();

    return true;
}

bool USBFocus::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineNumber(&TemperatureNP);
        defineNumber(&MaxTravelNP);
        defineSwitch(&StepModeSP);
        defineSwitch(&RotationSP);
        defineNumber(&TemperatureSettingNP);
        defineSwitch(&TemperatureCompensateSP);
        defineSwitch(&ResetSP);
        defineNumber(&FirmwareVersionNP);

        GetFocusParams();

        loadConfig(true);

        DEBUG(INDI::Logger::DBG_SESSION, "USBFocus paramaters updated, focuser ready for use.");
    }
    else
    {

        deleteProperty(TemperatureNP.name);
        deleteProperty(MaxTravelNP.name);
        deleteProperty(StepModeSP.name);
        deleteProperty(RotationSP.name);
        deleteProperty(TemperatureSettingNP.name);
        deleteProperty(TemperatureCompensateSP.name);
        deleteProperty(ResetSP.name);
        deleteProperty(FirmwareVersionNP.name);
    }
    return true;
}

bool USBFocus::Connect()
{
    int connectrc=0;
    char errorMsg[MAXRBUF];

    if ( (connectrc = tty_connect(PortT[0].text, 19200, 8, 0, 1, &PortFD)) != TTY_OK)
    {
        tty_error_msg(connectrc, errorMsg, MAXRBUF);

        DEBUGF(INDI::Logger::DBG_SESSION, "Failed to connect to port %s. Error: %s", PortT[0].text, errorMsg);

        return false;
    }

    if (Ack())
    {
        DEBUG(INDI::Logger::DBG_SESSION, "USBFocus is online. Getting focus parameters...");
        SetTimer(POLLMS);
        return true;
    }

    DEBUG(INDI::Logger::DBG_SESSION, "Error retreiving data from USBFocus, please ensure USBFocus controller is powered and the port is correct.");
    return false;
}

bool USBFocus::Disconnect()
{
    tty_disconnect(PortFD);
    DEBUG(INDI::Logger::DBG_SESSION, "USBFocus is offline.");
    return true;
}

const char * USBFocus::getDefaultName()
{
    return "USB_Focus V3";
}

bool USBFocus::Ack()
{
    int nbytes_written=0, nbytes_read=0, rc=-1;
    char errstr[MAXRBUF];
    char resp[5];
    int retries = 6; // Maximum that need to be send

    // Sync with the device by sending space characters until we get reply ER=1
    tcflush(PortFD, TCIOFLUSH);
    while(retries)
    {
        if ( (rc = tty_write(PortFD, " ", 1, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(rc, errstr, MAXRBUF);
            DEBUGF(INDI::Logger::DBG_ERROR, "ack error: %s.", errstr);
            return false;
        }

        // Synchronize commands with the device
        if ( (rc = tty_read(PortFD, resp, 4, 1, &nbytes_read)) == TTY_OK)
        {
            resp[4] = 0;
            DEBUGF(INDI::Logger::DBG_SESSION, "ack reply: %s.", resp);
            if (strncmp("ER=1", resp, 4) == 0)
            {
                return true; // Probably valid USB_Focus V3 found!
            }
        }
        retries--;
    }
    return false;
}

bool USBFocus::updateConfiguration()
{
    int nbytes_written=0, nbytes_read=0, rc=-1;
    char errstr[MAXRBUF];
    char resp[29];

    tcflush(PortFD, TCIOFLUSH);

    if ( (rc = tty_write(PortFD, GET_SETTINGS_CMD, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updateConfiguration error: %s.", errstr);
        return false;
    }

    if ( (rc = tty_read(PortFD, resp, 28, USBFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updateConfiguration error: %s.", errstr);
        return false;
    }

    resp[28]='\0';

    if (resp[0] == 'C' &&
        resp[1] == '=')
    {
        DEBUGF(INDI::Logger::DBG_DEBUG, "updateConfiguration string: %s", resp);

        // C=1-0-2-015-010-1505-00000
        int rotation, stepmode, speed, tempcoeff, tempmini, firmware, maxMove;
        rc = sscanf(resp, "C=%d-%d-%d-%d-%d-%d-%d", &rotation, &stepmode, &speed, &tempcoeff, &tempmini, &firmware, &maxMove);

        DEBUGF(INDI::Logger::DBG_DEBUG, "rc %d rot %d step %d speed %d co %d min %d firm %d max %d",
               rc, rotation, stepmode, speed, tempcoeff, tempmini, firmware, maxMove);

        if (rc == 7)
        {
            IUResetSwitch(&RotationSP);
            RotationS[rotation].s = ISS_ON;

            IUResetSwitch(&StepModeSP);
            StepModeS[stepmode^1].s = ISS_ON;

            FocusSpeedN[0].value = speed;
            TemperatureSettingN[0].value = tempcoeff;
            TemperatureSettingN[1].value = tempmini;
            FirmwareVersionN[0].value = firmware;
            MaxTravelN[0].value = maxMove;
        }
        else
        {
            DEBUGF(INDI::Logger::DBG_ERROR, "Unknown error: configuration value (%s)", resp);
            return false;
        }
    }

    // Read temperature coefficient sign
    if ( (rc = tty_write(PortFD, GET_SIGN_CMD, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updateConfiguration error: %s.", errstr);
        return false;
    }

    if ( (rc = tty_read(PortFD, resp, 3, USBFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updateConfiguration error: %s.", errstr);
        return false;
    }
    resp[3] = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "updateConfiguration string: %s", resp);
    if (resp[0] == 'A' &&
        resp[1] == '=')
    {
        int sign;
        rc = sscanf(resp, "A=%d", &sign);
        if (rc == 1 && sign)
        {
            TemperatureSettingN[0].value *= -1;
        }
    }
    return true;
}

bool USBFocus::updateTemperature()
{
    int nbytes_written=0, nbytes_read=0, rc=-1;
    char errstr[MAXRBUF];
    char resp[MAXRBUF];
    float temp;

    tcflush(PortFD, TCIOFLUSH);

    if ( (rc = tty_write(PortFD, GET_TEMPERATURE_CMD, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updateTemperature error: %s.", errstr);
        return false;
    }

    if ( (rc = tty_read_section(PortFD, resp, '\r', USBFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updateTemperature error: %s.", errstr);
        return false;
    }
    char* rsp = resp;
    resp[nbytes_read] = 0;
    if (resp[0] == '*')
    {
        // Got move finish ack
        moveInProgress = false;
        rsp++;
    }
    rc = sscanf(rsp, "T=%f", &temp);

    if (rc > 0)
    {
        TemperatureN[0].value = temp;
    }
    else
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Unknown error: focuser temperature value (%s)", resp);
        return false;
    }
    return true;
}

bool USBFocus::updatePosition()
{
    int nbytes_written=0, nbytes_read=0, rc=-1;
    char errstr[MAXRBUF];
    char resp[MAXRBUF];
    int pos=-1;

    tcflush(PortFD, TCIOFLUSH);

    if ( (rc = tty_write(PortFD, GET_POSITION_CMD, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updatePostion error: %s.", errstr);
        return false;
    }

    if ( (rc = tty_read_section(PortFD, resp, '\r', USBFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "updatePosition error: %s.", errstr);
        return false;
    }

    char* rsp = resp;
    resp[nbytes_read] = 0;
    if (resp[0] == '*')
    {
        // Got move finish ack
        moveInProgress = false;
        rsp++;
    }
//    DEBUGF(INDI::Logger::DBG_DEBUG, "updatePosition string: %s", resp);

    rc = sscanf(rsp, "P=%d", &pos);

    if (rc > 0)
    {
        FocusAbsPosN[0].value = pos;
        if (moveInProgress && pos == targetPosition)
        {
            moveInProgress = false;
        }
    }
    else
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Unknown error: focuser position value (%s)", resp);
        return false;
    }
    return true;
}

bool USBFocus::isMoving()
{
    int nbytes_read=0, rc=-1;
    char errstr[MAXRBUF];
    char resp[4];

    if (moveInProgress)
    {
        // Poll for * character
        resp[3] = 0;
        if ( (rc = tty_read(PortFD, resp, 3, 0, &nbytes_read)) != TTY_OK)
        {
            DEBUGF(INDI::Logger::DBG_SESSION, "Focuser still busy: %d %d: %s", rc, nbytes_read, resp);
            return true;
        }
        DEBUGF(INDI::Logger::DBG_SESSION, "Focuser reply: %s", resp);
        if (resp[0] == '*')
        {
            DEBUG(INDI::Logger::DBG_SESSION, "Focuser ack received!");
            moveInProgress = false;
        }
    }
    return moveInProgress;
}

bool USBFocus::setTemperatureMinMove(unsigned int minimum)
{
    int nbytes_written=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[7];

    snprintf(cmd, 7, ":PO%02hhX#", minimum);

    tcflush(PortFD, TCIOFLUSH);

#if 0
    if ( (rc = tty_write(PortFD, cmd, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setTemperatureCalibration error: %s.", errstr);
        return false;
    }
#endif
    return true;

}

bool USBFocus::setTemperatureCoefficient(unsigned int coefficient)
{
    int nbytes_written=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[7];

    snprintf(cmd, 7, ":SC%02hhX#", coefficient);

    tcflush(PortFD, TCIOFLUSH);

#if 0
    if ( (rc = tty_write(PortFD, cmd, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setTemperatureCoefficient error: %s.", errstr);
        return false;
    }
#endif
    return true;

}

bool USBFocus::reset()
{
    // TODO
    return true;
}

bool USBFocus::MoveFocuser(unsigned int position)
{
    int nbytes_written=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[7];

    if (position < FocusAbsPosN[0].min || position > FocusAbsPosN[0].max)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Requested position value out of bound: %d", position);
        return false;
    }

    /*if (fabs(position - FocusAbsPosN[0].value) > MaxTravelN[0].value)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Requested position value of %d exceeds maximum travel limit of %g", position, MaxTravelN[0].value);
        return false;
    }*/

    if (!updatePosition())
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "MoveFocuser error: %s.", errstr);
        return false;
    }

    int current_pos = FocusAbsPosN[0].value;
    int delta = position - current_pos;

    targetPosition = position;

    if (delta == 0)
    {
        return true;
    }

    if (delta > 0)
    {
        // Move outwards
        snprintf(cmd, 7, MOVE_OUT_CMD, delta);
    }
    else
    {
        // Move inwards
        snprintf(cmd, 7, MOVE_IN_CMD, -delta);
    }
    cmd[6] = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "MoveFocuser cmd %s", cmd);
    if ( (rc = tty_write(PortFD, cmd, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "MoveFocuser error: %s.", errstr);
        return false;
    }
    moveInProgress = true;
    return true;
}

bool USBFocus::setStepMode(FocusStepMode mode)
{
    int nbytes_written=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[7];

    tcflush(PortFD, TCIOFLUSH);

    if (mode == FOCUS_HALF_STEP)
        strncpy(cmd, SET_STEPMODE_H_CMD, 7);
    else
        strncpy(cmd, SET_STEPMODE_F_CMD, 7);

    if ( (rc = tty_write(PortFD, cmd, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setStepMode error: %s.", errstr);
        return false;
    }

    return true;
}

bool USBFocus::setRotation(MotorRotation rotation)
{
    int nbytes_written=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[7];

    tcflush(PortFD, TCIOFLUSH);

    if (rotation == ROTATION_CW)
        strncpy(cmd, SET_ROTATION_CW_CMD, 7);
    else
        strncpy(cmd, SET_ROTATION_ACW_CMD, 7);

    if ( (rc = tty_write(PortFD, cmd, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setRotation error: %s.", errstr);
        return false;
    }

    return true;
}

bool USBFocus::setSpeed(unsigned short speed)
{
    int nbytes_written=0, nbytes_read=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[7];
    char resp[7];

    snprintf(cmd, 7, SET_SPEED_CMD, speed);
    cmd[6] = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "setSpeed cmd %s", cmd);
    if ( (rc = tty_write(PortFD, cmd, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setSpeed error: %s.", errstr);
        return false;
    }

    // Read reply ("DONE\n\r")
    if ( (rc = tty_read(PortFD, resp, 6, USBFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setSpeed reply error: %s.", errstr);
        return false;
    }
    resp[6] = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "setSpeed reply %s", resp);
    return true;
}

bool USBFocus::setMaxPos(unsigned int maxPos)
{
    int nbytes_written=0, nbytes_read=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[7];
    char resp[7];

    snprintf(cmd, 7, SET_MAXPOS_CMD, maxPos);
    cmd[6] = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "setMaxPos cmd %s", cmd);
    if ( (rc = tty_write(PortFD, cmd, 6, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setMaxPos error: %s.", errstr);
        return false;
    }

    // Read reply ("DONE\n\r")
    if ( (rc = tty_read(PortFD, resp, 6, USBFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setMaxPos reply error: %s.", errstr);
        return false;
    }
    resp[6] = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "setMaxPos reply %s", resp);
    return true;
}

bool USBFocus::setTemperatureCompensation(bool enable)
{
    int nbytes_written=0, rc=-1;
    char errstr[MAXRBUF];
    char cmd[4];

    tcflush(PortFD, TCIOFLUSH);

    if (enable)
        strncpy(cmd, ":+#", 4);
    else
        strncpy(cmd, ":-#", 4);

#if 0
    if ( (rc = tty_write(PortFD, cmd, 3, &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        DEBUGF(INDI::Logger::DBG_ERROR, "setTemperatureCompensation error: %s.", errstr);
        return false;
    }
#endif
    return true;

}

bool USBFocus::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if(strcmp(dev,getDeviceName())==0)
    {
        // Focus Step Mode
        if (!strcmp(StepModeSP.name, name))
        {
            bool rc=false;
            int current_mode = IUFindOnSwitchIndex(&StepModeSP);
            IUUpdateSwitch(&StepModeSP, states, names, n);
            int target_mode = IUFindOnSwitchIndex(&StepModeSP);
            if (current_mode == target_mode);
            {
                StepModeSP.s = IPS_OK;
                IDSetSwitch(&StepModeSP, NULL);
            }

            if (target_mode == 0)
                rc = setStepMode(FOCUS_HALF_STEP);
            else
                rc = setStepMode(FOCUS_FULL_STEP);

            if (rc == false)
            {
                IUResetSwitch(&StepModeSP);
                StepModeS[current_mode].s = ISS_ON;
                StepModeSP.s = IPS_ALERT;
                IDSetSwitch(&StepModeSP, NULL);
                return false;
            }

            StepModeSP.s = IPS_OK;
            IDSetSwitch(&StepModeSP, NULL);
            return true;
        }

        // Motor Rotation
        if (!strcmp(RotationSP.name, name))
        {
            bool rc=false;
            int current_mode = IUFindOnSwitchIndex(&RotationSP);
            IUUpdateSwitch(&RotationSP, states, names, n);
            int target_mode = IUFindOnSwitchIndex(&RotationSP);
            if (current_mode == target_mode);
            {
                RotationSP.s = IPS_OK;
                IDSetSwitch(&RotationSP, NULL);
            }

            if (target_mode == 0)
                rc = setRotation(ROTATION_CW);
            else
                rc = setRotation(ROTATION_CCW);

            if (rc == false)
            {
                IUResetSwitch(&RotationSP);
                RotationS[current_mode].s = ISS_ON;
                RotationSP.s = IPS_ALERT;
                IDSetSwitch(&RotationSP, NULL);
                return false;
            }

            RotationSP.s = IPS_OK;
            IDSetSwitch(&RotationSP, NULL);
            return true;
        }

        if (!strcmp(TemperatureCompensateSP.name, name))
        {
            int last_index = IUFindOnSwitchIndex(&TemperatureCompensateSP);
            IUUpdateSwitch(&TemperatureCompensateSP, states, names, n);

            bool rc = setTemperatureCompensation((TemperatureCompensateS[0].s == ISS_ON));

            if (rc == false)
            {
                TemperatureCompensateSP.s = IPS_ALERT;
                IUResetSwitch(&TemperatureCompensateSP);
                TemperatureCompensateS[last_index].s = ISS_ON;
                IDSetSwitch(&TemperatureCompensateSP, NULL);
                return false;
            }

            TemperatureCompensateSP.s = IPS_OK;
            IDSetSwitch(&TemperatureCompensateSP, NULL);
            return true;
        }

        if (!strcmp(ResetSP.name, name))
        {
            IUResetSwitch(&ResetSP);

            if (reset())
                ResetSP.s = IPS_OK;
            else
                ResetSP.s = IPS_ALERT;

            IDSetSwitch(&ResetSP, NULL);
            return true;
        }

    }


    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool USBFocus::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
    int nset=0,i=0;

    if(strcmp(dev,getDeviceName())==0)
    {
        if (!strcmp (name, MaxTravelNP.name))
        {
             IUUpdateNumber(&MaxTravelNP, values, names, n);
             if (!setMaxPos(MaxTravelN[0].value))
             {
                MaxTravelNP.s = IPS_ALERT;
                IDSetNumber(&MaxTravelNP, NULL);
                return false;
             }
             MaxTravelNP.s = IPS_OK;
             IDSetNumber(&MaxTravelNP, NULL);
             return true;
        }

        if (!strcmp(name, TemperatureSettingNP.name))
        {
            IUUpdateNumber(&TemperatureSettingNP, values, names, n);
            if (!setTemperatureCoefficient(TemperatureSettingN[0].value) || !setTemperatureMinMove(TemperatureSettingN[1].value))
            {
                TemperatureSettingNP.s = IPS_ALERT;
                IDSetNumber(&TemperatureSettingNP, NULL);
                return false;
            }

            TemperatureSettingNP.s = IPS_OK;
            IDSetNumber(&TemperatureSettingNP, NULL);
        }

    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);

}

void USBFocus::GetFocusParams ()
{
    if (updatePosition())
        IDSetNumber(&FocusAbsPosNP, NULL);

    if (updateTemperature())
        IDSetNumber(&TemperatureNP, NULL);

    if (updateConfiguration())
    {
        IDSetNumber(&FocusSpeedNP, NULL);
        IDSetSwitch(&StepModeSP, NULL);
        IDSetSwitch(&RotationSP, NULL);

        IDSetNumber(&MaxTravelNP, NULL);
        IDSetNumber(&TemperatureSettingNP, NULL);
        IDSetNumber(&FirmwareVersionNP, NULL);
    }
}

bool USBFocus::SetFocuserSpeed(int speed)
{
    bool rc = false;

    rc = setSpeed(speed);

    if (rc == false)
        return false;

    currentSpeed = speed;

    FocusSpeedNP.s = IPS_OK;
    IDSetNumber(&FocusSpeedNP, NULL);

    return true;
}

IPState USBFocus::MoveAbsFocuser(uint32_t targetTicks)
{
    targetPos = targetTicks;

    bool rc = false;

    rc = MoveFocuser(targetPos);

    if (rc == false)
        return IPS_ALERT;

    FocusAbsPosNP.s = IPS_BUSY;

    return IPS_BUSY;
}

IPState USBFocus::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    double newPosition=0;
    bool rc=false;

    if (dir == FOCUS_INWARD)
        newPosition = FocusAbsPosN[0].value - ticks;
    else
        newPosition = FocusAbsPosN[0].value + ticks;

    rc = MoveFocuser(newPosition);

    if (rc == false)
        return IPS_ALERT;

    FocusRelPosN[0].value = ticks;
    FocusRelPosNP.s = IPS_BUSY;
    return IPS_BUSY;
}

void USBFocus::TimerHit()
{
    if (isConnected() == false)
    {
        SetTimer(POLLMS);
        return;
    }

    if (FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY)
    {
        if (isMoving() == false)
        {
            FocusAbsPosNP.s = IPS_OK;
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, NULL);
            IDSetNumber(&FocusRelPosNP, NULL);
            lastPos = FocusAbsPosN[0].value;
            DEBUG(INDI::Logger::DBG_SESSION, "Focuser reached requested position.");
        }
    }

    bool rc = updatePosition();
    if (rc)
    {
        if (fabs(lastPos - FocusAbsPosN[0].value) > 5)
        {
            IDSetNumber(&FocusAbsPosNP, NULL);
            lastPos = FocusAbsPosN[0].value;
        }
    }

    rc = updateTemperature();
    if (rc)
    {
        if (fabs(lastTemperature - TemperatureN[0].value) >= 0.5)
        {
            IDSetNumber(&TemperatureNP, NULL);
            lastTemperature = TemperatureN[0].value;
        }
    }
    SetTimer(POLLMS);
}

bool USBFocus::AbortFocuser()
{
    int nbytes_written;
    tcflush(PortFD, TCIOFLUSH);
    if (tty_write(PortFD, STOP_CMD, 6, &nbytes_written) == TTY_OK)
    {
        FocusAbsPosNP.s = IPS_IDLE;
        FocusRelPosNP.s = IPS_IDLE;
        IDSetNumber(&FocusAbsPosNP, NULL);
        IDSetNumber(&FocusRelPosNP, NULL);
        return true;
    }
    else
        return false;
}
