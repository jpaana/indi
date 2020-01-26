/*******************************************************************************
 ScopeDome Dome INDI Driver

 Copyright(c) 2017-2021 Jarno Paananen. All rights reserved.

 based on:

 ScopeDome Windows ASCOM driver version 5.1.30

 and

 Baader Planetarium Dome INDI Driver

 Copyright(c) 2014 Jasem Mutlaq. All rights reserved.

 Baader Dome INDI Driver

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include "scopedome_dome.h"
#include "scopedome_arduino.h"
#include "scopedome_sim.h"
#include "scopedome_usb21.h"

#include "connectionplugins/connectionserial.h"
#include "indicom.h"

#include <cmath>
#include <cstring>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <wordexp.h>

// We declare an auto pointer to ScopeDome.
std::unique_ptr<ScopeDome> scopeDome(new ScopeDome());

void ISGetProperties(const char *dev)
{
    scopeDome->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    scopeDome->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    scopeDome->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    scopeDome->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
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

void ISSnoopDevice(XMLEle *root)
{
    scopeDome->ISSnoopDevice(root);
}

ScopeDome::ScopeDome()
{
    setVersion(2, 0);
    targetAz         = 0;
    m_ShutterState   = SHUTTER_UNKNOWN;
    simShutterStatus = SHUTTER_CLOSED;

    status        = DOME_UNKNOWN;
    targetShutter = SHUTTER_CLOSE;

    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_ABS_MOVE | DOME_CAN_REL_MOVE | DOME_CAN_PARK | DOME_HAS_SHUTTER);

    stepsPerRevolution = ~0;

    // Load dome inertia table if present
    wordexp_t wexp;
    if (wordexp("~/.indi/ScopeDome_DomeInertia_Table.txt", &wexp, 0) == 0)
    {
        FILE *inertia = fopen(wexp.we_wordv[0], "r");
        if (inertia)
        {
            // skip UTF-8 marker bytes
            fseek(inertia, 3, SEEK_SET);
            char line[100];
            int lineNum = 0;

            while (fgets(line, sizeof(line), inertia))
            {
                int step, result;
                if (sscanf(line, "%d ;%d", &step, &result) != 2)
                {
                    sscanf(line, "%d;%d", &step, &result);
                }
                if (step == lineNum)
                {
                    inertiaTable.push_back(result);
                }
                lineNum++;
            }
            fclose(inertia);
            LOGF_INFO("Read inertia file %s", wexp.we_wordv[0]);
        }
        else
        {
            LOG_INFO("Could not read inertia file, please generate one with Windows "
                     "driver setup and copy to "
                     "~/.indi/ScopeDome_DomeInertia_Table.txt");
        }
    }
    wordfree(&wexp);
}

bool ScopeDome::initProperties()
{
    INDI::Dome::initProperties();

    IUFillNumber(&DomeHomePositionN[0], "DH_POSITION", "AZ (deg)", "%6.2f", 0.0, 360.0, 1.0, 0.0);
    IUFillNumberVector(&DomeHomePositionNP, DomeHomePositionN, 1, getDeviceName(), "DOME_HOME_POSITION",
                       "Home sensor position", SITE_TAB, IP_RW, 60, IPS_OK);

    IUFillSwitch(&FindHomeS[0], "START", "Start", ISS_OFF);
    IUFillSwitchVector(&FindHomeSP, FindHomeS, 1, getDeviceName(), "FIND_HOME", "Find home", MAIN_CONTROL_TAB, IP_RW,
                       ISR_ATMOST1, 0, IPS_OK);

    IUFillSwitch(&DerotateS[0], "START", "Start", ISS_OFF);
    IUFillSwitchVector(&DerotateSP, DerotateS, 1, getDeviceName(), "DEROTATE", "Derotate", MAIN_CONTROL_TAB, IP_RW,
                       ISR_ATMOST1, 0, IPS_OK);

    IUFillNumber(&FirmwareVersionsN[0], "MAIN", "Main part", "%2.2f", 0.0, 99.0, 1.0, 0.0);
    IUFillNumber(&FirmwareVersionsN[1], "ROTARY", "Rotary part", "%2.1f", 0.0, 99.0, 1.0, 0.0);
    IUFillNumberVector(&FirmwareVersionsNP, FirmwareVersionsN, 2, getDeviceName(), "FIRMWARE_VERSION",
                       "Firmware versions", INFO_TAB, IP_RO, 60, IPS_IDLE);

    IUFillNumber(&StepsPerRevolutionN[0], "STEPS", "Steps per revolution", "%5.0f", 0.0, 99999.0, 1.0, 0.0);
    IUFillNumberVector(&StepsPerRevolutionNP, StepsPerRevolutionN, 1, getDeviceName(), "CALIBRATION_VALUES",
                       "Calibration values", SITE_TAB, IP_RO, 60, IPS_IDLE);

    IUFillSwitch(&CalibrationNeededS[0], "CALIBRATION_NEEDED", "Calibration needed", ISS_OFF);
    IUFillSwitchVector(&CalibrationNeededSP, CalibrationNeededS, 1, getDeviceName(), "CALIBRATION_STATUS",
                       "Calibration status", SITE_TAB, IP_RO, ISR_ATMOST1, 0, IPS_IDLE);

    IUFillSwitch(&StartCalibrationS[0], "START", "Start", ISS_OFF);
    IUFillSwitchVector(&StartCalibrationSP, StartCalibrationS, 1, getDeviceName(), "RUN_CALIBRATION", "Run calibration",
                       SITE_TAB, IP_RW, ISR_ATMOST1, 0, IPS_OK);

    SetParkDataType(PARK_AZ);

    addAuxControls();

    IUFillSwitch(&CardTypeS[0], "USB_CARD_21", "USB Card 2.1", ISS_ON);
    IUFillSwitch(&CardTypeS[1], "ARDUINO", "Arduino Card", ISS_OFF);
    IUFillSwitchVector(&CardTypeSP, CardTypeS, 2, getDeviceName(), "CARD_TYPE", "Card type",
                       CONNECTION_TAB, IP_RW, ISR_1OFMANY, 0, IPS_OK);


    defineProperty(&CardTypeSP);
    if(!loadConfig(true, CardTypeSP.name))
    {
        // Set serial parameters
        // This is tricky as Arduino card communicates at 9600 bauds, but
        // USB Card 2.1 uses 115200 bauds
        serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
    }

    setPollingPeriodRange(1000, 3000); // Device doesn't like too long interval
    setDefaultPollingPeriod(1000);
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::SetupParms()
{
    targetAz = 0;

    stepsPerRevolution = interface->getStepsPerRevolution();
    LOGF_INFO("Steps per revolution read as %d", stepsPerRevolution);
    StepsPerRevolutionN[0].value = stepsPerRevolution;
    StepsPerRevolutionNP.s       = IPS_OK;
    IDSetNumber(&StepsPerRevolutionNP, nullptr);

    if (UpdatePosition())
        IDSetNumber(&DomeAbsPosNP, nullptr);

    if (UpdateShutterStatus())
        IDSetSwitch(&DomeShutterSP, nullptr);

    UpdateSensorStatus();
    UpdateRelayStatus();

    if (InitPark())
    {
        // If loading parking data is successful, we just set the default parking
        // values.
        SetAxis1ParkDefault(0);
    }
    else
    {
        // Otherwise, we set all parking data to default in case no parking data is
        // found.
        SetAxis1Park(0);
        SetAxis1ParkDefault(0);
    }

    bool calibrationNeeded = interface->isCalibrationNeeded();
    CalibrationNeededS[0].s = calibrationNeeded ? ISS_ON : ISS_OFF;
    CalibrationNeededSP.s   = IPS_OK;
    IDSetSwitch(&CalibrationNeededSP, nullptr);

    interface->getFirmwareVersions(FirmwareVersionsN[0].value, FirmwareVersionsN[1].value);
    FirmwareVersionsNP.s       = IPS_OK;
    IDSetNumber(&FirmwareVersionsNP, nullptr);
    return true;
}

/************************************************************************************
 *
 ************************************************************************************/
bool ScopeDome::Handshake()
{
    if(reconnecting)
        return true;

    sim = isSimulation();

    if (sim)
    {
        interface.reset(static_cast<ScopeDomeCard *>(new ScopeDomeSim()));
    }
    else
    {
        if(CardTypeS[0].s == ISS_ON)
        {
            interface.reset(static_cast<ScopeDomeCard *>(new ScopeDomeUSB21(this, PortFD)));
        }
        else
        {
            bool useEthernet = false; // (getDomeConnection() & CONNECTION_TCP) != 0;
            interface.reset(static_cast<ScopeDomeCard *>(new ScopeDomeArduino(useEthernet, PortFD)));
        }
    }
    if (interface->detect())
    {
        return true;
    }
    LOG_ERROR("Can't identify the card, check card type and baud rate (115200 for USB Card 2.1, 9600 for Arduino Card)");
    return false;
}

/************************************************************************************
 *
 ************************************************************************************/
const char *ScopeDome::getDefaultName()
{
    return (const char *)"ScopeDome Dome";
}

/************************************************************************************
 *
 ************************************************************************************/
bool ScopeDome::updateProperties()
{
    INDI::Dome::updateProperties();

    if (isConnected())
    {
        // Initialize dynamic properties
        {
            size_t numSensors = interface->getNumberOfSensors();
            SensorsN = std::unique_ptr<INumber[]>(new INumber[numSensors]);
            for(size_t i = 0; i < numSensors; i++)
            {
                auto info = interface->getSensorInfo(i);
                IUFillNumber(&SensorsN[i], info.propName.c_str(), info.label.c_str(),
                             info.format.c_str(), info.minValue, info.maxValue, 1.0, 0.0);
            }
            IUFillNumberVector(&SensorsNP, SensorsN.get(), numSensors, getDeviceName(), "SCOPEDOME_SENSORS",
                               "Sensors", INFO_TAB, IP_RO, 60, IPS_IDLE);
        }

        {
            size_t numRelays = interface->getNumberOfRelays();
            RelaysS = std::unique_ptr<ISwitch[]>(new ISwitch[numRelays]);
            for(size_t i = 0; i < numRelays; i++)
            {
                auto info = interface->getRelayInfo(i);
                IUFillSwitch(&RelaysS[i], info.propName.c_str(), info.label.c_str(), ISS_OFF);
            }
            IUFillSwitchVector(&RelaysSP, RelaysS.get(), numRelays, getDeviceName(), "RELAYS", "Relays", MAIN_CONTROL_TAB, IP_RW,
                               ISR_NOFMANY,
                               0, IPS_IDLE);
        }

        {
            size_t numInputs = interface->getNumberOfInputs();
            InputsS = std::unique_ptr<ISwitch[]>(new ISwitch[numInputs]);
            for(size_t i = 0; i < numInputs; i++)
            {
                auto info = interface->getInputInfo(i);
                IUFillSwitch(&InputsS[i], info.propName.c_str(), info.label.c_str(), ISS_OFF);
            }
            IUFillSwitchVector(&InputsSP, InputsS.get(), numInputs, getDeviceName(), "INPUTS", "Inputs", INFO_TAB, IP_RO,
                               ISR_NOFMANY, 0, IPS_IDLE);
        }

        defineProperty(&FindHomeSP);
        defineProperty(&DerotateSP);
        defineProperty(&DomeHomePositionNP);
        defineProperty(&SensorsNP);
        defineProperty(&RelaysSP);
        defineProperty(&InputsSP);
        defineProperty(&StepsPerRevolutionNP);
        defineProperty(&CalibrationNeededSP);
        defineProperty(&StartCalibrationSP);
        defineProperty(&FirmwareVersionsNP);
        SetupParms();
    }
    else
    {
        deleteProperty(FindHomeSP.name);
        deleteProperty(DerotateSP.name);
        deleteProperty(DomeHomePositionNP.name);
        deleteProperty(SensorsNP.name);
        deleteProperty(RelaysSP.name);
        deleteProperty(InputsSP.name);
        deleteProperty(StepsPerRevolutionNP.name);
        deleteProperty(CalibrationNeededSP.name);
        deleteProperty(StartCalibrationSP.name);
        deleteProperty(FirmwareVersionsNP.name);
    }

    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (strcmp(name, FindHomeSP.name) == 0)
        {
            if (status != DOME_HOMING)
            {
                LOG_INFO("Finding home sensor");
                status = DOME_HOMING;
                IUResetSwitch(&FindHomeSP);
                DomeAbsPosNP.s = IPS_BUSY;
                FindHomeSP.s   = IPS_BUSY;
                IDSetSwitch(&FindHomeSP, nullptr);
                interface->findHome();
            }
            return true;
        }

        if (strcmp(name, DerotateSP.name) == 0)
        {
            if (status != DOME_DEROTATING)
            {
                LOG_INFO("De-rotating started");
                status = DOME_DEROTATING;
                IUResetSwitch(&DerotateSP);
                DomeAbsPosNP.s = IPS_BUSY;
                DerotateSP.s   = IPS_BUSY;
                IDSetSwitch(&DerotateSP, nullptr);
            }
            return true;
        }

        if (strcmp(name, StartCalibrationSP.name) == 0)
        {
            if (status != DOME_CALIBRATING)
            {
                LOG_INFO("Calibration started");
                status = DOME_CALIBRATING;
                IUResetSwitch(&StartCalibrationSP);
                DomeAbsPosNP.s       = IPS_BUSY;
                StartCalibrationSP.s = IPS_BUSY;
                IDSetSwitch(&StartCalibrationSP, nullptr);
                interface->calibrate();
            }
            return true;
        }

        if (strcmp(name, RelaysSP.name) == 0)
        {
            IUUpdateSwitch(&RelaysSP, states, names, n);
            for(int i = 0; i < n; i++)
            {
                interface->setRelayState(i, RelaysS[i].s);
            }
            IDSetSwitch(&RelaysSP, nullptr);
            return true;
        }

        if (strcmp(name, CardTypeSP.name) == 0)
        {
            IUUpdateSwitch(&CardTypeSP, states, names, n);
            IDSetSwitch(&CardTypeSP, nullptr);

            if(CardTypeS[0].s == ISS_ON)
            {
                // USB Card 2.1 uses 115200 bauds
                serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
            }
            else
            {
                // Arduino Card uses 9600 bauds
                serialConnection->setDefaultBaudRate(Connection::Serial::B_9600);
            }
            return true;
        }
    }

    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}

bool ScopeDome::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (strcmp(name, DomeHomePositionNP.name) == 0)
        {
            IUUpdateNumber(&DomeHomePositionNP, values, names, n);
            DomeHomePositionNP.s = IPS_OK;
            IDSetNumber(&DomeHomePositionNP, nullptr);
            return true;
        }
    }
    return INDI::Dome::ISNewNumber(dev, name, values, names, n);
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::UpdateShutterStatus()
{
    int rc = 0;
    if (rc != 0)
    {
        LOGF_ERROR("Error reading input state: %d", rc);
        return false;
    }

    for(int i = 0; i < InputsSP.nsp; i++)
    {
        InputsS[i].s = interface->getInputValue(i) ? ISS_ON : ISS_OFF;
    }
    InputsSP.s    = IPS_OK;
    IDSetSwitch(&InputsSP, nullptr);

    DomeShutterSP.s = IPS_OK;
    IUResetSwitch(&DomeShutterSP);

    if (interface->getInputState(ScopeDomeCard::OPEN1) == ISS_ON) // shutter open switch triggered
    {
        if (m_ShutterState == SHUTTER_MOVING && targetShutter == SHUTTER_OPEN)
        {
            LOGF_INFO("%s", GetShutterStatusString(SHUTTER_OPENED));
            interface->controlShutter(ScopeDomeCard::STOP_SHUTTER);
            m_ShutterState = SHUTTER_OPENED;
            if (getDomeState() == DOME_UNPARKING)
                SetParked(false);
        }
        DomeShutterS[SHUTTER_OPEN].s = ISS_ON;
    }
    else if (interface->getInputState(ScopeDomeCard::CLOSED1) == ISS_ON) // shutter closed switch triggered
    {
        if (m_ShutterState == SHUTTER_MOVING && targetShutter == SHUTTER_CLOSE)
        {
            LOGF_INFO("%s", GetShutterStatusString(SHUTTER_CLOSED));
            interface->controlShutter(ScopeDomeCard::STOP_SHUTTER);
            m_ShutterState = SHUTTER_CLOSED;

            if (getDomeState() == DOME_PARKING && DomeAbsPosNP.s != IPS_BUSY)
            {
                SetParked(true);
            }
        }
        DomeShutterS[SHUTTER_CLOSE].s = ISS_ON;
    }
    else
    {
        m_ShutterState  = SHUTTER_MOVING;
        DomeShutterSP.s = IPS_BUSY;
    }
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::UpdatePosition()
{
    rotationCounter = interface->getRotationCounter();

    //    LOGF_INFO("Counters are %d - %d", counter, counter2);

    // We assume counter value 0 is at home sensor position
    double az = ((double)rotationCounter * -360.0 / stepsPerRevolution) + DomeHomePositionN[0].value;
    az        = fmod(az, 360.0);
    if (az < 0.0)
    {
        az += 360.0;
    }
    DomeAbsPosN[0].value = az;
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::UpdateSensorStatus()
{
    for (int i = 0; i < SensorsNP.nnp; ++i)
    {
        SensorsN[i].value = interface->getSensorValue(i);
    }
    //    SensorsN[10].value = getDewPoint(SensorsN[8].value, SensorsN[7].value);
    SensorsNP.s        = IPS_OK;

    IDSetNumber(&SensorsNP, nullptr);

    // My shutter unit occasionally disconnects so implement a simple watchdog
    // to check for link strength and reset the controller if link is lost for
    // more than 5 polling cycles
    static int count = 0;
    if (linkStrength == 0)
    {
        if (++count > 5)
        {
            // Issue reset
            interface->setOutputState(ScopeDomeCard::RESET, ISS_ON);
            count = 0;
        }
    }
    else
    {
        count = 0;
    }
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::UpdateRelayStatus()
{
    for(int i = 0; i < RelaysSP.nsp; i++)
    {
        RelaysS[i].s = interface->getRelayState(i);
    }
    RelaysSP.s   = IPS_OK;
    IDSetSwitch(&RelaysSP, nullptr);
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
void ScopeDome::TimerHit()
{
    if (!isConnected())
        return; //  No need to reset timer if we are not connected anymore

    interface->updateState();

    currentStatus = interface->getStatus();

    UpdatePosition();
    UpdateShutterStatus();
    IDSetSwitch(&DomeShutterSP, nullptr);

    UpdateRelayStatus();

    if (status == DOME_HOMING)
    {
        if ((currentStatus & (ScopeDomeCard::STATUS_HOMING | ScopeDomeCard::STATUS_MOVING)) == 0)
        {
            double azDiff = DomeHomePositionN[0].value - DomeAbsPosN[0].value;

            if (azDiff > 180)
            {
                azDiff -= 360;
            }
            if (azDiff < -180)
            {
                azDiff += 360;
            }

            if (interface->getInputState(ScopeDomeCard::HOME) || fabs(azDiff) <= DomeParamN[0].value)
            {
                // Found home (or close enough)
                LOG_INFO("Home sensor found");
                status   = DOME_READY;
                targetAz = DomeHomePositionN[0].value;

                // Reset counters
                interface->resetCounter();

                FindHomeSP.s   = IPS_OK;
                DomeAbsPosNP.s = IPS_OK;
                IDSetSwitch(&FindHomeSP, nullptr);
            }
            else
            {
                // We overshoot, go closer
                MoveAbs(DomeHomePositionN[0].value);
            }
        }
        IDSetNumber(&DomeAbsPosNP, nullptr);
    }
    else if (status == DOME_DEROTATING)
    {
        if ((currentStatus & ScopeDomeCard::STATUS_MOVING) == 0)
        {
            currentRotation = interface->getRotationCounterExt();
            LOGF_INFO("Current rotation is %d", currentRotation);
            if (abs(currentRotation) < 100)
            {
                // Close enough
                LOG_INFO("De-rotation complete");
                status         = DOME_READY;
                DerotateSP.s   = IPS_OK;
                DomeAbsPosNP.s = IPS_OK;
                IDSetSwitch(&DerotateSP, nullptr);
            }
            else
            {
                interface->move(currentRotation);
            }
        }
        IDSetNumber(&DomeAbsPosNP, nullptr);
    }
    else if (status == DOME_CALIBRATING)
    {
        if ((currentStatus & (ScopeDomeCard::STATUS_CALIBRATING | ScopeDomeCard::STATUS_MOVING)) == 0)
        {
            stepsPerRevolution = interface->getStepsPerRevolution();
            LOGF_INFO("Calibration complete, steps per revolution read as %d", stepsPerRevolution);
            StepsPerRevolutionN[0].value = stepsPerRevolution;
            StepsPerRevolutionNP.s       = IPS_OK;
            IDSetNumber(&StepsPerRevolutionNP, nullptr);
            StartCalibrationSP.s = IPS_OK;
            DomeAbsPosNP.s       = IPS_OK;
            IDSetSwitch(&StartCalibrationSP, nullptr);
            status = DOME_READY;
        }
    }
    else if (DomeAbsPosNP.s == IPS_BUSY)
    {
        if ((currentStatus & ScopeDomeCard::STATUS_MOVING) == 0)
        {
            // Rotation idle, are we close enough?
            double azDiff = targetAz - DomeAbsPosN[0].value;

            if (azDiff > 180)
            {
                azDiff -= 360;
            }
            if (azDiff < -180)
            {
                azDiff += 360;
            }
            if (!refineMove || fabs(azDiff) <= DomeParamN[0].value)
            {
                if (refineMove)
                    DomeAbsPosN[0].value = targetAz;
                DomeAbsPosNP.s = IPS_OK;
                LOG_INFO("Dome reached requested azimuth angle.");

                if (getDomeState() == DOME_PARKING)
                {
                    if (ShutterParkPolicyS[SHUTTER_CLOSE_ON_PARK].s == ISS_ON &&
                            interface->getInputState(ScopeDomeCard::CLOSED1) == ISS_OFF)
                    {
                        ControlShutter(SHUTTER_CLOSE);
                    }
                    else
                    {
                        SetParked(true);
                    }
                }
                else if (getDomeState() == DOME_UNPARKING)
                    SetParked(false);
                else
                    setDomeState(DOME_SYNCED);
            }
            else
            {
                // Refine azimuth
                MoveAbs(targetAz);
            }
        }

        IDSetNumber(&DomeAbsPosNP, nullptr);
    }
    else
        IDSetNumber(&DomeAbsPosNP, nullptr);

    // Read temperatures only every 10th time
    static int tmpCounter = 0;
    if (--tmpCounter <= 0)
    {
        UpdateSensorStatus();
        tmpCounter = 10;
    }

    SetTimer(getCurrentPollingPeriod());
}

/************************************************************************************
 *
 * ***********************************************************************************/
IPState ScopeDome::MoveAbs(double az)
{
    LOGF_DEBUG("MoveAbs (%f)", az);
    targetAz      = az;
    double azDiff = az - DomeAbsPosN[0].value;
    LOGF_DEBUG("azDiff = %f", azDiff);

    // Make relative (-180 - 180) regardless if it passes az 0
    if (azDiff > 180)
    {
        azDiff -= 360;
    }
    if (azDiff < -180)
    {
        azDiff += 360;
    }

    LOGF_DEBUG("azDiff rel = %f", azDiff);

    refineMove = true;
    return sendMove(azDiff);
}

/************************************************************************************
 *
 * ***********************************************************************************/
IPState ScopeDome::MoveRel(double azDiff)
{
    refineMove = false;
    return sendMove(azDiff);
}

/************************************************************************************
 *
 * ***********************************************************************************/
IPState ScopeDome::sendMove(double azDiff)
{
    if (azDiff < 0)
    {
        uint16_t steps = (uint16_t)(-azDiff * stepsPerRevolution / 360.0);
        LOGF_DEBUG("CCW (%d)", steps);
        steps = compensateInertia(steps);
        LOGF_DEBUG("CCW inertia (%d)", steps);
        if (steps == 0)
            return IPS_OK;
        interface->move(-steps);
    }
    else
    {
        uint16_t steps = (uint16_t)(azDiff * stepsPerRevolution / 360.0);
        LOGF_DEBUG("CW (%d)", steps);
        steps = compensateInertia(steps);
        LOGF_DEBUG("CW inertia (%d)", steps);
        if (steps == 0)
            return IPS_OK;
        interface->move(steps);
    }
    return IPS_BUSY;
}

/************************************************************************************
 *
 * ***********************************************************************************/
IPState ScopeDome::Move(DomeDirection dir, DomeMotionCommand operation)
{
    // Map to button outputs
    if (operation == MOTION_START)
    {
        refineMove = false;
        if (dir == DOME_CW)
        {
            interface->setOutputState(ScopeDomeCard::CW, ISS_ON);
            interface->setOutputState(ScopeDomeCard::CCW, ISS_OFF);
        }
        else
        {
            interface->setOutputState(ScopeDomeCard::CW, ISS_OFF);
            interface->setOutputState(ScopeDomeCard::CCW, ISS_ON);
        }
        return IPS_BUSY;
    }
    interface->setOutputState(ScopeDomeCard::CW, ISS_OFF);
    interface->setOutputState(ScopeDomeCard::CCW, ISS_OFF);
    return IPS_OK;
}

/************************************************************************************
 *
 * ***********************************************************************************/
IPState ScopeDome::Park()
{
    // First move to park position and then optionally close shutter
    targetAz  = GetAxis1Park();
    IPState s = MoveAbs(targetAz);
    if (s == IPS_OK && ShutterParkPolicyS[SHUTTER_CLOSE_ON_PARK].s == ISS_ON)
    {
        // Already at home, just close if needed
        return ControlShutter(SHUTTER_CLOSE);
    }
    return s;
}

/************************************************************************************
 *
 * ***********************************************************************************/
IPState ScopeDome::UnPark()
{
    if (ShutterParkPolicyS[SHUTTER_OPEN_ON_UNPARK].s == ISS_ON)
    {
        return ControlShutter(SHUTTER_OPEN);
    }
    return IPS_OK;
}

/************************************************************************************
 *
 * ***********************************************************************************/
IPState ScopeDome::ControlShutter(ShutterOperation operation)
{
    LOGF_INFO("Control shutter %d", (int)operation);
    targetShutter = operation;
    if (operation == SHUTTER_OPEN)
    {
        LOG_INFO("Opening shutter");
        if (interface->getInputState(ScopeDomeCard::OPEN1))
        {
            LOG_INFO("Shutter already open");
            return IPS_OK;
        }
        interface->controlShutter(ScopeDomeCard::OPEN_SHUTTER);
    }
    else
    {
        LOG_INFO("Closing shutter");
        if (interface->getInputState(ScopeDomeCard::CLOSED1))
        {
            LOG_INFO("Shutter already closed");
            return IPS_OK;
        }
        interface->controlShutter(ScopeDomeCard::CLOSE_SHUTTER);
    }

    m_ShutterState = SHUTTER_MOVING;
    return IPS_BUSY;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::Abort()
{
    interface->abort();
    status = DOME_READY;
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::saveConfigItems(FILE *fp)
{
    INDI::Dome::saveConfigItems(fp);

    IUSaveConfigNumber(fp, &DomeHomePositionNP);
    IUSaveConfigSwitch(fp, &CardTypeSP);
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
bool ScopeDome::SetCurrentPark()
{
    SetAxis1Park(DomeAbsPosN[0].value);
    return true;
}
/************************************************************************************
 *
 * ***********************************************************************************/

bool ScopeDome::SetDefaultPark()
{
    // By default set position to 90
    SetAxis1Park(90);
    return true;
}

/************************************************************************************
 *
 * ***********************************************************************************/
void ScopeDome::reconnect()
{
    // Reconnect serial port after write error
    LOG_INFO("Reconnecting serial port");
    reconnecting = true;
    serialConnection->Disconnect();
    usleep(1000000); // 1s
    serialConnection->Connect();
    PortFD = serialConnection->getPortFD();
    interface->setPortFD(PortFD);
    LOG_INFO("Reconnected");
    reconnecting = false;
}

/*
 * Saturation Vapor Pressure formula for range -100..0 Deg. C.
 * This is taken from
 *   ITS-90 Formulations for Vapor Pressure, Frostpoint Temperature,
 *   Dewpoint Temperature, and Enhancement Factors in the Range 100 to +100 C
 * by Bob Hardy
 * as published in "The Proceedings of the Third International Symposium on
 * Humidity & Moisture",
 * Teddington, London, England, April 1998
 */
static const float k0 = -5.8666426e3;
static const float k1 = 2.232870244e1;
static const float k2 = 1.39387003e-2;
static const float k3 = -3.4262402e-5;
static const float k4 = 2.7040955e-8;
static const float k5 = 6.7063522e-1;

static float pvsIce(float T)
{
    float lnP = k0 / T + k1 + (k2 + (k3 + (k4 * T)) * T) * T + k5 * log(T);
    return exp(lnP);
}

/**
 * Saturation Vapor Pressure formula for range 273..678 Deg. K.
 * This is taken from the
 *   Release on the IAPWS Industrial Formulation 1997
 *   for the Thermodynamic Properties of Water and Steam
 * by IAPWS (International Association for the Properties of Water and Steam),
 * Erlangen, Germany, September 1997.
 *
 * This is Equation (30) in Section 8.1 "The Saturation-Pressure Equation (Basic
 * Equation)"
 */

static const float n1  = 0.11670521452767e4;
static const float n6  = 0.14915108613530e2;
static const float n2  = -0.72421316703206e6;
static const float n7  = -0.48232657361591e4;
static const float n3  = -0.17073846940092e2;
static const float n8  = 0.40511340542057e6;
static const float n4  = 0.12020824702470e5;
static const float n9  = -0.23855557567849;
static const float n5  = -0.32325550322333e7;
static const float n10 = 0.65017534844798e3;

static float pvsWater(float T)
{
    float th = T + n9 / (T - n10);
    float A  = (th + n1) * th + n2;
    ;
    float B = (n3 * th + n4) * th + n5;
    float C = (n6 * th + n7) * th + n8;
    ;

    float p = 2.0f * C / (-B + sqrt(B * B - 4.0f * A * C));
    p *= p;
    p *= p;
    return p * 1e6;
}

static const float C_OFFSET = 273.15f;
static const float minT     = 173; // -100 Deg. C.
static const float maxT     = 678;

static float PVS(float T)
{
    if (T < minT || T > maxT)
        return 0;
    else if (T < C_OFFSET)
        return pvsIce(T);
    else
        return pvsWater(T);
}

static float solve(float (*f)(float), float y, float x0)
{
    float x      = x0;
    int maxCount = 10;
    int count    = 0;
    while (++count < maxCount)
    {
        float xNew;
        float dx = x / 1000;
        float z  = f(x);
        xNew     = x + dx * (y - z) / (f(x + dx) - z);
        if (fabs((xNew - x) / xNew) < 0.0001f)
            return xNew;
        x = xNew;
    }
    return 0;
}

float ScopeDome::getDewPoint(float RH, float T)
{
    T = T + C_OFFSET;
    return solve(PVS, RH / 100 * PVS(T), T) - C_OFFSET;
}

uint16_t ScopeDome::compensateInertia(uint16_t steps)
{
    if (inertiaTable.size() == 0)
    {
        LOGF_INFO("inertia passthrough %d", steps);
        return steps; // pass value as such if we don't have enough data
    }

    for (uint16_t out = 0; out < inertiaTable.size(); out++)
    {
        if (inertiaTable[out] > steps)
        {
            LOGF_INFO("inertia %d -> %d", steps, out - 1);
            return out - 1;
        }
    }
    // Check difference from largest table entry and assume we have
    // similar inertia also after that
    int lastEntry = inertiaTable.size() - 1;
    int inertia   = inertiaTable[lastEntry] - lastEntry;
    int movement  = (int)steps - inertia;
    LOGF_INFO("inertia %d -> %d", steps, movement);
    if (movement <= 0)
        return 0;

    return movement;
}
