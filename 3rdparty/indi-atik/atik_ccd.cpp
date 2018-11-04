/*
 ATIK CCD driver
  
 Copyright 2018 Jarno Paananen

 Using ATIK CCD SDK by CloudMakers, s. r. o
 
 and based on:
  
 Generic CCD
 CCD Template for INDI Developers
 Copyright (C) 2012 Jasem Mutlaq (mutlaqja@ikarustech.com)

 Multiple device support Copyright (C) 2013 Peter Polakovic (peter.polakovic@cloudmakers.eu)

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

#include <memory>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

#include "config.h"
#include "indidevapi.h"
#include "eventloop.h"

#include "atik_ccd.h"

#define MAX_CCD_TEMP 45    /* Max CCD temperature */
#define MIN_CCD_TEMP -55   /* Min CCD temperature */
#define MAX_X_BIN 16       /* Max Horizontal binning */
#define MAX_Y_BIN 16       /* Max Vertical binning */
#define MAX_PIXELS 4096    /* Max number of pixels in one dimension */
#define TEMP_THRESHOLD .25 /* Differential temperature threshold (C)*/
#define MAX_DEVICES 20     /* Max device cameraCount */

static int cameraCount;
static ATIKCCD *cameras[MAX_DEVICES];
static AtikCamera *atik_cameras[MAX_DEVICES];

static void cleanup()
{
    for (int i = 0; i < cameraCount; i++)
    {
        delete cameras[i];
    }
}

extern bool AtikDebug;

void ISInit()
{
    static bool isInit = false;
    if (!isInit)
    {
        //        AtikDebug   = true;
        int count   = AtikCamera::list(atik_cameras, MAX_DEVICES);
        cameraCount = 0;

        for (int i = 0; i < count; i++)
        {
            if (atik_cameras[i]->open())
            {
                cameras[cameraCount++] = new ATIKCCD(atik_cameras[i]);
            }
        }

        atexit(cleanup);
        isInit = true;
    }
}

void ISGetProperties(const char *dev)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        ATIKCCD *camera = cameras[i];
        if (dev == NULL || !strcmp(dev, camera->name))
        {
            camera->ISGetProperties(dev);
            if (dev != NULL)
                break;
        }
    }
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        ATIKCCD *camera = cameras[i];
        if (dev == NULL || !strcmp(dev, camera->name))
        {
            camera->ISNewSwitch(dev, name, states, names, num);
            if (dev != NULL)
                break;
        }
    }
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        ATIKCCD *camera = cameras[i];
        if (dev == NULL || !strcmp(dev, camera->name))
        {
            camera->ISNewText(dev, name, texts, names, num);
            if (dev != NULL)
                break;
        }
    }
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < cameraCount; i++)
    {
        ATIKCCD *camera = cameras[i];
        if (dev == NULL || !strcmp(dev, camera->name))
        {
            camera->ISNewNumber(dev, name, values, names, num);
            if (dev != NULL)
                break;
        }
    }
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
    ISInit();

    for (int i = 0; i < cameraCount; i++)
    {
        ATIKCCD *camera = cameras[i];
        camera->ISSnoopDevice(root);
    }
}

ATIKCCD::ATIKCCD(AtikCamera *device)
{
    this->device = device;
    const char *name;
    device->getCapabilities(&name, &cameraType, &capabilities);
    snprintf(this->name, 32, "%s", name);
    setDeviceName(this->name);

    setVersion(ATIK_VERSION_MAJOR, ATIK_VERSION_MINOR);
}

ATIKCCD::~ATIKCCD()
{
    device->close();
}

const char *ATIKCCD::getDefaultName()
{
    return "ATIK CCD";
}

bool ATIKCCD::initProperties()
{
    // Init parent properties first
    INDI::CCD::initProperties();

    //    uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_COOLER | CCD_HAS_SHUTTER | CCD_HAS_ST4_PORT;
    uint32_t cap = CCD_CAN_ABORT | CCD_CAN_SUBFRAME;
    if (capabilities.maxBinX > 1)
        cap |= CCD_CAN_BIN;
    if (capabilities.cooler != COOLER_NONE)
        cap |= CCD_HAS_COOLER;
    if (capabilities.hasShutter)
        cap |= CCD_HAS_SHUTTER;
    if (capabilities.hasGuidePort)
        cap |= CCD_HAS_ST4_PORT;
    SetCCDCapability(cap);

    minDuration = capabilities.minShortExposure;

    addConfigurationControl();
    addDebugControl();
    return true;
}

void ATIKCCD::ISGetProperties(const char *dev)
{
    INDI::CCD::ISGetProperties(dev);
}

bool ATIKCCD::updateProperties()
{
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        // Let's get parameters now from CCD
        setupParams();

        timerID = SetTimer(POLLMS);
    }
    else
    {
        rmTimer(timerID);
    }

    return true;
}

bool ATIKCCD::Connect()
{
    LOG_INFO("Attempting to find the ATIK CCD...");

    LOG_INFO("CCD is online. Retrieving basic data.");

    LOGF_INFO("QUICKER_START_EXPOSURE_DELAY %d", device->getParam(QUICKER_START_EXPOSURE_DELAY));
    LOGF_INFO("QUICKER_READ_CCD_DELAY %d", device->getParam(QUICKER_READ_CCD_DELAY));
    LOGF_INFO("MAX_PACKET_SIZE %d", device->getParam(MAX_PACKET_SIZE));

    device->setParam(MAX_PACKET_SIZE, 8 * 1024 * 1024);
    LOGF_INFO("MAX_PACKET_SIZE %d", device->getParam(MAX_PACKET_SIZE));

    return true;
}

bool ATIKCCD::Disconnect()
{
    LOG_INFO("CCD is offline.");
    return true;
}

bool ATIKCCD::setupParams()
{
    float x_pixel_size, y_pixel_size;
    int bit_depth = 16;
    int x_1, y_1, x_2, y_2;

    ///////////////////////////
    // 1. Get Pixel size
    ///////////////////////////
    // Actucal CALL to CCD to get pixel size here
    x_pixel_size = capabilities.pixelSizeX;
    y_pixel_size = capabilities.pixelSizeY;

    ///////////////////////////
    // 2. Get Frame
    ///////////////////////////

    x_1 = y_1 = 0;
    x_2       = capabilities.pixelCountX;
    y_2       = capabilities.pixelCountY;

    ///////////////////////////
    // 3. Get temperature
    ///////////////////////////
    float temperature;
    device->getTemperatureSensorStatus(1, &temperature);
    TemperatureN[0].value = temperature;
    LOGF_INFO("The CCD Temperature is %f", TemperatureN[0].value);
    IDSetNumber(&TemperatureNP, NULL);

    ///////////////////////////
    // 4. Set frame
    ///////////////////////////
    bit_depth = 16;
    SetCCDParams(x_2 - x_1, y_2 - y_1, bit_depth, x_pixel_size, y_pixel_size);

    // Now we usually do the following in the hardware
    // Set Frame to LIGHT or NORMAL
    // Set Binning to 1x1
    /* Default frame type is NORMAL */

    // Let's calculate required buffer
    int nbuf;
    nbuf = PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8; //  this is pixel cameraCount
    nbuf += 512;                                                                  //  leave a little extra at the end
    PrimaryCCD.setFrameBufferSize(nbuf);

    return true;
}

int ATIKCCD::SetTemperature(double temperature)
{
    // If there difference, for example, is less than 0.1 degrees, let's immediately return OK.
    if (fabs(temperature - TemperatureN[0].value) < TEMP_THRESHOLD)
        return 1;

    // Otherwise, we set the temperature request and we update the status in TimerHit() function.
    TemperatureRequest = temperature;
    device->setCooling((float)temperature);

    LOGF_INFO("Setting CCD temperature to %+06.2f C", temperature);
    return 0;
}

bool ATIKCCD::StartExposure(float duration)
{
    if (imageFrameType == INDI::CCDChip::BIAS_FRAME)
    {
        duration = minDuration;
        LOGF_INFO("Bias Frame (s) : %g\n", minDuration);
    }
    else if (duration < minDuration)
    {
        LOGF_WARN("Exposure shorter than minimum duration %g s requested. \n Setting exposure time to %g s.", duration,
                  minDuration);
        duration = minDuration;
    }

    PrimaryCCD.setExposureDuration(duration);
    ExposureRequest = duration;

    gettimeofday(&ExpStart, NULL);

    LOGF_INFO("Taking a %g seconds frame...", ExposureRequest);
    if (duration <= capabilities.maxShortExposure)
    {
        unsigned width  = device->imageWidth(PrimaryCCD.getSubW(), PrimaryCCD.getBinX());
        unsigned height = device->imageHeight(PrimaryCCD.getSubH(), PrimaryCCD.getBinY());

        device->readCCD(PrimaryCCD.getSubX() * PrimaryCCD.getBinX(), PrimaryCCD.getSubY() * PrimaryCCD.getBinY(),
                        width * PrimaryCCD.getBinX(), height * PrimaryCCD.getBinY(), PrimaryCCD.getBinX(),
                        PrimaryCCD.getBinY(), duration);

        ShortExposure = true;
    }
    else
    {
        device->startExposure(false);
        ShortExposure = false;
    }
    InExposure = true;
    return true;
}

bool ATIKCCD::AbortExposure()
{
    device->abortExposure();
    InExposure = false;
    return true;
}

bool ATIKCCD::UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType)
{
    INDI::CCDChip::CCD_FRAME imageFrameType = PrimaryCCD.getFrameType();

    if (fType == imageFrameType)
        return true;

    switch (imageFrameType)
    {
        case INDI::CCDChip::BIAS_FRAME:
        case INDI::CCDChip::DARK_FRAME:
            device->setDarkFrameMode(true);
            break;

        case INDI::CCDChip::LIGHT_FRAME:
        case INDI::CCDChip::FLAT_FRAME:
            device->setDarkFrameMode(false);
            break;
    }

    PrimaryCCD.setFrameType(fType);

    return true;
}

bool ATIKCCD::UpdateCCDFrame(int x, int y, int w, int h)
{
    /* Add the X and Y offsets */
    long x_1 = x;
    long y_1 = y;

    long bin_width  = x_1 + (w / PrimaryCCD.getBinX());
    long bin_height = y_1 + (h / PrimaryCCD.getBinY());

    if (bin_width > PrimaryCCD.getXRes() / PrimaryCCD.getBinX())
    {
        LOGF_INFO("Error: invalid width requested %d", w);
        return false;
    }
    else if (bin_height > PrimaryCCD.getYRes() / PrimaryCCD.getBinY())
    {
        LOGF_INFO("Error: invalid height request %d", h);
        return false;
    }

    /**********************************************************
   *
   *
   *
   *  IMPORTANT: Put here your CCD Frame dimension call
   *  The values calculated above are BINNED width and height
   *  which is what most CCD APIs require, but in case your
   *  CCD API implementation is different, don't forget to change
   *  the above calculations.
   *  If there is an error, report it back to client
   *  e.g.
   *  LOG_INFO( "Error, unable to set frame to ...");
   *  return false;
   *
   *
   **********************************************************/

    // Set UNBINNED coords
    PrimaryCCD.setFrame(x_1, y_1, w, h);

    int nbuf;
#if 0
    nbuf = (bin_width * bin_height * PrimaryCCD.getBPP() / 8); //  this is pixel count
#else
    nbuf = device->imageWidth(w, PrimaryCCD.getBinX()) * device->imageWidth(h, PrimaryCCD.getBinY()) * 2;
#endif
    nbuf += 512; //  leave a little extra at the end
    PrimaryCCD.setFrameBufferSize(nbuf);

    LOGF_DEBUG("Setting frame buffer size to %d bytes.", nbuf);

    return true;
}

bool ATIKCCD::UpdateCCDBin(int binx, int biny)
{
    PrimaryCCD.setBin(binx, biny);

    return UpdateCCDFrame(PrimaryCCD.getSubX(), PrimaryCCD.getSubY(), PrimaryCCD.getSubW(), PrimaryCCD.getSubH());
}

float ATIKCCD::CalcTimeLeft()
{
    if (ShortExposure)
    {
        return 0.0f;
    }

    double timesince;
    double timeleft;
    struct timeval now;
    gettimeofday(&now, NULL);

    timesince = (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) -
                (double)(ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec / 1000);
    timesince = timesince / 1000;

    timeleft = ExposureRequest - timesince;
    return timeleft;
}

/* Downloads the image from the CCD.
 N.B. No processing is done on the image */
int ATIKCCD::grabImage()
{
    uint16_t *image = (uint16_t *)PrimaryCCD.getFrameBuffer();
    unsigned width  = device->imageWidth(PrimaryCCD.getSubW(), PrimaryCCD.getBinX());
    unsigned height = device->imageHeight(PrimaryCCD.getSubH(), PrimaryCCD.getBinY());

    if (!ShortExposure)
    {
        device->readCCD(PrimaryCCD.getSubX() * PrimaryCCD.getBinX(), PrimaryCCD.getSubY() * PrimaryCCD.getBinY(),
                        width * PrimaryCCD.getBinX(), height * PrimaryCCD.getBinY(), PrimaryCCD.getBinX(),
                        PrimaryCCD.getBinY());
    }

    device->getImage(image, width * height);

    LOG_INFO("Download complete.");

    ExposureComplete(&PrimaryCCD);

    return 0;
}

void ATIKCCD::TimerHit()
{
    int timerID = -1;
    long timeleft;

    if (isConnected() == false)
        return; //  No need to reset timer if we are not connected anymore

    if (InExposure)
    {
        timeleft = CalcTimeLeft();

        if (timeleft < 1.0)
        {
            if (timeleft > 0.25)
            {
                //  a quarter of a second or more
                //  just set a tighter timer
                timerID = SetTimer(250);
            }
            else
            {
                if (timeleft > 0.07)
                {
                    //  use an even tighter timer
                    timerID = SetTimer(50);
                }
                else
                {
                    //  it's real close now, so spin on it
                    while (timeleft > 0)
                    {
                        /**********************************************************
             			*
			            *  IMPORTANT: If supported by your CCD API
			            *  Add a call here to check if the image is ready for download
			            *  If image is ready, set timeleft to 0. Some CCDs (check FLI)
			            *  also return timeleft in msec.
			            *
			            **********************************************************/

                        // Breaking in simulation, in real driver either loop until time left = 0 or use an API call to know if the image is ready for download
                        break;

                        //int slv;
                        //slv = 100000 * timeleft;
                        //usleep(slv);
                    }

                    /* We're done exposing */
                    LOG_INFO("Exposure done, downloading image...");

                    PrimaryCCD.setExposureLeft(0);
                    InExposure = false;
                    /* grab and save image */
                    grabImage();
                }
            }
        }
        else
        {
            if (isDebug())
            {
                IDLog("With time left %ld\n", timeleft);
                IDLog("image not yet ready....\n");
            }

            PrimaryCCD.setExposureLeft(timeleft);
        }
    }

    float temperature;
    device->getTemperatureSensorStatus(1, &temperature);
    TemperatureN[0].value = temperature;

    switch (TemperatureNP.s)
    {
        case IPS_IDLE:
        case IPS_OK:
            IDSetNumber(&TemperatureNP, NULL);
            break;

        case IPS_BUSY:
            // If we're within threshold, let's make it BUSY ---> OK
            if (fabs(TemperatureRequest - TemperatureN[0].value) <= TEMP_THRESHOLD)
                TemperatureNP.s = IPS_OK;

            IDSetNumber(&TemperatureNP, NULL);
            break;

        case IPS_ALERT:
            break;
    }

    if (timerID == -1)
        SetTimer(POLLMS);
    return;
}

IPState ATIKCCD::GuideNorth(float ms)
{
    INDI_UNUSED(ms);
    /**********************************************************
   *
   *
   *
   *  IMPORRANT: Put here your CCD Guide call
   *  Some CCD API support pulse guiding directly (i.e. without timers)
   *  Others implement GUIDE_ON and GUIDE_OFF for each direction, and you
   *  will have to start a timer and then stop it after the 'ms' milliseconds
   *  For an example on timer usage, please refer to indi-sx and indi-gpusb drivers
   *  available in INDI 3rd party repository
   *  If there is an error, report it back to client
   *  e.g.
   *  LOG_INFO( "Error, unable to guide due ...");
   *  return IPS_ALERT;
   *
   *
   **********************************************************/

    return IPS_OK;
}

IPState ATIKCCD::GuideSouth(float ms)
{
    INDI_UNUSED(ms);
    return IPS_OK;
}

IPState ATIKCCD::GuideEast(float ms)
{
    INDI_UNUSED(ms);
    return IPS_OK;
}

IPState ATIKCCD::GuideWest(float ms)
{
    INDI_UNUSED(ms);
    return IPS_OK;
}
