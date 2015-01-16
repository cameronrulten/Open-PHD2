/*
 *  cam_QHY5IIbase.cpp
 *  PHD Guiding
 *
 *  Created by Craig Stark.
 *  Copyright (c) 2012 Craig Stark.
 *  All rights reserved.
 *
 *  This source code is distributed under the following "BSD" license
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *    Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *    Neither the name of Craig Stark, Stark Labs nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "phd.h"

#if defined(QHY5II) || defined(QHY5LII)

#include "cam_QHY5II.h"

#ifdef __WINDOWS__  // Windows cameras
typedef DWORD (CALLBACK* Q5II_DW_V)(void);
//EXPORT DWORD _stdcall openUSB;  Open USB for the selected camera. Returnd 1 is a camera was found, 0 otherwise
Q5II_DW_V Q5II_OpenUSB;
//EXPORT DWORD _stdcall isExposing();  Indicates if the camera is currently performing an exposure
Q5II_DW_V Q5II_IsExposing;

typedef void (CALLBACK* Q5II_V_V)(void);
//EXPORT void _stdcall CancelExposure();  Cancels the current exposure
Q5II_V_V Q5II_CancelExposure;
//EXPORT void _stdcall closeUSB;  Closes the USB connection.
Q5II_V_V Q5II_CloseUSB;
//EXPORT void _stdcall StopCapturing();  Stops any started capturing Thread
Q5II_V_V Q5II_StopCapturing;
//EXPORT void _stdcall SingleExposure();  Clears the buffer and starts a new exposure, stops capturing after one image
Q5II_V_V Q5II_SingleExposure;
//EXPORT void _stdcall SetAutoBlackLevel();  Set automatic black level, QHY5-II Only
// Seems not in there...
//Q5II_V_V Q5II_SetAutoBlackLevel;

typedef void (CALLBACK* Q5II_V_DW)(DWORD);
//EXPORT void _stdcall SetBlackLevel( DWORD n );  QHY5-II Only Sted the blacklevel of the camera (direct register write)
// Seems to actually be the auto black level
Q5II_V_DW Q5II_SetBlackLevel;
//EXPORT void _stdcall SetGain(DWORD x );  Set gainlevel (0-100)
Q5II_V_DW Q5II_SetGain;
//EXPORT void _stdcall SetExposureTime( DWORD milisec ) ;  Set exposuretime in miliseconds
Q5II_V_DW Q5II_SetExposureTime;
//EXPORT void _stdcall SetSpeed( DWORD n );  Set camera speed (0=slow, 1=medium, 2=fast). QHY5L may not support all speeds in higher resolutions.
Q5II_V_DW Q5II_SetSpeed;
//EXPORT void _stdcall SetHBlank( DWORD hBlank) ; Sets the USB delay factor to a value between 0 and 2047
Q5II_V_DW Q5II_SetHBlank;
//EXPORT void _stdcall SetLineremoval ( DWORD lineremoval ) ; Enable line noise removal algorithm. QHY5-II Only
Q5II_V_DW Q5II_SetLineRemoval;


//EXPORT DWORD _stdcall CancelGuide( DWORD Axis );  Cancel guides on a specific axis

typedef DWORD (CALLBACK* Q5II_GFD)(PUCHAR, DWORD);
//EXPORT DWORD _stdcall getFrameData(PUCHAR _buffer , DWORD size ) ;  Gets the data after a SingleExposure(), 8 bit data
Q5II_GFD Q5II_GetFrameData;

typedef DWORD (CALLBACK* Q5II_GC)(DWORD, DWORD, DWORD);
//EXPORT DWORD _stdcall GuideCommand(DWORD GC, DWORD PulseTimeX, DWORD PulseTimeY) ;
Q5II_GC Q5II_GuideCommand;
#endif


Camera_QHY5IIBase::Camera_QHY5IIBase()
{
    Connected = false;
    m_hasGuideOutput = true;
    HasGainControl = true;
    RawBuffer = NULL;
}

#ifdef __WINDOWS__  // Windows cameras
static FARPROC WINAPI GetProc(HINSTANCE dll, LPCSTR name)
{
    FARPROC p = GetProcAddress(dll, name);
    if (!p)
    {
        FreeLibrary(dll);
        wxMessageBox(wxString::Format(_("Camera DLL missing entry %s"), name), _("Error"), wxOK | wxICON_ERROR);
    }
    return p;
}
#endif

bool Camera_QHY5IIBase::Connect()
{
    char cam_type[16] = {'\0'};
    char id[32] = {'\0'};
    int found = false;
    // returns true on error
#ifndef __WINDOWS__
    int imagew, imageh, bpp;
    double chipw, chiph, pixelw, pixelh;
#endif

#ifdef __WINDOWS__  // Windows cameras
    CameraDLL = LoadLibrary(m_cameraDLLName);

    if (CameraDLL == NULL)
    {
        wxMessageBox(wxString::Format(_("Cannot load camera dll %s.dll"), m_cameraDLLName), _("Error"),wxOK | wxICON_ERROR);
        return true;
    }

#define GET_PROC(p, type, name) do { \
    if ((p = (type) GetProc(CameraDLL, name)) == 0) \
        return true; \
} while (false)

    GET_PROC(Q5II_OpenUSB,           Q5II_DW_V, "openUSB");
    GET_PROC(Q5II_IsExposing,        Q5II_DW_V, "isExposing");
    GET_PROC(Q5II_CancelExposure,    Q5II_V_V,  "CancelExposure");
    GET_PROC(Q5II_CloseUSB,          Q5II_V_V,  "closeUSB");
    GET_PROC(Q5II_StopCapturing,     Q5II_V_V,  "StopCapturing");
    GET_PROC(Q5II_SingleExposure,    Q5II_V_V,  "SingleExposure");
//  GET_PROC(Q5II_SetAutoBlackLevel, Q5II_V_V,  "SetAutoBlackLevel");
    GET_PROC(Q5II_SetBlackLevel,     Q5II_V_DW, "SetBlackLevel");
    GET_PROC(Q5II_SetGain,           Q5II_V_DW, "SetGain");
    GET_PROC(Q5II_SetExposureTime,   Q5II_V_DW, "SetExposureTime");
    GET_PROC(Q5II_SetSpeed,          Q5II_V_DW, "SetSpeed");
    GET_PROC(Q5II_SetHBlank,         Q5II_V_DW, "SetHBlank");
    GET_PROC(Q5II_GetFrameData,      Q5II_GFD,  "getFrameData");
    GET_PROC(Q5II_GuideCommand,      Q5II_GC,   "GuideCommand");

#undef GET_PROC

    if (!Q5II_OpenUSB())
    {
        wxMessageBox(_("No camera"));
        return true;
    }
#endif

    if (InitQHYCCDResource() != QHYCCD_SUCCESS)
    {
        wxMessageBox(_("Fail to init libqhyccd"));
        return true;
    }

    int cam_num = ScanQHYCCD();
    if (cam_num <= 0)
    {
        wxMessageBox(_("No QHYCCD was connected"));
        ReleaseQHYCCDResource();
        return true;
    }

    for (int i = 0; i < cam_num; i++)
    {
        if (GetQHYCCDId(i, id) == QHYCCD_SUCCESS)
        {
            strncpy(cam_type, id, 8);
            cam_type[8] = '\0';
            if (NULL != strstr(cam_type, "QHY5"))
            {
                found = true;
                break;
            }
        }
    }

    if (found != true)
    {
        wxMessageBox(_("No QHY5LII series cameras was connected"));
        ReleaseQHYCCDResource();
        return true;
    }

    cam_handle = OpenQHYCCD(id);
    if (!cam_handle)
    {
        wxMessageBox(_("Fail to open QHY5LII camera"));
        ReleaseQHYCCDResource();
        return true;
    }

    if (InitQHYCCD(cam_handle) != QHYCCD_SUCCESS)
    {
        wxMessageBox(_("Fail to init QHY5LII camera"));
        ReleaseQHYCCDResource();
        return true;
    }

    if (RawBuffer)
        delete [] RawBuffer;

#ifdef __WINDOWS__
    size_t size = FullSize.GetWidth() * FullSize.GetHeight();
    RawBuffer = new unsigned char[size];
#else
    GetQHYCCDChipInfo(cam_handle, &chipw, &chiph, &imagew, &imageh, &pixelw, &pixelh, &bpp);
    FullSize = wxSize(imagew, imageh);
    RawBuffer = new unsigned char[imagew * imageh];
#endif

#ifdef __WINDOWS__  // Windows cameras
    //Q5II_SetAutoBlackLevel();
    Q5II_SetBlackLevel(1);
    Q5II_SetSpeed(0);
#elif defined (__LINUX__)
    //SetQHYCCDParam(cam_handle, CONTROL_SPEED, 0);
    SetQHYCCDParam(cam_handle, CONTROL_TRANSFERBIT, 8);
    //SetQHYCCDResolution(cam_handle, 0, 0, 1280, 960);
#endif
    Connected = true;

    return false;
}

bool Camera_QHY5IIBase::ST4PulseGuideScope(int direction, int duration)
{
#ifdef __WINDOWS__
    DWORD reg = 0;
    DWORD dur = (DWORD) duration / 10;
    DWORD ptx, pty;

    //if (dur >= 255) dur = 254; // Max guide pulse is 2.54s -- 255 keeps it on always
    switch (direction) {
        case WEST: reg = 0x80; ptx = dur; pty = 0xFFFFFFFF; break;
        case NORTH: reg = 0x20; pty = dur; ptx = 0xFFFFFFFF; break;
        case SOUTH: reg = 0x40; pty = dur; ptx = 0xFFFFFFFF; break;
        case EAST: reg = 0x10;  ptx = dur; pty = 0xFFFFFFFF; break;
        default: return true; // bad direction passed in
    }
    Q5II_GuideCommand(reg,ptx,pty);
    wxMilliSleep(duration + 10);
#elif defined (__LINUX__)
    if (ControlQHYCCDGuide(cam_handle, direction, duration) != QHYCCD_SUCCESS)
        return true;
    else
#endif
    return false;
}

void Camera_QHY5IIBase::ClearGuidePort()
{
    //Q5II_CancelGuide(3); // 3 clears on both axes
}

void Camera_QHY5IIBase::InitCapture()
{
#ifdef __WINDOWS__
    Q5II_SetGain(GuideCameraGain);
#elif defined (__LINUX__)
    SetQHYCCDParam(cam_handle, CONTROL_GAIN, GuideCameraGain);
#endif
}

bool Camera_QHY5IIBase::Disconnect()
{
#ifdef __WINDOWS__
    Q5II_CloseUSB();
#elif defined (__LINUX__)
    CloseQHYCCD(cam_handle);
    ReleaseQHYCCDResource();
#endif
    Connected = false;
    if (RawBuffer)
        delete [] RawBuffer;
    RawBuffer = NULL;
#ifdef __WINDOWS__
    FreeLibrary(CameraDLL);
#endif

    return false;
}

bool Camera_QHY5IIBase::Capture(int duration, usImage& img, wxRect subframe, bool recon)
{
// Only does full frames still
    static int last_dur = 0;
    static int last_gain = 60;
#ifdef __WINDOWS__
    unsigned char *bptr;
    unsigned short *dptr;
    int  x,y;
    int xsize = FullSize.GetWidth();
    int ysize = FullSize.GetHeight();
#endif
    int ret;
//  bool firstimg = true;

    if (img.Init(FullSize))
    {
        pFrame->Alert(_("Memory allocation error during capture"));
        Disconnect();
        return true;
    }

    if (duration != last_dur) {
#ifdef __WINDOWS__
        Q5II_SetExposureTime(duration);
#else
        ret = SetQHYCCDParam(cam_handle, CONTROL_EXPOSURE, duration * 1000);
        if (ret == QHYCCD_SUCCESS)
            Debug.AddLine("QHYCCD: Set Expo to %d ms.", duration);
        else
            Debug.AddLine("QHYCCD: Fail to set Expo to %d ms.", duration);
#endif
        last_dur = duration;
    }

    if (GuideCameraGain != last_gain) {
#ifdef __WINDOWS__
        Q5II_SetGain(GuideCameraGain);
#else
        ret = SetQHYCCDParam(cam_handle, CONTROL_GAIN, GuideCameraGain);
        if (ret == QHYCCD_SUCCESS)
            Debug.AddLine("QHYCCD: Set gain to %d .", GuideCameraGain);
        else
            Debug.AddLine("QHYCCD: Fail to set gain to %d .", GuideCameraGain);
#endif
        last_gain = GuideCameraGain;
    }

#ifdef __WINDOWS__
    Q5II_SingleExposure();
    wxMilliSleep(duration);
    //int foo = 0;
    while (Q5II_IsExposing()) {
        //pFrame->SetStatusText(wxString::Format("%d",foo));
        wxMilliSleep(100);
        //foo++;
    }

    Q5II_GetFrameData(RawBuffer,xsize*ysize);

    bptr = RawBuffer;
    // Load and crop from the 800 x 525 image that came in
    dptr = img.ImageData;
    for (y=0; y<ysize; y++) {
        for (x=0; x<xsize; x++, bptr++, dptr++) { // CAN SPEED THIS UP
            *dptr=(unsigned short) *bptr;
        }
    }

#else
    bool useSubframe = UseSubframes;

    if (subframe.width <= 0 || subframe.height <= 0)
        useSubframe = false;

    if (useSubframe)
    {
        ret = SetQHYCCDResolution(cam_handle, subframe.x, subframe.y, subframe.width + 1, subframe.height + 1);
        Debug.AddLine("QHYCCD: use subframe at (%d,%d)+(%d,%d)", subframe.x, subframe.y, subframe.width, subframe.height);
        fprintf(stderr, "QHYCCD: use subframe at (%d,%d)+(%d,%d)\n", subframe.x, subframe.y, subframe.width, subframe.height);
        if (ret != QHYCCD_SUCCESS)
            Debug.AddLine("QHYCCD: Fail to set new resolution");
    }
    else
    {
        ret = SetQHYCCDResolution(cam_handle, 0, 0, FullSize.GetWidth(), FullSize.GetHeight());
        Debug.AddLine("QHYCCD: use FullSize at (%d,%d)+(%d,%d)", 0, 0, FullSize.GetWidth(), FullSize.GetHeight());
        fprintf(stderr, "QHYCCD: use FullSize at (%d,%d)+(%d,%d)\n", 0, 0, FullSize.GetWidth(), FullSize.GetHeight());
        if (ret != QHYCCD_SUCCESS)
            Debug.AddLine("QHYCCD: Fail to set new resolution");
    }

    SetQHYCCDBinMode(cam_handle, 1, 1);
    ret = ExpQHYCCDSingleFrame(cam_handle);
    if (ret == QHYCCD_SUCCESS)
        Debug.AddLine("QHYCCD: Begin to Expo.");
    else
        Debug.AddLine("QHYCCD: Fail to begin Expo.");

    wxMilliSleep(duration + 1);

    int w, h, bpp, chn;
    ret = GetQHYCCDSingleFrame(cam_handle, &w, &h, &bpp, &chn, RawBuffer);
    if (ret == QHYCCD_SUCCESS) {
        Debug.AddLine("QHYCCD: Get image data (%d * %d) ok.", w, h);
        fprintf(stderr, "QHYCCD: Get image data (%d * %d) ok.\n", w, h);
    } else
        Debug.AddLine("QHYCCD: Fail to get image data .");

    if (useSubframe)
    {
        img.Subframe = subframe;
        img.Clear();

        const unsigned char *src = RawBuffer;
        for (int y = 0; y < subframe.height + 1; y++)
        {
            unsigned short *dst = img.ImageData + (y + subframe.y) * FullSize.GetWidth() + subframe.x;
            for (int x = 0; x < subframe.width + 1; x++)
                *dst++ = (unsigned short)*src++;
        }
    }
    else
    {
        for (int i = 0; i < img.NPixels; i++)
            img.ImageData[i] = (unsigned short)RawBuffer[i];
    }
#endif

    if (recon) SubtractDark(img);

    return false;
}

/*bool Camera_QHY5IIBase::CaptureCrop(int duration, usImage& img) {
    GenericCapture(duration, img, width,height,startX,startY);

return false;
}

bool Camera_QHY5IIBase::CaptureFull(int duration, usImage& img) {
    GenericCapture(duration, img, FullSize.GetWidth(),FullSize.GetHeight(),0,0);

    return false;
}
*/

#endif
