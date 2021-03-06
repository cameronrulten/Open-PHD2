/*
 *  cam_QGuide.h
 *  PHD Guiding
 *
 *  Created by Craig Stark.
 *  Copyright (c) 2007-2010 Craig Stark.
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
#ifndef QGUIDEDEF
#define QGUIDEDEF
#include "cmosDLL.h"
class Camera_QGuiderClass : public GuideCamera {
public:
    bool    Capture(int duration, usImage& img, wxRect subframe = wxRect(0,0,0,0), bool recon=false);
    bool    Connect();
    bool    Disconnect();
    void    InitCapture();

//  bool    SetGlobalGain(unsigned char gain);
    bool    ST4PulseGuideScope(int direction, int duration);
    void    ClearGuidePort();
    //bool  SetColorGain(unsigned char r_gain, unsigned char g_gain, unsigned char b_gain);
    //int   FindCameras(int DevNums[8]);
    bool    HasNonGuiCapture(void) { return true; }
    bool    ST4HasNonGuiMove(void) { return true; }

    Camera_QGuiderClass();
private:
//  bool GenericCapture(int duration, usImage& img, int xsize, int ysize, int xpos, int ypos);
    unsigned char *buffer;
    void RemoveLines(usImage& img);
};
#endif  //QGUIDEDEF


