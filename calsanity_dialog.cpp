/*
 *  calsanity_dialog.cpp
 *  PHD Guiding
 *
 *  Created by Bruce Waddington
 *  Copyright (c) 2014 Bruce Waddington
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
 *    Neither the name of Bret McKee, Dad Dog Development,
 *     Craig Stark, Stark Labs nor the names of its
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
#include "calsanity_dialog.h"

static const int MESSAGE_HEIGHT = 100;

static void HighlightCell(wxGrid *pGrid, int row, int col)
{
    pGrid->SetCellBackgroundColour(row, col, "DARK SLATE GREY");
    pGrid->SetCellTextColour(row, col, "white");
}

void CalSanityDialog::BuildMessage(wxStaticText *pText, Calibration_Issues etype)
{
    wxString msg;

    switch (etype)
    {
    case CI_Steps:
        msg = _("The calibration was done with a very small number of steps, which can produce inaccurate results. "
            "Consider reducing the size of the calibration step parameter until you see at least 8 steps in each direction.  The 'calculator' "
            "feature in the 'Mount' configuration tab can help you with this.");
        break;
    case CI_Angle:
        msg = wxString::Format(_("The RA and Declination angles computed in the calibration are questionable.  Normally, "
            "these angles will be nearly perpendicular, having an 'orthogonality error' of less than 10 degrees.  In this calibration, your error was %s degrees. This "
            "could mean the calibration is inaccurate, perhaps because of small or erratic star movement during the calibration."), m_newAngleDelta);
        break;
    case CI_Different:
        msg = wxString::Format(_("The most recent calibration produced results that are %s%% different from the previous calibration.  If this is because "
            "you changed equipment configurations, you may want to use different profiles.  Doing so will allow you to switch back "
            "and forth between configurations and still retain earlier settings and calibration results."), m_oldNewDifference);
        break;
    case CI_Rates:
        msg = wxString::Format(_("The RA and Declination guiding rates differ by an unexpected amount.  For your declination of %0.0f degrees, "
            "the RA rate should be about %0.0f%% of the Dec rate.  But your RA rate is %0.0f%% of the Dec rate.  "
            "This could mean the calibration is inaccurate, perhaps because of small or erratic star movement during the calibration."),
            degrees(m_newParams.declination), cos(m_newParams.declination) * 100.0, m_newParams.xRate / m_newParams.yRate * 100.0);
        break;
    default:
        break;
    }
    pText->SetLabel(msg);
    pText->Wrap(380);
}

CalSanityDialog::CalSanityDialog(const Calibration& oldParams, const Calibration& newParams,
    int lastRASteps, int lastDecSteps, Calibration_Issues issue, Scope *pScope) :
    wxDialog(pFrame, wxID_ANY, _("Calibration Sanity Check"), wxDefaultPosition, wxSize(800, 400), wxCAPTION | wxCLOSE_BOX)
{
    wxString raSteps = wxString::Format("%d", lastRASteps);
    wxString decSteps = wxString::Format("%d", lastDecSteps);
    wxString oldAngleDelta;
    double newRARate = newParams.xRate * 1000;                          // px per sec for UI purposes
    double newDecRate = newParams.yRate * 1000;
    double imageScale = pFrame->GetCameraPixelScale();
    bool oldValid = oldParams.declination < INVALID_DECLINATION;
    m_newParams = newParams;

    // Compute the orthogonality stuff
    double nonOrtho = degrees(fabs(fabs(norm_angle(m_newParams.xAngle - m_newParams.yAngle)) - M_PI / 2.));         // Delta from the nearest multiple of 90 degrees
    m_newAngleDelta = wxString::Format("%0.1f", nonOrtho);
    if (oldValid)
    {
        nonOrtho = degrees(fabs(fabs(norm_angle(oldParams.xAngle - oldParams.yAngle)) - M_PI / 2.));
        oldAngleDelta = wxString::Format("%0.1f", nonOrtho);
    }
    else
        oldAngleDelta = _("Unknown");

    if (m_newParams.yRate != 0. && oldParams.yRate != 0.)
        m_oldNewDifference = wxString::Format("%0.1f", fabs(1.0 - m_newParams.yRate / oldParams.yRate) * 100.0);
    else
        m_oldNewDifference = "";

    // Lay out the controls
    wxBoxSizer *pVSizer = new wxBoxSizer(wxVERTICAL);
    wxStaticBoxSizer *pMsgGrp = new wxStaticBoxSizer(wxVERTICAL, this, _("Explanation"));

    // Explanation area
    wxStaticText *pMsgArea = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(400, -1), wxALIGN_LEFT | wxST_NO_AUTORESIZE);
    BuildMessage(pMsgArea, issue);
    pMsgArea->SetSizeHints(wxSize(-1, MESSAGE_HEIGHT));
    wxFont font = pMsgArea->GetFont();
    font.SetWeight(wxFONTWEIGHT_BOLD);
    pMsgArea->SetFont(font);
    pMsgGrp->Add(pMsgArea, wxSizerFlags().Border(wxLEFT, 10).Border(wxBOTTOM, 10));
    pVSizer->Add(pMsgGrp, wxSizerFlags().Border(wxLEFT, 10).Border(wxBOTTOM, 10));

    // Grid control for details
    wxStaticBoxSizer *pGridGrp = new wxStaticBoxSizer(wxVERTICAL, this, _("Details"));
    wxGrid *pGrid = new wxGrid(this, wxID_ANY);
    pGrid->CreateGrid(3, 4);
    pGrid->SetRowLabelSize(1);
    pGrid->SetColLabelSize(1);
    pGrid->EnableEditing(false);

    int col = 0;
    int row = 0;
    pGrid->SetCellValue( _("Steps, RA:"), row, col++);
    pGrid->SetCellValue(raSteps, row, col++);
    pGrid->SetCellValue(_("Steps, Dec:"), row, col++);
    pGrid->SetCellValue(decSteps, row, col++);
    if (issue == CI_Steps){
      if (raSteps <= decSteps)
         HighlightCell(pGrid, row, 1);
      else
         HighlightCell(pGrid, row, 3);
    }
    row++;
    col = 0;
    pGrid->SetCellValue(_("Orthogonality error:"), row, col++);
    pGrid->SetCellValue( m_newAngleDelta, row, col++);
    pGrid->SetCellValue(_("Previous orthogonality error:"), row, col++);
    pGrid->SetCellValue(oldAngleDelta, row, col++);
    if (issue == CI_Angle)
    {
        HighlightCell(pGrid, row, 1);
    }

    row++;
    col = 0;
    // Show either the new RA and Dec rates or the new and old Dec rates depending on the issue
    if (issue == CI_Different)
    {
        pGrid->SetCellValue(_("This declination rate:"), row, col++);
        pGrid->SetCellValue(wxString::Format("%0.3f ''/sec\n%0.3f px/sec", newDecRate * imageScale, newDecRate), row, col++);
        pGrid->SetCellValue(_("Previous declination rate:"), row, col++);
        pGrid->SetCellValue(wxString::Format("\n%0.3f px/sec", oldParams.yRate * 1000), row, col++);
        HighlightCell(pGrid, row, 1);
        HighlightCell(pGrid, row, 3);
    }
    else
    {
        pGrid->SetCellValue(_("RA rate:"), row, col++);
        pGrid->SetCellValue(wxString::Format("%0.3f ''/sec\n%0.3f px/sec", newRARate * imageScale, newRARate), row, col++);
        pGrid->SetCellValue(_("Declination rate:"), row, col++);
        pGrid->SetCellValue(wxString::Format("%0.3f ''/sec\n%0.3f px/sec", newDecRate * imageScale, newDecRate), row, col++);
        if (issue == CI_Rates)
        {
            HighlightCell(pGrid, row, 1);
            HighlightCell(pGrid, row, 3);
        }
    }

    pGrid->AutoSize();
    pGrid->ClearSelection();
    pGridGrp->Add(pGrid);
    pVSizer->Add(pGridGrp, wxSizerFlags(0).Border(wxALL, 10));

    // Checkboxes for being quiet
    m_pBlockThis = new wxCheckBox(this, wxID_ANY, _("Don't show calibration alerts of this type"));
    pVSizer->Add(m_pBlockThis, wxSizerFlags(0).Border(wxALL, 15));


    // Now deal with the buttons
    wxBoxSizer *pButtonSizer = new wxBoxSizer( wxHORIZONTAL );

    wxButton *pIgnore = new wxButton( this, wxID_ANY, _("Accept calibration") );
    pIgnore->SetToolTip(_("Accept the calibration as being valid and continue guiding"));
    pIgnore->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &CalSanityDialog::OnIgnore, this);
    wxButton *pRecal = new wxButton(this, wxID_ANY, _("Discard calibration"));
    pRecal->SetToolTip(_("Stop guiding and discard the most recent calibration.  Calibration will be re-done the next time you start guiding"));
    pRecal->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &CalSanityDialog::OnRecal, this);
    wxButton *pRestore = new wxButton(this, wxID_ANY, _("Restore old calibration"));
    pRestore->SetToolTip(_("Stop guiding, discard the most recent calibration, then load the previous (good) calibration"));
    pRestore->Bind(wxEVT_COMMAND_BUTTON_CLICKED, &CalSanityDialog::OnRestore, this);
    pRestore->Enable(oldValid);

    pButtonSizer->Add(
        pIgnore,
        wxSizerFlags(0).Align(0).Border(wxRIGHT | wxLEFT | wxBOTTOM, 10));
    pButtonSizer->Add(
        pRecal,
        wxSizerFlags(0).Align(0).Border(wxRIGHT | wxLEFT | wxBOTTOM, 10));
    pButtonSizer->Add(
        pRestore,
        wxSizerFlags(0).Align(0).Border(wxRIGHT | wxLEFT | wxBOTTOM, 10));

     //position the buttons centered with no border
     pVSizer->Add(
        pButtonSizer,
        wxSizerFlags(0).Center() );
    SetSizerAndFit (pVSizer);

    m_priorCalibration = oldParams;
    m_issue = issue;
    m_pScope = pScope;
}

CalSanityDialog::~CalSanityDialog(void)
{
}

// Handle the user choices for blocking/restoring future alerts
void CalSanityDialog::SaveBlockingOptions()
{
    if (m_pBlockThis->IsChecked())
        m_pScope->SetCalibrationWarning(m_issue, false);
}

void CalSanityDialog::ShutDown()
{
    SaveBlockingOptions();
    wxDialog::Close();
}

void CalSanityDialog::OnIgnore(wxCommandEvent& evt)
{
    Debug.AddLine("Calibration sanity check: user chose to ignore alert");
    ShutDown();
}

// Stop guiding if it's active, then clear the calibration - recal will be triggered with the next guide-start
void CalSanityDialog::OnRecal(wxCommandEvent& evt)
{
    if (pFrame->pGuider && pFrame->pGuider->IsCalibratingOrGuiding())
        pFrame->StopCapturing();
    Debug.AddLine("Calibration sanity check: user discarded bad calibration");
    pMount->ClearCalibration();
    ShutDown();
}

// Stop guiding if it's active, then restore the data from the previous calibration
void CalSanityDialog::OnRestore(wxCommandEvent& evt)
{
    if (pFrame->pGuider && pFrame->pGuider->IsCalibratingOrGuiding())
        pFrame->StopCapturing();

    m_pScope->SetCalibration(m_priorCalibration);

    pFrame->LoadCalibration();
    Debug.AddLine("Calibration sanity check: user chose to restore old calibration");
    ShutDown();
}
