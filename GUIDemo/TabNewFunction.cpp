#include "TabNewFunction.h"
#include "ImageClient.h"
#include "QControlUtils.h"
#include "ui_GUIDemo.h"

//-----------------------------------------------------------------------------
// tTabNewFunction Implementation
//-----------------------------------------------------------------------------
tTabNewFunction::tTabNewFunction(Ui::GUIDemoClass& ui, QObject* pParent,
                                 QWidget& tab)
    : tTabBase(ui, pParent, tab) {
  connect(ui.checkBox_DataRecord, SIGNAL(clicked(bool)), this,
          SLOT(RecordData_check(bool)));
  connect(ui.checkBox_datatransmit, SIGNAL(clicked(bool)), this,
          SLOT(TransmitData_check(bool)));
  connect(ui.checkBox_TT, SIGNAL(clicked(bool)), this,
          SLOT(TransmitData_check(bool)));
  //  ConnectControls(true, *this, tab);
}

//-----------------------------------------------------------------------------
tTabNewFunction::~tTabNewFunction() {
  //    ConnectControls(false, *this, m_Tab);
}

//-----------------------------------------------------------------------------
void tTabNewFunction::OnConnect(
    Navico::Protocol::NRP::tImageClient* pImageClient) {
  tTabBase::OnConnect(pImageClient);
}

void tTabNewFunction::RecordData_check(bool checked) { IsDatabase_ = checked; }

void tTabNewFunction::TransmitData_check(bool checked) {
  IsDataTransmission_ = checked;
}

void tTabNewFunction::TrackTarget_check(bool checked) {
  IsTargetTracking_ = checked;
}
