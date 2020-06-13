#ifndef TABNEWFUNCTION_H
#define TABNEWFUNCTION_H

#include <Feature.h>
#include <NavRadarProtocol.h>
#include <QObject>
#include "TabBase.h"

//-----------------------------------------------------------------------------
// tTabNewFunction Class
//-----------------------------------------------------------------------------
class tTabNewFunction : public tTabBase {
  Q_OBJECT

 public:
  //-----------------------------------------------------------------------------
  // Constructor, destructor and initialization
  //-----------------------------------------------------------------------------
  tTabNewFunction(Ui::GUIDemoClass& ui, QObject* pParent, QWidget& tab);
  ~tTabNewFunction();

  void OnConnect(Navico::Protocol::NRP::tImageClient* pImageClient);
  bool IsDatabase() const noexcept { return IsDatabase_; }
  bool IsDataTransmission() const noexcept { return IsDataTransmission_; }
  bool IsTargetTracking() const noexcept { return IsTargetTracking_; }

 private slots:
  //-----------------------------------------------------------------------------
  // UI Signal Handling Interface
  //-----------------------------------------------------------------------------
  // group: Configuration
  void RecordData_check(bool checked);
  void TransmitData_check(bool checked);
  void TrackTarget_check(bool checked);

 private:
  bool IsDatabase_ = false;
  bool IsDataTransmission_ = false;
  bool IsTargetTracking_ = false;

};

//-----------------------------------------------------------------------------

#endif  // TABNEWFUNCTION_H
