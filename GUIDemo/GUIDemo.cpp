//-----------------------------------------------------------------------------
// Copyright (C) 2007-2016 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------

#include "GUIDemo.h"
#include "QControlUtils.h"

#include <algorithm>
#include <thread>

#include <NavRadarProtocol.h>
#include <cstring>

//-----------------------------------------------------------------------------
// Helper Functions
//-----------------------------------------------------------------------------

template <typename Type>
inline void InitStruct(Type*& ptr, unsigned count = 1) {
  if (ptr == nullptr) {
    if (count > 1)
      ptr = new Type[count];
    else
      ptr = new Type;
  }

  if (ptr != nullptr) {
    if (count > 1)
      memset(ptr, 0, count * sizeof(Type));
    else
      memset(ptr, 0, sizeof(Type));
  }
}

//-----------------------------------------------------------------------------
//  Helper Functions
//-----------------------------------------------------------------------------
QString ErrorToString(Navico::Protocol::eErrors error) {
  switch (error) {
    case Navico::Protocol::EOK:
      return "EOK";
    case Navico::Protocol::ELocked:
      return "ELocked";
    case Navico::Protocol::EPending:
      return "EPending";
    case Navico::Protocol::ETimedOut:
      return "ETimedOut";
    case Navico::Protocol::EBusy:
      return "EBusy";
    case Navico::Protocol::EBadSerialNumber:
      return "EBadSerialNumber";
    case Navico::Protocol::ENoUnlockKey:
      return "ENoUnlockKey";
    case Navico::Protocol::EBadUnlockKey:
      return "EBadUnlockKey";
    case Navico::Protocol::EWrongUnlockKey:
      return "EWrongUnlockKey";
    case Navico::Protocol::ENotRunning:
      return "ENotRunning";
    case Navico::Protocol::EUnknownRadar:
      return "EUnknownRadar";
    case Navico::Protocol::ENonStdAddress:
      return "ENonStdAddress";
    case Navico::Protocol::ECommsFailure:
      return "ECommsFailure";
    case Navico::Protocol::EThreadCreation:
      return "EThreadCreation";
    case Navico::Protocol::EBadParameter:
      return "EBadParameter";
    case Navico::Protocol::EUnused:
      return "EUnused";
    case Navico::Protocol::EBadUnlockLevel:
      return "EBadUnlockLevel";
    default:
      return QString::number(error);
  }
}

//-----------------------------------------------------------------------------
// GUIDemo Class Implementation
//-----------------------------------------------------------------------------
GUIDemo::GUIDemo(QWidget* pParent, Qt::WindowFlags flags)
    : QMainWindow(pParent, flags),
      m_pImageClient(nullptr),
      m_pMode(nullptr),
      m_pSetup(nullptr),
      m_pSetupExtended(nullptr),
      m_pProperties(nullptr),
      m_pConfiguration(nullptr),
      m_pAdvancedSTCState(nullptr),
      m_pGuardZoneAlarms(nullptr),
      m_pRadarError(nullptr),
      m_PixelCellSize_mm(0),
      m_pTargetClient(nullptr),
      m_pTargetAlarmSetup(nullptr),
      m_pTargetProperties(nullptr),
      m_pTargets(nullptr),
      m_pTargetLocations(nullptr),
      rt_marineradar_data({
          0,      // spoke_azimuth_deg
          0,      // spoke_samplerange_m
          {0x00}  // spokedata
      }),
      rt_motion_data({
          0,  // vessel_x
          0,  // vessel_y
          0,  // vessel_theta
          0,  // vessel_u
          0,  // vessel_v
          0   // vessel_r
      }) {
  m_pImageClient = new Navico::Protocol::NRP::tImageClient();
  m_pTargetClient = new Navico::Protocol::NRP::tTargetTrackingClient();
  InitProtocolData();

  // setup UI
  ui.setupUi(this);
  m_pMultiRadar = new tMultiRadar(ui, this);
  m_pScannerInfo = new tScannerInfo(ui, this);
  m_pTabImage = new tTabImage(ui, this, *ui.tabImage);
  m_pTabInstallation = new tTabInstallation(ui, this, *ui.tabInstallation);
  m_pTabNewFunction = new tTabNewFunction(ui, this, *ui.tabNewFunction);
  m_pTabAdvanced = new tTabAdvanced(ui, this, *ui.tabAdvanced);
  m_pTabGuardZone =
      new tTabGuardZone(ui, this, *ui.tabGuardZone, m_OverlayManager);
  m_pTabTargets = new tTabTargets(ui, this, *ui.tabTargets);
  m_pTabFeatures = new tTabFeatures(ui, this, *ui.tabFeatures);
  m_pTabSectorBlanking =
      new tTabSectorBlanking(ui, this, *ui.tabSectorBlanking, m_OverlayManager);
  m_pTabBScan = new tTabBScan(ui, m_pTargetLocations, cMaxTargets, this,
                              m_OverlayManager);
  m_pTabPPI =
      new tTabPPI(ui, m_pTargetLocations, cMaxTargets, this, m_OverlayManager);

  MakeConnections(true);

  // create new thread

  std::thread database_thread(&GUIDemo::DataBaseLoop, this);
  std::thread socket_thread(&GUIDemo::DataTransmissionLoop, this);

  sched_param sch;
  int policy;
  pthread_getschedparam(socket_thread.native_handle(), &policy, &sch);
  sch.sched_priority = 90;
  if (pthread_setschedparam(socket_thread.native_handle(), SCHED_FIFO, &sch)) {
    std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
  }

  database_thread.detach();
  socket_thread.detach();
}

//-----------------------------------------------------------------------------
GUIDemo::~GUIDemo() {
  MakeConnections(false);

  DisconnectTargetClient();
  DisconnectImageClient();

  delete m_pTabPPI;
  delete m_pTabBScan;
  delete m_pTabSectorBlanking;
  delete m_pTabFeatures;
  delete m_pTabTargets;
  delete m_pTabGuardZone;
  delete m_pTabAdvanced;
  delete m_pTabInstallation;
  delete m_pTabNewFunction;
  delete m_pTabImage;
  delete m_pScannerInfo;
  delete m_pMultiRadar;

  // cleanup target-tracking
  delete m_pTargetClient;
  delete m_pTargetAlarmSetup;
  delete m_pTargetProperties;
  delete[] m_pTargets;
  delete[] m_pTargetLocations;

  // cleanup image handlling
  delete m_pImageClient;
  delete m_pMode;
  delete m_pSetup;
  delete m_pSetupExtended;
  delete m_pProperties;
  delete m_pConfiguration;
  delete m_pAdvancedSTCState;
  delete m_pRadarError;
  delete[] m_pGuardZoneAlarms;
}

//-----------------------------------------------------------------------------
void GUIDemo::InitProtocolData() {
  // setup image protocol data structures
  InitStruct(m_pMode);
  InitStruct(m_pSetup);
  InitStruct(m_pSetupExtended);
  InitStruct(m_pProperties);
  InitStruct(m_pConfiguration);
  InitStruct(m_pAdvancedSTCState);
  InitStruct(m_pRadarError);
  InitStruct(m_pGuardZoneAlarms, Navico::Protocol::NRP::cMaxGuardZones);

  // setup trget-tracking protocol data structures
  InitStruct(m_pTargetAlarmSetup);
  InitStruct(m_pTargetProperties);
  InitStruct(m_pTargets, 1 + cMaxTargets);
  InitStruct(m_pTargetLocations, cMaxTargets);
}

//-----------------------------------------------------------------------------
void GUIDemo::MakeConnections(bool connect) {
  // connections for user requested connect/disconnect
  Connect(connect, m_pMultiRadar, SIGNAL(ConnectChanged(bool)), this,
          SLOT(MultiRadar_ConnectChanged(bool)));

  // connections for updated state data
  Connect(connect, this, SIGNAL(UpdateMode_signal()), this,
          SLOT(UpdateMode_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateProperties_signal()), this,
          SLOT(UpdateProperties_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateConfiguration_signal()), this,
          SLOT(UpdateConfiguration_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateSetup_signal()), this,
          SLOT(UpdateSetup_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateSetupExtended_signal()), this,
          SLOT(UpdateSetupExtended_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateAdvancedState_signal()), this,
          SLOT(UpdateAdvancedState_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateGuardZoneAlarm_signal(unsigned)), this,
          SLOT(UpdateGuardZoneAlarm_slot(unsigned)), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateRadarError_signal()), this,
          SLOT(UpdateRadarError_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateFeature_signal(unsigned)), this,
          SLOT(UpdateFeature_slot(unsigned)), Qt::QueuedConnection);

  // connections for updated target data
  Connect(connect, this, SIGNAL(UpdateTargetAlarmSetup_signal()), this,
          SLOT(UpdateTargetAlarmSetup_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateTargetProperties_signal()), this,
          SLOT(UpdateTargetProperties_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateNavigationState_signal()), this,
          SLOT(UpdateNavigationState_slot()), Qt::QueuedConnection);
  Connect(connect, this, SIGNAL(UpdateTargetTarget_signal(unsigned)), this,
          SLOT(UpdateTargetTarget_slot(unsigned)), Qt::QueuedConnection);

  // connections for target acquire
  Connect(connect, m_pTabBScan, SIGNAL(AcquireTarget(double, double)), this,
          SLOT(TargetAcquire(double, double)));
  Connect(connect, m_pTabPPI, SIGNAL(AcquireTarget(double, double)), this,
          SLOT(TargetAcquire(double, double)));
}

//-----------------------------------------------------------------------------
void GUIDemo::MultiRadar_ConnectChanged(bool connect) {
  if (connect) {
    QString serialNumber = m_pMultiRadar->GetRadarSerialNumber();
    unsigned instance = m_pMultiRadar->GetRadarInstance();

    Navico::Protocol::eErrors imageError =
        (Navico::Protocol::eErrors)ConnectImageClient(serialNumber, instance);
    Navico::Protocol::eErrors targetError =
        (Navico::Protocol::eErrors)ConnectTargetClient(serialNumber, instance);
    if (imageError != Navico::Protocol::EOK ||
        targetError != Navico::Protocol::EOK) {
      if (imageError == Navico::Protocol::EOK) DisconnectImageClient();
      if (targetError == Navico::Protocol::EOK) DisconnectTargetClient();

      if (imageError == Navico::Protocol::ELocked ||
          targetError == Navico::Protocol::ELocked) {
        m_pMultiRadar->SetConnectState(false);

        if (QMessageBox::warning(
                this, "Unlock Radar?",
                "Radar is locked - would you like to unlock the radar?",
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::Yes) == QMessageBox::Yes) {
          m_pMultiRadar->InitiateUnlock();
        }
      } else {
        QString err;
        QString msg = "Initialisation of ";
        if (imageError && targetError) {
          msg += "Image and Target-Tracking Clients";
          err = "errors " + ErrorToString(imageError) + " and " +
                ErrorToString(targetError) + " resp";
        } else if (imageError) {
          msg += "Image Client";
          err = "error " + ErrorToString(imageError);
        } else if (targetError) {
          msg += "Target-Tracking Client";
          err = "error " + ErrorToString(targetError);
        }
        msg += " for Radar \"" + m_pMultiRadar->GetRadarSelection() +
               "\" failed with " + err;

        QMessageBox::critical(this, "Error", msg);

        m_pMultiRadar->SetConnectState(false);
      }
    } else {
      m_OverlayManager.Clear();

      // client services connected ok - initialise all dependent user interfaces
      m_pScannerInfo->OnConnect(m_pImageClient);
      m_pTabImage->OnConnect(m_pImageClient);
      m_pTabInstallation->OnConnect(m_pImageClient);
      m_pTabNewFunction->OnConnect(m_pImageClient);
      m_pTabAdvanced->OnConnect(m_pImageClient);
      m_pTabGuardZone->OnConnect(m_pImageClient);
      m_pTabTargets->OnConnect(m_pTargetClient);
      m_pTabFeatures->OnConnect(m_pImageClient);
      m_pTabSectorBlanking->OnConnect(m_pImageClient);
      m_pTabBScan->OnConnect();
      m_pTabPPI->OnConnect();

      // Send the feature request message
      m_pImageClient->QueryAll();
    }
  } else {
    // now disconnect the client services
    DisconnectTargetClient();
    DisconnectImageClient();

    // first disconnect all dependent user interfaces
    m_pScannerInfo->OnDisconnect();
    m_pTabImage->OnDisconnect();
    m_pTabInstallation->OnDisconnect();
    m_pTabNewFunction->OnDisconnect();
    m_pTabAdvanced->OnDisconnect();
    m_pTabGuardZone->OnDisconnect();
    m_pTabTargets->OnDisconnect();
    m_pTabFeatures->OnDisconnect();
    m_pTabSectorBlanking->OnDisconnect();
    m_pTabBScan->OnDisconnect();
    m_pTabPPI->OnDisconnect();

    // zero data for next time we connect
    InitProtocolData();
  }
}

//-----------------------------------------------------------------------------
//  Image Handling
//-----------------------------------------------------------------------------
int GUIDemo::ConnectImageClient(const QString& serialNumber,
                                unsigned instance) {
  int error = Navico::Protocol::EOK + 1;
  if (m_pImageClient) {
    m_pImageClient->AddStateObserver(this);
    m_pImageClient->AddSpokeObserver(this);
    m_pImageClient->AddFeatureObserver(this);
    // m_pImageClient->SetAutoSendClientWatchdog( true );

    m_ForceSetup = true;
    m_ForceAdvancedSTCState = true;

    error = m_pImageClient->Connect(serialNumber.toLatin1().data(), instance);
  }
  return error;
}

//-----------------------------------------------------------------------------
void GUIDemo::DisconnectImageClient() {
  if (m_pImageClient) {
    m_pImageClient->Disconnect();
    m_pImageClient->RemoveStateObserver(this);
    m_pImageClient->RemoveSpokeObserver(this);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateMode(const Navico::Protocol::NRP::tMode* pMode) {
  if (memcmp(pMode, m_pMode, sizeof(Navico::Protocol::NRP::tMode)) != 0) {
    *m_pMode = *pMode;
    emit UpdateMode_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetup(const Navico::Protocol::NRP::tSetup* pSetup) {
  if (m_ForceSetup ||
      memcmp(pSetup, m_pSetup, sizeof(Navico::Protocol::NRP::tSetup)) != 0) {
    m_ForceSetup = false;
    *m_pSetup = *pSetup;
    emit UpdateSetup_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetupExtended(
    const Navico::Protocol::NRP::tSetupExtended* pSetupExtended) {
  if (memcmp(pSetupExtended, m_pSetupExtended,
             sizeof(Navico::Protocol::NRP::tSetupExtended)) != 0) {
    *m_pSetupExtended = *pSetupExtended;
    emit UpdateSetupExtended_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateProperties(
    const Navico::Protocol::NRP::tProperties* pProperties) {
  if (memcmp(pProperties, m_pProperties,
             sizeof(Navico::Protocol::NRP::tProperties)) != 0) {
    *m_pProperties = *pProperties;
    emit UpdateProperties_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateConfiguration(
    const Navico::Protocol::NRP::tConfiguration* pConfiguration) {
  if (memcmp(pConfiguration, m_pConfiguration,
             sizeof(Navico::Protocol::NRP::tConfiguration)) != 0) {
    *m_pConfiguration = *pConfiguration;
    emit UpdateConfiguration_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSpoke(
    const Navico::Protocol::NRP::Spoke::t9174Spoke* pSpoke) {
  m_PixelCellSize_mm =
      Navico::Protocol::NRP::Spoke::GetPixelCellSize_mm(pSpoke->header);
  m_pTabBScan->OnUpdateSpoke(pSpoke, &rt_marineradar_data);
  m_pTabPPI->OnUpdateSpoke(pSpoke);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateAdvancedState(
    const Navico::Protocol::NRP::tAdvancedSTCState* pState) {
  if (m_ForceAdvancedSTCState ||
      memcmp(pState, m_pAdvancedSTCState,
             sizeof(Navico::Protocol::NRP::tAdvancedSTCState)) != 0) {
    m_ForceAdvancedSTCState = false;
    *m_pAdvancedSTCState = *pState;
    emit UpdateAdvancedState_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateFeature(
    const Navico::Protocol::NRP::tFeatureEnum* pFeature) {
  if (pFeature != nullptr) {
    emit UpdateFeature_signal(*pFeature);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateGuardZoneAlarm(
    const Navico::Protocol::NRP::tGuardZoneAlarm* pAlarm) {
  unsigned zone = pAlarm->zone;
  if (zone < Navico::Protocol::NRP::cMaxGuardZones) {
    m_pGuardZoneAlarms[zone] = *pAlarm;
    emit UpdateGuardZoneAlarm_signal(zone);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateRadarError(
    const Navico::Protocol::NRP::tRadarError* pError) {
  // NOTE: this simplification may overwrite a previous error that hasn't yet
  // been reported
  *m_pRadarError = *pError;
  emit UpdateRadarError_signal();
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateMode_slot() {
  m_pScannerInfo->OnModeChanged(m_pMode);
  m_pTabImage->OnModeChanged(m_pMode);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateProperties_slot() {
  m_pScannerInfo->OnPropertiesChanged(m_pProperties);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateConfiguration_slot() {
  m_pTabImage->OnConfigurationChanged(m_pConfiguration);
  m_pTabInstallation->OnConfigurationChanged(m_pConfiguration);
  m_pTabSectorBlanking->OnConfigurationChanged(m_pConfiguration);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetup_slot() {
  m_pScannerInfo->OnSetupChanged(m_pSetup);
  m_pTabImage->OnSetupChanged(m_pSetup);
  m_pTabGuardZone->OnSetupChanged(m_pSetup);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateSetupExtended_slot() {
  m_pTabImage->OnSetupExtendedChanged(m_pSetupExtended);
  m_pTabAdvanced->OnSetupExtendedChanged(m_pSetupExtended);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateAdvancedState_slot() {
  m_pTabAdvanced->OnAdvancedSTCStateChanged(m_pAdvancedSTCState);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateGuardZoneAlarm_slot(unsigned zone) {
  m_pTabGuardZone->OnGuardZoneAlarmChanged(zone, &m_pGuardZoneAlarms[zone]);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateRadarError_slot() {
  m_pScannerInfo->OnRadarErrorChanged(m_pRadarError);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateFeature_slot(unsigned feature) {
  m_pTabImage->OnUpdateFeature(
      static_cast<Navico::Protocol::NRP::tFeatureEnum>(feature));
  m_pTabFeatures->OnUpdateFeature(
      static_cast<Navico::Protocol::NRP::tFeatureEnum>(feature));
  m_pTabSectorBlanking->OnUpdateFeature(
      static_cast<Navico::Protocol::NRP::tFeatureEnum>(feature));
  m_pTabInstallation->OnUpdateFeature(
      static_cast<Navico::Protocol::NRP::tFeatureEnum>(feature));
}

//-----------------------------------------------------------------------------
//  Target Tracking Handling
//-----------------------------------------------------------------------------
int GUIDemo::ConnectTargetClient(const QString& serialNumber,
                                 unsigned instance) {
  int error = Navico::Protocol::EOK + 1;
  if (m_pTargetClient) {
    m_pTargetClient->AddStateObserver(this);
    m_pTargetClient->AddTargetTrackingObserver(this);

    m_ForceTargetAlarmSetup = true;
    m_ForceTargetProperties = true;

    error = m_pTargetClient->Connect(serialNumber.toLatin1().data(), instance);
  }
  return error;
}

//-----------------------------------------------------------------------------
void GUIDemo::DisconnectTargetClient() {
  if (m_pTargetClient) {
    m_pTargetClient->RemoveStateObserver(this);
    m_pTargetClient->RemoveTargetTrackingObserver(this);
    m_pTargetClient->Disconnect();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::TargetAcquire(double sample, double degrees) {
  if (m_PixelCellSize_mm > 0) {
    unsigned range = (sample * m_PixelCellSize_mm) / 1000 + 0.5;
    m_pTargetClient->Acquire(0, range, int32_t(degrees + 0.5),
                             Navico::Protocol::NRP::eRelative);
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateAlarmSetup(
    const Navico::Protocol::NRP::tTargetTrackingAlarmSetup* pAlarmSetup) {
  if (m_ForceTargetAlarmSetup ||
      memcmp(pAlarmSetup, m_pTargetAlarmSetup,
             sizeof(Navico::Protocol::NRP::tTargetTrackingAlarmSetup)) != 0) {
    m_ForceTargetAlarmSetup = false;
    *m_pTargetAlarmSetup = *pAlarmSetup;
    emit UpdateTargetAlarmSetup_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateProperties(
    const Navico::Protocol::NRP::tTargetTrackingProperties* pProperties) {
  if (m_ForceTargetProperties ||
      memcmp(pProperties, m_pTargetProperties,
             sizeof(Navico::Protocol::NRP::tTargetTrackingProperties)) != 0) {
    m_ForceTargetProperties = false;
    *m_pTargetProperties = *pProperties;
    emit UpdateTargetProperties_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateNavigation(
    const Navico::Protocol::NRP::tNavigationState* pNavigationState) {
  if (m_ForceNavigationState ||
      memcmp(pNavigationState, m_pNavigationState,
             sizeof(Navico::Protocol::NRP::tNavigationState)) != 0) {
    m_ForceNavigationState = false;
    *m_pNavigationState = *pNavigationState;
    emit UpdateNavigationState_signal();
  }
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateTarget(
    const Navico::Protocol::NRP::tTrackedTarget* pTarget) {
  unsigned index = std::max(0, pTarget->serverTargetID);

  if (index > cMaxTargets) index = 0;

  m_pTargets[index] = *pTarget;
  if (index > 0 && m_PixelCellSize_mm > 0) {
    tTargetLocation* pLoc = &m_pTargetLocations[index - 1];
    uint32_t state = pTarget->targetState;

    pLoc->isValid = false;
    if (pTarget->targetValid && state <= 3) {
      pLoc->sample =
          (pTarget->infoRelative.distance_m * 1000.0 / m_PixelCellSize_mm);
      pLoc->degrees = pTarget->infoRelative.bearing_ddeg / 10.0;
      pLoc->isValid = true;
    }
  }
  emit UpdateTargetTarget_signal(index);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateTargetAlarmSetup_slot() {
  m_pTabTargets->OnTTAlarmSetupChanged(m_pTargetAlarmSetup);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateTargetProperties_slot() {
  m_pTabTargets->OnTTPropertiesChanged(m_pTargetProperties);
}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateNavigationState_slot() {}

//-----------------------------------------------------------------------------
void GUIDemo::UpdateTargetTarget_slot(unsigned target) {
  m_pTabTargets->OnTrackedTargetChanged(target, &m_pTargets[target]);
}

//-------------------------------------------------------------------------
//  New Function
//-------------------------------------------------------------------------
void GUIDemo::DataBaseLoop() {
  QString strBuffer =
      QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
  QDir().mkdir("../" + strBuffer);
  std::string folderp = "../" + strBuffer.toStdString() + "/";

  marineradar_db _marineradar_db(folderp);
  _marineradar_db.create_table();

  while (1) {
    if (m_pTabNewFunction->IsDatabase()) {
      _marineradar_db.update_motion_table({
          0,  // local_time
          1,  // state_x
          2,  // state_y
          3,  // state_theta
          4,  // state_u
          5,  // state_v
          6   // statt_r
      });
      _marineradar_db.update_spoke_table({
          0,                                        // local_time
          rt_marineradar_data.spoke_azimuth_deg,    // azimuth_deg
          rt_marineradar_data.spoke_samplerange_m,  // sample_range
          std::vector<uint8_t>(
              &rt_marineradar_data.spokedata[0],
              &rt_marineradar_data
                   .spokedata[SAMPLES_PER_SPOKE / 2])  // spokedata
      });

    } else
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}  // DataBaseLoop

void GUIDemo::DataTransmissionLoop() {
  datatransimission _datatransimission("9340");
  static const size_t recv_size = 50;
  static const size_t send_size = 100;

  while (1) {
    unsigned char recv_buffer[recv_size] = {0x00};
    unsigned char send_buffer[send_size] = {0x00};

    static double t_previous_azimuth = 0;

    if (std::fabs(t_previous_azimuth - rt_marineradar_data.spoke_azimuth_deg) >
        0.1) {
      printf("azimuth: %f\n", rt_marineradar_data.spoke_azimuth_deg);

      std::memcpy(send_buffer, rt_marineradar_data.spokedata, 80);
      pack(send_buffer + 80, "d", rt_marineradar_data.spoke_azimuth_deg);
      _datatransimission.selectserver(recv_buffer, send_buffer, recv_size,
                                      send_size);
      t_previous_azimuth = rt_marineradar_data.spoke_azimuth_deg;
    }
  }
}  // DataTransmissionLoop

void GUIDemo::TargetTrackingLoop() {
  // Spoke Processing
  SpokeProcessdata SpokeProcess_data{
      0.1,  // sample_time
      0.0,  // radar_x
      0.0   // radar_y
  };

  AlarmZone Alarm_Zone{
      5,            // start_range_m
      50,           // end_range_m
      -0.5 * M_PI,  // center_bearing_rad
      M_PI,         // width_bearing_rad
      0x90          // sensitivity_threhold
  };

  TrackingTargetData TrackingTarget_Data{
      1,    // min_squared_radius
      36,   // max_squared_radius
      0.8,  // speed_threhold
      20,   // max_speed
      4,    // max_acceleration
      600,  // max_roti
      1,    // safe_distance
      0.5,  // K_radius
      1,    // K_delta_speed
      1     // K_delta_yaw;
  };

  ClusteringData Clustering_Data{
      4.4,  // p_radius
      2     // p_minumum_neighbors
  };

  const int max_num_targets = 20;

  TargetTracking<max_num_targets> Target_Tracking(
      Alarm_Zone, SpokeProcess_data, TrackingTarget_Data, Clustering_Data);
}
