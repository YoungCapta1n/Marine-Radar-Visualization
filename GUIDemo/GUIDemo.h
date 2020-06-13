//-----------------------------------------------------------------------------
// Copyright (C) 2007-2011 Navico
// Confidential and proprietary.  All rights reserved.
//-----------------------------------------------------------------------------
//! \file GUIDemo.h
//!
//! Window for testing/demonstrating the NRP library protocol
//-----------------------------------------------------------------------------
#ifndef GUIDEMO_H
#define GUIDEMO_H

#include <QMainWindow>

#include <ClientErrors.h>
#include <Feature.h>
#include <FeatureManager.h>
#include <ImageClient.h>
#include <ImageClientObserver.h>
#include <MultiRadarClient.h>
#include <NavRadarProtocol.h>
#include <PPIController.h>
#include <TargetTrackingClient.h>

#include "MultiRadar.h"
#include "ScannerInfo.h"
#include "TabAdvanced.h"
#include "TabBScan.h"
#include "TabFeatures.h"
#include "TabGuardZone.h"
#include "TabImage.h"
#include "TabInstallation.h"
#include "TabNewFunction.h"
#include "TabPPI.h"
#include "TabSectorBlanking.h"
#include "TabTargets.h"

#include "OverlayManager.h"

#include "TargetTracking/TargetTracking.h"
#include "datarecorder.h"
#include "datatransimission.h"

#include "ui_GUIDemo.h"

//-----------------------------------------------------------------------------
// GUIDemo Class
//-----------------------------------------------------------------------------

class GUIDemo
    : public QMainWindow,
      public Navico::Protocol::NRP::iImageClientStateObserver,
      public Navico::Protocol::NRP::iImageClientSpokeObserver,
      public Navico::Protocol::NRP::iFeatureObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientObserver,
      public Navico::Protocol::NRP::iTargetTrackingClientStateObserver {
  Q_OBJECT

 public:
  GUIDemo(QWidget* parent = nullptr, Qt::WindowFlags flags = nullptr);
  ~GUIDemo();

 private:
  void InitProtocolData();
  void MakeConnections(bool connect);

  //-------------------------------------------------------------------------
  //  Observer Callbacks
  //-------------------------------------------------------------------------
 private:
  // iImageClientSpokeObserver callbacks
  virtual void UpdateSpoke(
      const Navico::Protocol::NRP::Spoke::t9174Spoke* pSpoke);

  // iImageClientStateObserver callbacks
  virtual void UpdateMode(const Navico::Protocol::NRP::tMode* pMode);
  virtual void UpdateSetup(const Navico::Protocol::NRP::tSetup* pSetup);
  virtual void UpdateSetupExtended(
      const Navico::Protocol::NRP::tSetupExtended* pSetupExtended);
  virtual void UpdateProperties(
      const Navico::Protocol::NRP::tProperties* pProperties);
  virtual void UpdateConfiguration(
      const Navico::Protocol::NRP::tConfiguration* pConfiguration);
  virtual void UpdateGuardZoneAlarm(
      const Navico::Protocol::NRP::tGuardZoneAlarm* pAlarm);
  virtual void UpdateRadarError(
      const Navico::Protocol::NRP::tRadarError* pAlarm);
  virtual void UpdateAdvancedState(
      const Navico::Protocol::NRP::tAdvancedSTCState* pState);

  // iFeatureObserver callbacks
  virtual void UpdateFeature(
      const Navico::Protocol::NRP::tFeatureEnum* pFeature);

  // iTargetTrackingClientObserver callbacks
  virtual void UpdateTarget(
      const Navico::Protocol::NRP::tTrackedTarget* pTarget);

  // iTargetTrackingClientStateObserver callbacks
  virtual void UpdateNavigation(
      const Navico::Protocol::NRP::tNavigationState* pNavigationState);
  virtual void UpdateAlarmSetup(
      const Navico::Protocol::NRP::tTargetTrackingAlarmSetup* pAlarmSetup);
  virtual void UpdateProperties(
      const Navico::Protocol::NRP::tTargetTrackingProperties* pProperties);

  //-------------------------------------------------------------------------
  //  Image Client
  //-------------------------------------------------------------------------
 private:
  int ConnectImageClient(const QString& serialNumber, unsigned instance);
  void DisconnectImageClient();

 signals:
  // signals from callbacks
  void UpdateMode_signal();
  void UpdateProperties_signal();
  void UpdateConfiguration_signal();
  void UpdateSetup_signal();
  void UpdateSetupExtended_signal();
  void UpdateAdvancedState_signal();
  void UpdateGuardZoneAlarm_signal(unsigned zone);
  void UpdateRadarError_signal();
  void UpdateFeature_signal(unsigned feature);

 private slots:
  // UI graph slots refresh
  void UpdateMode_slot();
  void UpdateProperties_slot();
  void UpdateConfiguration_slot();
  void UpdateSetup_slot();
  void UpdateSetupExtended_slot();
  void UpdateAdvancedState_slot();
  void UpdateGuardZoneAlarm_slot(unsigned zone);
  void UpdateRadarError_slot();
  void UpdateFeature_slot(unsigned feature);

 private:
  // image client protocol manager
  Navico::Protocol::NRP::tImageClient* m_pImageClient;

  // cached data from the radar
  Navico::Protocol::NRP::tMode* m_pMode;
  Navico::Protocol::NRP::tSetup* m_pSetup;
  Navico::Protocol::NRP::tSetupExtended* m_pSetupExtended;
  Navico::Protocol::NRP::tProperties* m_pProperties;
  Navico::Protocol::NRP::tConfiguration* m_pConfiguration;
  Navico::Protocol::NRP::tAdvancedSTCState* m_pAdvancedSTCState;
  Navico::Protocol::NRP::tGuardZoneAlarm* m_pGuardZoneAlarms;
  Navico::Protocol::NRP::tRadarError* m_pRadarError;
  unsigned m_PixelCellSize_mm;

  // flags to force cached data notification updates
  bool m_ForceSetup;
  bool m_ForceAdvancedSTCState;

  //-------------------------------------------------------------------------
  //  Target Tracking Client
  //-------------------------------------------------------------------------
 public:
  static const unsigned cMaxTargets = 10;

 private:
  int ConnectTargetClient(const QString& serialNumber, unsigned instance);
  void DisconnectTargetClient();

 signals:
  // signals from callbacks
  void UpdateTargetAlarmSetup_signal();
  void UpdateTargetProperties_signal();
  void UpdateNavigationState_signal();
  void UpdateTargetTarget_signal(unsigned targetID);

 private slots:
  // slots for callbacks
  void TargetAcquire(double sample, double degrees);
  void UpdateTargetAlarmSetup_slot();
  void UpdateTargetProperties_slot();
  void UpdateNavigationState_slot();
  void UpdateTargetTarget_slot(unsigned target);

 private:
  // working variables
  Navico::Protocol::NRP::tNavigationState* m_pNavigationState;
  Navico::Protocol::NRP::tTargetTrackingClient* m_pTargetClient;
  Navico::Protocol::NRP::tTargetTrackingAlarmSetup* m_pTargetAlarmSetup;
  Navico::Protocol::NRP::tTargetTrackingProperties* m_pTargetProperties;
  Navico::Protocol::NRP::tTrackedTarget* m_pTargets;
  tTargetLocation* m_pTargetLocations;

  // flags to force cached data notification updates
  bool m_ForceTargetAlarmSetup;
  bool m_ForceTargetProperties;
  bool m_ForceNavigationState;

  //-------------------------------------------------------------------------
  //  User Interface
  //-------------------------------------------------------------------------
 private slots:
  void MultiRadar_ConnectChanged(bool connect);

 private:
  tMultiRadar* m_pMultiRadar;
  tScannerInfo* m_pScannerInfo;
  tTabImage* m_pTabImage;
  tTabInstallation* m_pTabInstallation;
  tTabNewFunction* m_pTabNewFunction;
  tTabAdvanced* m_pTabAdvanced;
  tTabGuardZone* m_pTabGuardZone;
  tTabTargets* m_pTabTargets;
  tTabBScan* m_pTabBScan;
  tTabPPI* m_pTabPPI;
  tTabFeatures* m_pTabFeatures;
  tTabSectorBlanking* m_pTabSectorBlanking;

  Ui::GUIDemoClass ui;

  tOverlayManager m_OverlayManager;

  //-------------------------------------------------------------------------
  //  New Function
  //-------------------------------------------------------------------------

 private:
  void DataBaseLoop();
  void DataTransmissionLoop();
  void TargetTrackingLoop();

  RTSpokeData rt_marineradar_data;
  RTVesselData rt_motion_data;
};

//-----------------------------------------------------------------------------

#endif  // inclusion guard
