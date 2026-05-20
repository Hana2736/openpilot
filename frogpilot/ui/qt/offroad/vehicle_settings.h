#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotVehiclesPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotVehiclesPanel(FrogPilotSettingsWindow *parent);

signals:
  void openSubPanel();

protected:
  void showEvent(QShowEvent *event) override;

private:
  void updateState(const UIState &s);
  void updateToggles();

  bool forceOpenDescriptions;
  bool started;

  QString autoTuneStatusShown;
  QString longCollectStatusShown;
  QString latDelayStatusShown;

  std::map<QString, AbstractControl*> toggles;

  QSet<QString> gmKeys = {"ExperimentalGMTune", "LongPitch", "VoltSNG"};
  QSet<QString> hkgKeys = {"NewLongAPI", "TacoTuneHacks"};
  QSet<QString> hondaKeys = {"HondaAltTune", "HondaLowSpeedPedal", "HondaMaxBrake"};
  QSet<QString> mazda3Keys = {"Mazda3TuneA", "Mazda3TuneB", "Mazda3TuneC", "Mazda3TuneD"};
  QSet<QString> mazdaCX30Keys = {"MazdaCX30TuneA", "MazdaCX30TuneB", "MazdaCX30TuneC", "MazdaCX30TuneD"};
  QSet<QString> mazdaCX50Keys = {"MazdaCX50TuneA", "MazdaCX50TuneB", "MazdaCX50TuneC", "MazdaCX50TuneD"};
  QSet<QString> mazdaKeys = {"Mazda3TuneA", "Mazda3TuneB", "Mazda3TuneC", "Mazda3TuneD",
                             "MazdaCX30TuneA", "MazdaCX30TuneB", "MazdaCX30TuneC", "MazdaCX30TuneD",
                             "MazdaCX50TuneA", "MazdaCX50TuneB", "MazdaCX50TuneC", "MazdaCX50TuneD",
                             "MazdaTuneReset", "MazdaAutoTune", "MazdaAutoTuneApply",
                             "MazdaAutoTuneDeadzone", "LongAutoTuneCollect",
                             "LatDelayCollect", "LatDelayFit", "LatDelayApply", "LatDelayReset"};
  QSet<QString> longitudinalKeys = {"ExperimentalGMTune", "FrogsGoMoosTweak", "HondaAltTune", "HondaMaxBrake", "HondaLowSpeedPedal", "LongPitch", "NewLongAPI", "SNGHack", "SubaruSNG", "VoltSNG"};
  QSet<QString> subaruKeys = {"SubaruSNG"};
  QSet<QString> toyotaKeys = {"ClusterOffset", "FrogsGoMoosTweak", "LockDoorsTimer", "SNGHack", "ToyotaDoors"};
  QSet<QString> vehicleInfoKeys = {"BlindSpotSupport", "HardwareDetected", "OpenpilotLongitudinal", "PedalSupport", "RadarSupport", "SDSUSupport", "SNGSupport"};

  QSet<QString> parentKeys;

  FrogPilotSettingsWindow *parent;

  ParamControl *disableOpenpilotLong;
  ParamControl *forceFingerprint;

  Params params;
  Params params_default{"/dev/shm/params_default"};
  Params params_memory{"/dev/shm/params"};

  bool autoTuneRebootPrompted = false;

  QJsonObject frogpilotToggleLevels;

  QMap<QString, QString> carModels;
};
