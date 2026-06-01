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
  QString longDelayStatusShown;
  QString longStaticStatusShown;
  QString latOpenLoopStatusShown;
  QString tuneStackStatusShown;

  std::map<QString, AbstractControl*> toggles;

  QSet<QString> gmKeys = {"ExperimentalGMTune", "LongPitch", "VoltSNG"};
  QSet<QString> hkgKeys = {"NewLongAPI", "TacoTuneHacks"};
  QSet<QString> hondaKeys = {"HondaAltTune", "HondaLowSpeedPedal", "HondaMaxBrake"};
  QSet<QString> mazda3Keys = {"Mazda3TuneA", "Mazda3TuneB", "Mazda3TuneC", "Mazda3TuneD"};
  QSet<QString> mazdaCX30Keys = {"MazdaCX30TuneA", "MazdaCX30TuneB", "MazdaCX30TuneC", "MazdaCX30TuneD"};
  QSet<QString> mazdaCX50Keys = {"MazdaCX50TuneA", "MazdaCX50TuneB", "MazdaCX50TuneC", "MazdaCX50TuneD"};
  // Sub-panel routing: top-level Mazda panel shows the three nav headers
  // below; the model-specific siglin sliders/autotune live on the ABCD
  // sub-panel; delay collect/fit/apply on the delay sub-panel; long
  // collector on the long sub-panel. mazdaKeys is the union (used for
  // top-level visibility gating).
  // NOTE: MazdaAutoTune/Apply/Deadzone and LatOpenLoop* are HIDDEN (removed from
  // the vehicleToggles vector in the .cc). Their handlers + backend are kept; see
  // docs/MAZDA_AUTOTUNE_NOTES.md. Restore them to these sets + the vector to re-show.
  QSet<QString> mazdaLatAbcdKeys = {"Mazda3TuneA", "Mazda3TuneB", "Mazda3TuneC", "Mazda3TuneD",
                                    "MazdaCX30TuneA", "MazdaCX30TuneB", "MazdaCX30TuneC", "MazdaCX30TuneD",
                                    "MazdaCX50TuneA", "MazdaCX50TuneB", "MazdaCX50TuneC", "MazdaCX50TuneD",
                                    "MazdaTuneReset"};
  QSet<QString> mazdaLatDelayKeys = {"LatDelayCollect", "LatDelayFit", "LatDelayApply", "LatDelayReset"};
  QSet<QString> mazdaLongKeys = {"LongAutoTuneCollect", "LongDelayFit", "LongDelayApply", "LongDelayReset",
                                 "LongStaticFit", "LongStaticApply", "LongStaticReset"};
  QSet<QString> mazdaKeys = {"Mazda3TuneA", "Mazda3TuneB", "Mazda3TuneC", "Mazda3TuneD",
                             "MazdaCX30TuneA", "MazdaCX30TuneB", "MazdaCX30TuneC", "MazdaCX30TuneD",
                             "MazdaCX50TuneA", "MazdaCX50TuneB", "MazdaCX50TuneC", "MazdaCX50TuneD",
                             "MazdaTuneReset",
                             "LatDelayCollect", "LatDelayFit", "LatDelayApply", "LatDelayReset",
                             "LongAutoTuneCollect", "LongDelayFit", "LongDelayApply", "LongDelayReset",
                             "LongStaticFit", "LongStaticApply", "LongStaticReset",
                             "MazdaCollectAll", "MazdaFitAll",
                             "MazdaLatAbcdToggles", "MazdaLatDelayToggles", "MazdaLongToggles"};
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
