#include <QRegularExpression>
#include <QTextStream>

#include "frogpilot/ui/qt/offroad/vehicle_settings.h"

QStringList getCarNames(const QString &carMake, QMap<QString, QString> &carModels) {
  static QMap<QString, QString> makeMap = {
    {"acura", "honda"},
    {"audi", "volkswagen"},
    {"buick", "gm"},
    {"cadillac", "gm"},
    {"chevrolet", "gm"},
    {"chrysler", "chrysler"},
    {"cupra", "volkswagen"},
    {"dodge", "chrysler"},
    {"ford", "ford"},
    {"genesis", "hyundai"},
    {"gmc", "gm"},
    {"holden", "gm"},
    {"honda", "honda"},
    {"hyundai", "hyundai"},
    {"jeep", "chrysler"},
    {"kia", "hyundai"},
    {"lexus", "toyota"},
    {"lincoln", "ford"},
    {"man", "volkswagen"},
    {"mazda", "mazda"},
    {"nissan", "nissan"},
    {"ram", "chrysler"},
    {"seat", "volkswagen"},
    {"škoda", "volkswagen"},
    {"subaru", "subaru"},
    {"tesla", "tesla"},
    {"toyota", "toyota"},
    {"volkswagen", "volkswagen"}
  };

  QStringList carNameList;

  QFile valuesFile(QString("../car/%1/values.py").arg(makeMap.value(carMake, carMake)));
  if (!valuesFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return carNameList;
  }

  QString fileContent = QTextStream(&valuesFile).readAll();
  valuesFile.close();

  fileContent.remove(QRegularExpression("#[^\n]*"));
  fileContent.remove(QRegularExpression("footnotes=\\[[^\\]]*\\],\\s*"));

  static QRegularExpression carNameRegex("CarDocs\\(\\s*\"([^\"]+)\"[^)]*\\)");
  static QRegularExpression platformRegex("((\\w+)\\s*=\\s*\\w+\\s*\\(\\s*\\[([\\s\\S]*?)\\]\\s*,)");
  static QRegularExpression validNameRegex("^[A-Za-z0-9 \u0160.()-]+$");

  QRegularExpressionMatchIterator platformMatches = platformRegex.globalMatch(fileContent);
  while (platformMatches.hasNext()) {
    QRegularExpressionMatch platformMatch = platformMatches.next();
    QString platformName = platformMatch.captured(2);
    QString platformSection = platformMatch.captured(3);

    QRegularExpressionMatchIterator carNameMatches = carNameRegex.globalMatch(platformSection);
    while (carNameMatches.hasNext()) {
      QString carName = carNameMatches.next().captured(1);

      if (carName.contains(validNameRegex) && carName.count(" ") >= 1) {
        QString firstWord = carName.section(" ", 0, 0);

        if (firstWord.compare(carMake, Qt::CaseInsensitive) == 0) {
          carModels[carName] = platformName;
          carNameList.append(carName);
        }
      }
    }
  }

  carNameList.sort();
  return carNameList;
}

FrogPilotVehiclesPanel::FrogPilotVehiclesPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  QJsonObject shownDescriptions = QJsonDocument::fromJson(QString::fromStdString(params.get("ShownToggleDescriptions")).toUtf8()).object();
  QString className = this->metaObject()->className();

  if (!shownDescriptions.value(className).toBool(false)) {
    forceOpenDescriptions = true;
    shownDescriptions.insert(className, true);
    params.put("ShownToggleDescriptions", QJsonDocument(shownDescriptions).toJson(QJsonDocument::Compact).toStdString());
  }

  QStackedLayout *vehiclesLayout = new QStackedLayout();
  addItem(vehiclesLayout);

  FrogPilotListWidget *settingsList = new FrogPilotListWidget(this);

  ScrollView *vehiclesPanel = new ScrollView(settingsList, this);

  vehiclesLayout->addWidget(vehiclesPanel);

  QStringList makes = {
    "Acura", "Audi", "Buick", "Cadillac", "Chevrolet", "Chrysler",
    "CUPRA", "Dodge", "Ford", "Genesis", "GMC", "Holden", "Honda",
    "Hyundai", "Jeep", "Kia", "Lexus", "Lincoln", "MAN", "Mazda",
    "Nissan", "Ram", "SEAT", "Škoda", "Subaru", "Tesla", "Toyota",
    "Volkswagen"
  };

  ButtonControl *selectMakeButton = new ButtonControl(tr("Car Make"), tr("SELECT"));
  QObject::connect(selectMakeButton, &ButtonControl::clicked, [makes, selectMakeButton, this]() {
    QString makeSelection = MultiOptionDialog::getSelection(tr("Choose your car make"), makes, "", this);
    if (!makeSelection.isEmpty()) {
      params.put("CarMake", makeSelection.toStdString());
      selectMakeButton->setValue(makeSelection);
    }
  });
  settingsList->addItem(selectMakeButton);

  ButtonControl *selectModelButton = new ButtonControl(tr("Car Model"), tr("SELECT"));
  QObject::connect(selectModelButton, &ButtonControl::clicked, [selectModelButton, this]() {
    QString modelSelection = MultiOptionDialog::getSelection(tr("Choose your car model"), getCarNames(QString::fromStdString(params.get("CarMake")).toLower(), carModels), "", this);
    if (!modelSelection.isEmpty()) {
      params.put("CarModel", carModels.value(modelSelection).toStdString());
      params.put("CarModelName", modelSelection.toStdString());
      selectModelButton->setValue(modelSelection);
    }
  });
  settingsList->addItem(selectModelButton);

  forceFingerprint = new ParamControl("ForceFingerprint", tr("Disable Automatic Fingerprint Detection"), tr("<b>Force the selected fingerprint</b> and prevent it from ever changing."), "");
  settingsList->addItem(forceFingerprint);

  disableOpenpilotLong = new ParamControl("DisableOpenpilotLongitudinal", tr("Disable openpilot Longitudinal Control"), tr("<b>Disable openpilot longitudinal</b> and use the car's stock ACC instead."), "");
  QObject::connect(disableOpenpilotLong, &ToggleControl::toggleFlipped, [parent, this](bool state) {
    if (state) {
      if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to completely disable openpilot longitudinal control?"), this)) {
        if (started) {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        }
      } else {
        params.putBool("DisableOpenpilotLongitudinal", false);
        disableOpenpilotLong->refresh();
      }
    }

    parent->updateVariables();
    updateToggles();
  });
  settingsList->addItem(disableOpenpilotLong);

  FrogPilotListWidget *gmList = new FrogPilotListWidget(this);
  FrogPilotListWidget *hkgList = new FrogPilotListWidget(this);
  FrogPilotListWidget *hondaList = new FrogPilotListWidget(this);
  FrogPilotListWidget *mazdaList = new FrogPilotListWidget(this);
  FrogPilotListWidget *mazdaLatAbcdList = new FrogPilotListWidget(this);
  FrogPilotListWidget *mazdaLatDelayList = new FrogPilotListWidget(this);
  FrogPilotListWidget *mazdaLongList = new FrogPilotListWidget(this);
  FrogPilotListWidget *subaruList = new FrogPilotListWidget(this);
  FrogPilotListWidget *toyotaList = new FrogPilotListWidget(this);
  FrogPilotListWidget *vehicleInfoList = new FrogPilotListWidget(this);

  ScrollView *gmPanel = new ScrollView(gmList, this);
  ScrollView *hkgPanel = new ScrollView(hkgList, this);
  ScrollView *hondaPanel = new ScrollView(hondaList, this);
  ScrollView *mazdaPanel = new ScrollView(mazdaList, this);
  ScrollView *mazdaLatAbcdPanel = new ScrollView(mazdaLatAbcdList, this);
  ScrollView *mazdaLatDelayPanel = new ScrollView(mazdaLatDelayList, this);
  ScrollView *mazdaLongPanel = new ScrollView(mazdaLongList, this);
  ScrollView *subaruPanel = new ScrollView(subaruList, this);
  ScrollView *toyotaPanel = new ScrollView(toyotaList, this);
  ScrollView *vehicleInfoPanel = new ScrollView(vehicleInfoList, this);

  vehiclesLayout->addWidget(gmPanel);
  vehiclesLayout->addWidget(hkgPanel);
  vehiclesLayout->addWidget(hondaPanel);
  vehiclesLayout->addWidget(mazdaPanel);
  vehiclesLayout->addWidget(mazdaLatAbcdPanel);
  vehiclesLayout->addWidget(mazdaLatDelayPanel);
  vehiclesLayout->addWidget(mazdaLongPanel);
  vehiclesLayout->addWidget(subaruPanel);
  vehiclesLayout->addWidget(toyotaPanel);
  vehiclesLayout->addWidget(vehicleInfoPanel);

  std::vector<std::tuple<QString, QString, QString, QString>> vehicleToggles {
    {"GMToggles", tr("General Motors Settings"), tr("<b>FrogPilot features for General Motors vehicles.</b>"), ""},
    {"ExperimentalGMTune", tr("FrogsGoMoo's Experimental Tune"), tr("<b>Experimental GM tune by FrogsGoMoo</b> that attempts to smoothen stopping and takeoff control. Use at your own risk!"), ""},
    {"LongPitch", tr("Smooth Pedal Response on Hills"), tr("<b>Smoothen acceleration and braking</b> when driving downhill/uphill."), ""},
    {"VoltSNG", tr("Stop-and-Go Hack"), tr("<b>Force stop-and-go</b> on the 2017 Chevy Volt."), ""},

    {"HKGToggles", tr("Hyundai/Kia/Genesis Settings"), tr("<b>FrogPilot features for Genesis, Hyundai, and Kia vehicles.</b>"), ""},
    {"NewLongAPI", tr("comma's New Longitudinal API"), tr("<b>comma's new gas and brake control system</b> that improves acceleration and braking but may cause issues on some Genesis/Hyundai/Kia vehicles."), ""},
    {"TacoTuneHacks", tr("\"Taco Bell Run\" Torque Hack"), tr("<b>The steering torque hack from comma's 2022 \"Taco Bell Run\".</b> Designed to increase steering torque at low speeds for left and right turns."), ""},

    {"HondaToggles", tr("Acura/Honda Settings"), tr("<b>FrogPilot features for Acura and Honda vehicles.</b>"), ""},
    {"HondaAltTune", tr("Gentle Following"), tr("<b>Reduces jerky acceleration and braking when following a lead vehicle.</b> Ideal for stop-and-go traffic."), ""},
    {"HondaMaxBrake", tr("Increased Braking Force"), tr("<b>Increases the maximum braking force for improved stopping performance.</b>"), ""},
    {"HondaLowSpeedPedal", tr("Responsive Pedal at Low Speeds"), tr("<b>Improves acceleration from a standstill for a more responsive throttle feel in city driving.</b>"), ""},

    {"MazdaToggles", tr("Mazda Settings"), tr("Mazda lateral and longitudinal tuning."), ""},
    {"MazdaCollectAll", tr("Collect All (whole stack)"), tr("Sequentially ingest every Mazda rolling store: long (static+delay), lat delay, open-loop lat + K calibration. Parked only. Each store's per-feature button shows live progress."), ""},
    {"MazdaFitAll", tr("Fit All (whole stack)"), tr("Sequentially run every Mazda fitter: long delay, long static map, lat delay, open-loop lat. Apply remains per-feature so you can sanity-check each preview before committing."), ""},
    {"MazdaLatAbcdToggles", tr("Lateral ABCD Tuning"), tr("Sigmoid+linear tune (a/b/c/d) and per-model sliders."), ""},
    {"MazdaLatDelayToggles", tr("Lateral Delay Tuning"), tr("Per-speed steerActuatorDelay breakpoint table."), ""},
    {"MazdaLongToggles", tr("Longitudinal Tuning"), tr("Gen2 long static-map sample collector."), ""},
    {"Mazda3TuneA", tr("Mazda 3 — a"), tr("Sigmoid slope."), ""},
    {"Mazda3TuneB", tr("Mazda 3 — b"), tr("Sigmoid gain."), ""},
    {"Mazda3TuneC", tr("Mazda 3 — c"), tr("Linear gain (≥0)."), ""},
    {"Mazda3TuneD", tr("Mazda 3 — d"), tr("Torque offset; ± OK, 0 = none."), ""},
    {"MazdaCX30TuneA", tr("CX-30 — a"), tr("Sigmoid slope."), ""},
    {"MazdaCX30TuneB", tr("CX-30 — b"), tr("Sigmoid gain."), ""},
    {"MazdaCX30TuneC", tr("CX-30 — c"), tr("Linear gain (≥0)."), ""},
    {"MazdaCX30TuneD", tr("CX-30 — d"), tr("Torque offset; ± OK, 0 = none."), ""},
    {"MazdaCX50TuneA", tr("CX-50 — a"), tr("Sigmoid slope."), ""},
    {"MazdaCX50TuneB", tr("CX-50 — b"), tr("Sigmoid gain."), ""},
    {"MazdaCX50TuneC", tr("CX-50 — c"), tr("Linear gain (≥0)."), ""},
    {"MazdaCX50TuneD", tr("CX-50 — d"), tr("Torque offset; ± OK, 0 = none."), ""},
    {"MazdaAutoTune", tr("Auto-Tune Now"), tr("Fit the tune from drive logs. Parked only; previews first."), ""},
    {"MazdaAutoTuneApply", tr("Apply Auto-Tune"), tr("Write the previewed tune, then reboot."), ""},
    {"MazdaAutoTuneDeadzone", tr("Auto-Tune Deadzone"), tr("Ignore |lat accel| below this when fitting."), ""},
    {"MazdaTuneReset", tr("Reset Mazda Tune"), tr("Restore default coefficients."), ""},
    {"LongAutoTuneCollect", tr("Collect Long Samples"), tr("Ingest any new rlogs into the long-tune rolling store. Parked only."), ""},
    {"LatDelayCollect", tr("Collect Lat Delay Samples"), tr("Ingest any new rlogs into the lateral-delay rolling store. Parked only."), ""},
    {"LatDelayFit", tr("Fit Lat Delay Table"), tr("Build a per-speed delay breakpoint table from the store. Previews first."), ""},
    {"LatDelayApply", tr("Apply Lat Delay Table"), tr("Write the previewed delay table; lagd will interpolate it at runtime. Reboot after."), ""},
    {"LatDelayReset", tr("Reset Lat Delay Table"), tr("Clear the applied delay table; fall back to lagd's learned single value."), ""},
    {"LongDelayFit", tr("Fit Long Delay Table"), tr("Build a per-speed longitudinalActuatorDelay breakpoint table from the delay store. Previews first."), ""},
    {"LongDelayApply", tr("Apply Long Delay Table"), tr("Write the previewed long-delay table; the long planner/controller will interpolate it at runtime. Reboot after."), ""},
    {"LongDelayReset", tr("Reset Long Delay Table"), tr("Clear the applied long-delay table; fall back to the single scalar longitudinalActuatorDelay."), ""},
    {"LongStaticFit", tr("Fit Long Static Map"), tr("Build a per-speed piecewise-affine-with-deadband table from the long static store. Replaces the global accel_scale/accel_offset affine. Previews first."), ""},
    {"LongStaticApply", tr("Apply Long Static Map"), tr("Write the previewed static map; carcontroller will invert the plant per (target_accel, v_ego) at runtime. Reboot after."), ""},
    {"LongStaticReset", tr("Reset Long Static Map"), tr("Clear the applied static map; fall back to the global accel_scale/accel_offset affine."), ""},
    {"LatOpenLoopCollect", tr("Collect Open-Loop Lat Samples"), tr("Ingest rlogs into the open-loop lateral store (lat-off driver-steered windows). Parked only."), ""},
    {"LatOpenLoopFit", tr("Fit Open-Loop Lat"), tr("Fit siglin on the open-loop store, auto-compute K from the calibration store, and convert to OP-normalized a/b/c/d. Previews first."), ""},
    {"LatOpenLoopApply", tr("Apply Open-Loop Lat"), tr("Write the converted open-loop fit to Mazda{Model}TuneA-D. Requires K calibration (auto-accumulated from any lat-ON driving) and the ±1 coverage gate."), ""},
    {"LatOpenLoopReset", tr("Reset Open-Loop Lat"), tr("Clear the previewed open-loop fit."), ""},

    {"SubaruToggles", tr("Subaru Settings"), tr("<b>FrogPilot features for Subaru vehicles.</b>"), ""},
    {"SubaruSNG", tr("Stop and Go"), tr("Stop and go for supported Subaru vehicles."), ""},

    {"ToyotaToggles", tr("Toyota/Lexus Settings"), tr("<b>FrogPilot features for Lexus and Toyota vehicles.</b>"), ""},
    {"ToyotaDoors", tr("Automatically Lock/Unlock Doors"), tr("<b>Automatically lock/unlock doors</b> when shifting in and out of drive."), ""},
    {"ClusterOffset", tr("Dashboard Speed Offset"), tr("<b>The speed offset openpilot uses to match the speed on the dashboard display.</b>"), ""},
    {"FrogsGoMoosTweak", tr("FrogsGoMoo's Personal Tweaks"), tr("<b>Personal tweaks by FrogsGoMoo for quicker acceleration and smoother braking.</b>"), ""},
    {"LockDoorsTimer", tr("Lock Doors On Ignition Off After"), tr("<b>Automatically lock the doors on ignition off</b> when no one is detected in the front seats."), ""},
    {"SNGHack", tr("Stop-and-Go Hack"), tr("<b>Force stop-and-go</b> on Lexus/Toyota vehicles without stock stop-and-go functionality."), ""},

    {"VehicleInfo", tr("Vehicle Info"), tr("<b>Information about your vehicle in regards to openpilot support and functionality.</b>"), ""},
    {"HardwareDetected", tr("3rd Party Hardware Detected"), tr("<b>Detected 3rd party hardware.</b>"), ""},
    {"BlindSpotSupport", tr("Blind Spot Support"), tr("<b>Does openpilot use the vehicle's blind spot data?</b>"), ""},
    {"PedalSupport", tr("comma Pedal Support"), tr("<b>Does your vehicle support the \"comma pedal\"?</b>"), ""},
    {"OpenpilotLongitudinal", tr("openpilot Longitudinal Support"), tr("<b>Can openpilot control the vehicle's acceleration and braking?</b>"), ""},
    {"RadarSupport", tr("Radar Support"), tr("<b>Does openpilot use the vehicle's radar data</b> alongside the device's camera for tracking lead vehicles?"), ""},
    {"SDSUSupport", tr("SDSU Support"), tr("<b>Does your vehicle support \"SDSUs\"?</b>"), ""},
    {"SNGSupport", tr("Stop-and-Go Support"), tr("<b>Does your vehicle support stop-and-go driving?</b>"), ""}
  };

  for (const auto &[param, title, desc, icon] : vehicleToggles) {
    AbstractControl *vehicleToggle;

    if (param == "GMToggles") {
      ButtonControl *gmButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(gmButton, &ButtonControl::clicked, [vehiclesLayout, gmPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(gmPanel);
      });
      vehicleToggle = gmButton;

    } else if (param == "HKGToggles") {
      ButtonControl *hkgButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(hkgButton, &ButtonControl::clicked, [vehiclesLayout, hkgPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(hkgPanel);
      });
      vehicleToggle = hkgButton;

    } else if (param == "HondaToggles") {
      ButtonControl *hondaButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(hondaButton, &ButtonControl::clicked, [vehiclesLayout, hondaPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(hondaPanel);
      });
      vehicleToggle = hondaButton;

    } else if (param == "SubaruToggles") {
      ButtonControl *subaruButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(subaruButton, &ButtonControl::clicked, [vehiclesLayout, subaruPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(subaruPanel);
      });
      vehicleToggle = subaruButton;

    } else if (param == "MazdaToggles") {
      ButtonControl *mazdaButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(mazdaButton, &ButtonControl::clicked, [vehiclesLayout, mazdaPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(mazdaPanel);
      });
      vehicleToggle = mazdaButton;

    } else if (param == "MazdaCollectAll") {
      ButtonControl *mazdaCollectAllButton = new ButtonControl(title, tr("COLLECT"), desc);
      QObject::connect(mazdaCollectAllButton, &ButtonControl::clicked, [mazdaCollectAllButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Collect All can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("MazdaTuneStackStatus", "Queued...");
        params_memory.putBool("MazdaCollectAll", true);
        mazdaCollectAllButton->setValue(tr("Queued..."));
      });
      vehicleToggle = mazdaCollectAllButton;

    } else if (param == "MazdaFitAll") {
      ButtonControl *mazdaFitAllButton = new ButtonControl(title, tr("FIT"), desc);
      QObject::connect(mazdaFitAllButton, &ButtonControl::clicked, [mazdaFitAllButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Fit All can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("MazdaTuneStackStatus", "Queued...");
        params_memory.putBool("MazdaFitAll", true);
        mazdaFitAllButton->setValue(tr("Queued..."));
      });
      vehicleToggle = mazdaFitAllButton;

    } else if (param == "MazdaLatAbcdToggles") {
      ButtonControl *navButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(navButton, &ButtonControl::clicked, [vehiclesLayout, mazdaLatAbcdPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(mazdaLatAbcdPanel);
      });
      vehicleToggle = navButton;

    } else if (param == "MazdaLatDelayToggles") {
      ButtonControl *navButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(navButton, &ButtonControl::clicked, [vehiclesLayout, mazdaLatDelayPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(mazdaLatDelayPanel);
      });
      vehicleToggle = navButton;

    } else if (param == "MazdaLongToggles") {
      ButtonControl *navButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(navButton, &ButtonControl::clicked, [vehiclesLayout, mazdaLongPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(mazdaLongPanel);
      });
      vehicleToggle = navButton;

    } else if (param == "MazdaAutoTune") {
      ButtonControl *autoTuneButton = new ButtonControl(title, tr("RUN"), desc);
      QObject::connect(autoTuneButton, &ButtonControl::clicked, [autoTuneButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Auto-Tune can only run while parked. Try again when stopped."), this);
          return;
        }
        if (!FrogPilotConfirmationDialog::yesorno(tr("Process your driving logs and re-learn the Mazda lateral tune? This runs in the background while parked and may take a while."), this)) {
          return;
        }
        autoTuneRebootPrompted = false;
        params_memory.put("AutoTuneStatus", "Queued...");
        params_memory.putBool("MazdaAutoTune", true);
        autoTuneButton->setValue(tr("Queued..."));
      });
      vehicleToggle = autoTuneButton;

    } else if (param == "MazdaAutoTuneApply") {
      ButtonControl *applyButton = new ButtonControl(title, tr("APPLY"), desc);
      QObject::connect(applyButton, &ButtonControl::clicked, [this]() {
        QString status = QString::fromStdString(params_memory.get("AutoTuneStatus"));
        QStringList parts = status.split('|');
        QString detail = (parts.value(0) == "Preview" && parts.size() >= 3) ? parts.value(2) : QString();
        if (detail.isEmpty()) {
          ConfirmationDialog::alert(tr("No previewed tune to apply. Run Auto-Tune first."), this);
          return;
        }
        if (!FrogPilotConfirmationDialog::yesorno(tr("Apply this tune to your car?\n\n") + detail, this)) {
          return;
        }
        autoTuneRebootPrompted = false;
        params_memory.putBool("MazdaAutoTuneApply", true);
      });
      vehicleToggle = applyButton;

    } else if (param == "MazdaTuneReset") {
      ButtonControl *mazdaResetButton = new ButtonControl(title, tr("RESET"), desc);
      QObject::connect(mazdaResetButton, &ButtonControl::clicked, [this]() {
        if (!FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to reset the Mazda lateral tune to its defaults?"), this)) {
          return;
        }
        QSet<QString> tuneKeys = mazda3Keys;
        tuneKeys.unite(mazdaCX30Keys);
        tuneKeys.unite(mazdaCX50Keys);
        for (const QString &tuneKey : tuneKeys) {
          params.putFloat(tuneKey.toStdString(), params_default.getFloat(tuneKey.toStdString()));
          if (FrogPilotParamValueControl *valueToggle = qobject_cast<FrogPilotParamValueControl*>(toggles[tuneKey])) {
            valueToggle->refresh();
          }
        }
      });
      vehicleToggle = mazdaResetButton;

    } else if (param == "MazdaAutoTuneDeadzone") {
      vehicleToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.0f, 1.0f, tr(" m/s²"), std::map<float, QString>(), 0.0025f, true);

    } else if (param == "LongAutoTuneCollect") {
      ButtonControl *collectButton = new ButtonControl(title, tr("COLLECT"), desc);
      QObject::connect(collectButton, &ButtonControl::clicked, [collectButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Collection can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("LongAutoTuneStatus", "Queued...");
        params_memory.putBool("LongAutoTuneCollect", true);
        collectButton->setValue(tr("Queued..."));
      });
      vehicleToggle = collectButton;

    } else if (param == "LatDelayCollect") {
      ButtonControl *latDelayCollectButton = new ButtonControl(title, tr("COLLECT"), desc);
      QObject::connect(latDelayCollectButton, &ButtonControl::clicked, [latDelayCollectButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Collection can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("LatDelayStatus", "Queued...");
        params_memory.putBool("LatDelayCollect", true);
        latDelayCollectButton->setValue(tr("Queued..."));
      });
      vehicleToggle = latDelayCollectButton;

    } else if (param == "LatDelayFit") {
      ButtonControl *latDelayFitButton = new ButtonControl(title, tr("FIT"), desc);
      QObject::connect(latDelayFitButton, &ButtonControl::clicked, [latDelayFitButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Fit can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("LatDelayStatus", "Queued...");
        params_memory.putBool("LatDelayFit", true);
        latDelayFitButton->setValue(tr("Queued..."));
      });
      vehicleToggle = latDelayFitButton;

    } else if (param == "LatDelayApply") {
      ButtonControl *latDelayApplyButton = new ButtonControl(title, tr("APPLY"), desc);
      QObject::connect(latDelayApplyButton, &ButtonControl::clicked, [this]() {
        QString status = QString::fromStdString(params_memory.get("LatDelayStatus"));
        QStringList parts = status.split('|');
        QString detail = (parts.value(0) == "Preview" && parts.size() >= 3) ? parts.value(2) : QString();
        if (detail.isEmpty()) {
          ConfirmationDialog::alert(tr("No previewed delay table to apply. Run Fit first."), this);
          return;
        }
        if (!FrogPilotConfirmationDialog::yesorno(tr("Apply this lateral delay table?\n\n") + detail, this)) {
          return;
        }
        params_memory.putBool("LatDelayApply", true);
      });
      vehicleToggle = latDelayApplyButton;

    } else if (param == "LatDelayReset") {
      ButtonControl *latDelayResetButton = new ButtonControl(title, tr("RESET"), desc);
      QObject::connect(latDelayResetButton, &ButtonControl::clicked, [this]() {
        if (!FrogPilotConfirmationDialog::yesorno(tr("Clear the applied lateral delay table?"), this)) {
          return;
        }
        params_memory.putBool("LatDelayReset", true);
      });
      vehicleToggle = latDelayResetButton;

    } else if (param == "LongDelayFit") {
      ButtonControl *longDelayFitButton = new ButtonControl(title, tr("FIT"), desc);
      QObject::connect(longDelayFitButton, &ButtonControl::clicked, [longDelayFitButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Fit can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("LongDelayStatus", "Queued...");
        params_memory.putBool("LongDelayFit", true);
        longDelayFitButton->setValue(tr("Queued..."));
      });
      vehicleToggle = longDelayFitButton;

    } else if (param == "LongDelayApply") {
      ButtonControl *longDelayApplyButton = new ButtonControl(title, tr("APPLY"), desc);
      QObject::connect(longDelayApplyButton, &ButtonControl::clicked, [this]() {
        QString status = QString::fromStdString(params_memory.get("LongDelayStatus"));
        QStringList parts = status.split('|');
        QString detail = (parts.value(0) == "Preview" && parts.size() >= 3) ? parts.value(2) : QString();
        if (detail.isEmpty()) {
          ConfirmationDialog::alert(tr("No previewed long-delay table to apply. Run Fit first."), this);
          return;
        }
        if (!FrogPilotConfirmationDialog::yesorno(tr("Apply this longitudinal delay table?\n\n") + detail, this)) {
          return;
        }
        params_memory.putBool("LongDelayApply", true);
      });
      vehicleToggle = longDelayApplyButton;

    } else if (param == "LongDelayReset") {
      ButtonControl *longDelayResetButton = new ButtonControl(title, tr("RESET"), desc);
      QObject::connect(longDelayResetButton, &ButtonControl::clicked, [this]() {
        if (!FrogPilotConfirmationDialog::yesorno(tr("Clear the applied long-delay table?"), this)) {
          return;
        }
        params_memory.putBool("LongDelayReset", true);
      });
      vehicleToggle = longDelayResetButton;

    } else if (param == "LongStaticFit") {
      ButtonControl *longStaticFitButton = new ButtonControl(title, tr("FIT"), desc);
      QObject::connect(longStaticFitButton, &ButtonControl::clicked, [longStaticFitButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Fit can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("LongStaticStatus", "Queued...");
        params_memory.putBool("LongStaticFit", true);
        longStaticFitButton->setValue(tr("Queued..."));
      });
      vehicleToggle = longStaticFitButton;

    } else if (param == "LongStaticApply") {
      ButtonControl *longStaticApplyButton = new ButtonControl(title, tr("APPLY"), desc);
      QObject::connect(longStaticApplyButton, &ButtonControl::clicked, [this]() {
        QString status = QString::fromStdString(params_memory.get("LongStaticStatus"));
        QStringList parts = status.split('|');
        QString detail = (parts.value(0) == "Preview" && parts.size() >= 3) ? parts.value(2) : QString();
        if (detail.isEmpty()) {
          ConfirmationDialog::alert(tr("No previewed static map to apply. Run Fit first."), this);
          return;
        }
        if (!FrogPilotConfirmationDialog::yesorno(tr("Apply this longitudinal static map?\n\nThis replaces the carcontroller's affine accel→CAN map with a per-speed table. Reboot after to use.\n\n") + detail, this)) {
          return;
        }
        params_memory.putBool("LongStaticApply", true);
      });
      vehicleToggle = longStaticApplyButton;

    } else if (param == "LongStaticReset") {
      ButtonControl *longStaticResetButton = new ButtonControl(title, tr("RESET"), desc);
      QObject::connect(longStaticResetButton, &ButtonControl::clicked, [this]() {
        if (!FrogPilotConfirmationDialog::yesorno(tr("Clear the applied static map? carcontroller will revert to the stock global affine."), this)) {
          return;
        }
        params_memory.putBool("LongStaticReset", true);
      });
      vehicleToggle = longStaticResetButton;

    } else if (param == "LatOpenLoopCollect") {
      ButtonControl *latOpenLoopCollectButton = new ButtonControl(title, tr("COLLECT"), desc);
      QObject::connect(latOpenLoopCollectButton, &ButtonControl::clicked, [latOpenLoopCollectButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Collection can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("LatOpenLoopStatus", "Queued...");
        params_memory.putBool("LatOpenLoopCollect", true);
        latOpenLoopCollectButton->setValue(tr("Queued..."));
      });
      vehicleToggle = latOpenLoopCollectButton;

    } else if (param == "LatOpenLoopFit") {
      ButtonControl *latOpenLoopFitButton = new ButtonControl(title, tr("FIT"), desc);
      QObject::connect(latOpenLoopFitButton, &ButtonControl::clicked, [latOpenLoopFitButton, this]() {
        if (started) {
          ConfirmationDialog::alert(tr("Fit can only run while parked. Try again when stopped."), this);
          return;
        }
        params_memory.put("LatOpenLoopStatus", "Queued...");
        params_memory.putBool("LatOpenLoopFit", true);
        latOpenLoopFitButton->setValue(tr("Queued..."));
      });
      vehicleToggle = latOpenLoopFitButton;

    } else if (param == "LatOpenLoopApply") {
      ButtonControl *latOpenLoopApplyButton = new ButtonControl(title, tr("APPLY"), desc);
      QObject::connect(latOpenLoopApplyButton, &ButtonControl::clicked, [this]() {
        QString status = QString::fromStdString(params_memory.get("LatOpenLoopStatus"));
        QStringList parts = status.split('|');
        QString detail = (parts.value(0) == "Preview" && parts.size() >= 3) ? parts.value(2) : QString();
        if (detail.isEmpty()) {
          ConfirmationDialog::alert(tr("No previewed open-loop fit to apply. Run Fit first."), this);
          return;
        }
        if (!FrogPilotConfirmationDialog::yesorno(tr("Overwrite Mazda{Model}TuneA/B/C/D with the open-loop fit?\n\nApply will refuse internally if K calibration isn't ready or the ±1 coverage gate fails - safe to tap.\n\n") + detail, this)) {
          return;
        }
        params_memory.putBool("LatOpenLoopApply", true);
      });
      vehicleToggle = latOpenLoopApplyButton;

    } else if (param == "LatOpenLoopReset") {
      ButtonControl *latOpenLoopResetButton = new ButtonControl(title, tr("RESET"), desc);
      QObject::connect(latOpenLoopResetButton, &ButtonControl::clicked, [this]() {
        if (!FrogPilotConfirmationDialog::yesorno(tr("Clear the previewed open-loop fit?"), this)) {
          return;
        }
        params_memory.putBool("LatOpenLoopReset", true);
      });
      vehicleToggle = latOpenLoopResetButton;

    } else if (mazdaKeys.contains(param)) {
      // a: gain 0-30, b/c: gains 0-3, d: signed torque offset -1..1
      float mazdaMinValue = param.endsWith("D") ? -1.0f : 0.0f;
      float mazdaMaxValue = param.endsWith("A") ? 30.0f : (param.endsWith("D") ? 1.0f : 3.0f);
      vehicleToggle = new FrogPilotParamValueControl(param, title, desc, icon, mazdaMinValue, mazdaMaxValue, QString(), std::map<float, QString>(), 0.0001f, true);

    } else if (param == "ToyotaToggles") {
      ButtonControl *toyotaButton = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(toyotaButton, &ButtonControl::clicked, [vehiclesLayout, toyotaPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(toyotaPanel);
      });
      vehicleToggle = toyotaButton;
    } else if (param == "ToyotaDoors") {
      std::vector<QString> lockToggles{"LockDoors", "UnlockDoors"};
      std::vector<QString> lockToggleNames{tr("Lock"), tr("Unlock")};
      vehicleToggle = new FrogPilotButtonToggleControl(param, title, desc, icon, lockToggles, lockToggleNames);
    } else if (param == "LockDoorsTimer") {
      std::map<float, QString> autoLockLabels;
      for (int i = 0; i <= 300; ++i) {
        autoLockLabels[i] = i == 0 ? tr("Never") : QString::number(i) + tr(" seconds");
      }
      vehicleToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 300, QString(), autoLockLabels, 5);
    } else if (param == "ClusterOffset") {
      std::vector<QString> clusterOffsetButton{"Reset"};
      FrogPilotParamValueButtonControl *clusterOffsetToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 1.000, 1.050, "x", std::map<float, QString>(), 0.001, false, {}, clusterOffsetButton, false, false);
      QObject::connect(clusterOffsetToggle, &FrogPilotParamValueButtonControl::buttonClicked, [clusterOffsetToggle, this]() {
        params.putFloat("ClusterOffset", params_default.getFloat("ClusterOffset"));
        clusterOffsetToggle->refresh();
      });
      vehicleToggle = clusterOffsetToggle;

    } else if (param == "VehicleInfo") {
      ButtonControl *VehicleInfoButton = new ButtonControl(title, tr("VIEW"), desc);
      QObject::connect(VehicleInfoButton, &ButtonControl::clicked, [vehiclesLayout, vehicleInfoPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(vehicleInfoPanel);
      });
      vehicleToggle = VehicleInfoButton;
    } else if (vehicleInfoKeys.contains(param)) {
      vehicleToggle = new LabelControl(title, "", desc);

    } else {
      vehicleToggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param] = vehicleToggle;

    if (gmKeys.contains(param)) {
      gmList->addItem(vehicleToggle);
    } else if (hkgKeys.contains(param)) {
      hkgList->addItem(vehicleToggle);
    } else if (hondaKeys.contains(param)) {
      hondaList->addItem(vehicleToggle);
    } else if (mazdaLatAbcdKeys.contains(param)) {
      mazdaLatAbcdList->addItem(vehicleToggle);
    } else if (mazdaLatDelayKeys.contains(param)) {
      mazdaLatDelayList->addItem(vehicleToggle);
    } else if (mazdaLongKeys.contains(param)) {
      mazdaLongList->addItem(vehicleToggle);
    } else if (mazdaKeys.contains(param)) {
      mazdaList->addItem(vehicleToggle);
    } else if (subaruKeys.contains(param)) {
      subaruList->addItem(vehicleToggle);
    } else if (toyotaKeys.contains(param)) {
      toyotaList->addItem(vehicleToggle);
    } else if (vehicleInfoKeys.contains(param)) {
      vehicleInfoList->addItem(vehicleToggle);
    } else {
      settingsList->addItem(vehicleToggle);

      parentKeys.insert(param);
    }

    if (ButtonControl *buttonControl = qobject_cast<ButtonControl*>(vehicleToggle)) {
      QObject::connect(buttonControl, &ButtonControl::clicked, this, &FrogPilotVehiclesPanel::openSubPanel);
    }

    QObject::connect(vehicleToggle, &AbstractControl::hideDescriptionEvent, [this]() {
      update();
    });
    QObject::connect(vehicleToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  static_cast<FrogPilotParamValueControl*>(toggles["LockDoorsTimer"])->setWarning("<b>Warning:</b> openpilot can't detect if keys are still inside the car, so ensure you have a spare key to prevent accidental lockouts!");

  QSet<QString> rebootKeys = {"HondaAltTune", "NewLongAPI", "TacoTuneHacks"};
  for (const QString &key : rebootKeys) {
    QObject::connect(static_cast<ToggleControl*>(toggles[key]), &ToggleControl::toggleFlipped, [key, this](bool state) {
      if (started) {
        if (key == "HondaAltTune" || key == "TacoTuneHacks" && state) {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        } else if (key != "TacoTuneHacks") {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        }
      }
    });
  }

  openDescriptions(forceOpenDescriptions, toggles);

  QObject::connect(uiState(), &UIState::offroadTransition, [selectMakeButton, selectModelButton, this]() {
    std::thread([selectMakeButton, selectModelButton, this]() {
      selectMakeButton->setValue(QString::fromStdString(params.get("CarMake", true)));
      selectModelButton->setValue(QString::fromStdString(params.get(params.get("CarModelName").empty() ? "CarModel" : "CarModelName")));
    }).detach();
  });

  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubPanel, [vehiclesLayout, vehiclesPanel, this] {
    if (forceOpenDescriptions) {
      openDescriptions(forceOpenDescriptions, toggles);

      disableOpenpilotLong->showDescription();
      forceFingerprint->showDescription();
    }
    vehiclesLayout->setCurrentWidget(vehiclesPanel);
  });
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotVehiclesPanel::updateState);
}

void FrogPilotVehiclesPanel::showEvent(QShowEvent *event) {
  if (forceOpenDescriptions) {
    disableOpenpilotLong->showDescription();
    forceFingerprint->showDescription();
  }

  frogpilotToggleLevels = parent->frogpilotToggleLevels;

  QStringList detected;
  if (parent->hasPedal) detected << "comma Pedal";
  if (parent->hasSDSU) detected << "SDSU";
  if (parent->hasZSS) detected << "ZSS";
  static_cast<LabelControl*>(toggles["HardwareDetected"])->setText(detected.isEmpty() ? tr("None") : detected.join(", "));

  static_cast<LabelControl*>(toggles["BlindSpotSupport"])->setText(parent->hasBSM ? tr("Yes") : tr("No"));
  static_cast<LabelControl*>(toggles["OpenpilotLongitudinal"])->setText(parent->hasOpenpilotLongitudinal ? tr("Yes") : tr("No"));
  static_cast<LabelControl*>(toggles["PedalSupport"])->setText(parent->canUsePedal ? tr("Yes") : tr("No"));
  static_cast<LabelControl*>(toggles["RadarSupport"])->setText(parent->hasRadar ? tr("Yes") : tr("No"));
  static_cast<LabelControl*>(toggles["SDSUSupport"])->setText(parent->canUseSDSU ? tr("Yes") : tr("No"));
  static_cast<LabelControl*>(toggles["SNGSupport"])->setText(parent->hasSNG ? tr("Yes") : tr("No"));

  updateToggles();
}

void FrogPilotVehiclesPanel::updateState(const UIState &s) {
  if (!isVisible()) {
    return;
  }

  started = s.scene.started;

  if (ButtonControl *autoTuneButton = qobject_cast<ButtonControl*>(toggles["MazdaAutoTune"])) {
    // status protocol: "STATE|<short label>|<full wrapping detail>" for
    // Preview/Done; plain text otherwise (progress/idle/errors).
    QString status = QString::fromStdString(params_memory.get("AutoTuneStatus"));
    if (status != autoTuneStatusShown) {
      autoTuneStatusShown = status;
      QStringList parts = status.split('|');
      QString state = parts.value(0);
      if ((state == "Preview" || state == "Done" || state == "Refused") && parts.size() >= 3) {
        autoTuneButton->setValue(parts.value(1));
        autoTuneButton->setDescription(parts.value(2));   // full a/b/c/d, wraps - no truncation
        autoTuneButton->showDescription();
        if (state == "Done" && !autoTuneRebootPrompted) {
          autoTuneRebootPrompted = true;
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        }
      } else if (!status.isEmpty()) {
        autoTuneButton->setValue(status);
      }
    }
  }

  if (ButtonControl *collectButton = qobject_cast<ButtonControl*>(toggles["LongAutoTuneCollect"])) {
    // long collector status protocol: "STATE|<short>|<full detail>" for
    // Idle/Collect; plain text for queued/errors.
    QString status = QString::fromStdString(params_memory.get("LongAutoTuneStatus"));
    if (status != longCollectStatusShown) {
      longCollectStatusShown = status;
      QStringList parts = status.split('|');
      QString state = parts.value(0);
      if ((state == "Idle" || state == "Collect") && parts.size() >= 3) {
        collectButton->setValue(parts.value(1));
        collectButton->setDescription(parts.value(2));
        collectButton->showDescription();
      } else if (!status.isEmpty()) {
        collectButton->setValue(status);
      }
    }
  }

  // Lateral-delay status is shared across Collect/Fit/Apply buttons; show
  // the full detail on each so the user can see what's going on regardless
  // of which button they're looking at.
  QString latDelayStatus = QString::fromStdString(params_memory.get("LatDelayStatus"));
  if (latDelayStatus != latDelayStatusShown) {
    latDelayStatusShown = latDelayStatus;
    QStringList parts = latDelayStatus.split('|');
    QString state = parts.value(0);
    const bool isStructured = (state == "Idle" || state == "Collect" || state == "Preview"
                               || state == "Done" || state == "Refused") && parts.size() >= 3;
    for (const QString &key : {"LatDelayCollect", "LatDelayFit", "LatDelayApply"}) {
      if (ButtonControl *btn = qobject_cast<ButtonControl*>(toggles[key])) {
        if (isStructured) {
          btn->setValue(parts.value(1));
          btn->setDescription(parts.value(2));
          btn->showDescription();
        } else if (!latDelayStatus.isEmpty()) {
          btn->setValue(latDelayStatus);
        }
      }
    }
  }

  // Long-delay status is shared across Fit/Apply buttons (no Collect button -
  // the long collector populates the delay store as a side effect already).
  QString longDelayStatus = QString::fromStdString(params_memory.get("LongDelayStatus"));
  if (longDelayStatus != longDelayStatusShown) {
    longDelayStatusShown = longDelayStatus;
    QStringList parts = longDelayStatus.split('|');
    QString state = parts.value(0);
    const bool isStructured = (state == "Idle" || state == "Preview" || state == "Done"
                               || state == "Refused") && parts.size() >= 3;
    for (const QString &key : {"LongDelayFit", "LongDelayApply"}) {
      if (ButtonControl *btn = qobject_cast<ButtonControl*>(toggles[key])) {
        if (isStructured) {
          btn->setValue(parts.value(1));
          btn->setDescription(parts.value(2));
          btn->showDescription();
        } else if (!longDelayStatus.isEmpty()) {
          btn->setValue(longDelayStatus);
        }
      }
    }
  }

  // Long-static status is shared across Fit/Apply buttons (no Collect -
  // long_collect's static store population is shared with the delay path).
  QString longStaticStatus = QString::fromStdString(params_memory.get("LongStaticStatus"));
  if (longStaticStatus != longStaticStatusShown) {
    longStaticStatusShown = longStaticStatus;
    QStringList parts = longStaticStatus.split('|');
    QString state = parts.value(0);
    const bool isStructured = (state == "Idle" || state == "Preview" || state == "Done"
                               || state == "Refused") && parts.size() >= 3;
    for (const QString &key : {"LongStaticFit", "LongStaticApply"}) {
      if (ButtonControl *btn = qobject_cast<ButtonControl*>(toggles[key])) {
        if (isStructured) {
          btn->setValue(parts.value(1));
          btn->setDescription(parts.value(2));
          btn->showDescription();
        } else if (!longStaticStatus.isEmpty()) {
          btn->setValue(longStaticStatus);
        }
      }
    }
  }

  // Open-loop lateral status: shared across Collect/Fit/Apply.  Apply is
  // gated on calibration so its message will land here when poked.
  QString latOpenLoopStatus = QString::fromStdString(params_memory.get("LatOpenLoopStatus"));
  if (latOpenLoopStatus != latOpenLoopStatusShown) {
    latOpenLoopStatusShown = latOpenLoopStatus;
    QStringList parts = latOpenLoopStatus.split('|');
    QString state = parts.value(0);
    const bool isStructured = (state == "Idle" || state == "Collect" || state == "Preview"
                               || state == "Done" || state == "Refused") && parts.size() >= 3;
    for (const QString &key : {"LatOpenLoopCollect", "LatOpenLoopFit", "LatOpenLoopApply"}) {
      if (ButtonControl *btn = qobject_cast<ButtonControl*>(toggles[key])) {
        if (isStructured) {
          btn->setValue(parts.value(1));
          btn->setDescription(parts.value(2));
          btn->showDescription();
        } else if (!latOpenLoopStatus.isEmpty()) {
          btn->setValue(latOpenLoopStatus);
        }
      }
    }
  }

  // Tune-stack status: shared across the top-level Collect All / Fit All
  // buttons.  Per-feature statuses still drive the individual buttons; this
  // one is a separate channel so the stack progress doesn't fight the per-
  // feature messages.
  QString tuneStackStatus = QString::fromStdString(params_memory.get("MazdaTuneStackStatus"));
  if (tuneStackStatus != tuneStackStatusShown) {
    tuneStackStatusShown = tuneStackStatus;
    QStringList parts = tuneStackStatus.split('|');
    QString state = parts.value(0);
    const bool isStructured = (state == "Collect" || state == "Fit" || state == "Done"
                               || state == "Refused") && parts.size() >= 3;
    for (const QString &key : {"MazdaCollectAll", "MazdaFitAll"}) {
      if (ButtonControl *btn = qobject_cast<ButtonControl*>(toggles[key])) {
        if (isStructured) {
          btn->setValue(parts.value(1));
          btn->setDescription(parts.value(2));
          btn->showDescription();
        } else if (!tuneStackStatus.isEmpty()) {
          btn->setValue(tuneStackStatus);
        }
      }
    }
  }
}

void FrogPilotVehiclesPanel::updateToggles() {
  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      toggle->setVisible(false);
    }
  }

  for (auto &[key, toggle] : toggles) {
    if (parentKeys.contains(key)) {
      continue;
    }

    bool setVisible = parent->tuningLevel >= frogpilotToggleLevels[key].toDouble();

    if (gmKeys.contains(key)) {
      setVisible &= parent->isGM;
    } else if (hkgKeys.contains(key)) {
      setVisible &= parent->isHKG;
    } else if (hondaKeys.contains(key)) {
      setVisible &= parent->isHonda;
    } else if (mazdaKeys.contains(key)) {
      setVisible &= parent->isMazda;
    } else if (subaruKeys.contains(key)) {
      setVisible &= parent->isSubaru;
    } else if (toyotaKeys.contains(key)) {
      setVisible &= parent->isToyota;
    } else if (vehicleInfoKeys.contains(key)) {
      setVisible = true;
    }

    if (longitudinalKeys.contains(key)) {
      setVisible &= parent->hasOpenpilotLongitudinal;
    }

    if (key == "HondaAltTune") {
      setVisible &= parent->isHondaNidec;
    }

    else if (key == "HondaLowSpeedPedal") {
      setVisible &= parent->hasPedal;
    }

    else if (key == "HondaMaxBrake") {
      setVisible &= parent->isHondaNidec;
    }

    else if (mazda3Keys.contains(key)) {
      setVisible &= parent->isMazda3;
    }

    else if (mazdaCX30Keys.contains(key)) {
      setVisible &= parent->isMazdaCX30;
    }

    else if (mazdaCX50Keys.contains(key)) {
      setVisible &= parent->isMazdaCX50;
    }

    else if (key == "SNGHack") {
      setVisible &= !parent->hasPedal && !parent->hasSNG;
    }

    else if (key == "SubaruSNG") {
      setVisible &= parent->hasSNG;
    }

    else if (key == "TacoTuneHacks") {
      setVisible &= parent->isHKGCanFd;
    }

    else if (key == "VoltSNG") {
      setVisible &= parent->isVolt && !parent->hasSNG;
    }

    toggle->setVisible(setVisible);

    if (setVisible) {
      if (gmKeys.contains(key)) {
        toggles["GMToggles"]->setVisible(true);
      } else if (hkgKeys.contains(key)) {
        toggles["HKGToggles"]->setVisible(true);
      } else if (hondaKeys.contains(key)) {
        toggles["HondaToggles"]->setVisible(true);
      } else if (mazdaKeys.contains(key)) {
        toggles["MazdaToggles"]->setVisible(true);
      } else if (subaruKeys.contains(key)) {
        toggles["SubaruToggles"]->setVisible(true);
      } else if (toyotaKeys.contains(key)) {
        toggles["ToyotaToggles"]->setVisible(true);
      } else if (vehicleInfoKeys.contains(key)) {
        toggles["VehicleInfo"]->setVisible(true);
      }
    }
  }

  disableOpenpilotLong->setVisible((parent->hasOpenpilotLongitudinal || parent->openpilotLongitudinalControlDisabled) && !parent->hasExperimentalOpenpilotLongitudinal && parent->tuningLevel >= frogpilotToggleLevels["DisableOpenpilotLongitudinal"].toBool());
  forceFingerprint->setVisible(parent->tuningLevel >= frogpilotToggleLevels["ForceFingerprint"].toBool());

  openDescriptions(forceOpenDescriptions, toggles);

  update();
}
