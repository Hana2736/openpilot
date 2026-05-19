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
  FrogPilotListWidget *subaruList = new FrogPilotListWidget(this);
  FrogPilotListWidget *toyotaList = new FrogPilotListWidget(this);
  FrogPilotListWidget *vehicleInfoList = new FrogPilotListWidget(this);

  ScrollView *gmPanel = new ScrollView(gmList, this);
  ScrollView *hkgPanel = new ScrollView(hkgList, this);
  ScrollView *hondaPanel = new ScrollView(hondaList, this);
  ScrollView *mazdaPanel = new ScrollView(mazdaList, this);
  ScrollView *subaruPanel = new ScrollView(subaruList, this);
  ScrollView *toyotaPanel = new ScrollView(toyotaList, this);
  ScrollView *vehicleInfoPanel = new ScrollView(vehicleInfoList, this);

  vehiclesLayout->addWidget(gmPanel);
  vehiclesLayout->addWidget(hkgPanel);
  vehiclesLayout->addWidget(hondaPanel);
  vehiclesLayout->addWidget(mazdaPanel);
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

    {"MazdaToggles", tr("Mazda Settings"), tr("<b>FrogPilot features for Mazda vehicles.</b>"), ""},
    {"Mazda3TuneA", tr("Mazda 3 — Sigmoid Slope (a)"), tr("<b>The 'a' coefficient of the \"sigmoid + linear\" lateral-acceleration-to-torque fit for the Mazda 3 2019-24.</b> Controls how sharply torque ramps up near zero lateral acceleration."), ""},
    {"Mazda3TuneB", tr("Mazda 3 — Sigmoid Gain (b)"), tr("<b>The 'b' coefficient of the \"sigmoid + linear\" fit for the Mazda 3 2019-24.</b> Scales the sigmoid (non-linear) portion of the torque curve."), ""},
    {"Mazda3TuneC", tr("Mazda 3 — Linear Gain (c)"), tr("<b>The 'c' coefficient of the \"sigmoid + linear\" fit for the Mazda 3 2019-24.</b> Scales the linear portion of the torque curve."), ""},
    {"Mazda3TuneD", tr("Mazda 3 — Torque Offset (d)"), tr("<b>The 'd' constant torque offset of the \"sigmoid + linear + offset\" fit for the Mazda 3 2019-24.</b> Additive steering/torque bias; may be negative. 0 = no bias."), ""},
    {"MazdaCX30TuneA", tr("Mazda CX-30 — Sigmoid Slope (a)"), tr("<b>The 'a' coefficient of the \"sigmoid + linear\" lateral-acceleration-to-torque fit for the Mazda CX-30 2019-24.</b> Controls how sharply torque ramps up near zero lateral acceleration."), ""},
    {"MazdaCX30TuneB", tr("Mazda CX-30 — Sigmoid Gain (b)"), tr("<b>The 'b' coefficient of the \"sigmoid + linear\" fit for the Mazda CX-30 2019-24.</b> Scales the sigmoid (non-linear) portion of the torque curve."), ""},
    {"MazdaCX30TuneC", tr("Mazda CX-30 — Linear Gain (c)"), tr("<b>The 'c' coefficient of the \"sigmoid + linear\" fit for the Mazda CX-30 2019-24.</b> Scales the linear portion of the torque curve."), ""},
    {"MazdaCX30TuneD", tr("Mazda CX-30 — Torque Offset (d)"), tr("<b>The 'd' constant torque offset of the \"sigmoid + linear + offset\" fit for the Mazda CX-30 2019-24.</b> Additive steering/torque bias; may be negative. 0 = no bias."), ""},
    {"MazdaCX50TuneA", tr("Mazda CX-50 — Sigmoid Slope (a)"), tr("<b>The 'a' coefficient of the \"sigmoid + linear\" lateral-acceleration-to-torque fit for the Mazda CX-50 2022-24.</b> Controls how sharply torque ramps up near zero lateral acceleration."), ""},
    {"MazdaCX50TuneB", tr("Mazda CX-50 — Sigmoid Gain (b)"), tr("<b>The 'b' coefficient of the \"sigmoid + linear\" fit for the Mazda CX-50 2022-24.</b> Scales the sigmoid (non-linear) portion of the torque curve."), ""},
    {"MazdaCX50TuneC", tr("Mazda CX-50 — Linear Gain (c)"), tr("<b>The 'c' coefficient of the \"sigmoid + linear\" fit for the Mazda CX-50 2022-24.</b> Scales the linear portion of the torque curve."), ""},
    {"MazdaCX50TuneD", tr("Mazda CX-50 — Torque Offset (d)"), tr("<b>The 'd' constant torque offset of the \"sigmoid + linear + offset\" fit for the Mazda CX-50 2022-24.</b> Additive steering/torque bias; may be negative. 0 = no bias."), ""},
    {"MazdaAutoTune", tr("Auto-Tune Now"), tr("<b>Learn the Mazda lateral tune from your own driving logs.</b> Recursively scans your saved drives, keeps the clean lateral samples (steering untouched) in a rolling data set, refits the \"sigmoid + linear\" curve, and writes the result for your car. Runs while parked and can take a while the first time."), ""},
    {"MazdaAutoTuneDeadzone", tr("Auto-Tune Lateral Deadzone"), tr("<b>Lateral acceleration deadzone used by Auto-Tune.</b> Samples with |lateral acceleration| below this are ignored when fitting, so straight-line driving doesn't bias the curve. Applied at fit time, so changing it takes effect on the next Auto-Tune without reprocessing logs."), ""},
    {"MazdaTuneReset", tr("Reset Mazda Tune"), tr("<b>Reset all Mazda lateral tune coefficients back to their defaults.</b>"), ""},

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
    QString status = QString::fromStdString(params_memory.get("AutoTuneStatus"));
    if (status.startsWith("Done|")) {
      QString summary = status.mid(5);
      autoTuneButton->setValue(summary);
      if (!autoTuneRebootPrompted) {
        autoTuneRebootPrompted = true;
        if (FrogPilotConfirmationDialog::toggleReboot(this)) {
          Hardware::reboot();
        }
      }
    } else {
      autoTuneButton->setValue(status);
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
