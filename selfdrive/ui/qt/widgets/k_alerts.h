/*
 * included_by: home.cc
 *
 * WHEN IN DOUBT, DO NOT INCLUDE, DO NOT USE.
 */

#include <string>
#include <vector>

#include <QString>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/popup.h"

class AlertsManager {

  std::vector<std::string> alerts;
  Params params;

public:
  QString text;
  int unread = 0;

  AlertsManager();
  void refresh();
};

#ifdef K_IMPL

#include <QJsonDocument>
#include <QJsonObject>

AlertsManager::AlertsManager() {
  QString json = util::read_file("../controls/lib/alerts_offroad.json").c_str();
  auto obj = QJsonDocument::fromJson(json.toUtf8()).object();

  // descending sort labels by severity
  std::vector<std::pair<std::string, int>> sorted;
  for (auto it = obj.constBegin(); it != obj.constEnd(); ++it) {
    sorted.push_back({it.key().toStdString(), it.value()["severity"].toInt()});
  }
  std::sort(sorted.begin(), sorted.end(), [=](auto &l, auto &r) { return l.second > r.second; });

  for (const auto& [key, severity] : sorted) {
    alerts.push_back(key);
  }
}

void AlertsManager::refresh() {
  text = "";
  unread = 0;

  for (const auto& key : alerts) {
    auto bytes = params.get(key);
    if (bytes.size()) {
      auto doc_par = QJsonDocument::fromJson(bytes.c_str());
      text += doc_par["text"].toString();
      text += "\n\n";
      unread++;
    }
  }
}

#endif
