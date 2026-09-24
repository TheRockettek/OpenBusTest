#pragma once

#include <shared_mutex>
#include <string>
#include <unordered_map>

class Variables {
  public:
    Variables();

    void declare(const std::string& name);
    double get(const std::string& name) const;
    void set(const std::string& name, double value);
    void updateFrame(double timegap, double getTime, double mouseX, double mouseY);

  private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<std::string, double> values_;
};