#pragma once
#include "protocol.h"
#include <functional>
#include <memory>
namespace mt {
struct Connection {
    QString backend = "Demo", channel = "0", python = "python", driverDirectory;
    int deviceIndex = 0;
    Version version = Version::V2;
    size_t historyCapacity = 1000000;
};
class CanBackend {
  public:
    virtual ~CanBackend() = default;
    virtual void open(const Connection &) = 0;
    virtual void close() = 0;
    virtual void send(const Frame &) = 0;
    virtual std::vector<Frame> receive() = 0;
    virtual void setReceiveNotifier(std::function<void()> notify) { (void)notify; }
    virtual QString description() const { return {}; }
};
std::unique_ptr<CanBackend> createBackend(const Connection &);
QStringList backendNames();
} // namespace mt
