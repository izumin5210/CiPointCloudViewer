//
// Created by Masayuki IZUMI on 7/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_STORE_H
#define CIPOINTCLOUDVIEWERAPP_STORE_H

#include "cinder/Signals.h"

class Store {
public:
  using SignalT = cinder::signals::Signal<void ()>;

  using CallbackFn = std::function<void ()>;

  template <typename Class>
  using Method = void (Class::*)();

  using Connection = cinder::signals::Connection;

  ~Store() {
    disconnectAll();
  }

  Connection connect(const CallbackFn &callback) {
    return signal_.connect(callback);
  }

  Connection connect(int priority, const CallbackFn &callback) {
    return signal_.connect(priority, callback);
  }

  template<typename Class, typename Instance>
  Connection connect(Instance &object, Method<Class> method) {
    return signal_.connect(cinder::signals::slot(object, method));
  }

  template<typename Class>
  Connection connect(Class *object, Method<Class> method) {
    return signal_.connect(cinder::signals::slot(object, method));
  }


protected:
  void emit() {
    signal_.emit();
  }

  void addConnection(Connection connection) {
    connections_.push_back(connection);
  }

  void disconnectAll() {
    for (auto connection : connections_) {
      connection.disconnect();
    }
  }


private:
  SignalT signal_;
  std::vector<Connection> connections_;
};

#endif //CIPOINTCLOUDVIEWERAPP_STORE_H
