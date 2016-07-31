//
// Created by Masayuki IZUMI on 7/19/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_STORE_H
#define CIPOINTCLOUDVIEWERAPP_STORE_H

#include "nod/nod.hpp"

class Store {
public:
  using SignalT = nod::signal<void ()>;
  using CallbackFn = std::function<void ()>;

  template <typename Class>
  using Method = void (Class::*)();

  using Connection = nod::connection;

  ~Store() {
    disconnectAll();
  }

  Connection connect(const CallbackFn &callback) {
    return signal_.connect(callback);
  }


protected:
  void emit() {
    signal_();
  }

  void addConnection(Connection connection) {
    connections_.emplace_back(std::move(connection));
  }

  void disconnectAll() {
    for (size_t i = 0; i < connections_.size(); i++) {
      connections_[i].disconnect();
    }
  }


private:
  SignalT signal_;
  std::vector<Connection> connections_;
};

#endif //CIPOINTCLOUDVIEWERAPP_STORE_H
