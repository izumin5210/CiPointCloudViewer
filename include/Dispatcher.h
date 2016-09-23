//
// Created by Masayuki IZUMI on 2016/09/02.
//

#ifndef CIPOINTCLOUDVIEWERAPP_DISPATCHER_H
#define CIPOINTCLOUDVIEWERAPP_DISPATCHER_H

#include <map>
#include <mutex>
#include <typeindex>

#include <fruit/fruit.h>
#include "nod/nod.hpp"
#include "boost/any.hpp"

#include <iostream>

class Dispatcher {
public:
  using EventName = std::string;
  template <typename T>
  using SignalImpl = nod::signal<void (const T&)>;
  template <typename T>
  using CallbackFn = std::function<void (const T&)>;
  using Connection = nod::connection;

  Dispatcher() {}
  ~Dispatcher() {}

  template <typename T>
  void emit(const T t) {
//    std::lock_guard<std::mutex> lg(mutex_);
//    mutex_.lock();
    auto signal = std::dynamic_pointer_cast<Signal<T>>(signals_[typeid(T).name()]);
//    mutex_.unlock();
    signal->impl(t);
  }

  template <typename T>
  Connection connect(const CallbackFn<T> &callbackFn) {
//    mutex_.lock();
    std::string name = typeid(T).name();
    if (signals_.find(name) == signals_.end()) {
      signals_[name] = std::make_shared<Signal<T>>();
    }
    auto signal = std::dynamic_pointer_cast<Signal<T>>(signals_[name]);
//    mutex_.unlock();
    return signal->impl.connect(callbackFn);
  }


private:
  struct SignalBase {
    SignalBase() {}
    virtual ~SignalBase() {}
  };

  template <typename T>
  struct Signal : public SignalBase {
    Signal() : SignalBase() {}

    SignalImpl<T> impl;
  };

  template <typename T>
  static int get_id() {
    static int i = create_next_id();
    return i;
  }

  static int create_next_id() {
    static int i = 0;
    return i++;
  }

  std::map<std::string, std::shared_ptr<SignalBase>> signals_;
  std::mutex mutex_;
};


fruit::Component<Dispatcher> getDispatcherComponent();

#endif //CIPOINTCLOUDVIEWERAPP_DISPATCHER_H
