//
// Created by Masayuki IZUMI on 7/12/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SIGNAL_H
#define CIPOINTCLOUDVIEWERAPP_SIGNAL_H

#include "cinder/Signals.h"

#include "Singleton.h"

template <typename T>
using SignalT = cinder::signals::Signal<void (T&)>;

template <typename T>
using CallbackFn = std::function<void (const T&)>;

template <typename Class, typename T>
using Method = void (Class::*)(const T&);

using Connection = cinder::signals::Connection;

template <typename T>
class Signal : public Singleton<SignalT<T>> {
public:
  using Singleton<SignalT<T>>::get;

  static Connection connect(const CallbackFn<T>& callback) {
    return get().connect(callback);
  }

  static Connection connect(int priority, const CallbackFn<T>& callback) {
    return get().connect(priority, callback);
  }

  template<typename Class, typename Instance>
  static Connection connect(Instance &object, Method<Class, T> method) {
    return get().connect(cinder::signals::slot(object, method));
  }

  template<typename Class>
  static Connection connect(Class *object, Method<Class, T> method) {
    return get().connect(cinder::signals::slot(object, method));
  }

  static void emit(T t) {
    get().emit(t);
  }

};

#endif //CIPOINTCLOUDVIEWERAPP_SIGNAL_H
