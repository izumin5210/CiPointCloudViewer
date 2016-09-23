//
// Created by Masayuki IZUMI on 7/12/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_SIGNAL_H
#define CIPOINTCLOUDVIEWERAPP_SIGNAL_H


#include <fruit/fruit.h>
#include "nod/nod.hpp"

#include "Singleton.h"

template <typename T>
using SignalT = nod::signal<void (T&)>;

template <typename T>
using CallbackFn = std::function<void (const T&)>;

template <typename Class, typename T>
using Method = void (Class::*)(const T&);

using Connection = nod::connection;

template <typename T>
class Signal : public Singleton<SignalT<T>> {
public:
  using Singleton<SignalT<T>>::get;

  static Connection connect(const CallbackFn<T>& callback) {
    return get().connect(callback);
  }

  static void emit(T t) {
    get()(t);
  }
};

#endif //CIPOINTCLOUDVIEWERAPP_SIGNAL_H
