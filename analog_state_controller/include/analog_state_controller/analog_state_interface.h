///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, UW REACT
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of UW REACT, nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef ANALOG_STATE_INTERFACE_H
#define ANALOG_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface {

/** @brief A handle used to read the state of a generic analog sensor (eg. potentiometer). */
class AnalogStateHandle {
public:
  AnalogStateHandle() : name_(), state_(0) {}

  /**
   * @param name The name of the sensor
   * @param state A pointer to the storage for this sensor's state
   */
  AnalogStateHandle(const std::string& name, const double* state)
    : name_(name), state_(state) {
    if (!state)
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. State data pointer is null.");
  }

  std::string getName() const {return name_;}
  double getState() const {assert(state_); return *state_;}
  const double* getStatePtr() const {return state_;}

private:
  std::string name_;
  const double* state_;
};

/** @brief Hardware interface to support reading the state of an analog sensor. */
class AnalogStateInterface : public HardwareResourceManager<AnalogStateHandle> {};

} // namespace hardware_interface

#endif