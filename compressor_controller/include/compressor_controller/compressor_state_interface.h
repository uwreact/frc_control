///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019, UW REACT
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

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface {

/** @brief A handle used to read the state of a compressor. */
class CompressorStateHandle {
public:
  struct CompressorState {
    bool   enabled;          ///< If the compressor output is active
    bool   pressure_switch;  ///< If the pressure switch is triggered (pressure is low)
    double current;          ///< How much current the compressor is drawing
    bool   closed_loop;      ///< If closed loop control of the compressor is enabled

    bool fault_current_too_high;  ///< If the compressor output has been disabled due to high current draw
    bool fault_shorted;           ///< If the compressor output has been disabled due to a short circuit
    bool fault_not_connected;     ///< If the compressor output does not appear to be wired
  };

  CompressorStateHandle()
      : name_(),
        enabled_(nullptr),
        pressure_switch_(nullptr),
        current_(nullptr),
        closed_loop_(nullptr),
        fault_current_too_high_(nullptr),
        fault_shorted_(nullptr),
        fault_not_connected_(nullptr) {}

  CompressorStateHandle(const std::string& name, const CompressorState* state)
      : name_(name),
        closed_loop_(&state->closed_loop),
        enabled_(&state->enabled),
        pressure_switch_(&state->pressure_switch),
        current_(&state->current),
        fault_current_too_high_(&state->fault_current_too_high),
        fault_shorted_(&state->fault_shorted),
        fault_not_connected_(&state->fault_not_connected) {}

  CompressorStateHandle(const std::string& name,
                        const bool*        closed_loop,
                        const bool*        enabled,
                        const bool*        pressure_switch,
                        const double*      current,
                        const bool*        fault_current_too_high,
                        const bool*        fault_shorted,
                        const bool*        fault_not_connected)
      : name_(name),
        closed_loop_(closed_loop),
        enabled_(enabled),
        pressure_switch_(pressure_switch),
        current_(current),
        fault_current_too_high_(fault_current_too_high),
        fault_shorted_(fault_shorted),
        fault_not_connected_(fault_not_connected) {

    if (!closed_loop_)
      throw HardwareInterfaceException("Cannot create compressor state handle '" + name
                                       + "'. Closed loop mode pointer is null.");
    if (!enabled_)
      throw HardwareInterfaceException("Cannot create compressor state handle '" + name
                                       + "'. Enabled pointer is null.");
    if (!pressure_switch_)
      throw HardwareInterfaceException("Cannot create compressor state handle '" + name
                                       + "'. Pressure switch pointer is null.");
    if (!current_)
      throw HardwareInterfaceException("Cannot create compressor state handle '" + name
                                       + "'. Current pointer is null.");
    if (!fault_current_too_high_)
      throw HardwareInterfaceException("Cannot create compressor state handle '" + name
                                       + "'. Current too high fault pointer is null.");
    if (!fault_shorted_)
      throw HardwareInterfaceException("Cannot create compressor state handle '" + name
                                       + "'. Shorted fault pointer is null.");
    if (!fault_not_connected_)
      throw HardwareInterfaceException("Cannot create compressor state handle '" + name
                                       + "'. Not connected fault pointer is null.");
  }

  // clang-format off
  std::string getName() const                     {return name_;}

  bool   getClosedLoop() const                    {assert(closed_loop_);            return *closed_loop_;}
  bool   getEnabled() const                       {assert(enabled_);                return *enabled_;}
  bool   getPressureSwitch() const                {assert(pressure_switch_);        return *pressure_switch_;}
  double getCurrent() const                       {assert(current_);                return *current_;}
  bool   getFaultCurrentTooHigh() const           {assert(fault_current_too_high_); return *fault_current_too_high_;}
  bool   getFaultShorted() const                  {assert(fault_shorted_);          return *fault_shorted_;}
  bool   getFaultNotConnected() const             {assert(fault_not_connected_);    return *fault_not_connected_;}

  const bool*   getClosedLoopPtr() const          {return closed_loop_;}
  const bool*   getEnabledPtr() const             {return enabled_;}
  const bool*   getPressureSwitchPtr() const      {return pressure_switch_;}
  const double* getCurrentPtr() const             {return current_;}
  const bool*   getFaultCurrentTooHighPtr() const {return fault_current_too_high_;}
  const bool*   getFaultShortedPtr() const        {return fault_shorted_;}
  const bool*   getFaultNotConnectedPtr() const   {return fault_not_connected_;}
  // clang-format on

private:
  std::string   name_;
  const bool*   closed_loop_;
  const bool*   enabled_;
  const bool*   pressure_switch_;
  const double* current_;
  const bool*   fault_current_too_high_;
  const bool*   fault_shorted_;
  const bool*   fault_not_connected_;
};

/** @brief Hardware interface to support reading the state of a compressor. */
class CompressorStateInterface : public HardwareResourceManager<CompressorStateHandle> {};

}  // namespace hardware_interface
