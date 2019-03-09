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

/** @brief A handle used to read the state of a PowerDistributionPanel (PDP). */
class PDPStateHandle {
public:
  struct PDPState {
    double voltage;
    double temperature;
    double total_current;
    double total_power;
    double total_energy;
    double current[16];
  };

  PDPStateHandle()
      : name_(),
        voltage_(nullptr),
        temperature_(nullptr),
        total_current_(nullptr),
        total_power_(nullptr),
        total_energy_(nullptr),
        current_(nullptr) {}

  PDPStateHandle(const std::string& name, const PDPState& state)
      : name_(name),
        voltage_(&state.voltage),
        temperature_(&state.temperature),
        total_current_(&state.total_current),
        total_power_(&state.total_power),
        total_energy_(&state.total_energy),
        current_(state.current) {}

  PDPStateHandle(const std::string& name,
                 const double*      voltage,
                 const double*      temperature,
                 const double*      total_current,
                 const double*      total_power,
                 const double*      total_energy,
                 const double*      current)
      : name_(name),
        voltage_(voltage),
        temperature_(temperature),
        total_current_(total_current),
        total_power_(total_power),
        total_energy_(total_energy),
        current_(current) {

    if (!voltage_)
      throw HardwareInterfaceException("Cannot create PDP state handle '" + name + "'. Voltage pointer is null.");
    if (!temperature_)
      throw HardwareInterfaceException("Cannot create PDP state handle '" + name + "'. Temperature pointer is null.");
    if (!total_current_)
      throw HardwareInterfaceException("Cannot create PDP state handle '" + name + "'. Total current pointer is null.");
    if (!total_power_)
      throw HardwareInterfaceException("Cannot create PDP state handle '" + name + "'. Total power pointer is null.");
    if (!total_energy_)
      throw HardwareInterfaceException("Cannot create PDP state handle '" + name + "'. Total energy pointer is null.");
    if (!current)
      throw HardwareInterfaceException("Cannot create PDP state handle '" + name + "'. Current pointer is null.");
  }

  // clang-format off
    std::string getName() const               {return name_;}

    double getVoltage() const                 {assert(voltage_);        return *voltage_;}
    double getTemperature() const             {assert(temperature_);    return *temperature_;}
    double getTotalCurrent() const            {assert(total_current_);  return *total_current_;}
    double getTotalPower() const              {assert(total_power_);    return *total_power_;}
    double getTotalEnergy() const             {assert(total_energy_);   return *total_energy_;}
    double getCurrent(unsigned i) const       {assert(current_);        return current_[i];}

    const double* getVoltagePtr() const       {return voltage_;}
    const double* getTemperaturePtr() const   {return temperature_;}
    const double* getTotalCurrentPtr() const  {return total_current_;}
    const double* getTotalPowerPtr() const    {return total_power_;}
    const double* getTotalEnergyPtr() const   {return total_energy_;}
    const double* getCurrentPtr() const       {return current_;}
  // clang-format on

private:
  std::string   name_;
  const double* voltage_;
  const double* temperature_;
  const double* total_current_;
  const double* total_power_;
  const double* total_energy_;
  const double* current_;
};

/** @brief Hardware interface to support reading the state of the PowerDistributionPanel (PDP). */
class PDPStateInterface : public HardwareResourceManager<PDPStateHandle> {};

}  // namespace hardware_interface
