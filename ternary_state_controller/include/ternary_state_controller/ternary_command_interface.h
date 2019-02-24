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

#ifndef TERNARY_COMMAND_INTERFACE_H
#define TERNARY_COMMAND_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <ternary_state_controller/ternary_state_interface.h>

namespace hardware_interface {

/**
 * A handle used to read and command a ternary (-1, 0, 1 or reverse, off, forward) actuator
 * (eg. H-bridge relay, double-acting solenoid).
 */
class TernaryCommandHandle : public TernaryStateHandle {
public:
  TernaryCommandHandle() : TernaryStateHandle(), cmd_(nullptr) {}

  /**
   * \param state_handle This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   */
  TernaryCommandHandle(const TernaryStateHandle& state_handle, TernaryState* cmd)
      : TernaryStateHandle(state_handle), cmd_(cmd) {
    if (!cmd_)
      throw HardwareInterfaceException("Cannot create handle '" + getName() + "'. Command data pointer is null.");
  }

  void setCommand(TernaryState command) {
    assert(cmd_);
    *cmd_ = command;
  }

  TernaryState getCommand() const {
    assert(cmd_);
    return *cmd_;
  }

private:
  TernaryState* cmd_;
};

/** \brief Hardware interface to support commanding an array of ternary actuators. */
class TernaryCommandInterface : public HardwareResourceManager<TernaryCommandHandle, ClaimResources> {};

}  // namespace hardware_interface

#endif
