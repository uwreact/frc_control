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

#include <compressor_controller/compressor_state_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface {

/** @brief A handle used to read and command a compressor. */
class CompressorCommandHandle : public CompressorStateHandle {
public:
  CompressorCommandHandle() : CompressorStateHandle(), cmd_(nullptr) {}

  /**
   * @param state_handle This joint's state handle
   * @param cmd A pointer to the storage for this joint's output command
   */
  CompressorCommandHandle(const CompressorStateHandle& state_handle, bool* cmd)
      : CompressorStateHandle(state_handle), cmd_(cmd) {
    if (!cmd_)
      throw HardwareInterfaceException("Cannot create handle '" + getName() + "'. Command data pointer is null.");
  }

  void setCommand(bool command) {
    assert(cmd_);
    *cmd_ = command;
  }

  bool getCommand() const {
    assert(cmd_);
    return *cmd_;
  }

private:
  bool* cmd_;
};

/** @brief Hardware interface to support commanding a compressor. */
class CompressorCommandInterface : public HardwareResourceManager<CompressorCommandHandle, ClaimResources> {};

}  // namespace hardware_interface
