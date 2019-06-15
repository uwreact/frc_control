##############################################################################
# Copyright (C) 2019, UW REACT
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of UW REACT, nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##############################################################################

"""General helper functions."""

# Standard imports
import os
import subprocess
import threading

# ROS imports
import rospkg


def async_popen(popen_args, callback=None):
    """Asynchronously run subprocess.Popen, with the callback on completion."""

    def _run(popen_args, callback):
        proc = subprocess.Popen(*popen_args, stdout=open('/dev/null'), stderr=open('/dev/null'))
        proc.wait()
        if callback is not None:
            callback()
        return

    thread = threading.Thread(target=_run, args=(popen_args, callback))
    thread.start()
    return thread


def async_check_output(subprocess_args, success_callback=None, failure_callback=None):
    """Asynchronously run subprocess.check_output, with the callback on completion."""

    def _run(subprocess_args, success_callback, failure_callback):
        try:
            output = subprocess.check_output(*subprocess_args, stderr=open('/dev/null')).strip()
            if success_callback is not None:
                success_callback(output)
        except subprocess.CalledProcessError as error:
            if failure_callback is not None:
                failure_callback(error)

    thread = threading.Thread(target=_run, args=(subprocess_args, success_callback, failure_callback))
    thread.start()
    return thread


def load_resource(filename):
    """Load the specified resource from the driver_station package's resource dir."""
    return os.path.join(rospkg.RosPack().get_path('driver_station'), 'resources', filename)
