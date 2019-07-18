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

"""GUI-related utilities."""

# Standard imports
from enum import IntEnum

# ROS imports
from python_qt_binding import QtGui


class Color(IntEnum):
    """Standard colors used in the UI."""
    GRAY = 0x282828
    RED = 0xFF1400
    BAR_GREEN = 0x007310
    BTN_GREEN = 0x2edc00
    ORANGE = 0xDCAE00


def bool_style(element, enabled, use_red=False):
    """Format the specified element as a boolean indicator light.

    Args:
        element (QWidget):  The element to format. Typically a QLineEdit.
        enabled (bool):     Whether to format as enabled or disabled.
        use_red (bool):     Whether to use red or grey as the disabled color.
    """
    if use_red:
        color_enabled = Color.BTN_GREEN
        color_disabled = Color.RED
    else:
        color_enabled = Color.BTN_GREEN
        color_disabled = Color.GRAY

    if enabled:
        element.setStyleSheet('background-color: #{:06x}'.format(color_enabled))
    else:
        element.setStyleSheet('background-color: #{:06x}'.format(color_disabled))


def set_underlined(element, underlined):
    """Set whether the specified element should use an underlined font or not."""
    f = element.font()
    f.setUnderline(underlined)
    element.setFont(f)


def setup_int_validator(element, fixup, lower=0, upper=2**31 - 1):
    """Setup an int validator for the specified element.

    Args:
        element (QLineEdit):    The element on which to apply the validator.
        fixup   (function):     Callback to perform when fixing invalid inputs.
        lower   (int):          The lower limit of allowable values (Default=0).
        upper   (int):          The uppoer limit of allowable values (Default=2^31 - 1).
    """
    validator = QtGui.QIntValidator()
    validator.setRange(lower, upper)
    validator.fixup = fixup
    element.setValidator(validator)
