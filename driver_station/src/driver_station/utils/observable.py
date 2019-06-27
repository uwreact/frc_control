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

"""The Observable class."""

import copy


class Observable(object):
    """This class represents an observable object, or "data" in the model-view paradigm.

    It can be subclassed to represent an object that the application wants to have observed.

    This class is based off of the Java.util.Observable class.
    """

    def __init__(self):
        self.obs = []
        self.changed = False

    def add_observer(self, observer):
        """Add an observer to the set of observers for this object.

        `observer` should be a function expecting this observable object as the first argument
        and the observed value as the second.
        """
        if observer not in self.obs:
            self.obs.append(observer)

    def delete_observer(self, observer):
        """Delete an observer from the set of observers of this object."""
        self.obs.remove(observer)

    def delete_observers(self):
        """Clear the observer list so that this object no longer has any observers."""
        self.obs = []

    def notify_observers(self, arg=None):
        """If this object has changed, notify all of its observers."""
        if not self.changed:
            return
        for observer in self.obs:
            observer(self, arg)

    def force_notify(self, arg=None):
        """Notify all observers."""
        for observer in self.obs:
            observer(self, arg)

    def set_changed(self):
        """Marks this Observable object as having been changed."""
        self.changed = True

    def clear_changed(self):
        """Indicate that this object has no longer changed, or that it has
        already notified all of its observers of its most recent change."""
        self.changed = False

    def has_changed(self):
        """Test if this object has changed."""
        return self.changed

    def count_observers(self):
        """Return the number of observers of this Observable object."""
        return len(self.obs)


class ObservableData(Observable):
    """Generic implementation of the Observable class."""

    def __init__(self, init_val=None):
        super(ObservableData, self).__init__()
        self._val = init_val

    def set(self, new_val):
        """Set the value."""
        if self._val == new_val:
            return

        self._val = copy.deepcopy(new_val)
        self.set_changed()
        self.notify_observers()
        self.clear_changed()

    def get(self):
        """Get the stored value."""
        return copy.deepcopy(self._val)

    def force_notify(self, arg=None):
        super(ObservableData, self).force_notify(arg or self._val)

    def notify_observers(self, arg=None):
        super(ObservableData, self).notify_observers(arg or self._val)


class ObservableObj(ObservableData):
    """Generic object implementation of the Observable class."""

    def set_attr(self, attr, new_val):
        """Set the value."""
        if getattr(self._val, attr) == new_val:
            return

        setattr(self._val, attr, new_val)
        self.set_changed()
        self.notify_observers()
        self.clear_changed()


class ObservableDict(Observable):
    """Generic Dict implementation of the Observable class."""

    def __init__(self, init_dict=None):
        super(ObservableDict, self).__init__()

        if init_dict is None:
            self._dict = {}
        else:
            self._dict = init_dict

    def delete(self, key):
        """Delete the specified key from the dict."""
        if key not in self._dict:
            return

        del self._dict[key]
        self.notify_observers(self._dict)

    def set(self, key, new_val):
        """Set the value."""
        if key in self._dict and self._dict[key] == new_val:
            return

        self._dict[key] = new_val
        self.notify_observers(self._dict)

    def get(self, key):
        """Get the stored value."""
        return copy.deepcopy(self._dict[key])

    def get_all(self):
        """Get the stored value."""
        return copy.deepcopy(self._dict)

    def force_notify(self, arg=None):
        super(ObservableDict, self).force_notify(arg or self._dict)

    def notify_observers(self, arg=None):
        super(ObservableDict, self).notify_observers(arg or self._dict)
