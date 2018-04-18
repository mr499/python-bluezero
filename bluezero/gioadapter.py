"""DBus Adapter with GDBus"""
from collections import namedtuple

from gi.repository import GObject, Gio, GLib
from bluezero import constants

# Use this as a mixin (should mixins have __init__?)
class EventLoop:
    """Mixin for abstracting the event loop."""
    def __init__(self):
        self.mainloop = GObject.MainLoop()

    def run(self):
        self.mainloop.run()

    def quit(self):
        self.mainloop.quit()

    def is_running(self):
        self.mainloop.is_running()

    def add_timer(self, time, callback):
        GObject.timeout_add(time, callback)


class AdapterError(Exception):
    """Generic Exception for Adapter Errors."""
    pass


# Create a named tuple to represent DBus Properties
# Paramaters are:
#   name (as understood by Bluez)
#   sig (signature for set methods, the type of argument passed)
#     If sig is None it is a read only property
DBusProp = namedtuple('DBusProp', ['name', 'sig'])
DBusProp.__new__.__defaults__ = (None, None)


class DBusAccess(object):
    """Mixin for abstracting specific DBus access library."""
    def get_system_bus(self):
        return Gio.bus_get_sync(Gio.BusType.SYSTEM, None)

    def get_objectmanager(self, bus, name, path):
        return Gio.DBusObjectManagerClient.new_sync(
            bus, Gio.DBusObjectManagerClientFlags.NONE,
            name=name, object_path=path)

    def get_managedobjects(self, omhandle):
        return omhandle.get_objects()

    def get_interface(self, bus, service, path, iface):
        return Gio.DBusProxy.new_sync(
            bus, Gio.DBusProxyFlags.NONE, None,
            service, path, iface, None)

    def get_prop(self, handle, iface, name):
        return handle.Get('(ss)', iface, name)

    def set_prop(self, handle, iface, name, value, sig):
        return handle.Set(
            '(ssv)', iface, name, GLib.Variant(sig, value))


def interfaces_added(self, path, interfaces):
    if constants.DEVICE_INTERFACE in interfaces:
        print('Path {0}: Interfaces added - {1}'.format(path, interfaces))


def properties_changed(self, interface, changed, invalidated, path):
    """
    Properties Changed Callback.

    :param interface:
    :param changed:
    :param invalidated:
    :param path:
    :return:
    """
    if constants.DEVICE_INTERFACE in interface:
        for prop in changed:
            print('{}:{} Property {} new value {}'.format(
                  interface, path, prop, changed[prop]))


class DBusBluez(DBusAccess):
    """Representation of a Bluez DBus Object.

    Provides common functionality for DBus Objects.
    An alternative approach would be to restructure as a mixin.

    """
    _props = None  # Must initialise for get/set attr to work

    def __init__(self, path, iface=None, props=None):
        """Initialise the Bluez DBus Object.

        :param path: DBus path
        :param iface: Interface to use on the path
        :param props: Dictionary of DBusProps that describe
                      how to relate a python property name
                      to that found on DBus.
        """
        self.path = path
        self.iface = iface
        self._props = props

        # Handle to the System Bus
        self.bus = self.get_system_bus()

        # Handle to the Bluez Object Manager (Client)
        self._omclient = self.get_objectmanager(
            self.bus, constants.BLUEZ_SERVICE_NAME, '/')

        self._omproxy = self.get_interface(
            self.bus, constants.BLUEZ_SERVICE_NAME,
            '/', constants.DBUS_OM_IFACE)

        # Handle to the properties on the given path
        self._propproxy = self.get_interface(
            self.bus, constants.BLUEZ_SERVICE_NAME,
            self.path, constants.DBUS_PROP_IFACE)

        # Handle to the interface on the given path (if specified)
        if self.iface is not None:
            self._ifaceproxy = self.get_interface(
                self.bus, constants.BLUEZ_SERVICE_NAME,
                self.path, self.iface)

    def subscribe(self, iface, signal, arg0, func):
        """Subscribe to the bus on the given iface."""
        # Subscribing to signals
        # Sender (None matches all), interface name, member (signal name),
        # object path (None matches all), arg0 (first string to match on),
        # flags, callback, user data to pass to the callback
        self.bus.signal_subscribe(
            None, iface, signal, None, arg0, Gio.DBusSignalFlags.NONE,
            lambda conn, sender, path, interface, sig, parameters:
            func(*parameters.unpack(), path) if sig == signal else None)

    def subspropchange(self, iface, func):
        """Subscribe to PropertiesChanged."""
        self.subscribe(constants.DBUS_PROP_IFACE, 'PropertiesChanged',
                       iface, func)

    def omconnect(self, signalname, func):
        """Connect func to signalname on the objectmanager."""
        # A Gio DBusProxy g-signal has the following arguments -
        # dbus proxy, sender name, signal name, parameters
        self._omproxy.connect(
            'g-signal', lambda sender, connection_name, signal, parameters:
            func(*parameters.unpack()) if signal == signalname else None)

    def get_objects(self):
        """Return Bluez DBus Managed Objects."""
        return self.get_managedobjects(self._omclient)

    def __getattr__(self, name):
        # check whether to get the property from DBus or the class object
        if self._props is not None and name in self._props.keys():
            # Doesn't work without an interface to ask on
            if self.iface is None:
                raise AdapterError('No Interface to get Properties on')
            else:
                pname = self._props[name].name
                try:
                    return self.get_prop(self._propproxy, self.iface, pname)
                except GLib.Error as e:
                    if e.code == 16:
                        return 'Property Not Implemented (Optional?)'
                    else:
                        raise(e)
        else:
            return super().__getattribute__(name)

    def __setattr__(self, name, value):
        # check whether to set the property on the DBus or in the class object
        if self._props is not None and name in self._props.keys():
            # Doesn't work without an interface to set on
            if self.iface is None:
                raise AdapterError('No Interface to set Properties on')
            else:
                sig = self._props[name].sig
                # Use "None" to indicate a read-only property
                if sig is None:
                    raise AttributeError(
                        'Attribute {0} is not settable'.format(name))
                else:
                    pname = self._props[name].name
                    return self.set_prop(
                        self._propproxy, self.iface, pname, value, sig)
        else:
            super().__setattr__(name, value)

    def __dir__(self):
        # Give DBus property names as well
        if self._props is not None:
            return super().__dir__() + list(self._props.keys())
        else:
            return super().__dir__()

    def __repr__(self):
        # Print the class and properties
        if self._props is not None:
            plist = self._props.keys()
            pval = self.__getattr__
            strprops = '\n'.join(
                ['\t{0}: {1}'.format(x, pval(x)) for x in plist])
            strheader = '{0} with the following properties:'.format(
                self.__class__)
            return '\n'.join([strheader, strprops])


class DBusAdapter(DBusBluez, EventLoop):
    """Representation of a Bluetooth Adapter."""

    def __init__(self, path):
        props = {
            'address': DBusProp('Address'),
            'addresstype': DBusProp('AddressType'),
            'name': DBusProp('Name'),
            'alias': DBusProp('Alias', 's'),
            'bt_class': DBusProp('Class'),
            'powered': DBusProp('Powered', 'b'),
            'discoverable': DBusProp('Discoverable', 'b'),
            'pairable': DBusProp('Pairable', 'b'),
            'pairabletimeout': DBusProp('PairableTimeout', 'u'),
            'discoverabletimeout': DBusProp('DiscoverableTimeout', 'u'),
            'discovering': DBusProp('Discovering',),
            'uuids': DBusProp('UUIDs'),
            'modalias': DBusProp('Modalias')
        }

        super(DBusAdapter, self).__init__(
            path, constants.ADAPTER_INTERFACE, props)
        EventLoop.__init__(self)

        # Adapted from bluezero Adapter
        self.omconnect('InterfacesAdded', interfaces_added)
        self.subspropchange(constants.DEVICE_INTERFACE, properties_changed)
        self._nearby_timeout = 10
        self._nearby_count = 0

    def _discovering_timeout(self):
        """Test to see if discovering should stop."""
        self._nearby_count += 1
        if self._nearby_count > self._nearby_timeout:
            self.stop_discovery()
            self.quit()
            return False
        return True

    def nearby_discovery(self, timeout=10):
        """Start discovery of nearby Bluetooth devices."""
        self._nearby_timeout = timeout
        self._nearby_count = 0

        self.add_timer(1000, self._discovering_timeout)
        self._ifaceproxy.StartDiscovery()
        self.run()

    def stop_discovery(self):
        """Stop scanning of nearby Bluetooth devices."""
        self._ifaceproxy.StopDiscovery()


def list_adapters():
    """Return list of adapters address available on system."""
    addresses = []
    dobj = DBusBluez('/')

    for obj in dobj.get_objects():
        for iface in obj.get_interfaces():
            if iface.get_interface_name() == 'org.bluez.Adapter1':
                adobj = DBusAdapter(iface.get_object_path())
                addresses.append(adobj.address)

    if len(addresses) < 1:
        raise AdapterError('No Bluetooth adapter found')
    else:
        return addresses


class DBusDevice(DBusBluez, EventLoop):
    """Representation of a Bluetooth Device."""

    def __init__(self, path):
        props = {
            'address': DBusProp('Address'),
            'addresstype': DBusProp('AddressType'),
            'name': DBusProp('Name'),
            'icon': DBusProp('Icon'),
            'bt_class': DBusProp('Class'),
            'appearance': DBusProp('Appearance'),
            'uuids': DBusProp('UUIDs'),
            'paired': DBusProp('Paired'),
            'connected': DBusProp('Connected'),
            'trusted': DBusProp('Trusted', 'b'),
            'blocked': DBusProp('Blocked', 'b'),
            'alias': DBusProp('Alias', 's'),
            'adapter': DBusProp('Adapter'),
            'legacy_pairing': DBusProp('LegacyPairing'),
            'modalias': DBusProp('Modalias'),
            'rssi': DBusProp('RSSI'),
            'tx_power': DBusProp('TxPower'),
            'manufacturer_data': DBusProp('ManufacturerData'),
            'service_data': DBusProp('ServiceData'),
            'services_resolved': DBusProp('ServicesResolved'),
            'advertising_flags': DBusProp('AdvertisingFlags')
        }

        super(DBusDevice, self).__init__(
            path, constants.DEVICE_INTERFACE, props)
        EventLoop.__init__(self)

    def connect(self, profile=None):
        """
        Initiate a connection to the remote device.

        :param profile: (optional) profile to use for the connection.
        """
        if profile is None:
            self._ifaceproxy.Connect()
        else:
            self._ifaceproxy.ConnectProfile(profile)

    def disconnect(self):
        """Disconnect from the remote device."""
        self._ifaceproxy.Disconnect()


if __name__ == '__main__':
    print(list_adapters())

    DBA = DBusAdapter('/org/bluez/hci0')
    print(DBA)
    # DBA.powered = True
    # print(DBA.powered)
    # DBA.powered = False
    # print(DBA.powered)
    # DBA.pairabletimeout = 10
    # print(DBA.pairabletimeout)
    # DBA.pairabletimeout = 0
    # print(DBA.pairabletimeout)
    # DBA.alias = 'hal1'
    # print(DBA.alias)
    # DBA.alias = 'neptune'
    # print(DBA.alias)

    DBA.powered = True
    # DBA.nearby_discovery()

    DBD = DBusDevice('/org/bluez/hci0/dev_DD_86_5F_ED_57_CE')
    # DBD.connect()
    print(DBD)
    print(DBD.trusted)
    DBD.trusted = True
    print(DBD.trusted)
    DBD.trusted = False
    print(DBD.trusted)
    # DBD.disconnect()
