# MicroPython Library runtime USB device implementation
#
# These contain the classes and utilities that are needed to
# implement a USB device, not any complete USB drivers.
#
# MIT license; Copyright (c) 2022-2024 Angus Gratton
from micropython import const
import machine
import struct

try:
    from _thread import get_ident
except ImportError:

    def get_ident():
        return 0  # Placeholder, for no threading support


_EP_IN_FLAG = const(1 << 7)

# USB descriptor types
_STD_DESC_DEV_TYPE = const(0x1)
_STD_DESC_CONFIG_TYPE = const(0x2)
_STD_DESC_STRING_TYPE = const(0x3)
_STD_DESC_INTERFACE_TYPE = const(0x4)
_STD_DESC_ENDPOINT_TYPE = const(0x5)
_STD_DESC_INTERFACE_ASSOC = const(0xB)

_ITF_ASSOCIATION_DESC_TYPE = const(0xB)  # Interface Association descriptor

# Standard USB descriptor lengths
_STD_DESC_CONFIG_LEN = const(9)
_STD_DESC_ENDPOINT_LEN = const(7)
_STD_DESC_INTERFACE_LEN = const(9)

_DESC_OFFSET_LEN = const(0)
_DESC_OFFSET_TYPE = const(1)

_DESC_OFFSET_INTERFACE_NUM = const(2)  # for _STD_DESC_INTERFACE_TYPE
_DESC_OFFSET_ENDPOINT_NUM = const(2)  # for _STD_DESC_ENDPOINT_TYPE

# Standard control request bmRequest fields, can extract by calling split_bmRequestType()
_REQ_RECIPIENT_DEVICE = const(0x0)
_REQ_RECIPIENT_INTERFACE = const(0x1)
_REQ_RECIPIENT_ENDPOINT = const(0x2)
_REQ_RECIPIENT_OTHER = const(0x3)

# Offsets into the standard configuration descriptor, to fixup
_OFFS_CONFIG_iConfiguration = const(6)

_INTERFACE_CLASS_VENDOR = const(0xFF)
_INTERFACE_SUBCLASS_NONE = const(0x00)
_PROTOCOL_NONE = const(0x00)

# These need to match the constants in tusb_config.h
_USB_STR_MANUF = const(0x01)
_USB_STR_PRODUCT = const(0x02)
_USB_STR_SERIAL = const(0x03)

# Error constant to match mperrno.h
_MP_EINVAL = const(22)

_dev = None  # Singleton _Device instance


def get():
    # Getter to access the singleton instance of the MicroPython _Device object
    global _dev
    if not _dev:
        _dev = _Device()
    return _dev


class _Device:
    def __init__(self):
        self._itfs = {}
        self._eps = {}
        self._ep_cbs = {}
        self._cb_thread = None
        self._cb_ep = None
        self._usbd = machine.USBDevice()

    def init(self, *itfs, **kwargs):
        self.active(False)
        self.config(*itfs, **kwargs)
        self.active(True)

    def config(
        self,
        *itfs,
        builtin_driver=False,
        manufacturer_str=None,
        product_str=None,
        serial_str=None,
        configuration_str=None,
        id_vendor=None,
        id_product=None,
        bcd_device=None,
        device_class=0,
        device_subclass=0,
        device_protocol=0,
        config_str=None,
        max_power_ma=None,
        remote_wakeup=False,
    ):
        _usbd = self._usbd

        if self.active():
            raise OSError(_MP_EINVAL)

        if isinstance(builtin_driver, bool):
            builtin_driver = _usbd.BUILTIN_DEFAULT if builtin_driver else _usbd.BUILTIN_NONE
        _usbd.builtin_driver = builtin_driver

        strs = [None, manufacturer_str, product_str, serial_str]

        FMT = "<BBHBBBBHHHBBBB"
        f = struct.unpack(FMT, builtin_driver.desc_dev)

        def maybe_set(value, idx):
            if value is not None:
                return value
            return f[idx]

        desc_dev = struct.pack(
            FMT,
            f[0],
            f[1],
            f[2],
            device_class,
            device_subclass,
            device_protocol,
            f[6],
            maybe_set(id_vendor, 7),
            maybe_set(id_product, 8),
            maybe_set(bcd_device, 9),
            _USB_STR_MANUF,
            _USB_STR_PRODUCT,
            _USB_STR_SERIAL,
            1,
        )

        itf_num = builtin_driver.itf_max
        ep_num = max(builtin_driver.ep_max, 1)
        while len(strs) < builtin_driver.str_max:
            strs.append(None)
        initial_cfg = builtin_driver.desc_cfg or (b"\x00" * _STD_DESC_CONFIG_LEN)

        self._itfs = {}

        desc = Descriptor(None)
        desc.extend(initial_cfg)
        for itf in itfs:
            itf.desc_cfg(desc, 0, 0, [])

        desc = Descriptor(bytearray(desc.o))
        desc.extend(initial_cfg)
        for itf in itfs:
            itf.desc_cfg(desc, itf_num, ep_num, strs)

            for _ in range(itf.num_itfs()):
                self._itfs[itf_num] = itf
                itf_num += 1

            ep_num += itf.num_eps()

        bmAttributes = (
            (1 << 7)
            | (0 if max_power_ma else (1 << 6))
            | ((1 << 5) if remote_wakeup else 0)
        )

        iConfiguration = 0
        if configuration_str:
            iConfiguration = len(strs)
            strs.append(configuration_str)

        if max_power_ma is not None:
            max_power_ma //= 2
        else:
            try:
                max_power_ma = _usbd.BUILTIN_DEFAULT.desc_cfg[8]
            except IndexError:
                max_power_ma = 125

        desc.pack_into(
            "<BBHBBBBB",
            0,
            _STD_DESC_CONFIG_LEN,
            _STD_DESC_CONFIG_TYPE,
            len(desc.b),
            itf_num,
            1,
            iConfiguration,
            bmAttributes,
            max_power_ma,
        )

        _usbd.config(
            desc_dev,
            desc.b,
            strs,
            self._open_itf_cb,
            self._reset_cb,
            self._control_xfer_cb,
            self._xfer_cb,
        )

    def active(self, *optional_value):
        return self._usbd.active(*optional_value)

    def _open_itf_cb(self, desc):
        itf_num = desc[_DESC_OFFSET_INTERFACE_NUM]
        itf = self._itfs[itf_num]

        offs = 0
        max_itf = itf_num
        while offs < len(desc):
            dl = desc[offs + _DESC_OFFSET_LEN]
            dt = desc[offs + _DESC_OFFSET_TYPE]
            if dt == _STD_DESC_ENDPOINT_TYPE:
                ep_addr = desc[offs + _DESC_OFFSET_ENDPOINT_NUM]
                self._eps[ep_addr] = itf
                self._ep_cbs[ep_addr] = None
            elif dt == _STD_DESC_INTERFACE_TYPE:
                max_itf = max(max_itf, desc[offs + _DESC_OFFSET_INTERFACE_NUM])
            offs += dl

        if self._itfs.get(max_itf + 1, None) != itf:
            itf.on_open()

    def _reset_cb(self):
        for itf in self._itfs.values():
            itf.on_reset()

        self._eps = {}
        self._ep_cbs = {}

    def _submit_xfer(self, ep_addr, data, done_cb=None):
        if ep_addr not in self._eps:
            raise ValueError("ep_addr")
        if self._xfer_pending(ep_addr):
            raise RuntimeError("xfer_pending")

        self._ep_cbs[ep_addr] = done_cb or True
        return self._usbd.submit_xfer(ep_addr, data)

    def _xfer_pending(self, ep_addr):
        return self._ep_cbs[ep_addr] or (
            self._cb_ep == ep_addr and self._cb_thread != get_ident()
        )

    def _xfer_cb(self, ep_addr, result, xferred_bytes):
        cb = self._ep_cbs.get(ep_addr, None)
        self._cb_thread = get_ident()
        self._cb_ep = ep_addr
        self._ep_cbs[ep_addr] = None

        try:
            if callable(cb):
                cb(ep_addr, result, xferred_bytes)
        finally:
            self._cb_ep = None

    def _control_xfer_cb(self, stage, request):
        wIndex = request[4] + (request[5] << 8)
        recipient, _, _ = split_bmRequestType(request[0])

        itf = None
        result = None

        if recipient == _REQ_RECIPIENT_DEVICE:
            itf = self._itfs.get(wIndex & 0xFFFF, None)
            if itf:
                result = itf.on_device_control_xfer(stage, request)
        elif recipient == _REQ_RECIPIENT_INTERFACE:
            itf = self._itfs.get(wIndex & 0xFFFF, None)
            if itf:
                result = itf.on_interface_control_xfer(stage, request)
        elif recipient == _REQ_RECIPIENT_ENDPOINT:
            ep_num = wIndex & 0xFFFF
            itf = self._eps.get(ep_num, None)
            if itf:
                result = itf.on_endpoint_control_xfer(stage, request)

        if not itf:
            raise RuntimeError(f"Unexpected control request type {request[0]:#x}")

        return result


class Interface:
    def __init__(self):
        self._open = False

    def desc_cfg(self, desc, itf_num, ep_num, strs):
        raise NotImplementedError

    def num_itfs(self):
        return 1

    def num_eps(self):
        return 0

    def on_open(self):
        self._open = True

    def on_reset(self):
        self._open = False

    def is_open(self):
        return self._open

    def on_device_control_xfer(self, stage, request):
        return False

    def on_interface_control_xfer(self, stage, request):
        return False

    def on_endpoint_control_xfer(self, stage, request):
        return False

    def xfer_pending(self, ep_addr):
        return _dev and _dev._xfer_pending(ep_addr)

    def submit_xfer(self, ep_addr, data, done_cb=None):
        if not self._open:
            raise RuntimeError("Not open")
        if not _dev._submit_xfer(ep_addr, data, done_cb):
            raise RuntimeError("DCD error")

    def stall(self, ep_addr, *args):
        if not self._open or ep_addr not in self._eps:
            raise RuntimeError
        _dev._usbd.stall(ep_addr, *args)


class Descriptor:
    def __init__(self, b):
        self.b = b
        self.o = 0

    def pack(self, fmt, *args):
        self.pack_into(fmt, self.o, *args)

    def pack_into(self, fmt, offs, *args):
        end = offs + struct.calcsize(fmt)
        if self.b:
            struct.pack_into(fmt, self.b, offs, *args)
        self.o = max(self.o, end)

    def extend(self, a):
        if self.b:
            self.b[self.o : self.o + len(a)] = a
        self.o += len(a)

    def interface(
        self,
        bInterfaceNumber,
        bNumEndpoints,
        bInterfaceClass=_INTERFACE_CLASS_VENDOR,
        bInterfaceSubClass=_INTERFACE_SUBCLASS_NONE,
        bInterfaceProtocol=_PROTOCOL_NONE,
        iInterface=0,
    ):
        self.pack(
            "BBBBBBBBB",
            _STD_DESC_INTERFACE_LEN,
            _STD_DESC_INTERFACE_TYPE,
            bInterfaceNumber,
            0,
            bNumEndpoints,
            bInterfaceClass,
            bInterfaceSubClass,
            bInterfaceProtocol,
            iInterface,
        )

    def endpoint(self, bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval=1):
        if bmAttributes == "control":
            bmAttributes = 0
        elif bmAttributes == "bulk":
            bmAttributes = 2
        elif bmAttributes == "interrupt":
            bmAttributes = 3

        self.pack(
            "<BBBBHB",
            _STD_DESC_ENDPOINT_LEN,
            _STD_DESC_ENDPOINT_TYPE,
            bEndpointAddress,
            bmAttributes,
            wMaxPacketSize,
            bInterval,
        )

    def interface_assoc(
        self,
        bFirstInterface,
        bInterfaceCount,
        bFunctionClass,
        bFunctionSubClass,
        bFunctionProtocol=_PROTOCOL_NONE,
        iFunction=0,
    ):
        self.pack(
            "<BBBBBBBB",
            8,
            _ITF_ASSOCIATION_DESC_TYPE,
            bFirstInterface,
            bInterfaceCount,
            bFunctionClass,
            bFunctionSubClass,
            bFunctionProtocol,
            iFunction,
        )


def split_bmRequestType(bmRequestType):
    return (
        bmRequestType & 0x1F,
        (bmRequestType >> 5) & 0x03,
        (bmRequestType >> 7) & 0x01,
    )


class Buffer:
    def __init__(self, length):
        self._b = memoryview(bytearray(length))
        self._n = 0
        self._w = length

    def writable(self):
        return len(self._b) - self._n

    def readable(self):
        return self._n

    def pend_write(self, wmax=None):
        self._w = self._n
        end = (self._w + wmax) if wmax else len(self._b)
        return self._b[self._w : end]

    def finish_write(self, nbytes):
        ist = machine.disable_irq()
        try:
            assert nbytes <= len(self._b) - self._w
            if self._n == self._w:
                self._n += nbytes
            else:
                while nbytes > 0:
                    self._b[self._n] = self._b[self._w]
                    self._n += 1
                    self._w += 1
                    nbytes -= 1

            self._w = len(self._b)
        finally:
            machine.enable_irq(ist)

    def write(self, w):
        pw = self.pend_write()
        to_w = min(len(w), len(pw))
        if to_w:
            pw[:to_w] = w[:to_w]
            self.finish_write(to_w)
        return to_w

    def pend_read(self):
        return self._b[: self._n]

    def finish_read(self, nbytes):
        if not nbytes:
            return
        ist = machine.disable_irq()
        try:
            assert nbytes <= self._n
            i = 0
            self._n -= nbytes
            while i < self._n:
                self._b[i] = self._b[i + nbytes]
                i += 1
        finally:
            machine.enable_irq(ist)

    def readinto(self, b):
        pr = self.pend_read()
        to_r = min(len(pr), len(b))
        if to_r:
            b[:to_r] = pr[:to_r]
            self.finish_read(to_r)
        return to_r
