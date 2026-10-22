import ctypes
import ctypes.util

from PIL import Image


class WindowAttributes(ctypes.Structure):
    _fields_ = [(name, ctypes.c_int) for name in ('x', 'y', 'width', 'height', 'border_width', 'depth')]
    _fields_ += [('visual', ctypes.c_void_p), ('root', ctypes.c_ulong)]
    _fields_ += [(name, ctypes.c_int) for name in ('window_class', 'bit_gravity', 'win_gravity', 'backing_store')]
    _fields_ += [('backing_planes', ctypes.c_ulong), ('backing_pixel', ctypes.c_ulong),
                ('save_under', ctypes.c_int), ('colormap', ctypes.c_ulong), ('map_installed', ctypes.c_int),
                ('map_state', ctypes.c_int), ('all_event_masks', ctypes.c_long), ('your_event_mask', ctypes.c_long),
                ('do_not_propagate_mask', ctypes.c_long), ('override_redirect', ctypes.c_int), ('screen', ctypes.c_void_p)]


class XImage(ctypes.Structure):
    _fields_ = [(name, ctypes.c_int) for name in ('width', 'height', 'xoffset', 'format')]
    _fields_ += [('data', ctypes.c_void_p)]
    _fields_ += [(name, ctypes.c_int) for name in ('byte_order', 'bitmap_unit', 'bitmap_bit_order', 'bitmap_pad',
                                               'depth', 'bytes_per_line', 'bits_per_pixel')]
    _fields_ += [('red_mask', ctypes.c_ulong), ('green_mask', ctypes.c_ulong), ('blue_mask', ctypes.c_ulong),
                ('obdata', ctypes.c_void_p)]


def fit_frame(source, output=(1200, 900)):
    scale = min(output[0] / source.width, output[1] / source.height)
    size = (max(1, round(source.width * scale)), max(1, round(source.height * scale)))
    resized = source.resize(size, Image.Resampling.BILINEAR)
    frame = Image.new('RGB', output)
    left, top = (output[0] - size[0]) // 2, (output[1] - size[1]) // 2
    frame.paste(resized, (left, top))
    return frame, dict(native_width=source.width, native_height=source.height,
                       content_rect=[left, top, left + size[0], top + size[1]], entire_window=True)


class FullWindow:
    def __init__(self, window):
        self.window = int(window, 0) if isinstance(window, str) else window
        self.library = ctypes.CDLL(ctypes.util.find_library('X11'))
        self.library.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.library.XOpenDisplay.restype = ctypes.c_void_p
        self.library.XGetWindowAttributes.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(WindowAttributes)]
        self.library.XGetImage.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
                                         ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_int]
        self.library.XGetImage.restype = ctypes.POINTER(XImage)
        self.library.XDestroyImage.argtypes = [ctypes.POINTER(XImage)]
        self.library.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.display = self.library.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('X11 display unavailable')

    def attributes(self):
        attributes = WindowAttributes()
        if not self.library.XGetWindowAttributes(self.display, self.window, ctypes.byref(attributes)):
            raise RuntimeError('Gazebo window missing')
        if attributes.width < 100 or attributes.height < 100 or attributes.map_state != 2:
            raise RuntimeError('Gazebo window not viewable')
        return attributes

    def read(self):
        before = self.attributes()
        pointer = self.library.XGetImage(self.display, self.window, 0, 0, before.width, before.height,
                                        ctypes.c_ulong(-1), 2)
        if not pointer:
            raise RuntimeError('X11 full-window capture failed')
        try:
            source = pointer.contents
            if source.bits_per_pixel != 32 or source.byte_order != 0 or (source.red_mask, source.green_mask, source.blue_mask) != (0xff0000, 0xff00, 0xff):
                raise RuntimeError('Unreviewed X11 pixel format')
            raw = ctypes.string_at(source.data, source.bytes_per_line * source.height)
            result = Image.frombytes('RGB', (source.width, source.height), raw, 'raw', 'BGRX', source.bytes_per_line, 1)
        finally:
            self.library.XDestroyImage(pointer)
        after = self.attributes()
        if (before.width, before.height) != (after.width, after.height):
            return self.read()
        return result

    def close(self):
        self.library.XCloseDisplay(self.display)
