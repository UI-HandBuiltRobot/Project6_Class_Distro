# --- generalised missing-dependency mock layer --------------------------
# Injected when the operator knows (or the agent has observed) that the
# target GUI depends on packages that aren't installed in this Python
# environment (rclpy, vendor SDKs, custom msg types, hardware drivers…).
#
# Approach: a meta-path finder at the END of sys.meta_path. Real installed
# packages resolve normally through the existing finders. Anything that
# would raise ModuleNotFoundError instead reaches our finder, which
# fabricates a stub module. Attribute access on a stub module auto-creates
# a stub class (subclass of _Any). The stub classes are subclassable, so
# code like ``class MyNode(rclpy.node.Node)`` still works.
#
# GUI-critical frameworks are on a safelist and NEVER stubbed — we rely on
# them actually being installed, and a failure on those is a real error.
#
# The driver marks widgets.json with ``capture_mode: "layout_surrogate_deps_stubbed"``
# and records which modules were stubbed so the README can disclose the
# provenance of the screenshot.
import sys as _sys
import os as _os
import types as _types
import importlib as _importlib
import importlib.abc as _importlib_abc
import importlib.util as _importlib_util


# Modules that MUST resolve to real installed packages. Anything matching
# these prefixes is skipped by the stub finder.
_REAL_REQUIRED = (
    # GUI frameworks
    "tkinter", "_tkinter",
    "PySide6", "PySide2", "PyQt6", "PyQt5", "shiboken6", "shiboken2", "sip",
    "pygame", "pygame_ce",
    "playwright",
    # Image + screen capture
    "PIL", "Pillow", "mss",
    # Std-library + essentials
    "sys", "os", "io", "time", "threading", "subprocess", "socket",
    "pathlib", "types", "importlib", "traceback", "re", "math",
    "functools", "collections", "itertools", "operator", "random",
    "dataclasses", "typing", "weakref", "gc", "atexit", "struct",
    "copy", "string", "unicodedata", "codecs", "encodings",
    "datetime", "calendar", "timeit", "logging",
    "enum", "contextlib", "warnings", "inspect", "pickle", "shelve",
    "xml", "json", "html", "csv", "configparser",
    "zipfile", "tarfile", "gzip", "bz2", "lzma", "shutil", "tempfile",
    "abc", "numbers", "signal", "select", "queue", "heapq", "bisect",
    "ctypes", "_ctypes", "platform", "locale", "getpass", "hashlib",
    "base64", "binascii", "secrets", "uuid", "urllib", "http",
    "email", "textwrap", "difflib", "argparse", "getopt",
    "asyncio", "concurrent", "multiprocessing",
    # NOTE: we do NOT safelist numpy, scipy, packaging, cv2, torch, etc. —
    # those are commonly target-app dependencies, and if they're genuinely
    # missing the stub layer should cover them so the GUI construction
    # still runs. If they ARE installed, normal finders resolve them before
    # our stub finder (which sits at the END of sys.meta_path).
)


def _is_real(name: str) -> bool:
    for req in _REAL_REQUIRED:
        if name == req or name.startswith(req + "."):
            return True
    return False


class _Any:
    """Universal stand-in for stubbed attributes: callable like a constructor,
    attribute-accessible to arbitrary depth, indexable, subclassable."""
    def __init__(self, *a, **kw):
        for k, v in kw.items():
            try:
                object.__setattr__(self, k, v)
            except Exception:
                pass
    def __getattr__(self, name):
        if name.startswith("__") and name.endswith("__"):
            raise AttributeError(name)
        v = _Any()
        try:
            object.__setattr__(self, name, v)
        except Exception:
            pass
        return v
    def __call__(self, *a, **kw):
        return _Any()
    def __iter__(self):
        return iter([])
    def __len__(self):
        return 0
    def __bool__(self):
        return False
    def __getitem__(self, k):
        return _Any()
    def __setitem__(self, k, v):
        pass
    def __contains__(self, k):
        return False
    def __eq__(self, other):
        return self is other
    def __hash__(self):
        return id(self)
    def __repr__(self):
        return "<auto_labeler stub>"


class _StubModule(_types.ModuleType):
    """A module where any attribute access creates a stub class on demand.
    Pretends to be a package (has __path__) so `import foo.bar` works when
    `foo` was stubbed."""
    def __getattr__(self, name):
        if name.startswith("__") and name.endswith("__"):
            raise AttributeError(name)
        cls = type(name, (_Any,), {"__module__": self.__name__})
        setattr(self, name, cls)
        return cls


_STUBBED_MODULES: set = set()


class _StubFinder(_importlib_abc.MetaPathFinder, _importlib_abc.Loader):
    """Last-resort finder: fabricates a stub module for any non-safelisted
    name that no other finder resolved."""
    def find_spec(self, fullname, path=None, target=None):
        if _is_real(fullname):
            return None
        return _importlib_util.spec_from_loader(fullname, self, is_package=True)
    def create_module(self, spec):
        mod = _StubModule(spec.name)
        mod.__path__ = []  # behaves as a package
        return mod
    def exec_module(self, module):
        _STUBBED_MODULES.add(module.__name__)


def _install_dep_stubs() -> bool:
    """Install the meta-path stubber and pre-populate common ROS types."""
    if any(isinstance(f, _StubFinder) for f in _sys.meta_path):
        return False

    # IMPORTANT: probe for real rclpy BEFORE installing the finder, otherwise
    # the finder would happily resolve `import rclpy` as a generic stub and
    # we'd skip the more-realistic pre-population of QoS enums etc.
    rclpy_real_available = False
    try:
        import rclpy  # noqa: F401
        rclpy_real_available = True
    except ImportError:
        rclpy_real_available = False

    _sys.meta_path.append(_StubFinder())

    if not rclpy_real_available:
        class _Logger:
            def info(self, *a, **kw): pass
            def warn(self, *a, **kw): pass
            def warning(self, *a, **kw): pass
            def error(self, *a, **kw): pass
            def debug(self, *a, **kw): pass
            def fatal(self, *a, **kw): pass

        class _Parameter:
            def __init__(self, name="", value=None):
                self._name, self._value = name, value
            @property
            def value(self): return self._value
            @property
            def name(self): return self._name
            def get_parameter_value(self):
                return _types.SimpleNamespace(
                    string_value=str(self._value) if self._value is not None else "",
                    integer_value=int(self._value) if isinstance(self._value, (int, float)) and not isinstance(self._value, bool) else 0,
                    double_value=float(self._value) if isinstance(self._value, (int, float)) and not isinstance(self._value, bool) else 0.0,
                    bool_value=bool(self._value) if self._value is not None else False,
                    type=0,
                )

        class _Node:
            def __init__(self, node_name="stub_node", *a, **kw):
                self._node_name = node_name
            def get_name(self): return self._node_name
            def get_logger(self): return _Logger()
            def get_clock(self): return _Any()
            def create_publisher(self, *a, **kw): return _Any()
            def create_subscription(self, *a, **kw): return _Any()
            def create_service(self, *a, **kw): return _Any()
            def create_client(self, *a, **kw): return _Any()
            def create_timer(self, *a, **kw): return _Any()
            def create_rate(self, *a, **kw): return _Any()
            def destroy_node(self): pass
            def declare_parameter(self, name, value=None, *a, **kw):
                return _Parameter(name, value)
            def declare_parameters(self, namespace, parameters, *a, **kw):
                return [_Parameter(n, v) for n, v in (parameters or [])]
            def get_parameter(self, name): return _Parameter(name, None)
            def get_parameter_or(self, name, default=None):
                return _Parameter(name, default.value if default else None)
            def set_parameters(self, *a, **kw): return []
            def add_on_set_parameters_callback(self, *a, **kw): pass
            def __getattr__(self, name): return _Any()

        rclpy = _types.ModuleType("rclpy")
        rclpy.init = lambda args=None, context=None: None
        rclpy.shutdown = lambda *a, **kw: None
        rclpy.ok = lambda *a, **kw: True
        rclpy.spin = lambda node, *a, **kw: None
        rclpy.spin_once = lambda node, *a, **kw: None
        rclpy.spin_until_future_complete = lambda *a, **kw: None
        rclpy.create_node = lambda name="stub", *a, **kw: _Node(name)
        rclpy.Node = _Node
        _sys.modules["rclpy"] = rclpy
        _STUBBED_MODULES.add("rclpy")

        node_mod = _types.ModuleType("rclpy.node"); node_mod.Node = _Node
        rclpy.node = node_mod; _sys.modules["rclpy.node"] = node_mod

        qos_mod = _types.ModuleType("rclpy.qos")
        class _QoSProfile:
            def __init__(self, *a, **kw): pass
        qos_mod.QoSProfile = _QoSProfile
        qos_mod.QoSReliabilityPolicy = _types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2, SYSTEM_DEFAULT=0)
        qos_mod.QoSDurabilityPolicy = _types.SimpleNamespace(VOLATILE=1, TRANSIENT_LOCAL=2, SYSTEM_DEFAULT=0)
        qos_mod.QoSHistoryPolicy = _types.SimpleNamespace(KEEP_LAST=1, KEEP_ALL=2, SYSTEM_DEFAULT=0)
        qos_mod.QoSLivelinessPolicy = _types.SimpleNamespace(AUTOMATIC=1, MANUAL_BY_TOPIC=3, SYSTEM_DEFAULT=0)
        rclpy.qos = qos_mod; _sys.modules["rclpy.qos"] = qos_mod

        class _Executor:
            def __init__(self, *a, **kw): pass
            def add_node(self, *a, **kw): pass
            def remove_node(self, *a, **kw): pass
            def spin(self): pass
            def spin_once(self, *a, **kw): pass
            def spin_until_future_complete(self, *a, **kw): pass
            def shutdown(self, *a, **kw): pass
        exec_mod = _types.ModuleType("rclpy.executors")
        exec_mod.SingleThreadedExecutor = _Executor
        exec_mod.MultiThreadedExecutor = _Executor
        exec_mod.Executor = _Executor
        rclpy.executors = exec_mod; _sys.modules["rclpy.executors"] = exec_mod

    # Sentinel + disclosure bookkeeping. The driver checks these when
    # emitting widgets.json so the README can flag the capture as a surrogate.
    _os.environ["AUTO_LABEL_DEPS_MOCKED"] = "1"
    sentinel = _types.ModuleType("__auto_labeler_dep_stubs__")
    sentinel.stubbed_modules = _STUBBED_MODULES
    _sys.modules["__auto_labeler_dep_stubs__"] = sentinel
    return True


_install_dep_stubs()
# --- end dependency mock layer -----------------------------------------

"""Driver for a Tkinter screen. Rendered from a template.

Template variables (rendered by auto_labeler before execution):
    IMPORT_LINE   e.g. "from myproject.ui import MainWindow"
    CONSTRUCT     e.g. "MainWindow(root)"
    SCREEN_NAME   human screen name
    OUT_DIR       absolute path where PNG + JSON are written
    SETUP_PRE     optional pre-construct setup (sys.path lines, env vars)

The driver writes into its output dir:
    screen.png      cropped to target window's rootx/rooty/width/height
    widgets.json    {image_width, image_height, scaling_factor, origin_x, origin_y, widgets: [...]}
Exits 0 on success, non-zero with traceback on stderr on failure.
"""
from __future__ import annotations

import json
import sys
import traceback
from pathlib import Path

# Make the process DPI-aware BEFORE Tk initializes. Without this, on a
# high-DPI Windows display Tk reports logical coords (96 DPI virtualization)
# while mss captures physical pixels — the two frames disagree, and the
# crop rectangle lands on a fraction of the real window surrounded by
# whatever else is physically behind it. SetProcessDpiAwareness(1) makes Tk
# query the real system DPI; widgets scale naturally and winfo_rootx/width
# return physical coords matching what mss grabs.
if sys.platform.startswith("win"):
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)  # System DPI aware
        except (OSError, AttributeError):
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

import sys
sys.path.insert(0, r"C:\GITHUB Repos\UofIHandBuiltRobot")



import tkinter as tk
from tkinter import ttk

# Stub all blocking OS-level dialogs so driver captures never hang waiting
# for a human to interact with a file-picker or message box.
try:
    from tkinter import filedialog as _fd, messagebox as _mb, simpledialog as _sd
    _fd.askopenfilename   = lambda *a, **kw: ""
    _fd.askopenfilenames  = lambda *a, **kw: ()
    _fd.asksaveasfilename = lambda *a, **kw: ""
    _fd.askdirectory      = lambda *a, **kw: ""
    _mb.showinfo = _mb.showwarning = _mb.showerror = lambda *a, **kw: "ok"
    _mb.askyesno = _mb.askokcancel = _mb.askquestion = lambda *a, **kw: True
    _mb.askretrycancel = _mb.askyesnocancel = lambda *a, **kw: True
    _sd.askstring  = lambda *a, **kw: ""
    _sd.askinteger = lambda *a, **kw: 0
    _sd.askfloat   = lambda *a, **kw: 0.0
except Exception:
    pass


_CONTROL = (
    tk.Button, tk.Checkbutton, tk.Radiobutton, tk.Entry, tk.Listbox,
    tk.Scale, tk.Spinbox, tk.Menubutton,
    ttk.Button, ttk.Checkbutton, ttk.Radiobutton, ttk.Entry, ttk.Combobox,
    ttk.Scale, ttk.Spinbox, ttk.Notebook,
)

# Display widgets: things that show information but do not accept input.
# We capture these too so the agent can label readouts, status fields, etc.
# tk.Text is included here even though it's technically editable — most apps
# use it as a readout.
_DISPLAY = (
    tk.Label, ttk.Label, tk.Message,
    ttk.Progressbar, tk.Canvas, tk.Text,
)


def _classify(widget):
    for cls in _CONTROL:
        if isinstance(widget, cls):
            return "control"
    for cls in _DISPLAY:
        if isinstance(widget, cls):
            return "display"
    return None  # skip pure layout widgets


def _safe_text(widget):
    try:
        opts = widget.keys()
    except Exception:
        return None
    for key in ("text", "label"):
        if key in opts:
            try:
                val = widget.cget(key)
                if val:
                    return str(val)
            except Exception:
                pass
    return None


def _callback_repr(widget):
    try:
        if "command" in widget.keys():
            cmd = widget.cget("command")
            return str(cmd) if cmd else None
    except Exception:
        return None
    return None


def _target_window(root):
    tops = [w for w in root.winfo_children() if isinstance(w, tk.Toplevel)]
    if tops:
        return tops[-1]
    return root


def _walk(widget, out):
    for child in widget.winfo_children():
        role = _classify(child)
        if role is not None:
            try:
                text = _safe_text(child)
                # Skip display widgets that carry no user-visible content —
                # those are structural padding, not readouts worth labeling.
                if role == "display" and not text and not isinstance(
                    child, (ttk.Progressbar, tk.Canvas)
                ):
                    pass
                else:
                    out.append({
                        "id": str(child),
                        "cls": type(child).__name__,
                        "text": text,
                        "x": child.winfo_rootx(),
                        "y": child.winfo_rooty(),
                        "w": child.winfo_width(),
                        "h": child.winfo_height(),
                        "callback": _callback_repr(child),
                        "role": role,
                    })
            except Exception:
                pass
        _walk(child, out)


def _bring_to_front_win32(hwnd):
    """Use SetWindowPos(TOPMOST)→(NOTOPMOST)+SetForegroundWindow to force
    the window above any other app that may be in front. Returns True if we
    at least made the call without exception — actual foreground state should
    still be verified afterward."""
    try:
        import ctypes
        user32 = ctypes.windll.user32
        HWND_TOPMOST, HWND_NOTOPMOST = -1, -2
        SWP_NOMOVE, SWP_NOSIZE = 0x0002, 0x0001
        user32.SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE)
        user32.SetWindowPos(hwnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE)
        user32.SetForegroundWindow(hwnd)
        return True
    except Exception:
        return False


def _image_has_content(png_bytes_or_path) -> bool:
    """Return True when the image has meaningful variance — not a solid
    fill that would indicate the compositor hasn't painted the window yet.
    Uses a cheap per-channel min/max check via PIL."""
    try:
        from PIL import Image
        img = Image.open(str(png_bytes_or_path)) if isinstance(png_bytes_or_path, Path) else Image.frombytes("RGB", (1,1), b"\x00\x00\x00")
        try:
            extrema = img.convert("RGB").getextrema()
        finally:
            img.close()
        # `extrema` is ((r_min, r_max), (g_min, g_max), (b_min, b_max))
        spread = max(hi - lo for (lo, hi) in extrema)
        return spread >= 16  # anything below this is essentially solid
    except Exception:
        return True  # don't fail the capture on a probe error


def _slug(text: str) -> str:
    out = "".join(c if c.isalnum() or c in "-_" else "_" for c in (text or "").strip())
    out = out.strip("_")[:48]
    return out or "untitled"


def _enumerate_peer_toplevels(primary):
    """Return list of (toplevel_widget, slug) for every visible Tk Toplevel
    that is NOT the primary. Uses the Tk widget hierarchy rather than a gc
    scan — O(children) vs O(all Python objects), 50-100× faster.
    """
    out: list = []
    seen = {id(primary)}
    try:
        # Collect all tk.Tk root windows visible to Python via _default_root
        # and the primary itself, then walk their .winfo_children().
        roots = set()
        try:
            dr = tk._default_root
            if dr is not None:
                roots.add(dr)
        except Exception:
            pass
        roots.add(primary)

        def _walk_children(widget):
            try:
                children = widget.winfo_children()
            except Exception:
                return
            for child in children:
                if id(child) in seen:
                    continue
                seen.add(id(child))
                try:
                    if not isinstance(child, tk.Toplevel):
                        _walk_children(child)
                        continue
                    if not child.winfo_exists() or not child.winfo_viewable():
                        continue
                    w, h = int(child.winfo_width()), int(child.winfo_height())
                    if w < 100 or h < 60:
                        continue
                    title = ""
                    try: title = child.title() or ""
                    except Exception: pass
                    out.append((child, _slug(title) or f"sub_{id(child) & 0xffff:x}"))
                except Exception:
                    continue
                _walk_children(child)

        for root in roots:
            if id(root) not in seen or root is primary:
                _walk_children(root)
    except Exception:
        pass
    return out


def _capture_one(target, out_dir, screen_name, scaling) -> dict:
    """Bring `target` to front, wait for paint, screenshot, walk widgets.
    Retries on solid-color output up to 4 times with increasing sleeps.
    Writes screen.png + widgets.json into out_dir. Returns metadata dict."""
    import time as _time
    import mss, mss.tools

    out_dir.mkdir(parents=True, exist_ok=True)

    # Bring to front + force foreground
    try:
        target.lift()
        target.focus_force()
        try: target.attributes("-topmost", True)
        except Exception: pass
        target.update_idletasks(); target.update()
    except Exception:
        pass
    if sys.platform.startswith("win"):
        _bring_to_front_win32(_hwnd_of(target))

    png_path = out_dir / "screen.png"
    foreground_ok = True
    # Patient render loop: up to 5 attempts with escalating sleep.
    # On each attempt: re-settle events, sample geometry, grab pixels,
    # verify image variance. Keep retrying if the screenshot is solid.
    last_w = last_h = 0
    for attempt, delay in enumerate([0.30, 0.45, 0.80, 1.25, 2.00]):
        _time.sleep(delay)
        try: target.update_idletasks(); target.update()
        except Exception: pass

        # Verify foreground
        if sys.platform.startswith("win"):
            try:
                import ctypes
                fg = ctypes.windll.user32.GetForegroundWindow()
                our = _hwnd_of(target)
                foreground_ok = bool(our) and (fg == our)
                if not foreground_ok and attempt < 3:
                    _bring_to_front_win32(our)
                    continue
            except Exception:
                pass

        try:
            target.update_idletasks()
            x = int(target.winfo_rootx())
            y = int(target.winfo_rooty())
            w = int(target.winfo_width())
            h = int(target.winfo_height())
        except Exception:
            x = y = 0; w = h = 200

        # Move off negative coords so mss crop succeeds
        if x < 0 or y < 0:
            try:
                target.geometry(f"+{max(0, x)}+{max(0, y)}")
                target.update_idletasks(); target.update()
                _time.sleep(0.15)
                x = max(0, int(target.winfo_rootx()))
                y = max(0, int(target.winfo_rooty()))
            except Exception:
                pass

        sw, sh = int(target.winfo_screenwidth()), int(target.winfo_screenheight())
        x = max(0, x); y = max(0, y)
        w = max(1, min(w, sw - x)); h = max(1, min(h, sh - y))

        with mss.mss() as sct:
            shot = sct.grab({"left": x, "top": y, "width": w, "height": h})
            mss.tools.to_png(shot.rgb, shot.size, output=str(png_path))

        if _image_has_content(png_path):
            last_w, last_h = w, h
            break
        # Else: solid color, try again with longer delay
    else:
        last_w, last_h = w, h  # give up but accept the last attempt

    widgets: list = []
    _walk(target, widgets)

    meta = {
        "screen_name": screen_name,
        "image_width": last_w,
        "image_height": last_h,
        "scaling_factor": scaling,
        "origin_x": x,
        "origin_y": y,
        "foreground_ok": foreground_ok,
        "widgets": widgets,
    }
    _dep_sentinel = sys.modules.get("__auto_labeler_dep_stubs__")
    if _dep_sentinel is not None:
        meta["capture_mode"] = "layout_surrogate_deps_stubbed"
        meta["stubbed_modules"] = sorted(getattr(_dep_sentinel, "stubbed_modules", []))
    (out_dir / "widgets.json").write_text(json.dumps(meta, indent=2))
    return meta


def _hwnd_of(widget):
    """Best-effort retrieval of the Windows HWND for a Tk widget's frame."""
    try:
        frame_id = widget.winfo_toplevel().wm_frame()
        return int(frame_id, 16) if isinstance(frame_id, str) else int(frame_id)
    except Exception:
        try:
            return int(widget.winfo_id())
        except Exception:
            return 0


def main():
    out_dir = Path(r"C:\\GITHUB Repos\\UofIHandBuiltRobot\\docs\\work\\robot_gui")
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        import time
        root = tk.Tk()
        try:
            root.tk.call("tk", "scaling")
        except Exception:
            pass

        import rclpy
        rclpy.init(args=[])

        from system_manager_package.gui.robot_gui import RobotGUINode
        import rclpy
        rclpy.init(args=[])
        node = RobotGUINode()

        # If the app constructed its own tk.Tk() (a very common pattern:
        # `class App: def __init__(self): self.root = tk.Tk()`), our
        # pre-created root is empty while the real UI lives under a
        # different Tk instance. tk._default_root always points at the
        # FIRST Tk() (i.e. our empty one), so we can't use that. Instead,
        # scan live Python objects for any other Tk instance with children.
        try:
            if not root.winfo_children():
                import gc as _gc
                replacement = None
                for obj in _gc.get_objects():
                    try:
                        if isinstance(obj, tk.Tk) and obj is not root:
                            if obj.winfo_children():
                                replacement = obj
                                break
                    except Exception:
                        continue
                if replacement is not None:
                    try:
                        root.destroy()
                    except Exception:
                        pass
                    root = replacement
        except Exception:
            pass

        root.update_idletasks()
        root.update()

        # Give the event loop a few ticks to settle (tabs, late layout, etc.)
        for _ in range(3):
            root.update_idletasks()
            root.update()

        target = _target_window(root)
        try:
            target.update_idletasks()
            target.update()
        except Exception:
            pass

        # --- Auto-grow window to fit all children --------------------------
        # Apps frequently call root.geometry("WxH") with a size smaller than
        # the sum of their widgets — Tk happily lays widgets out past the
        # window border, and they end up clipped in the screenshot. To make
        # sure every labelable widget actually appears in the image, we
        # measure the children's bounding box and grow the window to match.
        try:
            target.update_idletasks()
            probe = []
            _walk(target, probe)
            tgt_x0 = int(target.winfo_rootx())
            tgt_y0 = int(target.winfo_rooty())
            cur_w = int(target.winfo_width())
            cur_h = int(target.winfo_height())
            laid = [p for p in probe if int(p["w"]) > 1 and int(p["h"]) > 1]
            if laid:
                needed_w = max(int(p["x"]) + int(p["w"]) for p in laid) - tgt_x0 + 20
                needed_h = max(int(p["y"]) + int(p["h"]) for p in laid) - tgt_y0 + 20
                grow_w = max(cur_w, needed_w)
                grow_h = max(cur_h, needed_h)
                if grow_w > cur_w or grow_h > cur_h:
                    try:
                        # Lift any fixed-size constraints the app may have set.
                        target.maxsize(10_000, 10_000)
                    except Exception:
                        pass
                    target.geometry(f"{grow_w}x{grow_h}")
                    for _ in range(3):
                        target.update_idletasks()
                        target.update()
        except Exception:
            pass

        scaling = 1.0
        try:
            scaling = float(root.tk.call("tk", "scaling"))
        except Exception:
            pass

        # --- Capture primary window (patient render + variance verify) ---
        _capture_one(target, out_dir, "robot_gui", scaling)

        # --- Capture any additional visible Toplevels (modals, prefs) -----
        # Treat each peer window as its own screen. Write to sibling work
        # directory `<parent>/<screen>__<slug>/` so run_driver can discover
        # them without needing intermediate coordination.
        additional: list[str] = []
        for peer, slug in _enumerate_peer_toplevels(target):
            sub_name = f"robot_gui__{slug}"
            sub_dir = out_dir.parent / f"{out_dir.name}__{slug}"
            try:
                _capture_one(peer, sub_dir, sub_name, scaling)
                additional.append(sub_dir.name)
            except Exception:
                import traceback as _tb
                _tb.print_exc()
        if additional:
            (out_dir / "additional_screens.json").write_text(
                json.dumps(additional, indent=2), encoding="utf-8")

        try:
            root.destroy()
        except Exception:
            pass
        return 0
    except Exception:
        traceback.print_exc()
        return 2


if __name__ == "__main__":
    sys.exit(main())
