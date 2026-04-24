"""Custom driver for Robot State GUI - layout surrogate for documentation."""

from __future__ import annotations
import json
import sys
import time
import threading

# DPI awareness for Windows
if sys.platform.startswith("win"):
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (OSError, AttributeError):
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

import tkinter as tk
from tkinter import ttk

# Create the main window
root = tk.Tk()
root.title('Robot State GUI — Debugging Dashboard')
root.geometry('1400x900')
root.configure(bg='#101010')

# Main grid container
grid = tk.Frame(root, bg='#101010')
grid.pack(fill=tk.BOTH, expand=True)
grid.rowconfigure(0, weight=1, uniform='row')
grid.rowconfigure(1, weight=1, uniform='row')
grid.columnconfigure(0, weight=1, uniform='col')
grid.columnconfigure(1, weight=1, uniform='col')

# Tile 1: Vision Stream (top-left)
vision_tile = tk.Frame(grid, bg='#101010', bd=1, relief=tk.FLAT)
vision_tile.grid(row=0, column=0, sticky='nsew', padx=4, pady=4)

tk.Label(
    vision_tile, text='Vision Stream (Camera Feed)',
    bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
).pack(side=tk.TOP, fill=tk.X)

cam_label = tk.Label(vision_tile, bg='black', text='Camera feed with\nobject detection overlays', fg='white', font=('Arial', 14))
cam_label.pack(fill=tk.BOTH, expand=True)

# Tile 2: WSKR Overlay (top-right)
overlay_tile = tk.Frame(grid, bg='#101010', bd=1, relief=tk.FLAT)
overlay_tile.grid(row=0, column=1, sticky='nsew', padx=4, pady=4)

tk.Label(
    overlay_tile, text='WSKR Overlay (Whiskers & Heading)',
    bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
).pack(side=tk.TOP, fill=tk.X)

overlay_label = tk.Label(overlay_tile, bg='black', text='Floor mask, whisker rays,\nheading arrow', fg='white', font=('Arial', 14))
overlay_label.pack(fill=tk.BOTH, expand=True)

# Tile 3: State & Telemetry (bottom, full width)
stats_tile = tk.Frame(grid, bg='#101010')
stats_tile.grid(row=1, column=0, columnspan=2, sticky='nsew', padx=4, pady=4)

tk.Label(
    stats_tile, text='Robot State & Telemetry',
    bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
).pack(side=tk.TOP, fill=tk.X)

# State display bar
state_bar = tk.Frame(stats_tile, bg='#1e1e1e', pady=4)
state_bar.pack(side=tk.TOP, fill=tk.X)

tk.Label(
    state_bar, text='State:', bg='#1e1e1e', fg='white',
    font=('Arial', 11, 'bold'),
).pack(side=tk.LEFT, padx=(10, 6))

state_label = tk.Label(
    state_bar, text='IDLE', bg='#1e1e1e', fg='#ffcc00',
    font=('Consolas', 14, 'bold'), anchor='w', width=20,
)
state_label.pack(side=tk.LEFT, padx=6)

tk.Label(
    state_bar, text='Search Phase:', bg='#1e1e1e', fg='white',
    font=('Arial', 11, 'bold'),
).pack(side=tk.LEFT, padx=(20, 6))

search_phase_label = tk.Label(
    state_bar, text='—', bg='#1e1e1e', fg='#80d0ff',
    font=('Consolas', 12), anchor='w', width=15,
)
search_phase_label.pack(side=tk.LEFT, padx=6)

# Telemetry canvas
numeric_canvas = tk.Canvas(stats_tile, bg='#181818', highlightthickness=0)
numeric_canvas.pack(fill=tk.BOTH, expand=True)

# Draw some sample telemetry items on the canvas
numeric_canvas.create_text(100, 30, text="Heading to Target:", fill="#ffffff", font=('Arial', 10, 'bold'), anchor='w')
numeric_canvas.create_text(220, 30, text="45.2°", fill="#ffcc00", font=('Consolas', 12), anchor='w')
numeric_canvas.create_text(100, 55, text="Tracking Mode:", fill="#ffffff", font=('Arial', 10, 'bold'), anchor='w')
numeric_canvas.create_text(220, 55, text="visual", fill="#80ff80", font=('Consolas', 12), anchor='w')
numeric_canvas.create_text(100, 80, text="cmd_vel:", fill="#ffffff", font=('Arial', 10, 'bold'), anchor='w')
numeric_canvas.create_text(220, 80, text="Vx=0.15 Vy=0.00 ω=0.05", fill="#80d0ff", font=('Consolas', 12), anchor='w')

# Draw whisker fan diagram
for i, angle in enumerate(range(-90, 91, 18)):
    x1, y1 = 600, 150
    import math
    angle_rad = math.radians(angle)
    x2 = x1 + 80 * math.cos(angle_rad)
    y2 = y1 - 80 * math.sin(angle_rad)
    color = f"#{100+i*15:02x}{200-i*10:02x}{100+i*5:02x}"
    numeric_canvas.create_line(x1, y1, x2, y2, fill=color, width=2)
numeric_canvas.create_text(600, 150, text="●", fill="#ffffff", font=('Arial', 12))

# Update the window
root.update_idletasks()
root.update()

# --- Capture logic ---
import mss
import mss.tools

# Get window geometry
root.update_idletasks()
root.update()

# Wait a bit for window to fully render
time.sleep(0.5)
root.update_idletasks()
root.update()

# Get window position and size
x = root.winfo_rootx()
y = root.winfo_rooty()
w = root.winfo_width()
h = root.winfo_height()

# Ensure minimum size
w = max(w, 1400)
h = max(h, 900)

# Bring window to front
root.lift()
root.focus_force()
try:
    root.attributes("-topmost", True)
except Exception:
    pass
root.update_idletasks()
root.update()

time.sleep(0.3)
root.update_idletasks()
root.update()

# Capture the window
with mss.mss() as sct:
    monitor = {"top": y, "left": x, "width": w, "height": h}
    output = sct.grab(monitor)
    mss.tools.to_png(output.rgb, output.size, output="screen.png")

# Enumerate widgets
widgets = []
widget_id = 0

def walk(widget, depth=0):
    global widget_id
    try:
        children = widget.winfo_children()
    except Exception:
        children = []
    
    # Classify this widget
    from tkinter import Button, Checkbutton, Radiobutton, Entry, Listbox, Scale, Spinbox, Menubutton
    from tkinter.ttk import Button as TtkButton, Checkbutton as TtkCheckbutton, Radiobutton as TtkRadiobutton, Entry as TtkEntry, Combobox, Scale as TtkScale, Spinbox as TtkSpinbox, Notebook
    
    _CONTROL = (Button, Checkbutton, Radiobutton, Entry, Listbox, Scale, Spinbox, Menubutton,
                TtkButton, TtkCheckbutton, TtkRadiobutton, TtkEntry, Combobox, TtkScale, TtkSpinbox, Notebook)
    _DISPLAY = (tk.Label, ttk.Label, tk.Message, ttk.Progressbar, tk.Canvas, tk.Text)
    
    role = None
    for cls in _CONTROL:
        if isinstance(widget, cls):
            role = "control"
            break
    if role is None:
        for cls in _DISPLAY:
            if isinstance(widget, cls):
                role = "display"
                break
    
    if role is not None:
        try:
            wx = widget.winfo_rootx()
            wy = widget.winfo_rooty()
            ww = widget.winfo_width()
            wh = widget.winfo_height()
            
            if ww > 5 and wh > 5:  # Filter out tiny widgets
                text = ""
                try:
                    if "text" in widget.keys():
                        text = str(widget.cget("text"))
                    elif "label" in widget.keys():
                        text = str(widget.cget("label"))
                except Exception:
                    pass
                
                widgets.append({
                    "id": f"widget_{widget_id}",
                    "cls": widget.__class__.__name__,
                    "text": text[:50] if text else "",
                    "x": wx,
                    "y": wy,
                    "w": ww,
                    "h": wh,
                    "role": role,
                })
                widget_id += 1
        except Exception:
            pass
    
    for child in children:
        walk(child, depth + 1)

walk(root)

# Save widgets.json
result = {
    "screen_name": "robot_gui",
    "image_width": w,
    "image_height": h,
    "scaling_factor": 1.0,
    "origin_x": x,
    "origin_y": y,
    "foreground_ok": True,
    "widgets": widgets,
    "capture_mode": "layout_surrogate_docs",
}

with open("widgets.json", "w") as f:
    json.dump(result, f, indent=2)

print(f"Captured {len(widgets)} widgets")
root.destroy()
