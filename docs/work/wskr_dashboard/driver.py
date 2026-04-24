"""Custom driver for WSKR Dashboard - layout surrogate for documentation."""

from __future__ import annotations
import json
import sys
import time

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
root.title('WSKR Dashboard')
root.geometry('1400x900')
root.configure(bg='#101010')

# Main grid container
grid = tk.Frame(root, bg='#101010')
grid.pack(fill=tk.BOTH, expand=True)
grid.rowconfigure(0, weight=1, uniform='row')
grid.rowconfigure(1, weight=1, uniform='row')
grid.columnconfigure(0, weight=1, uniform='col')
grid.columnconfigure(1, weight=1, uniform='col')

# Tile 1: ArUco Approach control (top-left)
approach_tile = tk.Frame(grid, bg='#101010', bd=1, relief=tk.FLAT)
approach_tile.grid(row=0, column=0, sticky='nsew', padx=4, pady=4)

tk.Label(
    approach_tile, text='ArUco Approach',
    bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
).pack(side=tk.TOP, fill=tk.X)

# Control bar
ctrl = tk.Frame(approach_tile, bg='#1e1e1e', pady=4)
ctrl.pack(side=tk.TOP, fill=tk.X)

tk.Label(
    ctrl, text='ArUco ID', bg='#1e1e1e', fg='white', font=('Arial', 12, 'bold'),
).pack(side=tk.LEFT, padx=(10, 4))

aruco_id_entry = tk.Entry(
    ctrl, width=4, font=('Arial', 20, 'bold'), justify='center',
    bg='#ffffe0', fg='black', relief=tk.SUNKEN, bd=2,
)
aruco_id_entry.insert(0, '1')
aruco_id_entry.pack(side=tk.LEFT, padx=4, ipady=2)

tag_status_label = tk.Label(
    ctrl, text='—', bg='#1e1e1e', fg='#ffcc00',
    font=('Arial', 11, 'bold'), anchor='w', width=26,
)
tag_status_label.pack(side=tk.LEFT, padx=8)

cancel_btn = tk.Button(
    ctrl, text='Cancel',
    bg='#c62828', fg='white', font=('Arial', 11, 'bold'), padx=10, pady=3,
)
cancel_btn.pack(side=tk.RIGHT, padx=(4, 10))

start_btn = tk.Button(
    ctrl, text='Start Approach',
    bg='#2e7d32', fg='white', font=('Arial', 11, 'bold'), padx=10, pady=3,
)
start_btn.pack(side=tk.RIGHT, padx=4)

approach_toy_btn = tk.Button(
    ctrl, text='Approach Toy',
    bg='#1565c0', fg='white', font=('Arial', 11, 'bold'), padx=10, pady=3,
)
approach_toy_btn.pack(side=tk.RIGHT, padx=4)

# Feedback and status labels
feedback_label = tk.Label(
    approach_tile, text='',
    bg='#1a2a1a', fg='#80ff80', font=('Consolas', 10), anchor='w',
)
feedback_label.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 3))

approach_status_label = tk.Label(
    approach_tile, text='Status: Ready',
    bg='#2a2a2a', fg='#80d0ff', font=('Arial', 10), anchor='w',
)
approach_status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(0, 1))

# Camera preview
cam_label = tk.Label(approach_tile, bg='black', text='Camera preview with\nArUco marker detection', fg='white', font=('Arial', 14))
cam_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Tile 2: WSKR Overlay (top-right)
overlay_tile = tk.Frame(grid, bg='#101010', bd=1, relief=tk.FLAT)
overlay_tile.grid(row=0, column=1, sticky='nsew', padx=4, pady=4)

tk.Label(
    overlay_tile, text='Consolidated WSKR Overlay (wskr_overlay/compressed)',
    bg='#202020', fg='#d0d0d0', font=('Arial', 11, 'bold'), anchor='w', padx=6,
).pack(side=tk.TOP, fill=tk.X)

overlay_label = tk.Label(overlay_tile, bg='black', text='Floor mask, whisker rays,\nheading arrow, telemetry', fg='white', font=('Arial', 14))
overlay_label.pack(fill=tk.BOTH, expand=True)

# Tile 3: Telemetry (bottom, full width)
stats_tile = tk.Frame(grid, bg='#101010')
stats_tile.grid(row=1, column=0, columnspan=2, sticky='nsew', padx=4, pady=4)

tk.Label(
    stats_tile, text='Telemetry', bg='#202020', fg='#d0d0d0',
    font=('Arial', 11, 'bold'), anchor='w', padx=6,
).pack(side=tk.TOP, fill=tk.X)

# Speed scale bar
speed_bar = tk.Frame(stats_tile, bg='#1e1e1e', pady=4)
speed_bar.pack(side=tk.TOP, fill=tk.X)

tk.Label(
    speed_bar, text='Speed scale', bg='#1e1e1e', fg='white',
    font=('Arial', 11, 'bold'),
).pack(side=tk.LEFT, padx=(10, 6))

speed_scale_var = tk.DoubleVar(value=1.0)
slider = tk.Scale(
    speed_bar, from_=0.0, to=1.0, resolution=0.01, orient=tk.HORIZONTAL,
    variable=speed_scale_var, length=260, showvalue=False,
    bg='#1e1e1e', fg='white', troughcolor='#333333',
    highlightthickness=0, sliderrelief=tk.FLAT,
)
slider.pack(side=tk.LEFT, padx=4)

speed_scale_value_label = tk.Label(
    speed_bar, text='1.00', bg='#1e1e1e', fg='#ffcc00',
    font=('Consolas', 12, 'bold'), width=5, anchor='w',
)
speed_scale_value_label.pack(side=tk.LEFT, padx=(4, 10))

apply_speed_btn = tk.Button(
    speed_bar, text='Apply',
    bg='#2e7d32', fg='white', font=('Arial', 11, 'bold'), padx=10, pady=2,
)
apply_speed_btn.pack(side=tk.LEFT, padx=4)

speed_scale_applied_label = tk.Label(
    speed_bar, text='applied: 1.00', bg='#1e1e1e', fg='#80d0ff',
    font=('Consolas', 11), anchor='w',
)
speed_scale_applied_label.pack(side=tk.LEFT, padx=10)

# Model bar
model_bar = tk.Frame(stats_tile, bg='#1e1e1e', pady=4)
model_bar.pack(side=tk.TOP, fill=tk.X)

tk.Label(
    model_bar, text='Model', bg='#1e1e1e', fg='white',
    font=('Arial', 11, 'bold'),
).pack(side=tk.LEFT, padx=(10, 6))

pick_model_btn = tk.Button(
    model_bar, text='Pick model…',
    bg='#2e7d32', fg='white', font=('Arial', 11, 'bold'), padx=10, pady=2,
)
pick_model_btn.pack(side=tk.LEFT, padx=4)

model_active_label = tk.Label(
    model_bar, text='active: (waiting…)', bg='#1e1e1e', fg='#80d0ff',
    font=('Consolas', 11), anchor='w',
)
model_active_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)

# Proximity bar
prox_bar = tk.Frame(stats_tile, bg='#1e1e1e', pady=4)
prox_bar.pack(side=tk.TOP, fill=tk.X)

tk.Label(
    prox_bar, text='Proximity (mm)', bg='#1e1e1e', fg='white',
    font=('Arial', 11, 'bold'),
).pack(side=tk.LEFT, padx=(10, 6))

tk.Label(prox_bar, text='min', bg='#1e1e1e', fg='#b0b0b0').pack(side=tk.LEFT)
prox_min_entry = tk.Entry(
    prox_bar, width=6, font=('Consolas', 11), justify='center',
    bg='#ffffe0', fg='black',
)
prox_min_entry.insert(0, '100')
prox_min_entry.pack(side=tk.LEFT, padx=(2, 8))

tk.Label(prox_bar, text='max', bg='#1e1e1e', fg='#b0b0b0').pack(side=tk.LEFT)
prox_max_entry = tk.Entry(
    prox_bar, width=6, font=('Consolas', 11), justify='center',
    bg='#ffffe0', fg='black',
)
prox_max_entry.insert(0, '500')
prox_max_entry.pack(side=tk.LEFT, padx=(2, 8))

apply_prox_btn = tk.Button(
    prox_bar, text='Apply',
    bg='#2e7d32', fg='white', font=('Arial', 11, 'bold'), padx=10, pady=2,
)
apply_prox_btn.pack(side=tk.LEFT, padx=4)

prox_applied_label = tk.Label(
    prox_bar, text='applied: —', bg='#1e1e1e', fg='#80d0ff',
    font=('Consolas', 11), anchor='w',
)
prox_applied_label.pack(side=tk.LEFT, padx=10)

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
import math
for i, angle in enumerate(range(-90, 91, 18)):
    x1, y1 = 600, 150
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
    "screen_name": "wskr_dashboard",
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
