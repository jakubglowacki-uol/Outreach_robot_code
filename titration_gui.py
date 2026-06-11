import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from Outreach_Core import OutreachDemo

try:
    import cv2
except Exception:
    cv2 = None

try:
    from PIL import Image, ImageTk
except Exception:
    Image = None
    ImageTk = None


INDICATOR_ML = 0.15


class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip_window = None
        self.widget.bind("<Enter>", self._show)
        self.widget.bind("<Leave>", self._hide)

    def _show(self, _event):
        if self.tip_window or not self.text:
            return

        x, y, _cx, cy = self.widget.bbox("insert") if self.widget.winfo_exists() else (0, 0, 0, 0)
        x += self.widget.winfo_rootx() + 20
        y += self.widget.winfo_rooty() + cy + 20

        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")

        label = tk.Label(
            tw,
            text=self.text,
            justify=tk.LEFT,
            background="#fff8dc",
            relief=tk.SOLID,
            borderwidth=1,
            padx=6,
            pady=4,
            wraplength=280,
        )
        label.pack()

    def _hide(self, _event):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None


class TitrationApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robotic Titration Teaching App")
        self.geometry("1220x760")

        self.demo = None
        self.initialized = False
        self.action_running = False

        self.totals = {
            "indicator": 0.0,
            "acid": 0.0,
            "base": 0.0,
        }

        self.current_frame = None
        self.cap = None
        self.camera_indices = []
        self.camera_running = False

        self.simulation_var = tk.BooleanVar(value=True)
        self.camera_var = tk.StringVar(value="0")

        self._build_ui()
        self._refresh_camera_list()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        container = ttk.Frame(self, padding=12)
        container.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(container)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))

        right = ttk.Frame(container)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        init_frame = ttk.LabelFrame(left, text="Session Setup", padding=10)
        init_frame.pack(fill=tk.X, pady=(0, 10))

        self.sim_check = ttk.Checkbutton(
            init_frame,
            text="Simulation mode (no robot/hotplate hardware calls)",
            variable=self.simulation_var,
        )
        self.sim_check.pack(anchor=tk.W)
        ToolTip(self.sim_check, "Enable for offline classroom demos. Hardware connections and motions are skipped.")

        self.init_button = ttk.Button(init_frame, text="Initialize Session", command=self._initialize_demo)
        self.init_button.pack(fill=tk.X, pady=(8, 4))

        self.shutdown_button = ttk.Button(init_frame, text="Shutdown Session", command=self._shutdown_demo, state=tk.DISABLED)
        self.shutdown_button.pack(fill=tk.X)

        control_frame = ttk.LabelFrame(left, text="Manual Additions", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        self.indicator_button = ttk.Button(control_frame, text="Add Indicator Drops", command=self._add_indicator, state=tk.DISABLED)
        self.indicator_button.pack(fill=tk.X, pady=(0, 6))
        ToolTip(self.indicator_button, "Adds a fixed small dose of universal indicator (0.15 mL).")

        acid_row = ttk.Frame(control_frame)
        acid_row.pack(fill=tk.X, pady=4)
        ttk.Label(acid_row, text="Acid (mL)").pack(side=tk.LEFT)
        self.acid_entry = ttk.Entry(acid_row, width=10)
        self.acid_entry.insert(0, "1.0")
        self.acid_entry.pack(side=tk.LEFT, padx=6)
        self.acid_button = ttk.Button(acid_row, text="Dispense Acid", command=self._add_acid, state=tk.DISABLED)
        self.acid_button.pack(side=tk.LEFT)
        ToolTip(self.acid_button, "Dispenses your entered acid volume. Educational range: 0-10 mL.")

        base_row = ttk.Frame(control_frame)
        base_row.pack(fill=tk.X, pady=4)
        ttk.Label(base_row, text="Base (mL)").pack(side=tk.LEFT)
        self.base_entry = ttk.Entry(base_row, width=10)
        self.base_entry.insert(0, "1.0")
        self.base_entry.pack(side=tk.LEFT, padx=6)
        self.base_button = ttk.Button(base_row, text="Dispense Base", command=self._add_base, state=tk.DISABLED)
        self.base_button.pack(side=tk.LEFT)
        ToolTip(self.base_button, "Dispenses your entered base volume. Educational range: 0-10 mL.")

        tally_frame = ttk.LabelFrame(left, text="Volume Tally", padding=10)
        tally_frame.pack(fill=tk.X, pady=(0, 10))

        self.indicator_label = ttk.Label(tally_frame, text="Indicator total: 0.00 mL")
        self.indicator_label.pack(anchor=tk.W)
        self.acid_label = ttk.Label(tally_frame, text="Acid total: 0.00 mL")
        self.acid_label.pack(anchor=tk.W)
        self.base_label = ttk.Label(tally_frame, text="Base total: 0.00 mL")
        self.base_label.pack(anchor=tk.W)
        self.total_label = ttk.Label(tally_frame, text="Overall total: 0.00 mL")
        self.total_label.pack(anchor=tk.W, pady=(6, 8))

        self.reset_button = ttk.Button(tally_frame, text="Reset Tally", command=self._reset_tally)
        self.reset_button.pack(fill=tk.X)
        ToolTip(self.reset_button, "Clears running totals to simplify repeat classroom trials.")

        self.status_label = ttk.Label(left, text="Status: Not initialized", foreground="#7a1f1f")
        self.status_label.pack(anchor=tk.W, pady=(6, 4))

        log_frame = ttk.LabelFrame(left, text="Action Log", padding=6)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=14, width=48, state=tk.DISABLED, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        cam_frame = ttk.LabelFrame(right, text="Camera + Approximate pH", padding=10)
        cam_frame.pack(fill=tk.BOTH, expand=True)

        cam_top = ttk.Frame(cam_frame)
        cam_top.pack(fill=tk.X)

        ttk.Label(cam_top, text="Camera").pack(side=tk.LEFT)
        self.camera_combo = ttk.Combobox(cam_top, textvariable=self.camera_var, width=8, state="readonly")
        self.camera_combo.pack(side=tk.LEFT, padx=(6, 6))

        self.refresh_cam_button = ttk.Button(cam_top, text="Refresh", command=self._refresh_camera_list)
        self.refresh_cam_button.pack(side=tk.LEFT, padx=(0, 6))

        self.open_cam_button = ttk.Button(cam_top, text="Open", command=self._open_camera)
        self.open_cam_button.pack(side=tk.LEFT, padx=(0, 6))

        self.close_cam_button = ttk.Button(cam_top, text="Close", command=self._close_camera, state=tk.DISABLED)
        self.close_cam_button.pack(side=tk.LEFT)

        self.video_label = ttk.Label(cam_frame, text="Camera preview will appear here.", anchor=tk.CENTER)
        self.video_label.pack(fill=tk.BOTH, expand=True, pady=10)

        ph_row = ttk.Frame(cam_frame)
        ph_row.pack(fill=tk.X)

        self.ph_label = ttk.Label(ph_row, text="Approximate pH: --")
        self.ph_label.pack(side=tk.LEFT)

        self.confidence_label = ttk.Label(ph_row, text="Confidence: --")
        self.confidence_label.pack(side=tk.LEFT, padx=(20, 0))

        self.color_patch = tk.Canvas(ph_row, width=44, height=20, highlightthickness=1, highlightbackground="#444")
        self.color_patch.pack(side=tk.LEFT, padx=(20, 0))
        self.color_patch.create_rectangle(0, 0, 44, 20, fill="#999999", outline="")

        educational_text = (
            "Educational note: pH is estimated from indicator color bands and lighting can shift results. "
            "Use this as a teaching aid, not an analytical measurement."
        )
        ttk.Label(cam_frame, text=educational_text, wraplength=520).pack(fill=tk.X, pady=(8, 0))

    def _log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _set_status(self, text, ok=False):
        self.status_label.configure(text=f"Status: {text}", foreground="#1b5e20" if ok else "#7a1f1f")

    def _set_controls_enabled(self, enabled):
        state = tk.NORMAL if enabled else tk.DISABLED
        self.indicator_button.configure(state=state)
        self.acid_button.configure(state=state)
        self.base_button.configure(state=state)

    def _initialize_demo(self):
        if self.initialized:
            return

        self.demo = OutreachDemo(simulation=self.simulation_var.get())
        self._set_status("Initializing...", ok=False)
        self._log("Initializing outreach session...")

        self.init_button.configure(state=tk.DISABLED)
        self.sim_check.configure(state=tk.DISABLED)

        def worker():
            try:
                self.demo.initialize()
                self.after(0, self._on_initialized)
            except Exception as exc:
                self.after(0, lambda: self._on_init_failed(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_initialized(self):
        self.initialized = True
        self._set_status("Ready", ok=True)
        self._set_controls_enabled(True)
        self.shutdown_button.configure(state=tk.NORMAL)
        self._log(
            "Session ready in {} mode.".format(
                "simulation" if self.simulation_var.get() else "hardware"
            )
        )

    def _on_init_failed(self, exc):
        self.init_button.configure(state=tk.NORMAL)
        self.sim_check.configure(state=tk.NORMAL)
        self._set_status("Initialization failed", ok=False)
        self._log(f"Initialization failed: {exc}")
        messagebox.showerror("Initialization Error", str(exc))

    def _shutdown_demo(self):
        if not self.demo:
            return

        self._set_status("Shutting down...", ok=False)
        self._set_controls_enabled(False)
        self.shutdown_button.configure(state=tk.DISABLED)

        def worker():
            try:
                self.demo.dispose()
            except Exception as exc:
                self.after(0, lambda: self._log(f"Shutdown warning: {exc}"))
            finally:
                self.after(0, self._on_shutdown_complete)

        threading.Thread(target=worker, daemon=True).start()

    def _on_shutdown_complete(self):
        self.initialized = False
        self.demo = None
        self.init_button.configure(state=tk.NORMAL)
        self.sim_check.configure(state=tk.NORMAL)
        self._set_status("Not initialized", ok=False)
        self._log("Session shut down.")

    def _run_action(self, action_name, action_callable, on_success):
        if not self.initialized or not self.demo:
            messagebox.showwarning("Not Ready", "Initialize the session before dispensing.")
            return

        if self.action_running:
            messagebox.showwarning("Busy", "Another action is currently running.")
            return

        self.action_running = True
        self._set_controls_enabled(False)
        self._set_status(f"Running: {action_name}", ok=False)
        self._log(f"Started action: {action_name}")

        def worker():
            try:
                action_callable()
                self.after(0, on_success)
                self.after(0, lambda: self._log(f"Completed action: {action_name}"))
            except Exception as exc:
                self.after(0, lambda: self._log(f"Action failed ({action_name}): {exc}"))
                self.after(0, lambda: messagebox.showerror("Action Error", str(exc)))
            finally:
                self.after(0, self._on_action_finished)

        threading.Thread(target=worker, daemon=True).start()

    def _on_action_finished(self):
        self.action_running = False
        if self.initialized:
            self._set_controls_enabled(True)
            self._set_status("Ready", ok=True)

    def _add_indicator(self):
        self._run_action(
            "Add Indicator",
            action_callable=lambda: self.demo.add_indicator(),
            on_success=lambda: self._add_to_tally("indicator", INDICATOR_ML),
        )

    def _add_acid(self):
        text = self.acid_entry.get().strip()

        def action():
            self.demo.add_acid(text)

        self._run_action(
            "Add Acid",
            action_callable=action,
            on_success=lambda: self._add_to_tally("acid", float(text)),
        )

    def _add_base(self):
        text = self.base_entry.get().strip()

        def action():
            self.demo.add_base(text)

        self._run_action(
            "Add Base",
            action_callable=action,
            on_success=lambda: self._add_to_tally("base", float(text)),
        )

    def _add_to_tally(self, key, value):
        self.totals[key] += value
        self._refresh_tally_labels()

    def _reset_tally(self):
        self.totals = {"indicator": 0.0, "acid": 0.0, "base": 0.0}
        self._refresh_tally_labels()
        self._log("Volume tally reset to zero.")

    def _refresh_tally_labels(self):
        indicator = self.totals["indicator"]
        acid = self.totals["acid"]
        base = self.totals["base"]
        total = indicator + acid + base

        self.indicator_label.configure(text=f"Indicator total: {indicator:.2f} mL")
        self.acid_label.configure(text=f"Acid total: {acid:.2f} mL")
        self.base_label.configure(text=f"Base total: {base:.2f} mL")
        self.total_label.configure(text=f"Overall total: {total:.2f} mL")

    def _refresh_camera_list(self):
        if cv2 is None:
            self.camera_indices = []
            self.camera_combo["values"] = []
            self._log("OpenCV not installed. Camera is unavailable.")
            return

        found = []
        for idx in range(6):
            cap = cv2.VideoCapture(idx)
            ok, _frame = cap.read()
            if ok:
                found.append(str(idx))
            cap.release()

        if not found:
            found = ["0"]

        self.camera_indices = found
        self.camera_combo["values"] = found
        if self.camera_var.get() not in found:
            self.camera_var.set(found[0])

    def _open_camera(self):
        if cv2 is None or Image is None or ImageTk is None:
            messagebox.showerror("Missing Dependencies", "Install opencv-python and Pillow to use camera features.")
            return

        self._close_camera()

        index = int(self.camera_var.get())
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            messagebox.showerror("Camera Error", f"Could not open camera index {index}.")
            return

        self.camera_running = True
        self.open_cam_button.configure(state=tk.DISABLED)
        self.close_cam_button.configure(state=tk.NORMAL)
        self._log(f"Camera {index} opened.")
        self._camera_loop()

    def _close_camera(self):
        self.camera_running = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            self._log("Camera closed.")

        self.open_cam_button.configure(state=tk.NORMAL)
        self.close_cam_button.configure(state=tk.DISABLED)
        self.video_label.configure(image="", text="Camera preview will appear here.")

    def _camera_loop(self):
        if not self.camera_running or self.cap is None:
            return

        ok, frame = self.cap.read()
        if ok:
            self.current_frame = frame
            ph_label, confidence, rgb = self._estimate_ph(frame)
            self.ph_label.configure(text=f"Approximate pH: {ph_label}")
            self.confidence_label.configure(text=f"Confidence: {confidence}")

            patch = "#{:02x}{:02x}{:02x}".format(rgb[0], rgb[1], rgb[2])
            self.color_patch.delete("all")
            self.color_patch.create_rectangle(0, 0, 44, 20, fill=patch, outline="")

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb_frame)
            image = image.resize((760, 520))
            tk_image = ImageTk.PhotoImage(image=image)
            self.video_label.configure(image=tk_image, text="")
            self.video_label.image = tk_image

        self.after(33, self._camera_loop)

    def _estimate_ph(self, frame):
        h, w, _ = frame.shape
        roi_size = 100
        y1 = max(0, (h // 2) - roi_size // 2)
        y2 = min(h, (h // 2) + roi_size // 2)
        x1 = max(0, (w // 2) - roi_size // 2)
        x2 = min(w, (w // 2) + roi_size // 2)

        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return "--", "Low", (127, 127, 127)

        avg_bgr = roi.mean(axis=(0, 1))
        avg_rgb = (int(avg_bgr[2]), int(avg_bgr[1]), int(avg_bgr[0]))

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue = float(hsv[:, :, 0].mean())
        sat = float(hsv[:, :, 1].mean())
        val = float(hsv[:, :, 2].mean())

        if sat < 35 and val > 120:
            ph_text = "6-8 (near neutral)"
        elif hue < 10 or hue >= 170:
            ph_text = "1-3 (acidic, red)"
        elif hue < 25:
            ph_text = "4-6 (orange/yellow)"
        elif hue < 75:
            ph_text = "7-8 (green)"
        elif hue < 130:
            ph_text = "9-11 (blue-green)"
        else:
            ph_text = "11-14 (blue-purple)"

        if sat >= 110 and 60 <= val <= 230:
            confidence = "High"
        elif sat >= 70:
            confidence = "Medium"
        else:
            confidence = "Low"

        return ph_text, confidence, avg_rgb

    def _on_close(self):
        self._close_camera()
        if self.demo is not None:
            try:
                self.demo.dispose()
            except Exception:
                pass
        self.destroy()


def main():
    app = TitrationApp()
    app.mainloop()


if __name__ == "__main__":
    main()
