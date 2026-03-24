#!/usr/bin/env python3
"""
Banco de Identificación de Sistemas - HMI Desktop v4.0
Atacama Dynamics

Protocolo binario puro (sin JSON):
  PC <-- USB/UART 460800 --> ESP32 (bypass) <-- SPI --> ATmega328P

Recepción: TelemetryPacket de 22 bytes crudos
Envío:     3 bytes crudos [CMD, VAL_H, VAL_L]

Modos de prueba (lazo abierto):
  - DC (Master + Slave simultáneo)
  - AC Master solo / Slave solo / Ambos
"""

import tkinter as tk
from tkinter import ttk, messagebox
import struct
import time
import threading
import csv
import os
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Intento de importar Pillow para el Logo
try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ============================================================================
# CONSTANTES DE PROTOCOLO Y UI
# ============================================================================
PKT_SIZE = 22
PKT_START = 0xAA
PKT_FORMAT = '<B I h h H H h h H H B'  # 22 bytes

# Comandos (3 bytes: CMD + VAL_H + VAL_L)
CMD_DC_BOTH    = 0x74
CMD_AC_MASTER  = 0x72
CMD_AC_SLAVE   = 0x73
CMD_AC_BOTH    = 0x75
CMD_STOP_ALL   = 0x76

# Paleta de colores Modern Light Flat (Soft UI)
BG_MAIN  = "#FFFFFF"
BG_PANEL = "#F8FAFC"
BG_INPUT = "#FFFFFF"
FG_TEXT  = "#1E293B"
FG_DIM   = "#64748B"
C_BORDER = "#E2E8F0"
C_BLUE   = "#3B82F6"
C_GREEN  = "#10B981"
C_RED    = "#EF4444"
C_ORANGE = "#F97316"
C_CYAN   = "#06B6D4"


class SysIdHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("Atacama Dynamics - DAQ Identificación v4.0")
        self.root.geometry("1280x850")
        self.root.configure(bg=BG_MAIN)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # === Estado ===
        self.ser = None
        self.serial_thread = None
        self.serial_running = False
        self.is_recording = False
        self.data_log = []
        self.last_rx_time = 0
        self.connection_ok = False
        self._closing = False

        # Variables Tk
        self.selected_port = tk.StringVar()

        # Variables independientes DC y AC
        self.dc_value = tk.IntVar(value=0)
        self.ac_value = tk.IntVar(value=1500)
        self.dc_entry_var = tk.StringVar(value="0")
        self.ac_entry_var = tk.StringVar(value="1500")
        self.dc_active = False
        self.ac_active = False

        # Últimos valores recibidos
        self.last_ts_ms = 0
        self.last_mPWM = 0
        self.last_sPWM = 0
        self.last_mESC = 1500
        self.last_sESC = 1500
        self.last_mRPM = 0
        self.last_sRPM = 0
        self.last_mHz = 0.0
        self.last_sHz = 0.0

        # Contadores de paquetes
        self.pkt_count = 0
        self.pkt_errors = 0

        # Buffer circular para gráfico en vivo (30s a ~100Hz)
        self.LIVE_BUFFER_SIZE = 6000
        self.live_times = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_mPWM  = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_sPWM  = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_mESC  = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_sESC  = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_mRPM  = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_sRPM  = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_mHz   = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_sHz   = deque(maxlen=self.LIVE_BUFFER_SIZE)
        self.live_t0_ms = None  # Primer timestamp_ms del ATmega

        self.setup_ui()
        self.refresh_ports()

    # ================================================================
    # UI SETUP (ESTILO LIGHT FLAT Y BORDES SUAVES)
    # ================================================================
    def create_header(self):
        header = tk.Frame(self.root, bg=BG_MAIN, height=80)
        header.pack(side=tk.TOP, fill=tk.X, pady=(10, 10))
        header.pack_propagate(False)

        self.logo_lbl = tk.Label(header, bg=BG_MAIN)
        self.logo_lbl.pack(side=tk.LEFT, padx=30, pady=5)
        self._load_logo()

        title_frame = tk.Frame(header, bg=BG_MAIN)
        title_frame.pack(side=tk.LEFT, padx=20, pady=10)
        tk.Label(title_frame, text="SISTEMA DAQ & IDENTIFICACIÓN",
                 font=("Segoe UI", 18, "bold"), bg=BG_MAIN, fg=FG_TEXT).pack(anchor=tk.W)
        tk.Label(title_frame, text="Banco de Pruebas de Lazo Abierto v4.0",
                 font=("Segoe UI", 10), bg=BG_MAIN, fg=FG_DIM).pack(anchor=tk.W)

    def _load_logo(self):
        if not HAS_PIL:
            self.logo_lbl.config(text="ATACAMA DYNAMICS", font=("Segoe UI", 16, "bold"), fg=C_BLUE)
            return

        script_dir = os.path.dirname(os.path.abspath(__file__))
        possible_names = ["logo.png", "Captura-de-pantalla-2025-07-15-162232.png"]

        for name in possible_names:
            path = os.path.join(script_dir, name)
            if os.path.exists(path):
                try:
                    img = Image.open(path)
                    ratio = 55.0 / img.height
                    new_size = (int(img.width * ratio), 55)
                    img = img.resize(new_size, Image.Resampling.LANCZOS)
                    self.logo_photo = ImageTk.PhotoImage(img)
                    self.logo_lbl.config(image=self.logo_photo)
                    return
                except Exception as e:
                    print(f"Error cargando logo: {e}")

        self.logo_lbl.config(text="ATACAMA DYNAMICS", font=("Segoe UI", 16, "bold"), fg=C_BLUE)

    def _create_flat_button(self, parent, text, bg_color, command, state=tk.NORMAL):
        btn = tk.Button(parent, text=text, bg=bg_color, fg="white",
                        font=("Segoe UI", 10, "bold"), command=command, state=state,
                        relief=tk.FLAT, bd=0, activebackground=bg_color, activeforeground="white",
                        cursor="hand2", pady=8, padx=10)
        return btn

    def _create_label_frame(self, parent, text, color=C_BLUE):
        lf = tk.LabelFrame(parent, text=f" {text} ", bg=BG_PANEL, fg=color,
                           font=("Segoe UI", 10, "bold"), bd=1, relief=tk.SOLID,
                           highlightbackground=C_BORDER, highlightthickness=1, padx=15, pady=15)
        return lf

    def setup_ui(self):
        style = ttk.Style()
        style.configure("TFrame", background=BG_MAIN)
        style.configure("TLabel", background=BG_PANEL, foreground=FG_TEXT,
                         font=("Segoe UI", 10))
        style.configure("TRadiobutton", background=BG_PANEL,
                         foreground=FG_TEXT, font=("Segoe UI", 10))

        self.create_header()

        main_frame = tk.Frame(self.root, bg=BG_MAIN)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=(0, 20))

        # --- PANEL IZQUIERDO ---
        ctrl = tk.Frame(main_frame, bg=BG_MAIN, width=340)
        ctrl.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 20))
        ctrl.pack_propagate(False)

        # ---- Conexión Serial ----
        conn_frame = self._create_label_frame(ctrl, "CONEXIÓN SERIAL")
        conn_frame.pack(fill=tk.X, pady=(0, 10))

        port_row = tk.Frame(conn_frame, bg=BG_PANEL)
        port_row.pack(fill=tk.X, pady=5)

        self.combo_port = ttk.Combobox(port_row,
                                        textvariable=self.selected_port,
                                        width=18, state="readonly",
                                        font=("Segoe UI", 10))
        self.combo_port.pack(side=tk.LEFT, padx=(0, 10))

        self._create_flat_button(port_row, "⟳", FG_DIM, self.refresh_ports).pack(side=tk.LEFT)

        self.btn_connect = self._create_flat_button(conn_frame, "CONECTAR", C_BLUE, self.toggle_connection)
        self.btn_connect.pack(fill=tk.X, pady=10)

        # ---- Control DC ----
        dc_frame = self._create_label_frame(ctrl, "MOTOR DC (PWM -255 a 255)", C_BLUE)
        dc_frame.pack(fill=tk.X, pady=10)

        dc_input_row = tk.Frame(dc_frame, bg=BG_PANEL)
        dc_input_row.pack(fill=tk.X, pady=5)
        self.dc_entry = tk.Entry(dc_input_row, textvariable=self.dc_entry_var,
                                 width=8, font=("Segoe UI", 14, "bold"),
                                 bg=BG_INPUT, fg=FG_TEXT, insertbackground=FG_TEXT,
                                 relief=tk.SOLID, bd=1, justify=tk.CENTER)
        self.dc_entry.pack(side=tk.LEFT, padx=(0, 10))
        self.dc_entry.bind("<Return>", self._dc_entry_changed)
        tk.Label(dc_input_row, text="(-255 a 255)", bg=BG_PANEL, fg=FG_DIM,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT)

        self.dc_slider = tk.Scale(dc_frame, from_=-255, to=255,
                                   orient=tk.HORIZONTAL,
                                   bg=BG_PANEL, fg=FG_TEXT,
                                   highlightthickness=0,
                                   troughcolor=C_BORDER,
                                   activebackground=C_BLUE,
                                   font=("Segoe UI", 8), showvalue=False,
                                   command=self._dc_slider_changed)
        self.dc_slider.set(0)
        self.dc_slider.pack(fill=tk.X, pady=10)

        dc_btn_row = tk.Frame(dc_frame, bg=BG_PANEL)
        dc_btn_row.pack(fill=tk.X, pady=5)
        self.btn_dc_send = self._create_flat_button(dc_btn_row, "▶ ENVIAR DC",
                                                      C_BLUE, self._send_dc, state=tk.DISABLED)
        self.btn_dc_send.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self.btn_dc_stop = self._create_flat_button(dc_btn_row, "⏹ STOP DC",
                                                      C_RED, self._stop_dc, state=tk.DISABLED)
        self.btn_dc_stop.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(4, 0))

        # ---- Control AC (ESC) ----
        ac_frame = self._create_label_frame(ctrl, "MOTOR AC / ESC (µs 1000-2000)", C_ORANGE)
        ac_frame.pack(fill=tk.X, pady=10)

        ac_target_row = tk.Frame(ac_frame, bg=BG_PANEL)
        ac_target_row.pack(fill=tk.X, pady=5)
        self.ac_target = tk.StringVar(value="AC_BOTH")
        for text, val in [("Ambos", "AC_BOTH"), ("Master", "AC_MASTER"),
                           ("Slave", "AC_SLAVE")]:
            ttk.Radiobutton(ac_target_row, text=text,
                            variable=self.ac_target,
                            value=val).pack(side=tk.LEFT, padx=5)

        ac_input_row = tk.Frame(ac_frame, bg=BG_PANEL)
        ac_input_row.pack(fill=tk.X, pady=5)
        self.ac_entry = tk.Entry(ac_input_row, textvariable=self.ac_entry_var,
                                 width=8, font=("Segoe UI", 14, "bold"),
                                 bg=BG_INPUT, fg=FG_TEXT, insertbackground=FG_TEXT,
                                 relief=tk.SOLID, bd=1, justify=tk.CENTER)
        self.ac_entry.pack(side=tk.LEFT, padx=(0, 10))
        self.ac_entry.bind("<Return>", self._ac_entry_changed)
        tk.Label(ac_input_row, text="(1000-2000 µs)", bg=BG_PANEL, fg=FG_DIM,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT)

        self.ac_slider = tk.Scale(ac_frame, from_=1000, to=2000,
                                   orient=tk.HORIZONTAL,
                                   bg=BG_PANEL, fg=FG_TEXT,
                                   highlightthickness=0,
                                   troughcolor=C_BORDER,
                                   activebackground=C_ORANGE,
                                   font=("Segoe UI", 8), showvalue=False,
                                   command=self._ac_slider_changed)
        self.ac_slider.set(1500)
        self.ac_slider.pack(fill=tk.X, pady=10)

        ac_btn_row = tk.Frame(ac_frame, bg=BG_PANEL)
        ac_btn_row.pack(fill=tk.X, pady=5)
        self.btn_ac_send = self._create_flat_button(ac_btn_row, "▶ ENVIAR AC",
                                                      C_ORANGE, self._send_ac, state=tk.DISABLED)
        self.btn_ac_send.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self.btn_ac_stop = self._create_flat_button(ac_btn_row, "⏹ STOP AC",
                                                      C_RED, self._stop_ac, state=tk.DISABLED)
        self.btn_ac_stop.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(4, 0))

        # ---- Grabación CSV ----
        rec_frame = self._create_label_frame(ctrl, "GRABACIÓN", C_GREEN)
        rec_frame.pack(fill=tk.X, pady=10)

        self.btn_rec = self._create_flat_button(rec_frame, "⏺ GRABAR CSV",
                                                  C_GREEN, self._toggle_recording, state=tk.DISABLED)
        self.btn_rec.pack(fill=tk.X)

        self.btn_stop_all = self._create_flat_button(rec_frame, "⏹ STOP TODO",
                                                       C_RED, self._stop_all, state=tk.DISABLED)
        self.btn_stop_all.pack(fill=tk.X, pady=(10, 0))

        # ---- Indicadores ----
        self.lbl_status = tk.Label(ctrl, text="DESCONECTADO",
                                    font=("Segoe UI", 11, "bold"),
                                    bg=BG_MAIN, fg=C_RED)
        self.lbl_status.pack(side=tk.BOTTOM, pady=10)

        self.lbl_warning = tk.Label(ctrl, text="", bg=BG_MAIN,
                                     fg=C_RED,
                                     font=("Segoe UI", 12, "bold"),
                                     wraplength=280)
        self.lbl_warning.pack(side=tk.BOTTOM, pady=5)

        # ---- Telemetría en Vivo ----
        live_frame = self._create_label_frame(ctrl, "TELEMETRÍA EN VIVO", C_CYAN)
        live_frame.pack(fill=tk.X, pady=(10, 0))

        self.lbl_live_ts   = tk.Label(live_frame, text="t(ms): ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_ts.pack(anchor=tk.W)
        self.lbl_live_mPWM = tk.Label(live_frame, text="M PWM: ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_mPWM.pack(anchor=tk.W)
        self.lbl_live_sPWM = tk.Label(live_frame, text="S PWM: ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_sPWM.pack(anchor=tk.W)
        self.lbl_live_mESC = tk.Label(live_frame, text="M ESC: ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_mESC.pack(anchor=tk.W)
        self.lbl_live_sESC = tk.Label(live_frame, text="S ESC: ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_sESC.pack(anchor=tk.W)
        self.lbl_live_mRPM = tk.Label(live_frame, text="M RPM: ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_mRPM.pack(anchor=tk.W)
        self.lbl_live_sRPM = tk.Label(live_frame, text="S RPM: ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_sRPM.pack(anchor=tk.W)
        self.lbl_live_mHz  = tk.Label(live_frame, text="M Hz:  ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_mHz.pack(anchor=tk.W)
        self.lbl_live_sHz  = tk.Label(live_frame, text="S Hz:  ---",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_live_sHz.pack(anchor=tk.W)

        # ---- Estadísticas ----
        stats_frame = self._create_label_frame(ctrl, "ESTADÍSTICAS", C_BLUE)
        stats_frame.pack(fill=tk.X, pady=(5, 0))
        self.lbl_pkt_count = tk.Label(stats_frame, text="Paquetes: 0",
                                       bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_pkt_count.pack(anchor=tk.W)
        self.lbl_pkt_errors = tk.Label(stats_frame, text="Errores:  0",
                                        bg=BG_PANEL, fg=FG_TEXT, font=("Consolas", 9))
        self.lbl_pkt_errors.pack(anchor=tk.W)
        self.lbl_pkt_rate = tk.Label(stats_frame, text="Rate:     --- Hz",
                                      bg=BG_PANEL, fg=C_BLUE, font=("Consolas", 9, "bold"))
        self.lbl_pkt_rate.pack(anchor=tk.W)

        # --- PANEL DERECHO: GRÁFICOS (4 subplots) ---
        graph_frame = tk.Frame(main_frame, bg=BG_MAIN, bd=1, relief=tk.SOLID,
                               highlightbackground=C_BORDER)
        graph_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig, ((self.ax_dc, self.ax_ac),
                   (self.ax_rpm, self.ax_hz)) = plt.subplots(
            2, 2, facecolor=BG_MAIN,
            gridspec_kw={"hspace": 0.40, "wspace": 0.30})

        self.fig.patch.set_facecolor(BG_MAIN)

        def style_ax(ax, title, ylabel, xlabel="Tiempo (s)"):
            ax.set_facecolor(BG_PANEL)
            ax.tick_params(colors=FG_TEXT, labelsize=8)
            ax.xaxis.label.set_color(FG_TEXT)
            ax.yaxis.label.set_color(FG_TEXT)
            ax.title.set_color(FG_TEXT)
            ax.title.set_fontsize(11)
            ax.title.set_weight("bold")
            ax.set_title(title)
            ax.set_ylabel(ylabel, fontsize=9)
            ax.set_xlabel(xlabel, fontsize=9)
            ax.grid(True, color=C_BORDER, linestyle="-", linewidth=0.5)
            for spine in ax.spines.values():
                spine.set_color("#CBD5E1")

        style_ax(self.ax_dc, "Actuación DC (PWM)", "PWM (-255..255)", "")
        style_ax(self.ax_ac, "Actuación AC (ESC µs)", "µs (1000-2000)", "")
        style_ax(self.ax_rpm, "Medición RPM", "RPM")
        style_ax(self.ax_hz, "Medición Hz (BLDC)", "Hz")

        # --- Subplot 1: Actuación DC (PWM applied) ---
        self.line_mPWM, = self.ax_dc.plot([], [], color=C_BLUE,
                                           linewidth=1.5, label="M PWM")
        self.line_sPWM, = self.ax_dc.plot([], [], color=C_GREEN,
                                           linewidth=1.5, label="S PWM")
        self.ax_dc.legend(facecolor=BG_MAIN, edgecolor=C_BORDER,
                          labelcolor=FG_TEXT, loc="upper right", fontsize=8)

        # --- Subplot 2: Actuación AC (ESC µs) ---
        self.line_mESC, = self.ax_ac.plot([], [], color=C_ORANGE,
                                           linewidth=1.5, label="M ESC")
        self.line_sESC, = self.ax_ac.plot([], [], color="#FBBF24",
                                           linewidth=1.5, label="S ESC")
        self.ax_ac.legend(facecolor=BG_MAIN, edgecolor=C_BORDER,
                          labelcolor=FG_TEXT, loc="upper right", fontsize=8)

        # --- Subplot 3: Medición RPM ---
        self.line_mRPM, = self.ax_rpm.plot([], [], color=C_BLUE,
                                            linewidth=1.5, label="M RPM")
        self.line_sRPM, = self.ax_rpm.plot([], [], color=C_GREEN,
                                            linewidth=1.5, label="S RPM")
        self.ax_rpm.legend(facecolor=BG_MAIN, edgecolor=C_BORDER,
                           labelcolor=FG_TEXT, loc="upper right", fontsize=8)

        # --- Subplot 4: Medición Hz ---
        self.line_mHz, = self.ax_hz.plot([], [], color=C_ORANGE,
                                          linewidth=1.5, label="M Hz")
        self.line_sHz, = self.ax_hz.plot([], [], color="#FBBF24",
                                          linewidth=1.5, label="S Hz")
        self.ax_hz.legend(facecolor=BG_MAIN, edgecolor=C_BORDER,
                          labelcolor=FG_TEXT, loc="upper right", fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    # ================================================================
    # SERIAL
    # ================================================================
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [p.device for p in ports]
        self.combo_port["values"] = port_list
        if port_list:
            preferred = next((p for p in port_list if "ttyUSB0" in p or "ttyUSB1" in p), port_list[0])
            self.selected_port.set(preferred)

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.selected_port.get()
        if not port:
            messagebox.showerror("Error", "Selecciona un puerto Serial.")
            return
        try:
            self.ser = serial.Serial(port, 460800, timeout=0.1)
            self.serial_running = True
            self.last_rx_time = time.time()
            self.connection_ok = True
            self.pkt_count = 0
            self.pkt_errors = 0
            self.live_t0_ms = None

            self.serial_thread = threading.Thread(target=self.serial_reader,
                                                   daemon=True)
            self.serial_thread.start()

            self.btn_connect.config(text="DESCONECTAR", bg=C_RED)
            for btn in (self.btn_dc_send, self.btn_dc_stop,
                        self.btn_ac_send, self.btn_ac_stop,
                        self.btn_rec, self.btn_stop_all):
                btn.config(state=tk.NORMAL)
            self.lbl_status.config(text=f"CONECTADO: {port}",
                                    fg=C_GREEN)
            self.lbl_warning.config(text="")

            self.check_watchdog()
            self._update_live_graph()

        except serial.SerialException as e:
            messagebox.showerror("Error Serial",
                                  f"No se pudo abrir {port}:\n{e}")

    def disconnect(self):
        self.serial_running = False
        if self.is_recording:
            self._stop_recording()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.connection_ok = False
        self.dc_active = False
        self.ac_active = False
        self.btn_connect.config(text="CONECTAR", bg=C_BLUE)
        for btn in (self.btn_dc_send, self.btn_dc_stop,
                    self.btn_ac_send, self.btn_ac_stop,
                    self.btn_rec, self.btn_stop_all):
            btn.config(state=tk.DISABLED)
        self.lbl_status.config(text="DESCONECTADO", fg=C_RED)

    def serial_reader(self):
        """Hilo de lectura Serial. Lee tramas binarias de 22 bytes."""
        buf = bytearray()
        while self.serial_running:
            try:
                if self.ser is None or not self.ser.is_open:
                    break
                # Leer datos disponibles
                chunk = self.ser.read(max(1, self.ser.in_waiting))
                if not chunk:
                    continue
                buf.extend(chunk)

                # Procesar todos los paquetes completos en el buffer
                while len(buf) >= PKT_SIZE:
                    # Buscar byte de inicio 0xAA
                    idx = buf.find(PKT_START)
                    if idx < 0:
                        buf.clear()
                        break
                    if idx > 0:
                        # Descartar bytes basura antes del 0xAA
                        self.pkt_errors += idx
                        del buf[:idx]
                    if len(buf) < PKT_SIZE:
                        break

                    # Extraer 22 bytes candidatos
                    frame = bytes(buf[:PKT_SIZE])

                    # Verificar checksum
                    xor_calc = 0
                    for i in range(PKT_SIZE - 1):
                        xor_calc ^= frame[i]
                    if xor_calc != frame[PKT_SIZE - 1]:
                        # Checksum fallo: descartar el 0xAA y buscar siguiente
                        self.pkt_errors += 1
                        del buf[:1]
                        continue

                    # Paquete válido — consumirlo
                    del buf[:PKT_SIZE]
                    self.pkt_count += 1

                    # Desempaquetar
                    datos = struct.unpack(PKT_FORMAT, frame)
                    # datos: (startByte, timestamp_ms, mPWM, sPWM, mESC, sESC,
                    #         mRPM, sRPM, mHz100, sHz100, checksum)
                    ts_ms  = datos[1]
                    m_pwm  = datos[2]
                    s_pwm  = datos[3]
                    m_esc  = datos[4]
                    s_esc  = datos[5]
                    m_rpm  = datos[6]
                    s_rpm  = datos[7]
                    m_hz   = datos[8] / 100.0
                    s_hz   = datos[9] / 100.0

                    # Guardar últimos valores
                    self.last_ts_ms = ts_ms
                    self.last_mPWM = m_pwm
                    self.last_sPWM = s_pwm
                    self.last_mESC = m_esc
                    self.last_sESC = s_esc
                    self.last_mRPM = m_rpm
                    self.last_sRPM = s_rpm
                    self.last_mHz  = m_hz
                    self.last_sHz  = s_hz

                    self.last_rx_time = time.time()
                    self.connection_ok = True

                    # Tiempo relativo del ATmega (en segundos)
                    if self.live_t0_ms is None:
                        self.live_t0_ms = ts_ms
                    t_sec = (ts_ms - self.live_t0_ms) / 1000.0

                    # Alimentar buffers para gráfico
                    self.live_times.append(t_sec)
                    self.live_mPWM.append(m_pwm)
                    self.live_sPWM.append(s_pwm)
                    self.live_mESC.append(m_esc)
                    self.live_sESC.append(s_esc)
                    self.live_mRPM.append(m_rpm)
                    self.live_sRPM.append(s_rpm)
                    self.live_mHz.append(m_hz)
                    self.live_sHz.append(s_hz)

                    # CSV: guardar con timestamp_ms del microcontrolador
                    if self.is_recording:
                        self.data_log.append({
                            "timestamp_ms": ts_ms,
                            "mPWM": m_pwm,
                            "sPWM": s_pwm,
                            "mESC": m_esc,
                            "sESC": s_esc,
                            "mRPM": m_rpm,
                            "sRPM": s_rpm,
                            "mHz":  round(m_hz, 2),
                            "sHz":  round(s_hz, 2),
                        })

                    # Actualizar panel de telemetría (schedule en main thread)
                    self.root.after(0, self._update_live_panel)

            except serial.SerialException:
                self.root.after(0, self.disconnect)
                break
            except Exception:
                continue

    def _update_live_panel(self):
        """Actualiza indicadores numéricos de telemetría."""
        if self._closing:
            return
        self.lbl_live_ts.config(text=f"t(ms): {self.last_ts_ms}")
        self.lbl_live_mPWM.config(text=f"M PWM: {self.last_mPWM}")
        self.lbl_live_sPWM.config(text=f"S PWM: {self.last_sPWM}")
        self.lbl_live_mESC.config(text=f"M ESC: {self.last_mESC} µs")
        self.lbl_live_sESC.config(text=f"S ESC: {self.last_sESC} µs")
        self.lbl_live_mRPM.config(text=f"M RPM: {self.last_mRPM}")
        self.lbl_live_sRPM.config(text=f"S RPM: {self.last_sRPM}")
        self.lbl_live_mHz.config(text=f"M Hz:  {self.last_mHz:.2f}")
        self.lbl_live_sHz.config(text=f"S Hz:  {self.last_sHz:.2f}")

        self.lbl_pkt_count.config(text=f"Paquetes: {self.pkt_count}")
        self.lbl_pkt_errors.config(text=f"Errores:  {self.pkt_errors}")
        if len(self.live_times) >= 10:
            dt = self.live_times[-1] - self.live_times[-10]
            if dt > 0:
                rate = 9.0 / dt
                self.lbl_pkt_rate.config(text=f"Rate:     {rate:.0f} Hz")

    def check_watchdog(self):
        """Watchdog del HMI: si no llegan datos en 2s, alertar."""
        if not self.serial_running or self._closing:
            return
        elapsed = time.time() - self.last_rx_time
        if elapsed > 2.0 and self.connection_ok:
            self.connection_ok = False
            self.lbl_warning.config(
                text="⚠ SIN DATOS DEL ATmega\n(REVISA SPI/USB/BATERÍA)",
                fg=C_RED)
        elif elapsed <= 2.0:
            self.lbl_warning.config(text="")
        self.root.after(500, self.check_watchdog)

    # ================================================================
    # COMANDOS — envío binario de 3 bytes
    # ================================================================
    def send_command(self, cmd_byte, val):
        """Envía 3 bytes crudos: [CMD, VAL_HIGH, VAL_LOW]"""
        if self.ser and self.ser.is_open:
            raw = (val if val >= 0 else val + 0x10000) & 0xFFFF
            try:
                self.ser.write(bytes([cmd_byte,
                                      (raw >> 8) & 0xFF,
                                      raw & 0xFF]))
            except serial.SerialException:
                self.disconnect()

    # ---- Slider / Entry sync ----
    def _dc_slider_changed(self, val):
        self.dc_value.set(int(val))
        self.dc_entry_var.set(str(int(val)))

    def _dc_entry_changed(self, event=None):
        try:
            v = int(self.dc_entry_var.get())
            v = max(-255, min(255, v))
            self.dc_value.set(v)
            self.dc_slider.set(v)
            self.dc_entry_var.set(str(v))
        except ValueError:
            pass

    def _ac_slider_changed(self, val):
        self.ac_value.set(int(val))
        self.ac_entry_var.set(str(int(val)))

    def _ac_entry_changed(self, event=None):
        try:
            v = int(self.ac_entry_var.get())
            v = max(1000, min(2000, v))
            self.ac_value.set(v)
            self.ac_slider.set(v)
            self.ac_entry_var.set(str(v))
        except ValueError:
            pass

    # ---- DC control ----
    def _send_dc(self):
        self._dc_entry_changed()
        val = self.dc_value.get()
        self.send_command(CMD_DC_BOTH, val)
        self.dc_active = val != 0
        self.lbl_status.config(text=f"DC ACTIVO: PWM={val}",
                                fg=C_BLUE)

    def _stop_dc(self):
        self.send_command(CMD_DC_BOTH, 0)
        self.dc_active = False
        self.dc_value.set(0)
        self.dc_slider.set(0)
        self.dc_entry_var.set("0")
        self.lbl_status.config(text="DC DETENIDO", fg=C_BLUE)

    # ---- AC (ESC) control ----
    def _send_ac(self):
        self._ac_entry_changed()
        val = self.ac_value.get()
        target = self.ac_target.get()
        cmd_map = {
            "AC_BOTH": CMD_AC_BOTH,
            "AC_MASTER": CMD_AC_MASTER,
            "AC_SLAVE": CMD_AC_SLAVE,
        }
        self.send_command(cmd_map[target], val)
        self.ac_active = val != 1500
        self.lbl_status.config(text=f"AC ACTIVO ({target}): {val}µs",
                                fg=C_ORANGE)

    def _stop_ac(self):
        target = self.ac_target.get()
        cmd_map = {
            "AC_BOTH": CMD_AC_BOTH,
            "AC_MASTER": CMD_AC_MASTER,
            "AC_SLAVE": CMD_AC_SLAVE,
        }
        self.send_command(cmd_map[target], 0)
        self.ac_active = False
        self.ac_value.set(1500)
        self.ac_slider.set(1500)
        self.ac_entry_var.set("1500")
        self.lbl_status.config(text="AC DETENIDO", fg=C_ORANGE)

    # ---- Stop all ----
    def _stop_all(self):
        self.send_command(CMD_STOP_ALL, 0)
        self.dc_active = False
        self.ac_active = False
        self.dc_value.set(0)
        self.dc_slider.set(0)
        self.dc_entry_var.set("0")
        self.ac_value.set(1500)
        self.ac_slider.set(1500)
        self.ac_entry_var.set("1500")
        self.lbl_status.config(text="TODO DETENIDO", fg=C_GREEN)
        if self.is_recording:
            self._stop_recording()
        self.root.after(500, self.on_close)

    # ---- Recording ----
    def _toggle_recording(self):
        if self.is_recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        self.is_recording = True
        self.data_log = []
        self.btn_rec.config(text="⏹ DETENER GRABACIÓN", bg=C_RED)
        self.lbl_status.config(text="GRABANDO...", fg=C_ORANGE)

    def _stop_recording(self):
        self.is_recording = False
        self.btn_rec.config(text="⏺ GRABAR CSV", bg=C_GREEN)

        if self.data_log:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"step_response_{ts}.csv"
            try:
                fields = ["timestamp_ms", "mPWM", "sPWM", "mESC", "sESC",
                           "mRPM", "sRPM", "mHz", "sHz"]
                with open(filename, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(self.data_log)
                messagebox.showinfo(
                    "Éxito",
                    f"Ensayo guardado en:\n{filename}\n"
                    f"{len(self.data_log)} muestras.\n\n"
                    f"Usa timestamp_ms como eje X en MATLAB/Excel\n"
                    f"para calcular τ y Tiempo Muerto.")
            except OSError as e:
                messagebox.showerror("Error CSV", str(e))

    # ================================================================
    # GRÁFICO EN VIVO
    # ================================================================
    def _update_live_graph(self):
        if not self.serial_running or self._closing:
            return
        if len(self.live_times) >= 2:
            times = list(self.live_times)
            mPWMs = list(self.live_mPWM)
            sPWMs = list(self.live_sPWM)
            mESCs = list(self.live_mESC)
            sESCs = list(self.live_sESC)
            mRPMs = list(self.live_mRPM)
            sRPMs = list(self.live_sRPM)
            mHzs  = list(self.live_mHz)
            sHzs  = list(self.live_sHz)

            # Ventana deslizante de 30s
            t_now = times[-1]
            t_min = max(0, t_now - 30)
            t_max = max(t_now, t_min + 5)

            # --- Subplot 1: Actuación DC ---
            self.line_mPWM.set_data(times, mPWMs)
            self.line_sPWM.set_data(times, sPWMs)
            all_pwm = mPWMs + sPWMs
            pmin = min(all_pwm) if all_pwm else -1
            pmax = max(all_pwm) if all_pwm else 1
            margin = max(abs(pmin), abs(pmax), 1) * 0.15
            self.ax_dc.set_xlim(t_min, t_max)
            self.ax_dc.set_ylim(pmin - margin, pmax + margin)

            # --- Subplot 2: Actuación AC ---
            self.line_mESC.set_data(times, mESCs)
            self.line_sESC.set_data(times, sESCs)
            all_esc = mESCs + sESCs
            emin = min(all_esc) if all_esc else 1000
            emax = max(all_esc) if all_esc else 2000
            em = max((emax - emin) * 0.1, 10)
            self.ax_ac.set_xlim(t_min, t_max)
            self.ax_ac.set_ylim(emin - em, emax + em)

            # --- Subplot 3: RPM ---
            self.line_mRPM.set_data(times, mRPMs)
            self.line_sRPM.set_data(times, sRPMs)
            all_rpm = mRPMs + sRPMs
            rmax = max(abs(v) for v in all_rpm) if all_rpm else 1
            self.ax_rpm.set_xlim(t_min, t_max)
            self.ax_rpm.set_ylim(0, max(rmax * 1.2, 1))

            # --- Subplot 4: Hz ---
            self.line_mHz.set_data(times, mHzs)
            self.line_sHz.set_data(times, sHzs)
            all_hz = mHzs + sHzs
            hmax = max(all_hz) if all_hz else 1
            self.ax_hz.set_xlim(t_min, t_max)
            self.ax_hz.set_ylim(0, max(hmax * 1.2, 1))

            self.canvas.draw_idle()

        self.root.after(200, self._update_live_graph)

    # ================================================================
    # CLEANUP
    # ================================================================
    def on_close(self):
        if self._closing:
            return
        self._closing = True
        self.serial_running = False
        if self.ser and self.ser.is_open:
            try:
                self.send_command(CMD_STOP_ALL, 0)
            except Exception:
                pass
            self.ser.close()
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
        plt.close('all')
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = SysIdHMI(root)
    root.mainloop()
