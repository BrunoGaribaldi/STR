# ============================================================================
#  attitude.py
#  Attitude indicator (horizonte artificial) en tkinter para el controlador
#  de vuelo del ala volante.
#
#  MATERIAL DE ESTUDIO -> [Guia §IV.5]. Instrumento recortado del proyecto
#  open-source "AirTelemetry": se conservo SOLO la clase del indicador y se
#  le agrego la linea del punto cero (el horizonte de referencia que se fija
#  con el boton del ESP32).
#
#  Como se dibuja (tres capas, de atras hacia adelante):
#    1. Fondo cielo/tierra: ROTA con el roll y se DESPLAZA con el pitch.
#    2. Linea amarilla punteada: el PUNTO CERO. Se dibuja como "el horizonte
#       del error": si el avion esta clavado en la referencia queda horizontal
#       y centrada; al desviarse, se mueve igual que un horizonte cuyo "nivel"
#       es la referencia. Con punto cero 0/0 coincide con el horizonte real.
#    3. Frente fijo (bisel + avioncito) y textos.
# ============================================================================

import math
from pathlib import Path
import tkinter as tk

from PIL import Image, ImageTk

ASSETS_DIR = Path(__file__).resolve().parent / "assets"

CENTER = 350          # centro del canvas de 700x700
ZERO_LINE_HALF = 240  # media longitud (px) de la linea del punto cero


class AttitudeIndicator(tk.Canvas):
    """Canvas de 700x700 con el instrumento. Actualizar con update_attitude()."""

    def __init__(self, parent, *args, **kwargs):
        kwargs.setdefault("width", 700)
        kwargs.setdefault("height", 700)
        super().__init__(parent, *args, **kwargs)

        self.bg_image_orig = Image.open(ASSETS_DIR / "attitudebg1.png")
        self.fg_image = Image.open(ASSETS_DIR / "attitudefg.png")

        # Cache de fondos rotados (roll redondeado a 0.25 grados -> imagen).
        # Evita re-rotar la imagen en cada frame; se vacia si crece demasiado.
        self._rotation_cache = {}

        self.bg_image_tk = ImageTk.PhotoImage(self.bg_image_orig)
        self.fg_image_tk = ImageTk.PhotoImage(self.fg_image)

        self.bg_item = self.create_image(CENTER, CENTER, anchor="center",
                                         image=self.bg_image_tk)
        self.zero_item = self.create_line(
            CENTER - ZERO_LINE_HALF, CENTER, CENTER + ZERO_LINE_HALF, CENTER,
            fill="#ffd21f", width=3, dash=(12, 8))
        self.fg_item = self.create_image(CENTER, CENTER, anchor="center",
                                         image=self.fg_image_tk)

        self.pitch_text = self.create_text(CENTER, 15, anchor="n",
                                           font=("Segoe UI", 16, "bold"),
                                           fill="white", text="Pitch: 0.0°")
        self.roll_text = self.create_text(CENTER, 45, anchor="n",
                                          font=("Segoe UI", 16, "bold"),
                                          fill="white", text="Roll: 0.0°")
        self.zero_text = self.create_text(CENTER, 685, anchor="s",
                                          font=("Segoe UI", 14, "bold"),
                                          fill="#ffd21f",
                                          text="Zero R/P: 0.0°/0.0°")

        self.pack(fill="both", expand=True)

    def pitch_to_pixels(self, pitch):
        # Mapeo lineal: -90..90 grados -> -262..262 pixeles del fondo
        return (pitch / 90.0) * 262.0

    def _get_rotated_bg(self, roll):
        rounded = round(roll * 4) / 4  # pasos de 0.25 grados
        if rounded not in self._rotation_cache:
            if len(self._rotation_cache) > 240:
                self._rotation_cache.clear()
            # BILINEAR y no BICUBIC: 2-3x mas rapido de rotar y en movimiento
            # continuo la diferencia de calidad no se percibe (evita que un
            # frame "trabe" cuando la rotacion no esta cacheada).
            rotated = self.bg_image_orig.rotate(
                -rounded, resample=Image.Resampling.BILINEAR, expand=False)
            self._rotation_cache[rounded] = ImageTk.PhotoImage(rotated)
        return self._rotation_cache[rounded]

    def update_attitude(self, pitch, roll, zero_pitch=0.0, zero_roll=0.0):
        self.itemconfig(self.pitch_text, text=f"Pitch: {pitch:+.1f}°")
        self.itemconfig(self.roll_text, text=f"Roll: {roll:+.1f}°")
        self.itemconfig(self.zero_text,
                        text=f"Zero R/P: {zero_roll:+.1f}°/{zero_pitch:+.1f}°")

        # Normalizar roll a (-180, 180]
        while roll > 180:
            roll -= 360
        while roll < -180:
            roll += 360

        # --- Fondo: rota -roll y se desplaza pitch_to_pixels(pitch) en la
        #     direccion perpendicular al horizonte rotado ---
        off = self.pitch_to_pixels(pitch)
        a = math.radians(roll)
        dx = -off * math.sin(a)
        dy = off * math.cos(a)

        self.bg_image_tk = self._get_rotated_bg(roll)
        self.itemconfig(self.bg_item, image=self.bg_image_tk)
        self.coords(self.bg_item, CENTER + dx, CENTER + dy)

        # --- Linea del punto cero: mismo dibujo que el horizonte, pero usando
        #     el ERROR (attitude actual menos referencia). Error nulo => linea
        #     horizontal y centrada sobre el avioncito. ---
        e = math.radians(roll - zero_roll)
        e_off = self.pitch_to_pixels(pitch - zero_pitch)
        cx = CENTER - e_off * math.sin(e)
        cy = CENTER + e_off * math.cos(e)
        ux, uy = math.cos(e), math.sin(e)
        self.coords(self.zero_item,
                    cx - ZERO_LINE_HALF * ux, cy - ZERO_LINE_HALF * uy,
                    cx + ZERO_LINE_HALF * ux, cy + ZERO_LINE_HALF * uy)


if __name__ == "__main__":
    # Prueba visual estatica (sin ESP32): actitud 10/20 con punto cero 2/-5.
    root = tk.Tk()
    root.title("Attitude Indicator — prueba estatica")
    root.geometry("700x700")
    ai = AttitudeIndicator(root)
    ai.update_attitude(pitch=10.0, roll=20.0, zero_pitch=2.0, zero_roll=-5.0)
    root.mainloop()
