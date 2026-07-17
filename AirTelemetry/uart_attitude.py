# ============================================================================
#  uart_attitude.py
#  Lector UART + attitude indicator para el controlador de vuelo del ESP32.
#
#  MATERIAL DE ESTUDIO -> [Guia §IV.5]. Es la contraparte en la PC de la
#  telemetria del firmware [Guia §II.9.b]: parsea la trama
#      $ATT,<roll>,<pitch>,<zero_roll>,<zero_pitch>      (50 Hz)
#  y la dibuja con AttitudeIndicator, incluida la linea del punto cero.
#
#  Arquitectura (mismo patron productor-consumidor que en el ESP32):
#    - Hilo lector (productor): bloquea leyendo el puerto serie, filtra las
#      lineas $ATT y ENCOLA los valores. No toca la GUI (tkinter no es
#      thread-safe).
#    - Bucle de la GUI (consumidor): cada ~16 ms (60 fps) vacia la cola,
#      suaviza el movimiento y redibuja.
#
#  Uso:  python3 uart_attitude.py [puerto]      (default /dev/ttyUSB0)
# ============================================================================

import queue
import sys
import threading
import tkinter as tk

import serial

from attitude import AttitudeIndicator

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 115200

data_queue = queue.Queue()


def uart_reader(ser):
    """Hilo productor: lee lineas del puerto y encola las tramas $ATT."""
    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line.startswith("$ATT,"):
            continue  # logs humanos, arranque, etc.: se ignoran
        try:
            roll, pitch, zero_roll, zero_pitch = map(float, line[5:].split(","))
        except ValueError:
            continue  # trama incompleta o corrupta: se descarta
        data_queue.put((pitch, roll, zero_pitch, zero_roll))


# Estado: objetivo (t*, ultimo recibido) y mostrado (d*, suavizado).
state = {"tp": 0.0, "tr": 0.0, "dp": 0.0, "dr": 0.0, "zp": 0.0, "zr": 0.0}

# Suavizado exponencial por frame (~60 fps): interpola entre tramas de 50 Hz.
# Mas alto = mas reactivo; mas bajo = mas suave (pero agrega retardo visual).
# Con datos a 50 Hz casi no hace falta suavizar: 0.6 responde en ~2 frames.
ALPHA = 0.6


def poll_queue(ai, root):
    """Consumidor (hilo de la GUI): vacia la cola, suaviza y redibuja."""
    while not data_queue.empty():
        state["tp"], state["tr"], state["zp"], state["zr"] = data_queue.get()

    state["dp"] += (state["tp"] - state["dp"]) * ALPHA
    state["dr"] += (state["tr"] - state["dr"]) * ALPHA
    # El punto cero no se suaviza: cambia por salto (al tocar el boton).
    ai.update_attitude(state["dp"], state["dr"], state["zp"], state["zr"])

    root.after(16, poll_queue, ai, root)


def main():
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.timeout = 1
    # DTR/RTS desactivados: NO resetear el ESP32 al abrir el puerto
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"No se pudo abrir {PORT}: {e}\n"
                 f"Uso: python3 uart_attitude.py [puerto]")

    root = tk.Tk()
    root.title("ESP32 Attitude Indicator")
    root.geometry("700x700")
    ai = AttitudeIndicator(root)

    threading.Thread(target=uart_reader, args=(ser,), daemon=True).start()
    root.after(16, poll_queue, ai, root)
    root.mainloop()


if __name__ == "__main__":
    main()
