# Attitude Indicator — visualizador del controlador de vuelo

Horizonte artificial en la PC para el controlador de vuelo del ala volante (ESP32).
Lee la trama de telemetría `$ATT,<roll>,<pitch>,<zero_roll>,<zero_pitch>` que el
firmware emite por UART a 20 Hz y dibuja la actitud del avión más la **línea
amarilla del punto cero** (el horizonte de referencia que se fija con el botón).

Instrumento basado en el proyecto open-source [AirTelemetry](https://github.com/tlkm4n/AirTelemetry),
recortado a solo el attitude indicator. Explicación completa: `RESUMEN_INTEGRAL.md` §IV.5.

## Dependencias

Python 3 con tkinter, más `pyserial` y `Pillow`. En esta máquina ya hay un
`venv/` armado con todo (creado con `uv venv --python 3.12` + `uv pip install
-r requirements.txt`; ese Python standalone trae tkinter incluido). Para
recrearlo en otra máquina: `sudo apt install python3-tk` y
`pip install -r requirements.txt`.

## Uso

```bash
venv/bin/python uart_attitude.py               # puerto default /dev/ttyUSB0
venv/bin/python uart_attitude.py /dev/ttyUSB1  # otro puerto
venv/bin/python attitude.py                    # prueba visual estática, sin ESP32
```
