import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
from collections import deque
import sys

# ================= CONFIGURACIÓN =================
PORT = 'COM5'      # <- CAMBIA ESTO (ej. 'COM3' en Windows, '/dev/ttyUSB0' en Linux/Mac)
BAUD = 115200      # Baudrate definido en tu setup()
MAX_POINTS = 100   # Cantidad de puntos mostrados en pantalla (ancho de la ventana)
Y_LIM = 20         # Límite del eje Y en cm (ej. -20 a +20 cm)
# =================================================

# Expresión regular para extraer Pos y Err del string del ESP32
# Busca "Pos: [numero] cm" y "Err: [numero] cm"
pattern = re.compile(r"Pos:\s*([-0-9.]+)\s*cm.*?Err:\s*([-0-9.]+)\s*cm")

# Colas (buffers) para almacenar los datos que se graficarán
y_pos = deque(maxlen=MAX_POINTS)
y_ref = deque(maxlen=MAX_POINTS)

# Iniciar conexión serie
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Conectado al puerto {PORT} a {BAUD} baudios.")
except serial.SerialException:
    print(f"ERROR: No se pudo abrir el puerto {PORT}.")
    print("Asegúrate de que la placa está conectada y CIERRA EL MONITOR SERIE DE ARDUINO.")
    sys.exit()

# Configurar la figura de Matplotlib
fig, ax = plt.subplots(figsize=(8, 5))
line_pos, = ax.plot([], [], label='Posición Real (cm)', color='#1f77b4', linewidth=2)
line_ref, = ax.plot([], [], label='Referencia (cm)', color='#d62728', linestyle='--', linewidth=2)

ax.set_xlim(0, MAX_POINTS)
ax.set_ylim(-Y_LIM, Y_LIM)  
ax.set_title("Ball and Beam: Posición vs Referencia en Tiempo Real")
ax.set_xlabel("Muestras (Tiempo)")
ax.set_ylabel("Distancia desde el centro (cm)")
ax.legend(loc='upper right')
ax.grid(True, linestyle=':', alpha=0.7)

def update(frame):
    # Leer todas las líneas disponibles en el buffer del puerto serie
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Filtrar e interpretar la línea con regex
            match = pattern.search(line)
            if match:
                pos_str = match.group(1)
                err_str = match.group(2)
                
                pos = float(pos_str)
                err = float(err_str)
                
                # Deducción matemática: Error = Referencia - Posición
                # Por lo tanto: Referencia = Error + Posición
                ref = err + pos
                
                y_pos.append(pos)
                y_ref.append(ref)
        except Exception as e:
            pass # Ignorar errores puntuales de lectura truncada

    # Actualizar las gráficas si hay datos
    if len(y_pos) > 0:
        line_pos.set_data(range(len(y_pos)), y_pos)
        line_ref.set_data(range(len(y_ref)), y_ref)
        
        # Efecto "scroll" estático (la línea avanza hasta llenar la pantalla)
        ax.set_xlim(0, len(y_pos) if len(y_pos) < MAX_POINTS else MAX_POINTS)

    return line_pos, line_ref

# Crear la animación (interval=50ms es más rápido que los 100ms de telemetría, asegurando fluidez)
ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

# Mostrar la ventana (el script se bloquea aquí hasta cerrar la ventana)
plt.tight_layout()
plt.show()

# Al cerrar la ventana, cerramos el puerto
ser.close()
print("Desconectado.")