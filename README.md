# ABB-YuMi-robot---SmartFactory-LabTecnologico-de-Monterrey
Este directorio incluye los documentos utiles y codigos usados para el proyect ABB YuMi Robot. 

YuMi Realtime Hand Control 🦾✋
Este proyecto implementa un sistema de control intuitivo en tiempo real para el robot colaborativo ABB YuMi IRB 14000, utilizando reconocimiento de manos mediante visión artificial.
El sistema permite al operador controlar los TCPs de ambos brazos del robot imitando el movimiento de sus manos, así como abrir y cerrar los Smart Grippers mediante gestos naturales, todo a través de una comunicación TCP/IP estable entre una Jetson Nano y el controlador IRC5.

🚀 Características principales
Seguimiento de manos en tiempo real con MediaPipe Hands.
Captura RGB + profundidad con Intel RealSense.
Control independiente de ambos brazos del YuMi.
Conversión de coordenadas cámara → robot (escala + offset).
Control de grippers mediante gesto ABIERTO / CERRADO.
Retorno automático a posición HOME segura si no se detectan manos.
Límites de seguridad aplicados en RAPID (LimitTCP).

🧩 Tecnologías utilizadas
Python 3, OpenCV, MediaPipe, PyRealSense2
NVIDIA Jetson Nano (Ubuntu Linux)
ABB RAPID (IRC5 Controller)
Comunicación Ethernet TCP/IP (Sockets)

📁 Estructura del proyecto
rs2YuMiv2.8.py → Script principal de visión y envío de datos.
posImitatorL.mod → Control RAPID del brazo izquierdo.
posImitatorR.mod → Control RAPID del brazo derecho.

⚠️ Notas importantes
El sistema imita únicamente dos dimensiones (X,Y de la cámara → Y,Z del robot) por motivos de seguridad.
Se recomienda verificar offsets, límites y conexión Ethernet antes de operar.
Mantener siempre disponible el botón de paro de emergencia.


Proyecto hecho por los ingenieros Carlos MARTINEZ GARCIA (mgcar_los@hotmail.com) y Kyllians GROUX (kyllias.groux@edu.ece.fr)
