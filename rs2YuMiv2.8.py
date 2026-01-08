import socket
import cv2 as cv
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np
import time

# Configuraciòn de IP del robot y de puertos por los brazos (en base a resultados voltear puertos si es necesario)
ROBOT_IP = "192.168.125.1"
portDer = 30010
portIzq = 30011

# Initializacion y configuraciòn de herramientas de detecciòn de manos y de dibujo
mpDrawing = mp.solutions.drawing_utils
mpHands = mp.solutions.hands

# Configuraciòn de las coordenadas iniciales del YuMi de acuerdo con su Base 
# (cambiar si cambia la posiciòn inicial de los brazos)
LEFT_HAND_BASE = np.array([295.15,260,267])
RIGHT_HAND_BASE = np.array([287,-190,244])
gripper_status = "ABIERTO"

# Definir el umbral de distancia entre las puntas de los dedos para mandar la señal de "Abierto" o "Cerrado"
GRIPPER_THRESH = 30


# Factores de escala (ajustar de acuerdo con la configuraciòn real)
ESCALA_X = 150
ESCALA_Y = 150

# Intervalo de tiempo para el reenvìo de la ùltima posiciòn guardada
RESEND_INTERVAL = 25.0  #ms


#####################################################################
   ######    LA CAMARA (RGB + profundidad)    #####

# Configurar la càmara Realsense y de la imagen que transmite
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# Inicializar de la càmara con la configuraciòn establecida
profile = pipeline.start(config)

# Obtener la escala de profundidad en mm
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()   #Se convierten los datos a mm


# Funciòn para obtener el valor de la profundidad en mm de un punto x,y
def get_depth_value(depth_frame, x, y):
    depth_image = np.asanyarray(depth_frame.get_data())   # Convertir el frame a una matriz de Numpy
    depth_value = depth_image[int(y),int(x)]*depth_scale
    return depth_value if depth_value > 0 else None



#####################################################################
   ######    CONNEXION A LOS BRAZOS (via sockets)    #####


# Funciòn para conectar sockets TCP con el puerto especificado
def connect_socket(port):
    # --> crear y mantener una conexiòn socket constante con el robot
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ROBOT_IP,port))
        print(f"Conectado al puerto: {port}")
        return s
    except Exception as e:
        print(f"Error al conectar al puerto {port}:{e}")
        return None

    
# Funciòn para enviar comandos a travès de la conexiòn Socket
def send_command(sock, command):
    # --> Enviar comandos al YuMi sin cortar la comunicaciòn
    try:
        if sock:
            sock.sendall(command.encode())
            #print(f"\n Comando enviado: {command}")
        else:
            print("Socket no disponible, no se pudo enviar el comando")
    
    except Exception as e:
        print(f"Error al enviar el comando: {e}")


# Iniciar conexiones de Sockets: 
sock_left = connect_socket(portIzq)
sock_right = connect_socket(portDer)

# Estado almacenado:
last_pos = {"left": None, "right": None}
last_grip = {"left": "ABIERTO", "right": "ABIERTO"}
last_time = {"left": time.time(), "right": time.time()}



#####################################################################
   ######    PROGRAMA DE DETECTION DE MANOS    #####


# Ahora se hace la instrucciòn para la detecciòn de manos y el envìo de los comandos
with mpHands.Hands(static_image_mode=False, 
                   max_num_hands = 2, 
                   min_detection_confidence = 0.65) as hands:
    
    # --> Definir un estado inicial de los grippers por seguridad
    previous_state = {"left":"ABIERTO","right":"CERRADO"}

    
    # --> siempre ejecutar este ciclo
    while True: 
        # --> Acquision de la imagen en color y con profundidad
            frames      = pipeline.wait_for_frames()   # Extraer fotogramas de la Càmara
            color_frame = frames.get_color_frame()     # Extraer el fotograma de color
            depth_frame = frames.get_depth_frame()     # Extraer la profundidad en mm (con escalas)
        

        # si no hay colores o profundidad
        # --> volver a ejecutar el ciclo 'while True:' para recuperarlas 
            if not color_frame or not depth_frame:
                continue   
        

        # --> Convertir fotograma de color a matriz Numpy
            frame = np.asanyarray(color_frame.get_data()) 
        # --> Volter la imagen para darle una imagen màs natural
            frame = cv.flip(frame,1) 
        # --> Convertir a formato RGB para Mediapipe
            frame_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB) 
        # --> Procesar el fotograma con Mediapipe
            resultado = hands.process(frame_RGB) 


        now = time.time()
        seen = {"left":False, "right":False}


        # Condiciòn para que SOLO se dibujen cosas CUANDO se detectan 2 manos
        if resultado.multi_hand_landmarks is not None:

            for i, hand_landmarks in enumerate(resultado.multi_hand_landmarks):
                # Obtener la clasificaciòn de la mano (Izquierda o derecha)
                if resultado.multi_handedness[i].classification[0].label == "Left" :
                    label = "left" 
                else:
                    label = "right"


                # Dibujar (draw) los landmarks en las manos detectadas en el fotograma
                # y empezar las conexiones/líneas entre cada landmark
                mpDrawing.draw_landmarks( frame, 
                                          hand_landmarks, 
                                          mpHands.HAND_CONNECTIONS,
                                          mpDrawing.DrawingSpec( color=(160,0,0), thickness = 4, circle_radius = 2),
                                          mpDrawing.DrawingSpec( color=(0,160,20), thickness = 3)  
                                        )
                # mpDrawing.draw_landmarks  es una función de MediaPipe que dibuja (draw) la mano detectada sobre la imagen
                # Incluye 21 puntos de referencia (landmarks) y líneas que conectan esos puntos (esqueleto)
                    #  
                    # La reparticion de los 21 landmarks segun los dedos es : 
                    # WRIST: 0 | THUMB: 1–2–3–4  | INDEX: 5–6–7–8 |  MIDDLE: 9–10–11–12 | RING: 13–14–15–16 | PINKY: 17–18–19–20


                # Obtener coordenadas de los landmarks importantes /de interès
                    muneca        =  hand_landmarks.landmark[ mpHands.HandLandmark.WRIST ]
                    indice_mcp    =  hand_landmarks.landmark[ mpHands.HandLandmark.INDEX_FINGER_MCP ]
                    menique_mcp   =  hand_landmarks.landmark[ mpHands.HandLandmark.PINKY_MCP ]
                    pulgar_punta  =  hand_landmarks.landmark[ mpHands.HandLandmark.THUMB_TIP ]
                    medio_punta   =  hand_landmarks.landmark[ mpHands.HandLandmark.MIDDLE_FINGER_TIP ]


                # Calcul del centro de la palma (que servirà como TCP), sirven solo para los dibujos en la imagen
                # !NO USAR COMO COORDENADAS A ENVIAR AL ROBOT!
                    tcp_x  =  (muneca.x + indice_mcp.x + menique_mcp.x) /3
                    tcp_y  =  (muneca.y + indice_mcp.y + menique_mcp.y) /3
                    tcp_z  =  (muneca.z + indice_mcp.z + menique_mcp.z) /3
                

                # Convertir coordenadas de 0-1 en pixeles
                    height, width , _  =  frame.shape
                    tcp_px  =  int( tcp_x *width )
                    tcp_py  =  int( tcp_y *height )
                    #profundidad = get_depth_value( depth_frame, tcp_px, tcp_py )
                # Son las coordenadas que se enviaràn al robot en mm



                # Se voltearon los valores para que el robot actùe como un espejo del usuario 
                    # Mano izquierda  ->  Brazo derecho del robot
                    # Mano derecha    ->  Brazo izquierdo del robot
                
                # Actualizar las coordenadas del brazo DERECHO si 'left' (left hand for right arm)
                    if label == "left":
                        ESCALA_X = 300
                        ESCALA_Y = 300
                        tcp_rx = int( (tcp_x * ESCALA_X) + RIGHT_HAND_BASE[1] )
                        tcp_ry = int( ( (1 - tcp_y) * ESCALA_Y) + RIGHT_HAND_BASE[2] )
                        tcp_rz = 450

                # sino actualizar las del brazo IZQUIERDO
                    else: 
                        ESCALA_X = 300
                        ESCALA_Y = 300
                        tcp_rx = int( (tcp_x * ESCALA_X) + LEFT_HAND_BASE[0] )
                        tcp_ry = int( ( (1 - tcp_y) * ESCALA_Y) + LEFT_HAND_BASE[1] )
                        tcp_rz = 450
                

                # Crear el mensaje de cordenadas para el robot 
                    coordenadas = f"{tcp_rz:02},{tcp_rx:02},{tcp_ry:02}"

                # Enviar los datos segun el brazo que corresponda a la mano detectada
                    if label == "left":
                        send_command( sock_left, coordenadas )
                        last_time["left"] = now
                        seen["left"] = True
                        time.sleep(0.5)
                        print( f" Coordenadas TCP mano izquierda: {tcp_rz} , {tcp_rx} , {tcp_ry} ")
                        
                    else:
                        send_command( sock_right, coordenadas )
                        last_time["right"] = now
                        seen["right"] = True
                        time.sleep(0.5)
                        print( f" Coordenadas TCP mano derecha: {tcp_rz} , {tcp_rx} , {tcp_ry} ")



                # Dibujar sobre la pantalla el TCP de cada mano, de la imagen, como un punto de color azul celeste
                    cv.circle( frame, (tcp_px , tcp_py), 9, (255,255,0), -1)

                # Calcular distancia entre las puntas del pulgar y del dedo medio para activar/desactivar el gripper
                    if pulgar_punta is not None and medio_punta is not None:
                        # Por el pulgar
                        if 0 <= pulgar_punta.x <= 1 and 0 <= pulgar_punta.y <= 1:
                            pulgar_px = ( int(pulgar_punta.x *width) , int(pulgar_punta.y *height) )
                        else:
                            pulgar_px = None
                        # Por el dedo medio 
                        if 0 <= medio_punta.x <=1 and 0 <= medio_punta.y <= 1:
                            medio_px = ( int(medio_punta.x *width) , int(medio_punta.y *height) )
                        else:
                            medio_px = None
                

                # Comparar la distancia lineal normal entre las coordenadas, en pixeles, de la punta del dedo pulgar menos las del dedo medio
                    if pulgar_px is not None and medio_px is not None:
                        distancia = np.linalg.norm( np.array(pulgar_px) - np.array(medio_px) )

                        # Definir el estado variable del Gripper segun la distancia calculada
                            # Si la distancia es menor al margen, se cambia el estado a cerrado 
                            if distancia < GRIPPER_THRESH:
                                gripper_status = "CERRADO"
                            else:
                                gripper_status = "ABIERTO"
                        

                # Condiciòn para que el comando solo se envìe si es diferente al estado anterior
                    if gripper_status != previous_state[label]:
                        previous_state[label] = gripper_status
                        
                        #Como en el còdigo de RAPID se espera el texto "ABIERTO" o "CERRADO"
                        if label == "left":
                            send_command( sock_left , gripper_status )
                            time.sleep(0.5)
                        else:
                            send_command( sock_right , gripper_status )
                            time.sleep(0.5)



        #print("Left: " , now - last_time["left"] )
        #print("Right: " , now - last_time["right"] )

    # Cuando la camara NO detecta manos por un tiempo, enviar las previas coordenadas para no perder la conexion con el robot
        if (now - last_time["left"] > RESEND_INTERVAL) and (now - last_time["right"] > RESEND_INTERVAL):
            send_command( sock_left  , "L-NO-DATA" )
            send_command( sock_right , "R-NO-DATA" )                        
            time.sleep(0.5)
            last_time["left"]  = now
            last_time["right"] = now    

    # Luego sigue mostrando la image
        cv.imshow( "rs2YuMi v.7.8" , frame )

    # Opcion para quitar el ciclo y apagar el programa
        if cv.waitKey(1) == ord('q'):
            break

# Cerrar los programas usados y liberar espacio RAM                
pipeline.stop()
cv.destroyAllWindows()