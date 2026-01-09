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

# Definir el umbral de distancia entre las puntas de los dedos para mandar la señal de "Abierto" o "Cerrado"
GRIPPER_THRESH = 30

# Factores de escala (ajustar de acuerdo con la configuraciòn real)
ESCALA_X = 300      # antes fue 150
ESCALA_Y = 300      # antes fue 150
CAM_MAX_X = 300      #mm   por el mapping de las coordenadas
CAM_MIN_X = 0 
CAM_MAX_Y = 300
CAM_MIN_Y = 0 
ROB_MAX_X = 450 
ROB_MIN_X = -450
ROB_L_MAX_X = 450
ROB_L_MIN_X = 80
ROB_R_MAX_X = -80
ROB_R_MIN_X = -450
ROB_MAX_Y = 650
ROB_MIN_Y = 200

# Intervalo de tiempo para el reenvìo de la ùltima posiciòn guardada
RESEND_INTERVAL = 15.0  #s
DEAD_TIMER      = 1.0   #s
DEAD_ZONE       = 15.0  #mm   


# Configuraciòn de las coordenadas iniciales del YuMi de acuerdo con su Base 
# (cambiar si cambia la posiciòn inicial de los brazos)
LEFT_HAND_BASE = np.array([295.15 , 260 , 267])
RIGHT_HAND_BASE = np.array([287 , -190 , 244])
pulgar_px = None
medio_px  = None
gripper_status = "ABIERTO"
# Estado almacenado:
last_pos = {"left": None, "right": None}
last_grip = {"left": "ABIERTO", "right": "ABIERTO"}
last_time = {"left": time.time(), "right": time.time()}
seen = {"left":False, "right":False}
tcp_p = {"x": 0, "y": 0}
tcp_r = {"x": 0, "y": 0, "z": 0}
prev_tcp = {
    "left":  {"x": 150, "y": 350, "z": 450},
    "right": {"x": -150, "y": 350, "z": 450}
}
coordenadas = f"{tcp_r['z']:03},{tcp_r['x']:04},{tcp_r['y']:03}\n"



#####################################################################
   ######    LA CAMARA (RGB + profundidad)    #####

# Configurar la càmara Realsense y de la imagen que transmite
pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

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
#EndOfFunction

# Cuando la camara NO detecta 1 mano por un tiempo, enviar las previas coordenadas para no perder la conexion con el robot (por cada brazo)
def detection(now, last_time, last_pos): 
    if (now - last_time["left"] > RESEND_INTERVAL) :
        send_command( sock_left  , "L-NO-DATA" )    
        #send_command(sock_left, last_pos["left"])                 
        time.sleep(0.5)    # 0.5segundos
        last_time["left"]  = now    
    if (now - last_time["right"] > RESEND_INTERVAL) :
        send_command( sock_right , "R-NO-DATA" )   
        #send_command(sock_left, last_pos["right"])                     
        time.sleep(0.5)    # 0.5segundos
        last_time["right"] = now   
#EndOfFunction



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
#EndOfFunction

# Iniciar conexiones de Sockets: 
sock_left = connect_socket(portIzq)
sock_right = connect_socket(portDer)

    
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
#EndOfFunction



smooth_prev = {
    "left":  {"x": None, "y": None, "z": None},
    "right": {"x": None, "y": None, "z": None}
}

def smooth(tcp_r, smooth_prev, label) :
    alpha = 0.25

    for axis in ["x", "y", "z"] :
        if smooth_prev[label][axis] is None :
            smooth_prev[label][axis] = tcp_r[axis]
        else:
            smooth_prev[label][axis] = (
                smooth_prev[label][axis]
                + alpha * (tcp_r[axis] - smooth_prev[label][axis])
            )
        tcp_r[axis] = smooth_prev[label][axis]
    return tcp_r
#EndOfFunction


def map_range(tcp_r, label) :
    if label == "left" :
        ROB_MAX_X = -80
        ROB_MIN_X = -450
    else :
        ROB_MAX_X = 450
        ROB_MIN_X = 80

    mapX = int( ROB_MIN_X + (tcp_r["x"] - CAM_MIN_X) * (ROB_MAX_X - ROB_MIN_X) / (CAM_MAX_X - CAM_MIN_X) )
    if mapX > ROB_MAX_X:
        tcp_r["x"] = ROB_MAX_X
    elif mapX < ROB_MIN_X:
        tcp_r["x"] = ROB_MIN_X
    else:
        tcp_r["x"] = mapX

    mapY = int( ROB_MIN_Y + (tcp_r["y"] - CAM_MIN_Y) * (ROB_MAX_Y - ROB_MIN_Y) / (CAM_MAX_Y - CAM_MIN_Y) )
    if mapY > ROB_MAX_Y:
        tcp_r["y"] = ROB_MAX_Y
    elif mapY < ROB_MIN_Y:
        tcp_r["y"] = ROB_MIN_Y
    else:
        tcp_r["y"] = mapY

    #mapZ = ROB_MIN_Z + (tcp_r["z"] - CAM_MIN_Z) * (ROB_MAX_Z - ROB_MIN_Z) / (CAM_MAX_Z - CAM_MIN_Z)
    #if mapZ > ROB_MAX_Z:
    #    tcp_r["z"] = ROB_MAX_Z
    #elif mapZ < ROB_MIN_Z:
    #    tcp_r["z"] = ROB_MIN_Z
    #else:
    #    tcp_r["z"] = mapZ

    return tcp_r
#EndOfFunction


# Recuperar las coordenadas de la camara y generar variables de posicion
def generate_coordinates(hand_landmarks, label, tcp_p, tcp_r, width, height):
  # Calcul del centro de la palma (que servirà como TCP), sirven solo para los dibujos en la imagen
  # !NO USAR COMO COORDENADAS A ENVIAR AL ROBOT!
    tcp = hand_landmarks.landmark[mpHands.HandLandmark.MIDDLE_FINGER_MCP]
    tcp_x = tcp.x
    tcp_y = tcp.y

  # Convertir coordenadas de 0-1 en pixeles
    tcp_p["x"]  =  int( tcp_x *width )
    tcp_p["y"]  =  int( tcp_y *height )
    #profundidad = get_depth_value( depth_frame, tcp_p["x"], tcp_p["z"] )
  # Son las coordenadas que se enviaràn al robot en mm


  # Se voltearon los valores para que el robot actùe como un espejo del usuario 
    # Mano izquierda  ->  Brazo derecho del robot
    # Mano derecha    ->  Brazo izquierdo del robot
               
    tcp_r['x']  =  int( (tcp_x) * ESCALA_X)  # le -0.5 permet de partager l'écran en 2 
    #avec l'axe des ords (0) au milieu et donc d'avoir du négatif pour la Izquierda
    tcp_r['y']  =  int( (1 - tcp_y) * ESCALA_Y)
    tcp_r['z']  =  450
    tcp_r  =  map_range(tcp_r, label)

  # Mapear y suavizar los valores para permanecer en la zona operativa del robot
    #tcp_r  =  smooth(tcp_r, prev_tcp, label)
    
  # Crear el mensaje de cordenadas para el robot 
    coordenadas = f"{tcp_r['z']:03},{tcp_r['x']:04},{tcp_r['y']:03}\n"
    socket.sendall(coordenadas.encode())
    return coordenadas
#EndOfFunction



# Funciòn para enviar las coordenadas al robot (via el socket)
def send_coordinates(label, coordenadas, tcp_r, prev_tcp, now):
    if label == "left":
        send_command( sock_left, coordenadas )
        last_time["left"]  = now
        print( f" Coordenadas TCP mano izquierda: {tcp_r['x']:04} , {tcp_r['y']:03} , {tcp_r['z']:03} ")

    else:
        send_command( sock_right, coordenadas )
        last_time["right"] = now
        print( f" Coordenadas TCP mano derecha: {tcp_r['x']:04} , {tcp_r['y']:03} , {tcp_r['z']:03}  ")

    seen[label]           = True
    prev_tcp[label]["x"]  =  tcp_r["x"]
    prev_tcp[label]["y"]  =  tcp_r["y"]
    prev_tcp[label]["z"]  =  tcp_r["z"]
    last_pos[label]       =  coordenadas
    #time.sleep(0.5)    # los 'sleep()' frenan todo el programa (por accumulacion)
#EndOfFunction



# Funciòn para calcular distancia entre las puntas del pulgar y del dedo medio para Abrir/Cerra el gripper
def gripper_state(np, pulgar_punta, medio_punta, pulgar_px, medio_px, gripper_status, width, height) :
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
    
    return gripper_status
#EndOfFunction
                        

# Funciòn para que el comando gripper solo se envìe si es diferente al estado anterior
def send_gripper(gripper_status, previous_state, label) :
    if gripper_status != previous_state[label]:
        previous_state[label] = gripper_status
        
    #Como en el còdigo de RAPID se espera el texto "ABIERTO" o "CERRADO"
    if label == "left":
        send_command( sock_left , gripper_status )
        #time.sleep(0.5)    # los 'sleep()' frenan todo el programa (por accumulacion)
    else:
        send_command( sock_right , gripper_status )
        #time.sleep(0.5)    # los 'sleep()' frenan todo el programa (por accumulacion)
#EndOfFunction





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


        # Condiciòn para que SOLO se dibujen cosas CUANDO se detectan 2 manos
            if resultado.multi_hand_landmarks is not None:

                for i, hand_landmarks in enumerate(resultado.multi_hand_landmarks):
                    # Obtener la clasificaciòn de la mano (Izquierda o derecha)
                    if resultado.multi_handedness[i].classification[0].label == "Left" :
                        label = "left" 
                    else:
                        label = "right"


                    height, width , _  =  frame.shape

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


                # Reuperar las coordenadas de la camara y generar variables de posicion
                    coordenadas = generate_coordinates(hand_landmarks, label, tcp_p, tcp_r, width, height)
                    
                # Enviar los datos segun el brazo que corresponda a la mano detectada
                    # if (NOT (X AND Y AND Z !IN! their deadzones) OR timer > DEAD_TIMER) :
                    # <==> if (X AND Y AND Z !OUT! of their deadzones) OR data sent more than 1s ago) : 
                    if( not( 
                            (prev_tcp[label]["x"] -DEAD_ZONE < tcp_r["x"] < prev_tcp[label]["x"] +DEAD_ZONE) 
                             and (prev_tcp[label]["y"] -DEAD_ZONE < tcp_r["y"] < prev_tcp[label]["y"] +DEAD_ZONE) 
                             and (prev_tcp[label]["z"] -DEAD_ZONE < tcp_r["z"] < prev_tcp[label]["z"] +DEAD_ZONE) ) 
                        or (now - last_time[label] > DEAD_TIMER) 
                    ) :
                            send_coordinates(label, coordenadas, tcp_r, prev_tcp, now)
                # estas condiciones permitten de evitar la saturación del socket y mantener el fluidez 


                # Dibujar sobre la pantalla el TCP de cada mano, de la imagen, como un punto de color azul celeste
                    cv.circle(frame, (tcp_p["x"] , tcp_p["y"]), 9, (255,255,0), -1)

                # Calcular distancia entre las puntas del pulgar y del dedo medio para activar/desactivar el gripper
                    gripper_status = gripper_state(np, pulgar_punta, medio_punta, pulgar_px, medio_px, gripper_status, width, height)
                    send_gripper(gripper_status, previous_state, label)
        

        # Cuando la camara NO detecta 1 mano por un tiempo, enviar las previas coordenadas para no perder la conexion con el robot (por cada brazo)
            detection(now, last_time, last_pos)  

        # Luego sigue mostrando la image
            cv.imshow( "rs2YuMi v.7.8.2" , frame )

        # Opcion para quitar el ciclo y apagar el programa
            if cv.waitKey(1) == ord('q'):
                break

# Cerrar los programas usados y liberar espacio RAM                
pipeline.stop()
cv.destroyAllWindows()