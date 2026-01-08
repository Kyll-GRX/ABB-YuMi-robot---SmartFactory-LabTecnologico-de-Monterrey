MODULE MainModule
!!! 
!! Documentacion completa sobre las funcciones y los processos, abajo de este codigo
!!! 

    VAR socketdev serverSocket_R;
    VAR socketdev clientSocket_R;
    VAR string receivedData_R;
    VAR string x_str;
    VAR string y_str;
    VAR string z_str;
    VAR bool ok1;
    VAR bool ok2;
    VAR bool ok3;
    VAR num x_val;
    VAR num y_val; 
    VAR num z_val;
    VAR num elapsed;
    VAR robtarget target_pos_R := [[0,0,0],[0.558278,-0.650056,0.366482,-0.362552],[1,2,-1,4],[-127.013,9E+09,9E+09,9E+09,9E+09,9E+09]];
    ! target_pos_R  is initialized here just give it a shape, otherway it wouldn't that it must have this shape/form/structure... like an array...
    CONST num MAX_X := 650;
    CONST num MIN_X := 280;
    CONST num MAX_Y := -80;
    CONST num MIN_Y := -450;
    CONST num MAX_Z := 650;
    CONST num MIN_Z := 200;
    CONST robtarget INICIAL_POS_R := [[295.00,-190.00,235.00],[0.558278,-0.650056,0.366482,-0.362552],[1,2,-1,4],[-127.013,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST num PUERTO_BRAZO_R := 30011;
    
    !Función para limitar coordenadas de TCP en rango seguro
    FUNC robtarget LimitTCP(robtarget tcp)
        VAR robtarget safeTCP;
        safeTCP := tcp;
        IF safeTCP.trans.x > MAX_X THEN safeTCP.trans.x := MAX_X; ENDIF
        IF safeTCP.trans.x < MIN_X THEN safeTCP.trans.x := MIN_X; ENDIF
        IF safeTCP.trans.y > MAX_Y THEN safeTCP.trans.y := MAX_Y; ENDIF
        IF safeTCP.trans.y < MIN_Y THEN safeTCP.trans.y := MIN_Y; ENDIF
        IF safeTCP.trans.z > MAX_Z THEN safeTCP.trans.z := MAX_Z; ENDIF
        IF safeTCP.trans.z < MIN_Z THEN safeTCP.trans.z := MIN_Z; ENDIF
        IF (safeTCP.trans.y < -225 AND safeTCP.trans.z < 360) THEN safeTCP.trans.y := -225; safeTCP.trans.z := 360; ENDIF
        RETURN safeTCP;
    ENDFUNC

    PROC main()
        ! Initializar a la posicion iniciale (y segura)
        MoveJ INICIAL_POS_R, v1000, z50, tool0;
        
        ! Crear y configurar el socket por la recepcion de los datos
        SocketCreate serverSocket_R;
        SocketBind serverSocket_R, "192.168.125.1", PUERTO_BRAZO_R;
        SocketListen serverSocket_R;

        TPWrite "Esperando conexión en el puerto 30011...";
        g_Calibrate;   ! Calibramos el gripper antes de usarlo

        WHILE TRUE DO
            SocketAccept serverSocket_R, clientSocket_R;
            TPWrite "Conexión establecida con el cliente (brazo de derecha) ";

            WHILE TRUE DO
                ! Verificar el estado del socket antes de recibir datos
                IF SocketGetStatus(clientSocket_R) = ERR_SOCK_CLOSED THEN
                    ! Checar si disconectado, si es.. conectar de vuelta..
                    TPWrite "Conexión cerrada. Esperando nueva conexión...";
                    EXIT;
                ENDIF
                
                
                elapsed := 0;                
                WHILE SocketPeek(clientSocket_R) = 0 DO
                    ! SocketPeek(clientSocket_R) checa si esta recibiendo bytes datas y cuentos
                    ! Checar aqui la conexión, si =0 significa que no hay bytes, y entonces, no conexión..
                    !WaitTime 100;   !this instruction blocks the entire program...
                    elapsed := elapsed + 100;
                    ! Esperar por intentar volver a conectarse
                    IF elapsed >= 100000000 THEN
                        ! Si no conexión despues de 100000000ms, el programa se apaga
                        TPWrite "Timeout esperando datos, esperando nueva conexión...";
                        EXIT;
                    ENDIF
                ENDWHILE

                
                ! Recibir datos
                SocketReceive clientSocket_R \Str := receivedData_R;
                
                IF receivedData_R = "ABIERTO" THEN
                    g_GripOut;
                    TPWrite "Gripper DERECHO Abierto";
                    
                ELSEIF receivedData_R = "CERRADO" THEN
                    g_GripIn;
                    TPWrite "Gripper DERECHO Cerrado";
                ELSE
                    ! Verificar que la cadena tiene datos válidos y mas de 12 (3 coordenadas 3D y 2 separaciones como " ; " + 1 especial al gripper de derecha)
                    IF StrLen(receivedData_R) >= 12 THEN
                        ! Convertir la cadena en valores numéricos
                        x_str := StrPart(receivedData_R,1,3);  ! Extraer el la primer cadena numerica (de 1 a 1+3), correspondiente a las primeras coordenadas
                        ok1 := StrToVal(x_str,x_val);          ! Convertir esta cadena numerica en numeros/cifras 
                        y_str := StrPart(receivedData_R,5,4);  
                        ok2 := StrToVal(y_str,y_val);
                        z_str := StrPart(receivedData_R,10,3);  ! 4 caracteras porque hay el signo - (so 3cifras mas un caractera por el signo)
                        ok3 := StrToVal(z_str,z_val);
                        TPWrite "Valores X,Y,Z recibidos: (camX/robY:" + y_str + " ; camY/robZ:" + z_str + " ; camZ/robX:" + x_str + ") ";            
                            
                        ! Asignar los valores a la estructura de robtarget, SOLO SI la conversion es validada (los 3 ok=TRUE)
                        IF ok1 AND ok2 AND ok3 THEN
                            target_pos_R.trans.x := x_val;
                            target_pos_R.trans.y := y_val;
                            target_pos_R.trans.z := z_val;
                            target_pos_R := LimitTCP(target_pos_R); ! Respectar las limites
                            
                            ! Mover el brazo (tool0) a la posición recibida (target_pos_L) con una velocidad de 600mm/s en la zona 50
                            MoveJ target_pos_R, v600, z50, tool0;
                        ENDIF
                        
                    ELSE
                        TPWrite "ERR: Datos inválidos -> ";
                    ENDIF
                ENDIF
            ENDWHILE
        ENDWHILE
    ENDPROC




! Here is some documentation : 
!
!  --> function :   SocketPeek(stringVar)
!     it checks how many bytes are waiting in the receive buffer, without removing them.. 
!     so it checks if it's receiving some bytes and how many... to check the coordonates data reception 
!     It returns:
!         0  ? no data available
!         >0 ? number of bytes ready to read
!         <0 ? error codes
! 
!
!  --> function :   MoverJ(positionVar[], Velocity, Zone, toolName)
!      executes a NON-straight movement of the COMPONENT 'toolNAme' 
!      from its CURRENT position TO the coordonates position in 'positionVAR[]'
!      with a specific VELOCITY, into a specific ZONE/AREA
! 
!
!  --> Coordonates Conversion Process :
!     The external system (camera+Jetson) sends coordinates as text as it's simple and robust over TCP/IP liaison 
!     So the robot receives a string and as to convert it into integer coordonates... using the following fuctions :
!     
!  --> function :   StrPart(variableName, start, length)
!         Extracts substring from a string... so can extract a part of string 'variableName', from START to START+LENGHT included
!  --> function :   StrToVal(strVar, numVar)
!         Converts a numeric string into a number     (returns TRUE if conversion succeed, FALSE if failed)
! 

! 
!  --> What is zone in MoveJ() function ? (z50, fine, z1, z10, ...)
!      Zone = how precisely the robot must pass through the target point.
!         fine ? robot must stop exactly at the target (no rounding, slowest, safest)
!         z1 ? allow 1 mm rounding
!         z10 ? 10 mm
!         z50 ? 50 mm
!         z100 ? 100 mm
!     Higher zone = smoother & faster motion, but less precise
!
!
!  --> CoordonatesVar := [ [X,Y,Z], [Q1,Q2,Q3,Q4], [CF1,CF2,CF3,CF4], [E1,E2,E3,E4,E5,E6] ]
!                        [X,Y,Z]                  <-- position of the TCP (Tool Center Point) in cartesian
!                        [Q1,Q2,Q3,Q4]            <-- orientation (or quaternion) it represents the tool's rotation in 3D
!                        [CF1,CF2,CF3,CF4]        <-- configuration data, it contains robot arm posture information
!                        [E1,E2,E3,E4,E5,E6]      <-- external axes (9E+09 = ignore this axis)
!
!
!  --> More details about the configuration data part: it contains robot arm posture information:
!         CF1: Arm configuration (elbow up/down, shoulder left/right)
!         CF2: Wrist configuration
!         CF3: Turn number (revolutions of axes)
!         CF4: external kinematic configuration
!      ABB robots need this to choose one of several possible robot postures that reach the same point
!
!
!  --> More details about the external axis: it's data for robots with:
!         linear tracks
!         positioners
!         extra rotary axes
!      9E+09 means: ignore this axis / no external axis used
!
!
ENDMODULE