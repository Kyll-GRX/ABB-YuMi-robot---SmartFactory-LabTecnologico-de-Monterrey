MODULE MainModule    
    VAR socketdev serverSocket_L;
    VAR socketdev clientSocket_L;
    VAR string receivedData_L;
    VAR string x_str;
    VAR string y_str;
    VAR string z_str;
    VAR string grip_str;
    VAR bool ok1;
    VAR bool ok2;
    VAR bool ok3;
    VAR bool ok4;
    VAR num x_val;
    VAR num y_val; 
    VAR num z_val;
    VAR num grip_statue;
    VAR num elapsed;
    VAR robtarget target_pos_L := [[340.00,315.00,265.00],[0.385214,-0.336824,0.640231,-0.572944],[-1,0,1,4],[138.956,9E+09,9E+09,9E+09,9E+09,9E+09]];
    ! target_pos_L  is initialized here just give it a shape, otherway it wouldn't that it must have this shape/form/structure... like an array...
    CONST num MAX_X := 650;
    CONST num MIN_X := 280;
    CONST num MAX_Y := 450;
    CONST num MIN_Y := 80;
    CONST num MAX_Z := 650;
    CONST num MIN_Z := 200;
    CONST robtarget INICIAL_POS_L := [[340.00,315.00,265.00],[0.385214,-0.336824,0.640231,-0.572944],[-1,0,1,4],[138.956,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST num PUERTO_BRAZO_L := 30010;
    
    ! Función para limitar coordenadas de TCP en rango seguro
    FUNC robtarget LimitTCP(robtarget tcp)
        VAR robtarget safeTCP;
        safeTCP := tcp;
        IF safeTCP.trans.x > MAX_X THEN safeTCP.trans.x := MAX_X; ENDIF
        IF safeTCP.trans.x < MIN_X THEN safeTCP.trans.x := MIN_X; ENDIF
        IF safeTCP.trans.y > MAX_Y THEN safeTCP.trans.y := MAX_Y; ENDIF
        IF safeTCP.trans.y < MIN_Y THEN safeTCP.trans.y := MIN_Y; ENDIF
        IF safeTCP.trans.z > MAX_Z THEN safeTCP.trans.z := MAX_Z; ENDIF
        IF safeTCP.trans.z < MIN_Z THEN safeTCP.trans.z := MIN_Z; ENDIF
        RETURN safeTCP;
    ENDFUNC

    
    
    PROC main()
        ! Initializar a la posicion iniciale (y segura)
        MoveJ INICIAL_POS_L, v1000, z50, tool0;
        
        ! Crear y configurar el socket por la recepcion de los datos
        SocketCreate serverSocket_L;
        SocketBind serverSocket_L, "192.168.125.1", PUERTO_BRAZO_L;  
        SocketListen serverSocket_L;

        TPWrite "Esperando conexión en el puerto 30010...";
        g_Calibrate;   ! Calibramos el Gripper antes de usarlo

        WHILE TRUE DO
            SocketAccept serverSocket_L, clientSocket_L;
            TPWrite "Conexión establecida con el cliente (brazo de izquierda) ";

            WHILE TRUE DO
                ! Verificar el estado del socket antes de recibir datos
                IF SocketGetStatus(clientSocket_L) = ERR_SOCK_CLOSED THEN
                    ! Checar si disconectado, si es.. conectar de vuelta..
                    TPWrite "Conexión cerrada. Esperando nueva conexión...";
                    EXIT;
                ENDIF
                
                elapsed := 0;        
                WHILE SocketPeek(clientSocket_L) = 0 DO
                    ! SocketPeek(clientSocket_L) checa si esta recibiendo bytes datas y cuentos
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
                SocketReceive clientSocket_L \Str := receivedData_L;
                
                IF StrLen(receivedData_L) >= 14 THEN
                    ! Convertir la cadena en valores numéricos
                    x_str := StrPart(receivedData_L,1,3);    ! Extraer el la primer cadena numerica (de 1 a 1+3), correspondiente a las primeras coordenadas
                    ok1 := StrToVal(x_str,x_val);          ! Convertir esta cadena numerica en numeros/cifras 
                    y_str := StrPart(receivedData_L,5,4);  
                    ok2 := StrToVal(y_str,y_val);
                    z_str := StrPart(receivedData_L,10,3);   ! 4 caracteras porque hay el signo - (so 3cifras mas un caractera por el signo)
                    ok3 := StrToVal(z_str,z_val);
                    grip_str := StrPart(receivedData_L,14,1);  
                    ok4 := StrToVal(grip_str,grip_statue);
                    TPWrite "recibidos (camX/robY:" + y_str + " ; camY/robZ:" + z_str + " ; camZ/robX:" + x_str + ") ";            
                    
                    IF grip_statue = 0 THEN
                        g_GripOut;
                        TPWrite "Gripper DERECHA Abierto";
                    ELSEIF grip_statue = 1 THEN
                        g_GripIn;
                        TPWrite "Gripper DERECHA Cerrado";
                    ENDIF    
                        
                    ! Asignar los valores a la estructura de robtarget, SOLO SI la conversion es validada (los 3 ok=TRUE)
                    IF ok1 AND ok2 AND ok3 THEN
                        target_pos_L.trans.x := x_val;
                        target_pos_L.trans.y := y_val;
                        target_pos_L.trans.z := z_val;
                        target_pos_L := LimitTCP(target_pos_L); ! Respectar las limites
                    ENDIF
                            
                ELSE 
                    TPWrite "ERR: Datos inválidos -> ";
                ENDIF
        
                ! Mover el brazo (tool0) a la posición recibida (target_pos_L) con una velocidad de 600mm/s en la zona 50
                MoveJ target_pos_L, v600, z50, tool0;
                        
                !IF NOT IsMoving() THEN
                !    MoveJ target_pos_R, v600, z50, tool0;
                !ENDIF
            ENDWHILE
        ENDWHILE
    ENDPROC
ENDMODULE