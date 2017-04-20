MODULE ServerMike
    !***********************************************************
    !
    ! Module:  ServerMike
    !
    ! Description:
    !   Communicates and moves arm according to the sent string from TCP/IP connection
    !
    ! Author: Michael Pickett
    !
    ! Version: 1.0
    !
    !***********************************************************
    
    !//Robot configuration
    PERS tooldata currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
    PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];   
    PERS speeddata currentSpeed;
    PERS zonedata currentZone;
    
    !//PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    
    PERS string ipController:= "192.168.125.3"; !local IP for testing in simulation
    PERS num serverPort:= 5515;
    
    VAR extjoint externalAxis;
    !Motion
    VAR bool moveCompleted; !true if move successful
    VAR robtarget cartesianTarget;
    VAR jointtarget jointsTarget;
    
    !Procedure for parsing incoming string
    !strings should be the desired type of command followed by % then a space and the params each followed by a space and the end character is a #
    !EXAMPLE STRING: "home% 10, 20, 50, 105, 105, 13, 15,#"
    VAR string params{9}; !collection of parameters for movement goes up to 8 cuz the python lib had something similar plus one for end character to indicate end of params
    VAR string commandType;
    VAR num numParams:= 0; !number of paramters
    
    PROC ParseString(string msg) 
        VAR num barrier;
        VAR num search;
        VAR num length := 0;
        VAR bool end := FALSE;
        VAR num paramLevel := 1;
        VAR num startParam;
        VAR num endParam;
        VAR string current;
        
        barrier:= StrMatch(msg,1,"%");
        search := barrier + 1;
        commandType:= StrPart(msg,1,barrier-1); !goes to grab the first part of the string that is separated by the # sign
        length := StrLen(msg);
        WHILE end = FALSE DO
            current := StrPart(msg,search,1); !looks after where search is set
            IF current = " " THEN
                startParam := search + 1; !go to after " "
                endParam := StrFind(msg,startParam,","); !returns the next instance of a space (needs +1 or it will just see the one it already grabbed
                IF endParam = length+1 THEN
                    !Do nothing this means that we are at the end and another " " couldnt be found after the last one
                ELSE
                    params{paramLevel} := StrPart(msg,startParam,endParam-startParam); !grabs the string from start to before end
                    Incr paramLevel;
                ENDIF
            ELSEIF current = "#" THEN
                params{paramLevel} := "#";
                numParams := paramLevel-1;
                end := TRUE;
            ENDIF
            Incr search; !keep incrementing search 
        ENDWHILE
        
    ENDPROC
    
    ! Procedure for socketConnection
    ! basically the same socket connect from open_abb library
    PROC attemptSocketConnect(string ip, num port)
        
        
        SocketClose clientSocket;
        SocketCreate clientSocket;
        SocketConnect clientSocket,ip,port;
        TPWrite "Client: Client connected";
    ENDPROC
    
    !FROM open_abb lib
    !//Parameter initialization
    !// Loads default values for
    !// - Tool.
    !// - WorkObject.
    !// - Zone.
    !// - Speed.
    PROC Initialize()
        currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
        currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
        currentSpeed := [100, 50, 0, 0];
        currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
    	
    	!Find the current external axis values so they don't move when we start
    	jointsTarget := CJointT();
    	externalAxis := jointsTarget.extax;
    ENDPROC
    
    !***********************************************************
    !
    ! Procedure main
    !
    !   This is the entry point of your program
    !
    !***********************************************************
    PROC main()
        
        VAR string receivedMsg; !Received String
        VAR string sendMsg; !Reply String
        VAR string addToMsg; !Additional info for send if needed
        VAR bool connected; !says if if cliet connected
        VAR robtarget cartesianPose;
        VAR jointtarget jointsPose;
        VAR num coors{9};
        VAR bool ok;
        
        !Motion Config
        ConfL \Off;
        SingArea \Wrist;
        moveCompleted:= TRUE;
        
        !initialize robot
        Initialize;
        
        !Connect to socket
        connected := FALSE;
        attemptSocketConnect ipController, serverPort;
        connected := TRUE;
        WHILE TRUE DO
            
            !Receive Msg
            SocketReceive clientSocket \Str:= receivedMsg \Time:=WAIT_MAX;
            ParseString receivedMsg;
            
            TEST commandType
                CASE "home": !home the robot
                    TPWrite "Homing robot...";
                    
                    cartesianTarget := [[1000,0,1000],[0,0,1,0],[0,0,0,0],externalAxis];
                    moveCompleted := FALSE;
                    MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                    moveCompleted := TRUE;
                    
                    TPWrite "Homing Successful";
                CASE "cmove": !cartesian move
                    TPWrite "Cartesian moving...";
                    !make sure we got enough params
                    IF numParams = 7 THEN
                        !convert string values into num values
                        ok := StrToVal(params{1},coors{1});
                        ok := StrToVal(params{2},coors{2});
                        ok := StrToVal(params{3},coors{3});
                        ok := StrToVal(params{4},coors{4});
                        ok := StrToVal(params{5},coors{5});
                        ok := StrToVal(params{6},coors{6});
                        ok := StrToVal(params{7},coors{7});
                        
                        cartesianTarget := [[coors{1},coors{2},coors{3}],[coors{4},coors{5},coors{6},coors{7}],[0,0,0,0],externalAxis];
                        moveCompleted := FALSE;
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                        moveCompleted := TRUE;
                    ELSE
                        TPWrite "Bad msg. Not enough or too many params";
                    ENDIF
                    TPWrite "Cartesian Movement Successful";
                CASE "jmove": !joint move (just homes rn)
                    TPWrite "Joint moving...";
                     IF numParams = 6 THEN
                        jointsTarget:=[[0,0,0,0,0,0], externalAxis];
                        
                        moveCompleted := FALSE;
                        MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                        moveCompleted := TRUE;
                    ELSE
                        TPWrite "Bad msg. Not enough or too many params";
                    ENDIF
                    TPWrite "Joint Movement Successful";
                CASE "circmove": !circular move
                    TPWrite "Circle moving...";
                    !add code
                    TPWrite "Circle Movement Successful";
                CASE "grab": !grab on\off
                    TPWrite "Grabbing or Releasing...";
                    !add code
                    TPWrite "Grabbed";
                    !in other side of if
                    TPWrite "Released";
                CASE "poke": !poke tower
                    TPWrite "Poking...";
                    !add code
                    TPWrite "Poked";
                    !if too dangerous
                    TPWrite "Too dangerous, pulling out....";
                    TPWrite "Pulled Out";
                CASE "setwobj": !set workobject
                    TPWrite "Setting workobject...";
                    !add code
                    TPWrite "Set workobject";
                CASE "settool": !set tool coordinates
                    TPWrite "Setting tool...";
                    !add code
                    TPWrite "Set tool";
                CASE "setzone": !set zone
                    TPWrite "Setting Zone...";
                    !add code
                    TPWrite "Set zone";
                CASE "setspeed": !set speed
                    TPWrite "Setting Speed...";
                    !add code
                    TPWrite "Set Speed";
                CASE "getcart": !get cartesian coordinates
                    TPWrite "Getting Cartesian Coordinates...";
                    IF numParams = 0 THEN
                        cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                        addToMsg := " x = " + NumToStr(cartesianPose.trans.x,2);
                        addToMsg := addToMsg + " y = " + NumToStr(cartesianPose.trans.y,2);
                        addToMsg := addToMsg + " z = " + NumToStr(cartesianPose.trans.z,2);
                        addToMsg := addToMsg + " q1 = " + NumToStr(cartesianPose.rot.q1,3);
                        addToMsg := addToMsg + " q2 = " + NumToStr(cartesianPose.rot.q2,3);
                        addToMsg := addToMsg + " q3 = " + NumToStr(cartesianPose.rot.q3,3);
                        addToMsg := addToMsg + " q4 = " + NumToStr(cartesianPose.rot.q4,3);
                        
                        TPWrite "Cartesian Coordinate is:" + addToMsg;
                        
                    ELSE
                        TPWrite "Bad message, too many params";
                    ENDIF
                    
                CASE "getjoint": !get joint coordinates
                DEFAULT:
                    TPWrite "Client: Received Bad Instruction";
                    commandType := "home"; !home the robot by default
            ENDTEST
            
            !send message back
            IF connected = TRUE THEN
                IF SocketGetStatus(clientSocket) = SOCKET_CONNECTED THEN
                    sendMsg := "Received Following Command Type: " + commandType; !tells client the command it recieved
                    TPWrite "Sending message back to client";
                    SocketSend clientSocket \Str:=sendMsg;
                ENDIF
            ENDIF
                    
        ENDWHILE
            
        
    ENDPROC
ENDMODULE