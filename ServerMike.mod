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
    PERS wobjdata pokerWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    PERS speeddata currentSpeed;
    PERS zonedata currentZone;
    PERS num jointYCoors{6} := [-122.856,22.9693,36.8172,139.507,67.1776,-71.8572];
    PERS num jointXCoors{6} := [-112.992,34.6445,-4.01666,-94.2035,82.6203,-142.63];
    
    
    !//PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    
    PERS string ipController:= "192.168.125.3"; !local IP for testing in simulation
    PERS num serverPort:= 5515;
    
    !Jenga block dimensions
    PERS num towerOffset := 40; !75mm offset from poker to tower distance at start of approach
    PERS num towerGripOffset := 40; !mm for offset from gripper to tower
    PERS num length := 75; !block length
    PERS num width := 25; !block width
    PERS num gripperOffset := 13; !difference in reach length between poker and gripper
    PERS num aboveTower := 300; !mm above tower for movearound
    PERS num lengthTool := 300; !mm of length of tooling
    
    PERS num zGrab := 56.61;
    VAR extjoint externalAxis;
    
    PERS robtarget pokeOffset; !true offset
    PERS robtarget pokeXOffset; !poke offset in x dir
    PERS robtarget grabXOffset; !grab offset in x dir
    PERS robtarget pokeYOffset; !poke offset in y dir
    PERS robtarget grabYOffset; !grab offset in y dir
    
    
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
        
        pokeOffset := [[0,0,0],[0,0,0,0],[0,0,0,0],externalAxis]; !true offset
        pokeXOffset := [[-581.11,-965.31,420.5],[0.001,0.024,1,0.001],[0,0,0,0],externalAxis]; !poke offset in x dir
        grabXOffset := [[-209.49,-974.43,363.89],[0.483,0.508,-0.513,-0.496],[0,0,0,0],externalAxis]; !grab offset in x dir
        pokeYOffset := [[-318.32,-1153.4,316.18],[0.483,0.508,-0.513,-0.496],[0,0,0,0],externalAxis]; !poke offset in y dir
        grabYOffset := [[-369.23,-800.90,315.51],[0.705,0.709,-0.013,-0.005],[0,0,0,0],externalAxis]; !grab offset in y dir
        
        
        
        
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
        VAR num pokeCount := 0; !number of times its checked with pi for limit
        VAR bool triggered := FALSE;
        
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
                    
                CASE "jmove": !joint move 
                    TPWrite "Joint moving...";
                     IF numParams = 6 THEN
                        !convert string values into num values
                        ok := StrToVal(params{1},coors{1});
                        ok := StrToVal(params{2},coors{2});
                        ok := StrToVal(params{3},coors{3});
                        ok := StrToVal(params{4},coors{4});
                        ok := StrToVal(params{5},coors{5});
                        ok := StrToVal(params{6},coors{6});
                        
                        jointsTarget:=[[coors{1},coors{2},coors{3},coors{4},coors{5},coors{6}], externalAxis];
                        
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
                CASE "moveAround":
                    TPWrite "Moving Around...";
                    IF numParams = 1 THEN
                            !first move up
                            coors{3} := coors{3} + aboveTower; !goes up 900 mm
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            TPWrite "Moved to Point 2 of Circumnav";
                            WaitTime 0.5; !wait before next move
                            TPWrite "Moving to Point 3...";
                        
                        IF params{1} = "x" THEN
                            
                            
                            !TPWrite "Rotating End-Effector";
                            !jointsTarget:=[[jointXCoors{1},jointXCoors{2},jointXCoors{3},jointXCoors{4},jointXCoors{5},jointXCoors{6}], externalAxis];
                        
                            !moveCompleted := FALSE;
                            !MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                            !moveCompleted := TRUE;
                            !WaitTime 0.5;
                            
                            !TPWrite "Rotated";
                            !have to keep tooling orientation
                            !might mess with tcp?????
                            !cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                            !coors{4} := cartesianPose.rot.q1;
                            !coors{5} := cartesianPose.rot.q2;
                            !coors{6} := cartesianPose.rot.q3;
                            !coors{7} := cartesianPose.rot.q4;
                            
                            coors{1} := coors{1} + towerOffset + towerOffset + length + lengthTool; !might be an issue not going far enough?
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            TPWrite "Moved to Point 3";
                            WaitTime 0.5;
                            
                            !set to current joint angles to not mess with linear movement
                            !jointsPose := CJointT();
                            !jointXCoors{1} := jointsPose.robax.rax_1;
                            !jointXCoors{2} := jointsPose.robax.rax_2;
                            !jointXCoors{3} := jointsPose.robax.rax_3;
                            
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[grabXOffset.rot.q1,grabXOffset.rot.q2,grabXOffset.rot.q3,grabXOffset.rot.q4],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            TPWrite "Moved to Point 3";
                            WaitTime 0.5;
                            
                            coors{3} := pokeOffset.trans.z - zGrab; !go back down should message 
                            
                            coors{1} := 0; !might be wrong math so make sure to check (minus 21 for block extruding)
                            TPWrite "Moving to grab...";
                            cartesianTarget := [[grabXOffset.trans.x + coors{1},grabXOffset.trans.y +coors{2},coors{3}],[grabXOffset.rot.q1+coors{4},grabXOffset.rot.q2+coors{5},grabXOffset.rot.q3+coors{6},grabXOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            TPWrite "Moved to grab";
                            !grab
                            !add code here
                            TPWrite "I grabbed it";
                            
                            TPWrite "Pulling block out";
                            coors{1} := coors{1} + 90; !using 90 to be safe distance
                            cartesianTarget := [[grabXOffset.trans.x + coors{1},grabXOffset.trans.y +coors{2},coors{3}],[grabXOffset.rot.q1+coors{4},grabXOffset.rot.q2+coors{5},grabXOffset.rot.q3+coors{6},grabXOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            
                            TPWrite "Pulled out";
                            
                        ELSEIF params{1} = "y" THEN
                            
                            !TPWrite "Rotating End-Effector";
                            !jointsTarget:=[[jointYCoors{1},jointYCoors{2},jointYCoors{3},jointYCoors{4},jointYCoors{5},jointYCoors{6}], externalAxis];
                        
                            !moveCompleted := FALSE;
                            !MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                            !moveCompleted := TRUE;
                            !WaitTime 0.5;
                            
                            !TPWrite "Rotated";
                            !have to keep tooling orientation
                            !might mess with tcp?????
                            !cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                            !coors{4} := cartesianPose.rot.q1;
                            !coors{5} := cartesianPose.rot.q2;
                            !coors{6} := cartesianPose.rot.q3;
                            !coors{7} := cartesianPose.rot.q4;
                            
                            coors{2} := coors{2} + towerOffset + towerOffset + length + lengthTool;
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            TPWrite "Moved to Point 3";
                            WaitTime 0.5;
                            !set to current joint angles to not mess with linear movement
                            
                            !jointsPose := CJointT();
                            !jointXCoors{1} := jointsPose.robax.rax_1;
                            !jointXCoors{2} := jointsPose.robax.rax_2;
                            !jointXCoors{3} := jointsPose.robax.rax_3;
                            
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[grabYOffset.rot.q1,grabYOffset.rot.q2,grabYOffset.rot.q3,grabYOffset.rot.q4],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            TPWrite "Moved to Point 3";
                            WaitTime 0.5;
                            
                            coors{3} := pokeOffset.trans.z - zGrab; !go back down
                            
                            coors{2} := 0; !check math here or you might regret it (minus 21 for block extruding)
                            TPWrite "Moving to grab...";
                            cartesianTarget := [[grabYOffset.trans.x + coors{1},grabYOffset.trans.y +coors{2},coors{3}],[grabYOffset.rot.q1+coors{4},grabYOffset.rot.q2+coors{5},grabYOffset.rot.q3+coors{6},grabYOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            TPWrite "Moved to grab";
                            !grab
                            Set D652_10_DO1;
                            TPWrite "I grabbed it";
                            
                            TPWrite "Pulling block out";
                            coors{2} := coors{2} + 90; !using 90 to be safe distance
                            cartesianTarget := [[grabYOffset.trans.x + coors{1},grabYOffset.trans.y +coors{2},coors{3}],[grabYOffset.rot.q1+coors{4},grabYOffset.rot.q2+coors{5},grabYOffset.rot.q3+coors{6},grabYOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            
                            TPWrite "Pulled out";
                            
                        ELSE
                            TPWrite "Bad msg, didn't give a good coordinate";
                            WaitTime 10; !waits for a while
                        ENDIF
                        
                        
                        
                    ELSE
                        TPWrite "Bad msg, wrong number of params";
                    ENDIF
                    
                    
                    
                    
                CASE "setJointCoors": !sets the movement of the joint twist
                    TPWrite "Setting joint coors...";
                    IF numParams = 1 THEN 
                        IF params{1} = "x" THEN
                            jointsPose := CJointT();
                            jointXCoors{1} := jointsPose.robax.rax_1;
                            jointXCoors{2} := jointsPose.robax.rax_2;
                            jointXCoors{3} := jointsPose.robax.rax_3;
                            jointXCoors{4} := jointsPose.robax.rax_4;
                            jointXCoors{5} := jointsPose.robax.rax_5;
                            jointXCoors{6} := jointsPose.robax.rax_6;    
                        ELSEIF params{1} = "y" THEN
                            jointsPose := CJointT();
                            jointYCoors{1} := jointsPose.robax.rax_1;
                            jointYCoors{2} := jointsPose.robax.rax_2;
                            jointYCoors{3} := jointsPose.robax.rax_3;
                            jointYCoors{4} := jointsPose.robax.rax_4;
                            jointYCoors{5} := jointsPose.robax.rax_5;
                            jointYCoors{6} := jointsPose.robax.rax_6;
                        ELSE
                            TPWrite "Bad msg, didn't give a good coordinate";
                            WaitTime 10; !waits for a while
                        ENDIF
                    ELSE
                        TPWrite "Bad message, wrong number of params";
                    ENDIF
                CASE "poke": !poke tower
                    TPWrite "Poke commencing...";
                    !initial positioning in front of block
                    IF numParams = 8 and pokeCount = 0 THEN
                        !convert string values into num values
                        ok := StrToVal(params{1},coors{1});
                        ok := StrToVal(params{2},coors{2});
                        ok := StrToVal(params{3},coors{3});
                        ok := StrToVal(params{4},coors{4});
                        ok := StrToVal(params{5},coors{5});
                        ok := StrToVal(params{6},coors{6});
                        ok := StrToVal(params{7},coors{7});
                        
                        IF params{8} = "x" THEN
                            pokeOffset := pokeXOffset;
                        ELSEIF params{8} = "y" THEN
                            pokeOffset := pokeYOffset;
                        ELSE
                            TPWrite "not x or y param";
                        ENDIF
                        
                        cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                        moveCompleted := FALSE;
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                        moveCompleted := TRUE;
                        TPWrite "Moved in front of block";
                        !Wait for a little before poking
                        WaitTime 0.5;
                    ELSE
                        TPWrite "Bad msg. Not enough or too many params";
                    ENDIF
                    
                    
                    !while for pokecount to make small movements to test poke
                    !need to set the pokerwobj before doing this
                    !will need to adjust so that when moving cartesian is accurate (set pokerwobj to corner of tower)
                    !at a poke count of 3 the while stops
                    WHILE pokeCount <> 3 DO
                        IF pokeCount = 0 AND triggered = FALSE THEN !move up to the edge of the tower
                            TPWrite "Moving up on tower";
                            !check if poke is occuring along x or y
                            !tower will have to be placed so that its parallel to the axis of the wobj
                            IF params{8} = "y" THEN
                                coors{2} := coors{2} + towerOffset; ! might need to be subtracting depending on directions
                            ELSEIF params{8} = "x" THEN
                                coors{1} := coors{1} + towerOffset; ! might need to be subtracting depending on directions
                            ELSE
                                TPWrite "Do not recognize poke direction";
                            ENDIF
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            !send message asking for triggered update
                            sendMsg := "trigger";
                            SocketSend clientSocket \Str:=sendMsg;
                            SocketReceive clientSocket \Str:=receivedMsg \Time:=WAIT_MAX;
                            IF receivedMsg = "true" THEN
                                triggered := TRUE;
                            ENDIF
                            TPWrite "Moved up on tower successfully";
                        ELSEIF pokeCount = 1 AND triggered = FALSE THEN
                            !move slightly further to engage more poke
                            TPWrite "Giving tower just the tip";
                            IF params{8} = "y" THEN
                                coors{2} := coors{2} + 1; ! might need to be subtracting depending on directions
                            ELSEIF params{8} = "x" THEN
                                coors{1} := coors{1} + 1; ! might need to be subtracting depending on directions
                            ELSE
                                TPWrite "Do not recognize poke direction";
                            ENDIF
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            !send message asking for triggered update
                            sendMsg := "trigger";
                            SocketSend clientSocket \Str:=sendMsg;
                            SocketReceive clientSocket \Str:=receivedMsg \Time:=WAIT_MAX;
                            IF receivedMsg = "true" THEN
                                triggered := TRUE;
                            ENDIF
                            TPWrite "Gave tower the tip";
                        ELSEIF pokeCount = 2 AND triggered = FALSE THEN
                            !move for full poke
                            TPWrite "Going full poke...";
                            IF params{8} = "y" THEN
                                !move by half block size minus the 1 mm from first poke attempt
                                coors{2} := coors{2} + 20; ! might need to be subtracting depending on directions
                            ELSEIF params{8} = "x" THEN
                                !move by half block size minus the 1 mm from first poke attempt
                                coors{1} := coors{1} + 20; ! might need to be subtracting depending on directions
                            ELSE
                                TPWrite "Do not recognize poke direction";
                            ENDIF
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            !send message asking for triggered update
                            sendMsg := "trigger";
                            SocketSend clientSocket \Str:=sendMsg;
                            SocketReceive clientSocket \Str:=receivedMsg \Time:=WAIT_MAX;
                            IF receivedMsg = "true" THEN
                                triggered := TRUE;
                            ENDIF
                            TPWrite "Did the full poke";
                        ELSE
                            !triggered so needs to go back to before poke
                            TPWrite "Triggered limit";
                            ok := StrToVal(params{1},coors{1});
                            ok := StrToVal(params{2},coors{2});
                            cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                            moveCompleted := FALSE;
                            MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                            moveCompleted := TRUE;
                            
                        ENDIF
                        !increment pokeCount
                        Incr pokeCount;
                    
                    ENDWHILE
                    !go back to before poke after completeing poke
                    TPWrite "Pulling out...";
                    ok := StrToVal(params{1},coors{1});
                    ok := StrToVal(params{2},coors{2});
                    cartesianTarget := [[pokeOffset.trans.x + coors{1},pokeOffset.trans.y +coors{2},pokeOffset.trans.z+coors{3}],[pokeOffset.rot.q1+coors{4},pokeOffset.rot.q2+coors{5},pokeOffset.rot.q3+coors{6},pokeOffset.rot.q4+coors{7}],[0,0,0,0],externalAxis];
                    moveCompleted := FALSE;
                    MoveL cartesianTarget,currentSpeed,currentZone,currentTool \WObj:=currentWobj;
                    moveCompleted := TRUE;
                    pokeCount := 0;
                    TPWrite "Pulled out";
                    !send message asking for triggered update
                    sendMsg := "complete";
                    SocketSend clientSocket \Str:=sendMsg;
                    TPWrite "Poke Attempt Completed";
                    
                    
                CASE "setwobj": !set workobject with our own coordinates
                    TPWrite "Setting workobject...";
                    !grabs coordinates and puts them into currentWobj
                    IF numParams = 7 THEN
                        ok := StrToVal(params{1},coors{1});
                        ok := StrToVal(params{2},coors{2});
                        ok := StrToVal(params{3},coors{3});
                        ok := StrToVal(params{4},coors{4});
                        ok := StrToVal(params{5},coors{5});
                        ok := StrToVal(params{6},coors{6});
                        ok := StrToVal(params{7},coors{7});
                        
                        currentWobj.oframe.trans.x:=coors{1};
                        currentWobj.oframe.trans.y:=coors{2};
                        currentWobj.oframe.trans.z:=coors{3};
                        currentWobj.oframe.rot.q1:=coors{4};
                        currentWobj.oframe.rot.q2:=coors{5};
                        currentWobj.oframe.rot.q3:=coors{6};
                        currentWobj.oframe.rot.q4:=coors{7};
                    ELSE
                       TPWrite "Bad msg. Wrong number of params";
                    ENDIF
                    TPWrite "Set workobject";
                    
                CASE "setPokeOffset":
                    TPWrite "Setting poke offset...";
                    IF numParams = 1 THEN
                        IF params{1} = "x" THEN
                            cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                    
                            pokeXOffset.trans.x := cartesianPose.trans.x;
                            pokeXOffset.trans.y := cartesianPose.trans.y;
                            pokeXOffset.trans.z := cartesianPose.trans.z;
                            pokeXOffset.rot.q1 := cartesianPose.rot.q1;
                            pokeXOffset.rot.q2 := cartesianPose.rot.q2;
                            pokeXOffset.rot.q3 := cartesianPose.rot.q3;
                            pokeXOffset.rot.q4 := cartesianPose.rot.q4;
                            
                        ELSEIF params{1} = "y" THEN
                            cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                    
                            pokeYOffset.trans.x := cartesianPose.trans.x;
                            pokeYOffset.trans.y := cartesianPose.trans.y;
                            pokeYOffset.trans.z := cartesianPose.trans.z;
                            pokeYOffset.rot.q1 := cartesianPose.rot.q1;
                            pokeYOffset.rot.q2 := cartesianPose.rot.q2;
                            pokeYOffset.rot.q3 := cartesianPose.rot.q3;
                            pokeYOffset.rot.q4 := cartesianPose.rot.q4;
                        ELSE
                            TPWrite "Wrong params";
                        ENDIF
                    ELSE
                        TPWrite "Wrong number of params";
                    ENDIF
                    
                    
                    TPWrite "Set poke offset";
                CASE "setGrabOffset":
                    TPWrite "Setting grab offset...";
                    IF numParams = 1 THEN
                        IF params{1} = "x" THEN
                            cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                    
                            grabXOffset.trans.x := cartesianPose.trans.x;
                            grabXOffset.trans.y := cartesianPose.trans.y;
                            grabXOffset.trans.z := cartesianPose.trans.z;
                            grabXOffset.rot.q1 := cartesianPose.rot.q1;
                            grabXOffset.rot.q2 := cartesianPose.rot.q2;
                            grabXOffset.rot.q3 := cartesianPose.rot.q3;
                            grabXOffset.rot.q4 := cartesianPose.rot.q4;
                            
                        ELSEIF params{1} = "y" THEN
                            cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                    
                            grabYOffset.trans.x := cartesianPose.trans.x;
                            grabYOffset.trans.y := cartesianPose.trans.y;
                            grabYOffset.trans.z := cartesianPose.trans.z;
                            grabYOffset.rot.q1 := cartesianPose.rot.q1;
                            grabYOffset.rot.q2 := cartesianPose.rot.q2;
                            grabYOffset.rot.q3 := cartesianPose.rot.q3;
                            grabYOffset.rot.q4 := cartesianPose.rot.q4;
                        ELSE
                            TPWrite "Wrong params";
                        ENDIF
                    ELSE
                        TPWrite "Wrong number of params";
                    ENDIF
                    
                CASE "setwobjCurrent":
                    TPWrite "Setting workobject at current position...";
                    !grab current cartesian pose
                    IF numParams = 1 THEN
                        IF params{1} = "poker" THEN
                            cartesianPose := CRobT(\Tool:=currentTool \WObj:= pokerWobj);
                    
                            pokerWobj.oframe.trans.x := cartesianPose.trans.x;
                            pokerWobj.oframe.trans.y := cartesianPose.trans.y;
                            pokerWobj.oframe.trans.z := cartesianPose.trans.z;
                            pokerWobj.oframe.rot.q1 := cartesianPose.rot.q1;
                            pokerWobj.oframe.rot.q2 := cartesianPose.rot.q2;
                            pokerWobj.oframe.rot.q3 := cartesianPose.rot.q3;
                            pokerWobj.oframe.rot.q4 := cartesianPose.rot.q4;
                        ENDIF
                    
                        TPWrite "Set pokerWobj at current position";
                    ELSE
                        TPWrite "Setting current position to currentWobj";
                        cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                    
                        currentWobj.oframe.trans.x := cartesianPose.trans.x;
                        currentWobj.oframe.trans.y := cartesianPose.trans.y;
                        currentWobj.oframe.trans.z := cartesianPose.trans.z;
                        currentWobj.oframe.rot.q1 := cartesianPose.rot.q1;
                        currentWobj.oframe.rot.q2 := cartesianPose.rot.q2;
                        currentWobj.oframe.rot.q3 := cartesianPose.rot.q3;
                        currentWobj.oframe.rot.q4 := cartesianPose.rot.q4;
                    ENDIF
                    
                    
                CASE "settool": !set tool coordinates
                    TPWrite "Setting tool...";
                    !add code
                    TPWrite "Set tool";
                    
                CASE "setzone": !set zone
                    TPWrite "Setting Zone...";
                    !takes 4 params but will look to see if the first param is a "fine" which means fine zone
                    IF numParams = 4 THEN
                        IF params{1} = "fine" THEN
                            currentZone.finep := TRUE;
                            currentZone.pzone_tcp := 0.0;
                            currentZone.pzone_ori := 0.0;
                            currentZone.zone_ori := 0.0;
                        ELSE
                            currentZone.finep := FALSE;
                            ok := StrToVal(params{2},coors{2});
                            ok := StrToVal(params{3},coors{3});
                            ok := StrToVal(params{4},coors{4});

                            currentZone.pzone_tcp := coors{2};
                            currentZone.pzone_ori := coors{3};
                            currentZone.zone_ori := coors{4};
                        ENDIF
                    ELSE
                        TPWrite "Bad msg, wrong number of params";
                    ENDIF
                    
                    TPWrite "Set zone";
                    
                CASE "setspeed": !set speed
                    TPWrite "Setting Speed...";
                    !need to research how to use this better but this is the current code
                    IF numParams = 4 THEN
                        !convert to num
                        ok := StrToVal(params{1},coors{1});
                        ok := StrToVal(params{2},coors{2});
                        ok := StrToVal(params{3},coors{3});
                        ok := StrToVal(params{4},coors{4});
                        
                        currentSpeed.v_tcp:=coors{1};
                        currentSpeed.v_ori:=coors{2};
                        currentSpeed.v_leax:=coors{3};
                        currentSpeed.v_reax:=coors{4};
                        
                    ELSEIF numParams = 2 THEN
                        
                        ok := StrToVal(params{1},coors{1});
                        ok := StrToVal(params{2},coors{2});
                        
    					currentSpeed.v_tcp:=coors{1};
    					currentSpeed.v_ori:=coors{2};
    					
    				ELSE
                        TPWrite "Bad msg, wrong number of params";
                    ENDIF
                    TPWrite "Set Speed";
                    
                CASE "getcart": !get cartesian coordinates
                    TPWrite "Getting Cartesian Coordinates...";
                    IF numParams = 0 THEN
                        cartesianPose := CRobT(\Tool:=currentTool \WObj:= currentWobj);
                        addToMsg := " x = " + NumToStr(cartesianPose.trans.x,2);
                        TPWrite "Cartesian Coordinate: " + addToMsg;
                        addToMsg := " y = " + NumToStr(cartesianPose.trans.y,2);
                        TPWrite "Cartesian Coordinate: " + addToMsg;
                        addToMsg := " z = " + NumToStr(cartesianPose.trans.z,2);
                        TPWrite "Cartesian Coordinate: " + addToMsg;
                        addToMsg := " q1 = " + NumToStr(cartesianPose.rot.q1,3);
                        TPWrite "Cartesian Coordinate: " + addToMsg;
                        addToMsg := " q2 = " + NumToStr(cartesianPose.rot.q2,3);
                        TPWrite "Cartesian Coordinate: " + addToMsg;
                        addToMsg := " q3 = " + NumToStr(cartesianPose.rot.q3,3);
                        TPWrite "Cartesian Coordinate: " + addToMsg;
                        addToMsg := " q4 = " + NumToStr(cartesianPose.rot.q4,3);
                        TPWrite "Cartesian Coordinate: " + addToMsg;
                        !message is too big to for tpwrite (should just send to python but just gonna split it up)
                        
                        
                    ELSE
                        TPWrite "Bad message, too many params";
                    ENDIF
                    
                CASE "getjoint": !get joint coordinates
                    TPWrite "Getting Joint Angles...";
                    IF numParams = 0 THEN
                        jointsPose := CJointT();
                        addToMsg := " Joint 1 = " + NumToStr(jointsPose.robax.rax_1,2);
                        TPWrite "Joint Angles are:" + addToMsg;
                        addToMsg := " Joint 2 = " + NumToStr(jointsPose.robax.rax_2,2);
                        TPWrite "Joint Angle:" + addToMsg;
                        addToMsg := " Joint 3 = " + NumToStr(jointsPose.robax.rax_3,2);
                        TPWrite "Joint Angle:" + addToMsg;
                        addToMsg := " Joint 4 = " + NumToStr(jointsPose.robax.rax_4,2);
                        TPWrite "Joint Angle:" + addToMsg;
                        addToMsg := " Joint 5 = " + NumToStr(jointsPose.robax.rax_5,2);
                        TPWrite "Joint Angle:" + addToMsg;
                        addToMsg := " Joint 6 = " + NumToStr(jointsPose.robax.rax_6,2);
                        TPWrite "Joint Angle:" + addToMsg;
                        
                    ELSE
                        TPWrite "Bad msg, too many params";
                    ENDIF
                    
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