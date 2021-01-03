MODULE Module1
    CONST robtarget Target_10:=[[364.353829072,0,594],[0.5,0,0.866025404,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[471.057655995,0,347.222351303],[0.234175779,0,0.972194273,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[127.502499513,453.473734509,347.222351303],[0.186656812,-0.587082673,0.774916537,0.141412623],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40:=[[367.543139828,294.630880317,347.222351303],[0.220936554,-0.322256477,0.917230869,0.077623026],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_50:=[[492,0,480],[0,0,1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];     !Demonstration Square Corner 1
	CONST robtarget Target_60:=[[492,110,480],[0,0,1,0],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];  !Demonstration Square Corner 2
	CONST robtarget Target_70:=[[330,110,480],[0,0,1,0],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];  !Demonstration Square Corner 3
	CONST robtarget Target_80:=[[330,0,480],[0,0,1,0],[0,1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];     !Demonstration Square Corner 4
	CONST robtarget Target_90:=[[482.389508449,0,480],[0,0,1,0],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_100:=[[482.389508449,0,480],[0,0,1,0],[0,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    
    !Declaration joint variables
    VAR string jnt1PosDegStr;
    VAR string jnt2PosDegStr;
    VAR string jnt3PosDegStr;
    VAR string jnt4PosDegStr;
    VAR string jnt5PosDegStr;
    VAR string jnt6PosDegStr;
    
    VAR num jnt1PosDeg;
    VAR num jnt2PosDeg;
    VAR num jnt3PosDeg;
    VAR num jnt4PosDeg;
    VAR num jnt5PosDeg;
    VAR num jnt6PosDeg;
    
    VAR num jntArray1PosDeg{681};
    VAR num jntArray2PosDeg{681};
    VAR num jntArray3PosDeg{681};
    VAR num jntArray4PosDeg{681};
    VAR num jntArray5PosDeg{681};
    VAR num jntArray6PosDeg{681};
    VAR num MoveTimeArray{681};
    CONST num arraysize := 681;
    
    !Declaration TCP movement speed variable
    VAR string MoveAbsJ_TCPTimeStr;
    VAR num MoveAbsJ_TCPTime;
    
    VAR bool ok1;
    VAR bool ok2;
    VAR bool ok3;
    VAR bool ok4;
    VAR bool ok5;
    VAR bool ok6;
    VAR bool ok7;
    
    !Declaration Server variables
	VAR socketdev server;
    VAR socketdev client;
    VAR string message1;
    VAR rawbytes data;
    
    !Initialize jointtarget variable rotJoints
    VAR jointtarget rotJoints1:= [[0,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	  
    !***********************************************************
    !
    ! Module:  Module1
    !
    ! Description:
    !   This module receives data packet from Matlab (or any TCP IP client).  The data packet is changed from a string to numbers,
    !    which is thenentered as absolute joint position (MoveAbsJ).
    ! Author: Marlon Scott (mscott@wpi.edu)
    !
    ! Version: 1.0
    !
    !***********************************************************
    
    
    !***********************************************************
    !
    ! Procedure main
    !
    !   This is the entry point of your program
    !
    !***********************************************************
    PROC main()
        !Add your code here
        MoveAngleMatlab;
!        Path_10;
!        Path_20;
    ENDPROC
    
    PROC Path_10()
        MoveL Target_10,v500,z10,tool1\WObj:=RBE502_SqTest;
        MoveL Target_20,v500,z10,tool1\WObj:=RBE502_SqTest;
        MoveL Target_30,v500,z10,tool1\WObj:=RBE502_SqTest;
        MoveL Target_40,v500,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_50,v500,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_60,v500,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_70,v500,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_80,v500,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_90,v500,z10,tool1\WObj:=RBE502_SqTest;
        MoveL Target_100,v500,z10,tool1\WObj:=RBE502_SqTest;
    ENDPROC
    
    PROC Path_20() 
		MoveL Target_50,v500,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_60,v500,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_70,v200,z10,tool1\WObj:=RBE502_SqTest;
		MoveL Target_80,v500,z10,tool1\WObj:=RBE502_SqTest;
    ENDPROC
    
    PROC MoveAngleMatlab ()
	
	!Create communcation
	SocketCreate server;
	SocketBind server,"127.0.0.1", 55000;
    SocketListen server;
    
    
    !Waiting for connection request
    WHILE TRUE DO
        
        FOR i FROM 1 TO arraysize DO
            SocketAccept server, client;
    
            !Receive a message from the client
            SocketReceive client,\RawData:=data;
            !UnpackRawBytes data,1,message1,\ASCII:=15;    
        
            !Ensure sent string data is 8 byte increments, each digit = 1 byte
            UnpackRawBytes data,1,jnt1PosDegStr,\ASCII:=8; !Joint1 raw data starts at byte 1
            UnpackRawBytes data,9,jnt2PosDegStr,\ASCII:=8; !Joint2 raw data starts at byte 9
            UnpackRawBytes data,17,jnt3PosDegStr,\ASCII:=8; !Joint3 raw data starts at byte 17
            UnpackRawBytes data,25,jnt4PosDegStr,\ASCII:=8; !Joint4 raw data starts at byte 25
            UnpackRawBytes data,33,jnt5PosDegStr,\ASCII:=8; !Joint5 raw data starts at byte 33
            UnpackRawBytes data,41,jnt6PosDegStr,\ASCII:=8; !Joint6 raw data starts at byte 41
            UnpackRawBytes data,49,MoveAbsJ_TCPTimeStr,\ASCII:=4; !EE velocity raw data starts at byte 49  
        
            ok1 := StrToVal(jnt1PosDegStr, jntArray1PosDeg{i}); 
            ok2 := StrToVal(jnt2PosDegStr, jntArray2PosDeg{i});
            ok3 := StrToVal(jnt3PosDegStr, jntArray3PosDeg{i});
            ok4 := StrToVal(jnt4PosDegStr, jntArray4PosDeg{i});
            ok5 := StrToVal(jnt5PosDegStr, jntArray5PosDeg{i});
            ok6 := StrToVal(jnt6PosDegStr, jntArray6PosDeg{i});
            ok7 := StrToVal(MoveAbsJ_TCPTimeStr, MoveTimeArray{i});
            
            SocketClose client;! Added here
        ENDFOR
    
        !Add FOR loop here
        FOR i FROM 1 TO arraysize DO
        
        rotJoints1.robax.rax_1:=jntArray1PosDeg{i};  !Save value jnt1PosDeg to jointtarget rotJoints joint 1
        rotJoints1.robax.rax_2:=jntArray2PosDeg{i};  !Save value jnt2PosDeg to jointtarget rotJoints joint 2
        rotJoints1.robax.rax_3:=jntArray3PosDeg{i};  !etc...
        rotJoints1.robax.rax_4:=jntArray4PosDeg{i};
        rotJoints1.robax.rax_5:=jntArray5PosDeg{i};
        rotJoints1.robax.rax_6:=jntArray6PosDeg{i};
        
        !MoveAbsJ rotJoints1, v500,z10, tool1\WObj:=RBE502_SqTest;
        !MoveAbsJ rotJoints1, v500\T:=MoveAbsJ_TCPTime,z10, tool1\WObj:=RBE502_SqTest;
        MoveAbsJ rotJoints1, v500\T:=MoveTimeArray{i},z10, tool1\WObj:=RBE502_SqTest;
        
        ENDFOR   
        
    ENDWHILE
    
    ERROR   
        RETRY;
        
    UNDO
        !Close communication        
        SocketClose server;
        SocketClose client;
    ENDPROC
ENDMODULE