///////////////////////////////////////////////////////////////////////////////
//
// IAR ANSI C/C++ Compiler V9.32.1.338/W64 for ARM        01/Jun/2023  10:14:36
// Copyright 1999-2022 IAR Systems AB.
//
//    Cpu mode     =  thumb
//    Endian       =  little
//    Source file  =
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l.c
//    Command line =
//        -f
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\Obj\FlashApollo4l.o.rsp
//        (C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l.c
//        -lCN
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\List\
//        -la
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\List\
//        -o
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\Obj\
//        --no_clustering --debug --endian=little --cpu=Cortex-M4 -e
//        --fpu=VFPv4_sp --dlib_config "C:\Program Files\IAR Systems\Embedded
//        Workbench 9.1\arm\inc\c\DLib_Config_Normal.h" -I "C:\Program
//        Files\IAR Systems\Embedded Workbench
//        9.1\arm\src\flashloader\framework2\\" -I
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\
//        -Ohs --use_c++_inline) --dependencies=n
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\Obj\FlashApollo4l.o.iar_deps
//    Locale       =  C
//    List file    =
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\List\FlashApollo4l.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__CPP_Runtime", "1"
        RTMODEL "__SystemLibrary", "DLib"
        RTMODEL "__dlib_version", "6"
        AAPCS BASE,INTERWORK,VFP
        PRESERVE8
        REQUIRE8

        #define SHT_PROGBITS 0x1

        PUBLIC FlashErase
        PUBLIC FlashInit
        PUBLIC FlashWrite
        PUBLIC g_am_hal_bootrom_helper
        
          CFI Names cfiNames0
          CFI StackFrame CFA R13 DATA
          CFI Resource R0:32, R1:32, R2:32, R3:32, R4:32, R5:32, R6:32, R7:32
          CFI Resource R8:32, R9:32, R10:32, R11:32, R12:32, R13:32, R14:32
          CFI Resource D0:64, D1:64, D2:64, D3:64, D4:64, D5:64, D6:64, D7:64
          CFI Resource D8:64, D9:64, D10:64, D11:64, D12:64, D13:64, D14:64
          CFI Resource D15:64
          CFI EndNames cfiNames0
        
          CFI Common cfiCommon0 Using cfiNames0
          CFI CodeAlign 2
          CFI DataAlign 4
          CFI ReturnAddress R14 CODE
          CFI CFA R13+0
          CFI R0 Undefined
          CFI R1 Undefined
          CFI R2 Undefined
          CFI R3 Undefined
          CFI R4 SameValue
          CFI R5 SameValue
          CFI R6 SameValue
          CFI R7 SameValue
          CFI R8 SameValue
          CFI R9 SameValue
          CFI R10 SameValue
          CFI R11 SameValue
          CFI R12 Undefined
          CFI R14 SameValue
          CFI D0 Undefined
          CFI D1 Undefined
          CFI D2 Undefined
          CFI D3 Undefined
          CFI D4 Undefined
          CFI D5 Undefined
          CFI D6 Undefined
          CFI D7 Undefined
          CFI D8 SameValue
          CFI D9 SameValue
          CFI D10 SameValue
          CFI D11 SameValue
          CFI D12 SameValue
          CFI D13 SameValue
          CFI D14 SameValue
          CFI D15 SameValue
          CFI EndCommon cfiCommon0
        

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
g_am_hal_bootrom_helper:
        DATA32
        DC32 0x800'004d, 0x800'0051, 0x800'0055, 0x800'0059, 0x800'006d
        DC32 0x800'0075, 0x800'0079, 0x800'0081, 0x800'0099, 0x800'009d

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function FlashInit
        THUMB
FlashInit:
        PUSH     {R4,LR}        
          CFI R14 Frame(CFA, -4)
          CFI R4 Frame(CFA, -8)
          CFI CFA R13+8
        LSLS     R0,R3,#+31     
        SUB      SP,SP,#+8      
          CFI CFA R13+16
        BPL.N    ??FlashInit_0  
        MOV      R4,#+499712    
        STR      R4,[SP, #+0]   
        MOV      R3,#+24576     
        MOV      R2,#+4294967295
        MOVS     R1,#+0         
        LDR.N    R0,??DataTable2
        LDR.N    R4,??DataTable2_1
          CFI IndirectCall
        BLX      R4             
        CMP      R0,#+0         
        ITE      EQ                
        MOVEQ    R0,#+3         
        MOVNE    R0,#+1         
        ADD      SP,SP,#+8      
          CFI CFA R13+8
        POP      {R4,PC}        
          CFI CFA R13+16
??FlashInit_0:
        MOVS     R0,#+0         
        ADD      SP,SP,#+8      
          CFI CFA R13+8
        POP      {R4,PC}        
          CFI EndBlock cfiBlock0

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function FlashWrite
        THUMB
FlashWrite:
        PUSH     {R4,LR}        
          CFI R14 Frame(CFA, -4)
          CFI R4 Frame(CFA, -8)
          CFI CFA R13+8
        MOV      R4,R3          
        ADDS     R0,R1,R0       
        SUB      SP,SP,#+8      
          CFI CFA R13+16
        ANDS     R1,R4,#0x3     
        ITE      EQ                
        TSTEQ    R0,#0x3        
        MOVNE    R0,#+1         
        BNE.N    ??FlashWrite_0 
        LSRS     R2,R2,#+2      
        STR      R2,[SP, #+0]   
        LSRS     R3,R0,#+2      
        MOV      R2,R4          
        MOVS     R1,#+1         
        LDR.N    R0,??DataTable2
        LDR.N    R4,??DataTable2_1
          CFI IndirectCall
        BLX      R4             
??FlashWrite_0:
        ADD      SP,SP,#+8      
          CFI CFA R13+8
        POP      {R4,PC}        
          CFI EndBlock cfiBlock1

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function FlashErase
        THUMB
FlashErase:
        PUSH     {R4,LR}        
          CFI R14 Frame(CFA, -4)
          CFI R4 Frame(CFA, -8)
          CFI CFA R13+8
        TST      R0,#0x3        
        SUB      SP,SP,#+8      
          CFI CFA R13+16
        BNE.N    ??FlashErase_0 
        CMP      R0,#+98304     
        BCS.N    ??FlashErase_1 
??FlashErase_0:
        MOVS     R0,#+1         
        ADD      SP,SP,#+8      
          CFI CFA R13+8
        POP      {R4,PC}        
          CFI CFA R13+16
??FlashErase_1:
        LSRS     R1,R1,#+2      
        STR      R1,[SP, #+0]   
        LSRS     R3,R0,#+2      
        MOV      R2,#+4294967295
        MOVS     R1,#+0         
        LDR.N    R0,??DataTable2
        LDR.N    R4,??DataTable2_1
          CFI IndirectCall
        BLX      R4             
        ADD      SP,SP,#+8      
          CFI CFA R13+8
        POP      {R4,PC}        
          CFI EndBlock cfiBlock2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2:
        DATA32
        DC32     0x12344321     

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable2_1:
        DATA32
        DC32     0x800006d      

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//  40 bytes in section .rodata
// 142 bytes in section .text
// 
// 142 bytes of CODE  memory
//  40 bytes of CONST memory
//
//Errors: none
//Warnings: none
