///////////////////////////////////////////////////////////////////////////////
//
// IAR ANSI C/C++ Compiler V9.32.1.338/W64 for ARM        01/Jun/2023  10:14:36
// Copyright 1999-2022 IAR Systems AB.
//
//    Cpu mode     =  thumb
//    Endian       =  little
//    Source file  =
//        C:\Program Files\IAR Systems\Embedded Workbench
//        9.1\arm\src\flashloader\framework2\flash_loader.c
//    Command line =
//        -f
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\Obj\framework\flash_loader.o.rsp
//        ("C:\Program Files\IAR Systems\Embedded Workbench
//        9.1\arm\src\flashloader\framework2\flash_loader.c" -lCN
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\List\framework
//        -la
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\List\framework
//        -o
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\Obj\framework
//        --no_clustering --debug --endian=little --cpu=Cortex-M4 -e
//        --fpu=VFPv4_sp --dlib_config "C:\Program Files\IAR Systems\Embedded
//        Workbench 9.1\arm\inc\c\DLib_Config_Normal.h" -I "C:\Program
//        Files\IAR Systems\Embedded Workbench
//        9.1\arm\src\flashloader\framework2\\" -I
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\
//        -Ohs --use_c++_inline) --dependencies=n
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\Obj\framework\flash_loader.o.iar_deps
//    Locale       =  C
//    List file    =
//        C:\code\gitlab\ide\ide-support\interim_releases\Apollo4l\IAR\src\flashloader\AmbiqMicro\Apollo4l\FlashApollo4l_RAM2816K\List\framework\flash_loader.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__CPP_Runtime", "1"
        RTMODEL "__SystemLibrary", "DLib"
        RTMODEL "__dlib_version", "6"
        AAPCS BASE,INTERWORK,VFP
        PRESERVE8
        REQUIRE8

        #define SHT_PROGBITS 0x1
        #define SHT_IAR_NOINIT 0xabdc5467
        #define SHF_WRITE 0x1

        EXTERN FlashChecksum
        EXTERN FlashErase
        EXTERN FlashInit
        EXTERN FlashSignoff
        EXTERN FlashWrite
        EXTERN memset

        PUBLIC Crc16
        PUBLIC Crc16_helper
        PUBLIC Fl2FlashChecksumEntry
        PUBLIC Fl2FlashEraseWriteEntry
        PUBLIC Fl2FlashInitEntry
        PUBLIC Fl2FlashSignoffEntry
        PUBLIC Fl2FlashWriteEntry
        PUBLIC FlashBreak
        PUBLIC __argc
        PUBLIC __argv
        PUBLIC __argvbuf
        PUBLIC frameworkVersion
        PUBLIC theFlashParams
        
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
        

        SECTION `.rodata`:CONST:REORDER:ROOT(1)
        DATA
frameworkVersion:
        DATA16
        DC16 200

        SECTION `.noinit`:DATA:REORDER:ROOT(2)
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        DATA
theFlashParams:
        DS8 20

        SECTION `.noinit`:DATA:REORDER:NOROOT(2)
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        DATA
__argc:
        DS8 4

        SECTION `.noinit`:DATA:REORDER:NOROOT(2)
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        DATA
__argvbuf:
        DS8 64

        SECTION `.noinit`:DATA:REORDER:NOROOT(2)
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        DATA
__argv:
        DS8 28
        REQUIRE __argvbuf

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function Fl2FlashInitEntry
        THUMB
Fl2FlashInitEntry:
        PUSH     {R4,LR}        
          CFI R14 Frame(CFA, -4)
          CFI R4 Frame(CFA, -8)
          CFI CFA R13+8
        LDR.N    R4,??DataTable4
        LDR      R3,[R4, #+4]   
        LDR      R2,[R4, #+8]   
        LDR      R1,[R4, #+16]  
        LDR      R0,[R4, #+0]   
          CFI FunCall FlashInit
        BL       FlashInit      
        STR      R0,[R4, #+4]   
        POP      {R4,PC}        
          CFI EndBlock cfiBlock0

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function Fl2FlashWriteEntry
        THUMB
Fl2FlashWriteEntry:
        PUSH     {R4,LR}        
          CFI R14 Frame(CFA, -4)
          CFI R4 Frame(CFA, -8)
          CFI CFA R13+8
        LDR.N    R4,??DataTable4
        LDR      R3,[R4, #+12]  
        LDR      R2,[R4, #+4]   
        LDR      R1,[R4, #+8]   
        LDR      R0,[R4, #+0]   
          CFI FunCall FlashWrite
        BL       FlashWrite     
        STR      R0,[R4, #+4]   
        POP      {R4,PC}        
          CFI EndBlock cfiBlock1

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function Fl2FlashEraseWriteEntry
        THUMB
Fl2FlashEraseWriteEntry:
        PUSH     {R4-R6,LR}     
          CFI R14 Frame(CFA, -4)
          CFI R6 Frame(CFA, -8)
          CFI R5 Frame(CFA, -12)
          CFI R4 Frame(CFA, -16)
          CFI CFA R13+16
        LDR.N    R4,??DataTable4
        LDR      R1,[R4, #+16]  
        CBNZ.N   R1,??Fl2FlashEraseWriteEntry_0
        LDR      R5,[R4, #+12]  
        MOVS     R6,#+0         
        MOVS     R0,#+0         
??Fl2FlashEraseWriteEntry_1:
        LDR      R2,[R4, #+4]   
        CMP      R6,R2          
        BCS.N    ??Fl2FlashEraseWriteEntry_2
        LDR      R1,[R5, #+4]   
        LDR      R0,[R5, #+0]   
          CFI FunCall FlashErase
        BL       FlashErase     
        CMP      R0,#+0         
        ITT      EQ                
        ADDEQ    R5,R5,#+8      
        ADDEQ    R6,R6,#+1      
        BEQ.N    ??Fl2FlashEraseWriteEntry_1
        STR      R0,[R4, #+4]   
        POP      {R4-R6,PC}     
??Fl2FlashEraseWriteEntry_0:
        LDR      R0,[R4, #+0]   
          CFI FunCall FlashErase
        BL       FlashErase     
        CBNZ.N   R0,??Fl2FlashEraseWriteEntry_2
        LDR      R3,[R4, #+12]  
        LDR      R2,[R4, #+4]   
        LDR      R1,[R4, #+8]   
        LDR      R0,[R4, #+0]   
          CFI FunCall FlashWrite
        BL       FlashWrite     
??Fl2FlashEraseWriteEntry_2:
        STR      R0,[R4, #+4]   
        POP      {R4-R6,PC}     
          CFI EndBlock cfiBlock2

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function Fl2FlashChecksumEntry
        THUMB
Fl2FlashChecksumEntry:
        PUSH     {R4,LR}        
          CFI R14 Frame(CFA, -4)
          CFI R4 Frame(CFA, -8)
          CFI CFA R13+8
        LDR.N    R4,??DataTable4
        LDR      R1,[R4, #+4]   
        LDR      R0,[R4, #+0]   
          CFI FunCall FlashChecksum
        BL       FlashChecksum  
        STR      R0,[R4, #+4]   
        POP      {R4,PC}        
          CFI EndBlock cfiBlock3

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function Fl2FlashSignoffEntry
        THUMB
Fl2FlashSignoffEntry:
        PUSH     {LR}           
          CFI R14 Frame(CFA, -4)
          CFI CFA R13+4
        SUB      SP,SP,#+4      
          CFI CFA R13+8
          CFI FunCall FlashSignoff
        BL       FlashSignoff   
        LDR.N    R1,??DataTable4
        STR      R0,[R1, #+4]   
        ADD      SP,SP,#+4      
          CFI CFA R13+4
        POP      {PC}           
          CFI EndBlock cfiBlock4

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable4:
        DATA32
        DC32     theFlashParams 

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function Crc16
        THUMB
Crc16:
        PUSH     {R4,R5,LR}     
          CFI R14 Frame(CFA, -4)
          CFI R5 Frame(CFA, -8)
          CFI R4 Frame(CFA, -12)
          CFI CFA R13+12
        SUB      SP,SP,#+4      
          CFI CFA R13+16
        MOV      R4,R0          
        MOV      R5,R1          
        MOVS     R2,#+2         
        MOVS     R1,#+0         
        MOV      R0,SP          
          CFI FunCall memset
        BL       memset         
        MOVS     R2,#+0         
        MOV      R1,R5          
        MOV      R0,R4          
          CFI FunCall Crc16_helper
        BL       Crc16_helper   
        MOV      R2,R0          
        MOVS     R1,#+2         
        MOV      R0,SP          
          CFI FunCall Crc16_helper
        BL       Crc16_helper   
        ADD      SP,SP,#+4      
          CFI CFA R13+12
        POP      {R4,R5,PC}     
          CFI EndBlock cfiBlock5

        SECTION `.text`:CODE:NOROOT(1)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function Crc16_helper
          CFI NoCalls
        THUMB
Crc16_helper:
        PUSH     {R4-R7}        
          CFI R7 Frame(CFA, -4)
          CFI R6 Frame(CFA, -8)
          CFI R5 Frame(CFA, -12)
          CFI R4 Frame(CFA, -16)
          CFI CFA R13+16
        MOVW     R4,#+4129      
??Crc16_helper_0:
        CBZ.N    R1,??Crc16_helper_1
        LDRB     R5,[R0], #+1   
        SUBS     R1,R1,#+1      
        MOVS     R3,#+8         
??Crc16_helper_2:
        LSLS     R6,R2,#+16     
        LSLS     R2,R2,#+1      
        LSLS     R7,R5,#+24     
        IT       MI                
        ORRMI    R2,R2,#0x1     
        CMP      R6,#+0         
        IT       MI                
        EORMI    R2,R4,R2       
        LSLS     R5,R5,#+1      
        SUBS     R3,R3,#+1      
        BNE.N    ??Crc16_helper_2
        B.N      ??Crc16_helper_0
??Crc16_helper_1:
        POP      {R4-R7}        
          CFI R4 SameValue
          CFI R5 SameValue
          CFI R6 SameValue
          CFI R7 SameValue
          CFI CFA R13+0
        UXTH     R0,R2          
        BX       LR             
          CFI EndBlock cfiBlock6

        SECTION `.text`:CODE:ROOT(1)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function FlashBreak
          CFI NoCalls
        THUMB
FlashBreak:
??FlashBreak_0:
        B.N      ??FlashBreak_0 
          CFI EndBlock cfiBlock7

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
// 116 bytes in section .noinit
//   2 bytes in section .rodata
// 234 bytes in section .text
// 
// 234 bytes of CODE  memory
//   2 bytes of CONST memory
// 116 bytes of DATA  memory
//
//Errors: none
//Warnings: none
