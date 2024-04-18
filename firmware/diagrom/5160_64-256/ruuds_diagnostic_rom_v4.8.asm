

;
; 	          Diagnostic ROM for IBM-PC/XT and compatibles
;
;		   Written by Ruud Baltissen - up to version V3
;
;			from version 3.0 on: modem7 at the Vintage Computer Federation Forums
;
;
;   To be compiled with NASM 2.11 or higher. Not tested with older versions.
;
;
; Notes:
; -	This ROM can be used to diagnose the IBM-PC, IBM-XT and compatible 
;	computers.
; -	This version only supports the MDA and CGA video adapter. The reason: 
;	the EGA and VGA cards need to be initialized by their own ROMs and I'm
;	sure that they run subroutines and therefore need RAM for the stack. 
;	The Landmark diagnostic ROM supports EGA by using its own routines but
;	1) IMHO that needs more effort than it is worth the trouble and time
;	and 2) and EGA monitors are rare nowadays.
;
; -	This test simply expects that a MDA or a CGA video card is present. In
;	case both cards are present, in this version only the MDA card will be
;	used.
; -	After the checksum test the program will beep to inform the user that
;	the test is running fine so far. But if there is nothing on the screen
;	at that point, there is either something wrong with the board or the
;	video system.
; -	If the CPU is to be found bad, the test stops. Why continuing with a
;	bad processor? Remark: a bad EPROM can cause this error as well.
; -	If the checksum of the EPROM is not correct, the test stops. If I can't
;	trust the EPROM, can I trust the outcome of the following tests?
; -	If the program can't find 2 KB of good memory from 0000:0000h on, the
;	test will stop as well. The rest of the program needs RAM for the
;	interrupt vectors and subroutines. Without RAM that is not possible. 
;	I know Landmark can continue by using this ROM-Stack method but this is
;	a very labour intensive way of programming. I'm interested in repairing
;	PCs with this tool and if the memory is bad, then repair that problem
;	first before looking for possible other problems.
; -	General remark: broken glue logic and buffers can cause errors that are
;	interpreted wrong by this diagnostic ROM or errors that won't be 
;	detected at all. For example: a broken data buffer can cause that 
;	nothing shows up at the screen althought the video card itself is fine.


; Versions
;	This is a project I do just for fun. So I had no intention to keep
;	track of any change I made. In case of a major change I will save a
;	dated copy for myself but that was all.
; V2:
;	But in october 2022 I decided for an important change: use the not-used
;	video RAM as stack. The advantage: the whole is MUCH more easier to 
;	program. The minor disadvantage: only MDA and CGA video cards are supported.


; Disclaimer
; -	All names with a copyright are acknowledged.
; -	Some information is derived from deduction after reading a lot of
;	documents and can be unreliable due to the lack of real proof.
; -	If you use this source code and the resulting binary, then it is on
;	your own account. I cannot be held responsible for blowing up your
;	computer, mother-in-law or whatever; it will always be your own fault.
;	USING THIS SOURCE CODE AND ITS BINARY IS COMPLETELY AT YOUR OWN RISK !!!
; -	I'm a Dutchman and English is not my native language. So you can expect
;	to find several linguistic mistakes.



;-------------------------------------------------------------------------------
; I/O port information
;-------------------------------------------------------------------------------
DMA8237_0_ar	equ	0		; channel 0 address register
DMA8237_0_wc	equ	1		; channel 0 word count
DMA8237_1_ar	equ	2		; channel 1 address register
DMA8237_1_wc	equ	3		; channel 1 word count
DMA8237_2_ar	equ	4		; channel 2 address register
DMA8237_2_wc	equ	5		; channel 2 word count
DMA8237_3_ar	equ	6		; channel 3 address register
DMA8237_3_wc	equ	7		; channel 3 word count
DMA8237_scr	equ	8		; status/command register
DMA8237_req	equ	9		; Request register
DMA8237_mask	equ	0Ah		; Mask register
DMA8237_mode	equ	0Bh		; Mode register
DMA8237_cmlff	equ	0Ch		; Clear MSB/LSB flip flop
DMA8237_mc	equ	0Dh		; Master clear

; Programmable Interrupt Controller
PIC8259_cmd	equ	20h		; command port
PIC8259_imr	equ	21h		; interrupt mask register

; Programmable Interval Timer
PIT8253_0	equ	40h		; channel 0
PIT8253_1	equ	41h		; channel 1
PIT8253_2	equ	42h		; channel 2
PIT8253_ctrl	equ	43h		; control

; Programmable Peripheral Interface 8255
PPI8255_A	equ	60h		; port A
PPI8255_B	equ	61h		; port B
PPI8255_C	equ	62h		; port C
PPI8255_ctrl	equ	63h		; control

DebugIO		equ	80h

; DMA page register
DmaPageRegCh2	equ	81h
DmaPageRegCh3	equ	82h
DmaPageRegCh1	equ	83h

NmiCtrlPort	equ	0A0h

LPT1		equ	378h
LPT2		equ	278h
LPT3		equ	3BCh		; Found on most MDA cards.

COM1_tx_rx_dll	equ	3F8h		; Tx/Rx or Divisor Latch LS (DLL)
COM1_ier_dlm	equ	3F9h		; Interrupt Enable Register (IER) or Divisor Latch MS (DLM)
COM1_iir	equ	3FAh		; Interrupt Ident Register (IIR)
COM1_lcr	equ	3FBh		; Line Control Register (LCR)
COM1_mcr	equ	3FCh		; MODEM Control Register (MCR)
COM1_lsr	equ	3FDh		; Line Status Register (LSR)

MDA_index	equ	3B4h		; 6845 Index Register
MDA_data	equ	3B5h		; 6845 Data Register
MDA_ctrl	equ	3B8h		; CRT Control Port 1

CGA_index	equ	3D4h		; 6845 Index Register
CGA_data	equ	3D5h		; 6845 Data Register
CGA_ctrl	equ	3D8h		; Mode Control Register
CGA_pal		equ	3D9h		; Color Select Register

FdcDor		equ	3F2h		; Digital Output Register (DOR)
FdcMsr		equ	3F4h		; Main Status Register (MSR)
FdcCdr0		equ	3F5h		; Command/Data Register 0

SegMdaRam	equ	0B000h
SegCgaRam	equ	0B800h

OurSpAddress	equ	4094		; What we will set the stack pointer to.

BrCornerOffset	equ	3940		; Screen offset at bottom right corner where a segment:offset gets displayed.

START		equ	0E000h		; Code starts at offset E000h.


;-------------------------------------------------------------------------------
; Variables stored in low motherboard RAM (after that RAM has been FULLY tested).
;-------------------------------------------------------------------------------
IntIsrRecord	equ	0400h		; Address = 0000:0400 - Where the ISR record is stored.
KybTestResult	equ	0401h		; Address = 0000:0401 - Where the result of the keyboard test is stored.
FdcResetCount	equ	0402h		; Address = 0000:0402 - Used by the FDC reset subroutine.
FloppyReadCount	equ	0403h		; Address = 0000:0403 - Used by the 'read a floppy' test.
NmiFlag		equ	0404h		; Address = 0000:0404 - Normally 0. NMI handler will set this to 1.

;-------------------------------------------------------------------------------
;Variables stored in unused MDA/CGA video RAM, or 4 KB at A0000.
;-------------------------------------------------------------------------------
InvScreenRAM	equ	4000			; Offset of unused portion of MDA/CGA video RAM
ControlWord	equ	InvScreenRAM		; ????????
Err8253Ch0	equ	InvScreenRAM+2
Err8253Ch1	equ	InvScreenRAM+3
Err8253Ch2	equ	InvScreenRAM+4
Err8237DMA	equ	InvScreenRAM+5
ErrHotTimer1	equ	InvScreenRAM+6
Err8255Parity	equ	InvScreenRAM+7
Err2KbRAM	equ	InvScreenRAM+8
Err8259PIC	equ	InvScreenRAM+9
ErrHotInterrupt	equ	InvScreenRAM+10
ErrInterrupt0	equ	InvScreenRAM+11
ErrNMI		equ	InvScreenRAM+12
ErrMemoryMDA	equ	InvScreenRAM+13
ErrMemoryCGA	equ	InvScreenRAM+14
Err8087		equ	InvScreenRAM+15
ErrKeybReset	equ	InvScreenRAM+16
ErrKeybStuck	equ	InvScreenRAM+17
ErrFDC		equ	InvScreenRAM+18
ErrFdcRead	equ	InvScreenRAM+19
ErrRomF4000	equ	InvScreenRAM+20
ErrRomF6000	equ	InvScreenRAM+21
ErrRomF8000	equ	InvScreenRAM+22
ErrRomFA000	equ	InvScreenRAM+23
ErrRomFC000	equ	InvScreenRAM+24
PassCount	equ	InvScreenRAM+25

SegTopOfRam	equ	InvScreenRAM+30		; 30 and 31 for word. The segment address of the top-of-RAM found. E.g. A000 for 640 KB.
BadAddrSegment	equ	InvScreenRAM+32		; 32 and 33 for word. For addressing or data error, the segement of the 'bad' address.
BadAddrOffset	equ	InvScreenRAM+34		; 34 and 35 for word. For addressing or data error, the offset of the 'bad' address.
BadDataParity	equ	InvScreenRAM+36		; 36 and 37 for word. If a data error, the high byte = 'bad' bits, low byte (parity) = 00.
                                                ;                     If a parity error, the high byte = 00, the low byte (parity) will be 01.

do_not_use	equ	InvScreenRAM+38		; Do not use this location. It caused a problem if a Mini G7 video card was used.
Com1Exists	equ	InvScreenRAM+39		; Normally 0. Will be set to 1 if COM1 exists.

AbsoluteAddress	equ	InvScreenRAM+43		; 43, 44 and 45 for an absolute address (eg. 084A3F hex). Populated by subroutine 'CalcAbsFromSegOff'


;=========================================================================
; Setloc - Set location. Insert 0FFh bytes until specifed location is reached.
;-------------------------------------------------------------------------
%imacro setloc  1.nolist
%assign pad_bytes (%1-($-$$)-START)
%if pad_bytes < 0
%assign over_bytes -pad_bytes
%error Preceding code extends beyond setloc location by over_bytes bytes
%endif
%if pad_bytes > 0
%warning Inserting pad_bytes bytes
 times  pad_bytes db 0FFh
%endif
%endm



; ****************************************************************************
; MACRO:
; Output the passed byte/code to:
;     - the three standard LPT ports; and
;     - IBM's debug port; and
;     - if the byte is less than 10h, the serial port of 3F8h ('COM1').
;
; REQUIREMENT: For XLATB, DS is set to the CS (where Tbl_ASCII is). This is normally the case in this program.
;
; REMARK: Intended for when:
;         - MDA/CGA/A0000 RAM available, but the stack has yet to be set to use that RAM; or
;         - Such RAM does not exist.
; ****************************************************************************
%macro macroCheckpointNoStack 1
	mov	al,%1

	;------------------------
	; Three standard LPT ports
	; -----------------------
	mov	dx,LPT1		; I/O port 378h
	out	dx,al
	dec	dh		; I/O port 278h
	out	dx,al
	mov	dx,LPT3		; I/O port 3BCh. The parallel/LPT port on most MDA cards.
	out	dx,al

	;------------------------
	; IBM's debug port
	; -----------------------
	out	80h,al		; IBM AT's debug port. Rarely works for PC's and XT's.

	;------------------------
	; If the byte is less than 10h, covert the least significant (LS) nibble to ASCII, then output that to the serial port of 3F8h ('COM1').
	; Why only the lower nibble?
	; We are in a macro that cannot call subroutines (because no stack yet).
	; This macro is only used for checkpoints less than 10h.
	; Therefore, to shorten the code (important for this 8 KB sized ROM), only the LS nibble is processed.
	; -----------------------
	cmp	al,10h
	jge	.EXIT		; --> if 10h or greater.
	mov	ah,al		; Save the byte for later.

	; Wait for the COM1 UART to indicate that it is ready for a TX byte.
	mov	dx,COM1_lsr	; Line status register (LSR).
.L10:	in	al,dx
	and	al,00100000b
	jz	.L10		; --> if UART is not ready.

	; Output an ASCII space.
	mov	dx,COM1_tx_rx_dll
	mov	al,' '
	out	dx,al

	; Wait for the COM1 UART to indicate that it is ready for a TX byte.
	mov	dx,COM1_lsr	; Line status register (LSR).
.L20:	in	al,dx
	and	al,00100000b
	jz	.L20		; --> if UART is not ready.

	; Ouput the LS nibble.
	mov	dx,COM1_tx_rx_dll
	mov	al,ah		; Get the byte to send, back into AL.
	mov	bx,Tbl_ASCII
	xlatb			; Convert AL into ASCII.
	out	dx,al		; Output it.

.EXIT:
%endmacro



; ****************************************************************************
; MACRO:
; Output a debug code to the three standard LPT ports and to COM1 and to IBM's debug port.
;
; REMARK: Only for use when the stack is available !!!
; ****************************************************************************
%macro macroCheckpointStack 1
	mov	al,%1
	call	CheckpointStack
%endmacro




BITS 16

CPU 8086


; ****************************************************************************
; * Start of the code.
; ****************************************************************************

	org	START

; ****************************************************************************
; * Author code at very start of ROM.
; ****************************************************************************
	db "Ruud's Diagnostic ROM for PC/XT - "
	db 'No copyright, all open source - '
	db 'Author: Ruud Baltissen  '


; ****************************************************************************
; * Cold start. Entry point after a hard reset.
; ****************************************************************************
ColdStart:

	; CPU - Disable maskable interrupts, and set the direction flag to increment.
	cli
	cld

	; Set DS to CS.
	mov	ax,cs
	mov	ds,ax


; ****************************************************************************
; * Cater for the IBM made MDA card.
; * 1 must be sent to the 'CRT control port 1' before the card is accessed - see IBM's technical documentation for the IBM MDA.
; * According to a book, if this is not done: "Otherwise the computer goes into an infinite wait loop."
; * I experienced that myself.
; *
; * We will do this pre-initialisation now, because the next step sends 33h to the MDA's parallel port.
; *
; ****************************************************************************
	mov	dx,MDA_ctrl	; MDA control register ('CRT Control Port 1')
	mov	al,1
	out	dx,al


; ****************************************************************************
; * Send 33h to the three standard LPT ports and to the debug port used by the IBM AT.
; * 33h chosen because it is very distinctive.
; * For those without an MDA/CGA card, the 33h indicates that the code has started to execute.
; ****************************************************************************
	mov	al,33h
	mov	dx,LPT1		; I/O port 378h
	out	dx,al
	dec	dh		; I/O port 278h
	out	dx,al
	mov	dx,LPT3		; I/O port 3BCh. The parallel/LPT port on most MDA cards.
	out	dx,al
	mov	dx,80h		; IBM AT's debug port. Rarely works for PC's and XT's.
	out	dx,al


; ****************************************************************************
; If a serial port at I/O port 3F8h ('COM1') exists:
; 1. Initialise the port to: 9600 baud, no parity, 8 data bits, 1 stop bit. (9600,N,8,1), then
; 2. Send '33' to it.
; ****************************************************************************
InitCom1:

	; See if a serial port at 3F8 ('COM1') is present.
	; Consider it present if bits 7 to 3 of the IIR are zero.
	mov	dx,COM1_iir
	in	al,dx
	and	al,11111000b
	jnz	.EXIT		; --> COM1 does not exist

	; Set 9600 baud.
	; Note: The DLAB bit will be restored to zero later.
	mov	dx,COM1_lcr	; Line control register (LCR).
	in	al,dx
	or	al,10000000b	; Bit 8 is the DLAB bit.
	out	dx,al		; In LCR, set the DLAB bit so that DLL and DLM are the targets.
	jmp	short $+2	; Delay for I/O.
	mov	dx,COM1_tx_rx_dll
	mov	al,0Ch    	; Divisor Latch LS (DLL) = 0Ch.
	out	dx,al
	jmp	short $+2	; Delay for I/O.
	mov	dx,COM1_ier_dlm	; Interrupt Enable Register (IER) or Divisor Latch MS (DLM).
	mov	al,0    	; Divisor Latch MS (DLM) = 0.
	out	dx,al
	jmp	short $+2	; Delay for I/O.

	; Note: At this point in time, in one of my serial configurations, a strange character appears on the serial terminal.

	; Restore the DLAB bit to zero, and set: No parity, 8 data bits, 1 stop bit.
	mov	dx,COM1_lcr	; Line control register (LCR).
	mov 	al,00000011b
	out	dx,al
	jmp	short $+2	; Delay for I/O.

	; Delay - found to be required.
	; If not here, in one of my serial configurations, the "33 0 2" appears as strange characters.
	xor	cx,cx
	loop	$

	; Wait for the COM1 UART to indicate that it is ready for a TX byte.
	mov	dx,COM1_lsr	; Line status register (LSR).
.L10:	in	al,dx
	and	al,00100000b
	jz	.L10		; --> if UART is not ready.

	; Send first '3'
	mov	dx,COM1_tx_rx_dll
	mov	al,'3'
	out	dx,al

	; Wait for the COM1 UART to indicate that it is ready for a TX byte.
	mov	dx,COM1_lsr	; Line status register (LSR).
.L20:	in	al,dx
	and	al,00100000b
	jz	.L20		; --> if UART is not ready.

	; Send second '3'
	mov	dx,COM1_tx_rx_dll
	mov	al,'3'
	out	dx,al

.EXIT:



; ****************************************************************************
; * Checkpoint 00
; ****************************************************************************
OD0:	macroCheckpointNoStack 0


; ****************************************************************************
; * Initialise the MDA card or CGA card.
; *
; * At this point in time, we do not know which is present, MDA or CGA.
; * Maybe neither (e.g. user is using a serial port to see checkpoints).
; * Just assume that an MDA or CGA is fitted.
; *
; * Note that earlier, is some initialisation code specifically targeting the IBM made MDA.
; *
; ****************************************************************************
InitMdaCga:
	mov	ax,cs
	mov	ds,ax			; DS := CS

	; ------------------------
	; First, do MDA.
	; This will result in 8Ox25 text mode.
	; ------------------------
	;
	; Set appropriate values into the 6845 controller's internal data registers.
	;
	mov	dx,MDA_index		; 6845 Index Register, on MDA card
	mov	si,Tbl_dataMDA		; 16 values
	mov	cx,16			; 16 registers to do
	xor	bx,bx			; Start our register counter (BL) at 0
.L10:	mov	al,bl
	out	dx,al			; Specify target register (0 to 15)
	lodsb				; Get the value for that register into AL  {Copy contents of [DS:SI] into AL, then increment SI}
	inc	dl			; DL now pointing to the 6845 Data Register
	out	dx,al			; Send value
	dec	dl			; DL back to pointing to the 6845 Index Register
	inc	bx			; Next register
	loop	.L10
	;
	; Set the mode, and enable video signal.
	mov	dx,MDA_ctrl		; MDA control register ('CRT Control Port 1')
	mov	al,00101001b		; High Resolution mode, with blinking enabled, and enable the video signal
	out	dx,al

	; ------------------------
	; Next, do CGA.
	; This will result in 8Ox25 text mode.
	; ------------------------
	;
	; Set appropriate values into the 6845 controller's internal data registers.
	;
	mov	dx,CGA_index		; 6845 Index Register, on CGA card
	mov	si,Tbl_dataCGA		; 16 values
	mov	cx,16			; 16 registers to do
	xor	bx,bx			; Start our register counter (BL) at 0
.L20:	mov	al,bl
	out	dx,al			; Specify target register (0 to 15)
	lodsb				; Get the value for that register into AL  {Copy contents of [DS:SI] into AL, then increment SI}
	inc	dl			; DL now pointing to the 6845 Data Register
	out	dx,al			; Send value
	dec	dl			; DL back to pointing to the 6845 Index Register
	inc	bx			; Next register
	loop	.L20
	;
	; Set the mode, and enable video signal.
	mov	dx,CGA_ctrl		; CGA mode control register
	mov	al,00101001b		; 80 by 25 text mode, with blinking enabled, and enable the video signal
	out	dx,al
	;
	; Set the palette.
	mov	dx,CGA_pal		; CGA palette register ('Color Select Register')
	mov	al,00110000b
	out	dx,al


; ****************************************************************************
; * Checkpoint 01
; ****************************************************************************
OD1:	macroCheckpointNoStack 1


; ****************************************************************************
; * Make the cursor invisible.
; *
; * At this point in time, we do not know which is present, MDA or CGA.
; * Maybe neither (e.g. user is using a serial port to see checkpoints).
; * Just assume that an MDA or CGA is fitted.
; ****************************************************************************
SetVideoMode:

	; ------------------------
	; First, do MDA.
	; ------------------------
	mov	cx,2607h
	;
	; Send 26h to 6845 register 10
	mov	ah,10			; 6845 register 10 (0Ah) - 'Cursor start'
	mov	al,ch			; 26h
	mov	dx,MDA_index		; 6845 Index Register, on MDA card
	xchg	al,ah
	out	dx,al			; Send register number, which is 10 (0Ah)
	xchg	al,ah
	inc	dl			; DL now pointing to the 6845 Data Register
	out	dx,al			; Send 26h for that register
	;
	; Send 07h to 6845 register 11
	inc	ah			; 6845 register 11 (0Bh) - 'Cursor end'
	mov	al,cl			; 07h
	mov	dx,MDA_index		; 6845 Index Register, on MDA card
	xchg	al,ah
	out	dx,al			; Send register number, which is 11 (0Bh)
	xchg	al,ah
	inc	dl			; DL now pointing to the 6845 Data Register
	out	dx,al			; Send 07h for that register

	; ------------------------
	; Next, do CGA.
	; ------------------------
	mov	cx,2607h
	;
	; Send 26h to 6845 register 10
	mov	ah,10			; 6845 register 10 (0Ah) - 'Cursor start'
	mov	al,ch			; 26h
	mov	dx,CGA_index		; 6845 Index Register, on CGA card
	xchg	al,ah
	out	dx,al			; Send register number, which is 10 (0Ah)
	xchg	al,ah
	inc	dl			; DL now pointing to the 6845 Data Register
	out	dx,al			; Send 26h for that register
	;
	; Send 07h to 6845 register 11
	inc	ah			; 6845 register 11 (0Bh) - 'Cursor end'
	mov	al,cl			; 07h
	mov	dx,CGA_index		; 6845 Index Register, on CGA card
	xchg	al,ah
	out	dx,al			; Send register number, which is 11 (0Bh)
	xchg	al,ah
	inc	dl			; DL now pointing to the 6845 Data Register
	out	dx,al			; Send 07h for that register


; ****************************************************************************
; * Checkpoint 02
; ****************************************************************************
OD2:	macroCheckpointNoStack 2


; ****************************************************************************
; * Clear the MDA/CGA video screen.
; *
; * At this point in time, we do not know which is present, MDA or CGA.
; * Maybe neither (e.g. user is using a serial port to see checkpoints).
; * Just assume that an MDA or CGA is fitted.
; ****************************************************************************

	; ------------------------
	; First, do MDA.
	; ------------------------
	mov	ax,SegMdaRam
	mov	es,ax		; Point ES to segment address of MDA video RAM.
	mov	ax,0700h+' '	; Attribute + space
	xor	di,di		; Start at first screen position.
	mov	cx,4000		; 4000 words.
	rep	stosw		; STOSW: AX-->[ES:DI], then DI=DI+2

	; ------------------------
	; Next, do CGA.
	; ------------------------
	mov	ax,SegCgaRam
	mov	es,ax		; Point ES to segment address of MDA/CGA video RAM.
	mov	ax,0700h+' '	; Attribute + space
	xor	di,di		; Start at first screen position.
	mov	cx,4000		; 4000 words.
	rep	stosw		; STOSW: AX-->[ES:DI], then DI=DI+2


; ****************************************************************************
; * Checkpoint 03
; ****************************************************************************
OD3:	macroCheckpointNoStack 3


; ****************************************************************************
; * Display "Ruud's Diagnostic ROM for PC/XT ......"
; *
; * At this point in time, we do not know which is present, MDA or CGA.
; * Maybe neither (e.g. user is using a serial port to see checkpoints).
; * Just assume that an MDA or CGA is fitted.
; ****************************************************************************
DispRdr:
	; ------------------------
	; First, do MDA.
	; ------------------------
	mov	si,TxtTitle
	mov	di,8			; DI := starting postion of text (offset into MDA/CGA video RAM)
	mov	ax,SegMdaRam
	mov	es,ax			; Point ES to segment address of MDA video RAM
	mov	ah,07h			; Char attribute = normal
.L10:	mov	al,[cs:si]		; Read character
	inc	si
	and	al,al			; End of the text?
	jz	.S10			; --> if yes 
	stosw				; Write character plus attribute { STOSW: AX-->[ES:DI], then DI=DI+2 }
	jmp	.L10			; -->

.S10:	; ------------------------
	; Next, do CGA.
	; ------------------------
	mov	si,TxtTitle
	mov	di,8			; DI := starting postion of text (offset into MDA/CGA video RAM)
	mov	ax,SegCgaRam
	mov	es,ax			; Point ES to segment address of CGA video RAM
	mov	ah,07h			; Char attribute = normal
.L20:	mov	al,[cs:si]		; Read character
	inc	si
	and	al,al			; End of the text?
	jz	.EXIT			; --> if yes 
	stosw				; Write character plus attribute { STOSW: AX-->[ES:DI], then DI=DI+2 }
	jmp	.L20			; -->

.EXIT:


; ****************************************************************************
; * Checkpoint 04
; ****************************************************************************
OD7:	macroCheckpointNoStack 4


; ****************************************************************************
; * Disable NMI interrupts from reaching the CPU.
; * (FYI. The hardware on a good motherboard will automatically do that at power-on time.)
; ****************************************************************************

	xor	al,al
	out	NmiCtrlPort,al


; ****************************************************************************
; * Checkpoint 06
; ****************************************************************************
OD6:	macroCheckpointNoStack 6


; ****************************************************************************
; * Initialise the 8255 PPI chip.
; ****************************************************************************

	mov	al,99h
	out	PPI8255_ctrl,al		; Set port B to outputs, and ports A and C to inputs.

	mov	al,0F8h
	out	PPI8255_B,al		; 1. Reset/clear the two 'RAM parity error' latches.
					; 2. Turns off turbo mode if the clone XT motherboard supports turbo control via bit 2.


; ****************************************************************************
; * Checkpoint 08
; ****************************************************************************
OD8:	macroCheckpointNoStack 8


; ****************************************************************************
; * At power-on, NMI's are prevented from reaching the CPU.
; * But a motherboard can fail in a way that the NMI pin of the CPU goes HIGH intermittently. Or perhaps it is the 8088 that is faulty, internally trigging an NMI.
; * That would be very very rare, but we will attempt to cater for that by pointing the NMI vector to our NMI handler code for an unexpected NMI.
; * If an unexpected NMI happens, that code will clear the screen and display '*** UNEXPECTED NMI ***'.
; * 
; * At this time, the low RAM where the CPU interrupt 2 vector is, has not been tested.
; * Take a chance that the RAM is there and good.
; * If missing or bad, we will not be able to detect unexpected NMI's happening during the many tests done.
; ****************************************************************************

	; An NMI triggers CPU interrupt 2.
	; Change the CPU interrupt 2 vector to point to our NMI handler code for this test.
	xor	ax,ax
	mov	es,ax			; ES := 0000
	mov	di,0008h		; Offset of vector for CPU interrupt 2. (2 x 4 bytes)
	mov	ax,NmiHandler_2
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2
	mov	ax,cs
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2


; ****************************************************************************
; * Checkpoint 09h
; ****************************************************************************
OD9:	macroCheckpointNoStack 9


; ****************************************************************************
; * Beep the speaker three times: short-long-short.
; *
; * >>> Yes, we are taking the chance that the circuitry involved in beeping the speaker is working. <<<
; *
; * For those without an MDA/CGA card, and without a parallel/LPT reader, and without a serial reader, this beeping indicates that the code has started to execute.
; *
; ****************************************************************************
InitBeep:

	; Comment: Port direction on the 8255 chip was set up earlier.

	; Configure channel 2 of the 8253 PIT chip for a square wave output of about 904 Hz.
	; 1.193 MHz / 1320 = about 904 Hz
	mov	al,10110110b		; Square waves for channel 2
	out	PIT8253_ctrl,al
	mov	ax,0528h		; Countdown constant word = 1320 decimal
	out	PIT8253_2,al		;   send low order
	mov	al,ah			;   load high order
	out	PIT8253_2,al		;   send high order

	; One short beep.
	mov	al,00000011b
	out	PPI8255_B,al		; Start beep by setting 8255 pins PB1 and PB0 to HIGH.
	mov	al,1			; 1 = short beep.
	xor	cx,cx
.L10:	loop	.L10
	dec	al
	jnz	.L10
	mov	al,0
	out	PPI8255_B,al		; Stop the beep by setting 8255 pins PB1 and PB0 to LOW.

	; Inter-beep delay.
	xor	cx,cx
	loop	$

	; One long beep.
	mov	al,00000011b
	out	PPI8255_B,al		; Start beep by setting 8255 pins PB1 and PB0 to HIGH.
	mov	al,3			; 3 = long beep.
	xor	cx,cx
.L30:	loop	.L30
	dec	al
	jnz	.L30
	mov	al,0
	out	PPI8255_B,al		; Stop the beep by setting 8255 pins PB1 and PB0 to LOW.

	; Inter-beep delay.
	xor	cx,cx
	loop	$

	; One short beep.
	mov	al,00000011b
	out	PPI8255_B,al		; Start beep by setting 8255 pins PB1 and PB0 to HIGH.
	mov	al,1			; 1 = short beep.
	xor	cx,cx
.L50:	loop	.L50
	dec	al
	jnz	.L50
	mov	al,0
	out	PPI8255_B,al		; Stop the beep by setting 8255 pins PB1 and PB0 to LOW.


; ****************************************************************************
; * Checkpoint 0A
; ****************************************************************************
ODA:	macroCheckpointNoStack 0Ah


; ****************************************************************************
; * From here on, we need RAM for variables and for the stack.
; *
; * Look for RAM in the following locations:
; *    - Address B0000 - MDA video RAM.
; *    - Address B8000 - CGA video RAM.
; *    - Address A0000 - A RAM card configured to put RAM there.
; *
; * If any of those found, set ES to it.
; * If none found, send checkpoint 8A then halt the CPU.
; *
; ****************************************************************************
CheckVideoRAM:

	; ---------------------------------------
	; First, see if MDA video RAM is present.
	; ---------------------------------------
	mov	ax,SegMdaRam		; Start with MDA
	mov	es,ax
	mov	word [es:0],0720h	; Try one address for the moment.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	cmp	word [es:0],0720h	; Read it, the same value?
	jne	TRYCGA			; --> if no, try CGA
ODB:	macroCheckpointNoStack 0Bh	; Signal that MDA video RAM is present.
	jmp	CVREXIT			; -->

	; ---------------------------------------
	; No MDA - See if CGA video RAM present.
	; ---------------------------------------
TRYCGA:	mov	ax,SegCgaRam		; Try CGA.
	mov	es,ax
	mov	word [es:0],0720h	; Try one address for the moment.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	cmp	word [es:0],0720h	; Read it, the same value?
	jne	TRYA0000		; --> if no, try RAM at A0000.
ODC:	macroCheckpointNoStack 0Ch	; Signal that CGA video RAM is present.
	jmp	CVREXIT			; -->

	; ---------------------------------------
	; No CGA - See if RAM at address A0000 is present.
	; ---------------------------------------
TRYA0000:
	mov	ax,0A000h
	mov	es,ax
	mov	word [es:0],0720h	; Try one address for the moment.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	cmp	word [es:0],0720h	; Read it, the same value?
	jne	NOVID			; --> if no
ODD:	macroCheckpointNoStack 0Dh	; Signal that RAM at A0000 is present.
	jmp	CVREXIT			; -->


NOVID:	;------------------------
	; No MDA/CGA/A0000 video RAM found.
	; -----------------------

	; Send 8Ah to the LPT ports, and to port 80h.
OD8A:	macroCheckpointNoStack 8Ah

	; Send '8A' to the COM1 port (i.e. I/O port 3F8h).
	mov	dx,COM1_lsr	; Line status register (LSR).
.L66:	in	al,dx
	and	al,00100000b
	jz	.L66		; --> if UART is not ready.
	mov	dx,COM1_tx_rx_dll
	mov	al,'8'
	out	dx,al
	mov	dx,COM1_lsr	; Line status register (LSR).
.L77:	in	al,dx
	and	al,00100000b
	jz	.L77		; --> if UART is not ready.
	mov	dx,COM1_tx_rx_dll
	mov	al,'A'
	out	dx,al

	; Halt the CPU.
	cli
	hlt

CVREXIT:


; ****************************************************************************
; * MDA video RAM, or CGA video RAM, or RAM at address A0000, was found.
; *
; * On entry, ES is pointing to either:
; * - Address B0000 if MDA video card RAM is present; or
; * - Address B8000 if CGA video card RAM is present; or
; * - Address A0000 if a RAM card is configured to put some RAM at A0000.
; * 
; * This diagnostic uses only 96 bytes of the RAM. We will now test those 96 bytes before we commit to using them.
; * 
; *   MDA: 4 KB of video RAM. The last 96 bytes are unused by the card - what this diagnostic will use.
; *   CGA: The last 96 bytes of the first 4 KB are unused by the card - what this diagnostic will use.
; * A0000: The 96 bytes starting at offset 4000h.
; * 
; * Test the 96 bytes. If that test fails, halt the CPU.
; * 
; ****************************************************************************
TVAR:
	mov	ax,0FFFFh		; First test data pattern

.L16:	; Fill 96 bytes with test data.
	mov	di,4000			; End of screen RAM.
	mov	cx,48			; 48 words = 96 bytes
	rep	stosw			; Fill 48 words with test data  { STOSW: AX-->[ES:DI], then DI=DI+2 }

	; A short delay.
	xor	cx,cx
	loop	$

	; See if the 96 bytes have the expected content.
	mov	di,4000			; End of screen RAM
	mov	cx,48
	repe	scasw			; Compare 48 words with test data, OK?
	jne	.FAIL			; --> if no

	; They do, so on to next test pattern.
	sub	ax,5555h		; Next pattern, overflow?
	jnc	.L16			; --> if no 

	jmp	.PASS			; -->

.FAIL:	;---------------------------------------------------
	; MDA/CGA: Failure of the last 96 bytes of MDA/CGA video RAM.
	;   A0000: Failure of the 96 bytes at offset 4000h within the RAM.
	; --------------------------------------------------

	; Send 8Dh to the LPT ports, and to port 80h.
	macroCheckpointNoStack 8Dh

	; Send '8D' to the COM1 port (i.e. I/O port 3F8h).
	mov	dx,COM1_lsr	; Line status register (LSR).
.L66:	in	al,dx
	and	al,00100000b
	jz	.L66		; --> if UART is not ready.
	mov	dx,COM1_tx_rx_dll
	mov	al,'8'
	out	dx,al
	mov	dx,COM1_lsr	; Line status register (LSR).
.L77:	in	al,dx
	and	al,00100000b
	jz	.L77		; --> if UART is not ready.
	mov	dx,COM1_tx_rx_dll
	mov	al,'D'
	out	dx,al

	; Display "Test of video RAM failed. Aborting.  <-----"
	mov	si,TxtVrtf
	mov	di,482			; DI := starting postion of text (offset into MDA/CGA video RAM)
	mov	ax,SegCgaRam
	mov	es,ax			; Point ES to segment address of MDA video RAM
	mov	ah,07h			; Char attribute = normal
.L40:	mov	al,[cs:si]		; Read character
	inc	si
	and	al,al			; End of the text?
	jz	.S40			; --> if yes 
	stosw				; Write character plus attribute { STOSW: AX-->[ES:DI], then DI=DI+2 }
	jmp	.L40			; -->

	; Halt the CPU
.S40:	cli
	hlt


.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; MDA/CGA: Set the stack top to near the top of video RAM.
	;   A0000: Set the stack top at 4094 bytes into RAM.

	mov	ax,es
	mov	ss,ax			; SS = ES = segment address of video RAM (either MDA or CGA or A0000).
	mov	sp,OurSpAddress		; 4094



;
; Info:	SS won't change anymore and will serve as a permanent pointer to the video RAM segment.
;
; Info:	We now have a stack. Therefore, we can now use 'macroCheckpointStack' (uses the stack) instead 
;	of 'macroCheckpointNoStack' (does not use the stack). This will reduce the amount of code.
;


; ****************************************************************************
; * Checkpoint 0Eh
; ****************************************************************************
	macroCheckpointStack 0Eh


; ****************************************************************************
; * Clear the variables that are stored in the unused 96 bytes at the end of 4 KB sized video RAM.
; * For CGA, that 4 KB is the first block of 4 KB.
; ****************************************************************************

	mov	ax,ss
	mov	es,ax			; Point ES to segment address of MDA/CGA video RAM.
	xor	ax,ax			; The bytes are to be zeroed.
	mov	di,InvScreenRAM+2	; Start at the first variable (Err8253Ch0).
	mov	cx,47			; 47 words = 94 bytes
	rep	stosw			; STOSW: AX-->[ES:DI], then DI=DI+2


; ****************************************************************************
; * If an RS-232 serial port at the base I/O address of 3F8h (i.e. COM1) exists,
; * then set the 'Com1Exists' variable to 1.
; *
; * That variable will be checked by later code that intends to send bytes to COM1.
; ****************************************************************************
Com1Check:

	; Consider it present if bits 7 to 3 of the IIR are zero.
	mov	dx,COM1_iir
	in	al,dx
	and	al,11111000b
	jnz	.EXIT		; --> COM1 does not exist
	mov	byte [ss:Com1Exists],1
.EXIT:


; ****************************************************************************
; * Checkpoint 10h
; * From here on it will be one big loop.
; ****************************************************************************
DiagLoop:

	macroCheckpointStack 10h

	; Disable maskable interrupts, and set the direction flag to increment.
	cli
	cld

	; Display 00 in the top right corner of the screen.
	mov	al,0
	call	DispAlTopCorner		; ( Destroys: BX, CL, DI )


; ****************************************************************************
; * Checkpoint 12h
; ****************************************************************************
	macroCheckpointStack 12h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Testing CPU".
; ****************************************************************************
TestCPU:

	; Display "Testing CPU" in inverse, with an arrow to the left of it.
	mov	si,TxtTestCPU
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; ------------------------------------------
	; SUBTEST 1 OF 2: Test the branches/flags.
	; ------------------------------------------
	xor	ax,ax
	jb	.FAIL1
	jo	.FAIL1
	js	.FAIL1
	jnz	.FAIL1
	jpo	.FAIL1

	add	ax,1
	jz	.FAIL1
	jpe	.FAIL1

	sub	ax,8002h
	js	.FAIL1

	inc	ax
	jno	.FAIL1

	shl	ax,1
	jnb	.FAIL1
	jnz	.FAIL1

	shl	ax,1
	jb	.FAIL1

	xor	ax,ax
	jnz	.FAIL1		; Jump if not zero

	add	al,80h
	jns	.FAIL1		; Jump if not sign
	jc	.FAIL1		; Jump if carry Set
	jp	.FAIL1		; Jump if parity=1

	add	al,80h
	jnz	.FAIL1		; Jump if not zero
	jno	.FAIL1		; Jump if not overflw
	jnc	.FAIL1		; Jump if carry=0

	lahf			; Load ah from flags
	or	ah,41h
	sahf			; Store ah into flags
	jnz	.FAIL1		; Jump if not zero
	jnc	.FAIL1		; Jump if carry=0

	xor	si,si		; Zero register
	lodsb			; Copy contents of [SI] into AL, then increment SI.
	dec	si
	or	si,si		; SI = zero?
	jz	.SS2		; --> if yes, all good, on to subtest 2

.FAIL1:	; Indicate failure of SUBTEST 1
	macroCheckpointStack 92h
	jmp	.FAIL

	; ------------------------------------------
	; SUBTEST 2 OF 2: Test the REGISTERS (except SS and SP).
	; ------------------------------------------

.SS2:	macroCheckpointStack 14h

	mov	bx,5555h
.L10:	mov	bp,bx
	mov	cx,bp
	mov	dx,cx
	mov	si,dx
	mov	es,si
	mov	di,es
	mov	ds,di
	mov	ax,ds
	cmp	ax,5555h	; AX = 5555h ?
	jne	.S20		; --> if no 

	not	ax		; AX := AAAAh
	mov	bx,ax
	jmp	.L10

.S20:	; Either:
	; 5555h is the test word, and that failed; or
	; AAAAh is the test word, and we have yet to see if that worked.
	xor	ax,0AAAAh	; AX = AAAAh ?
	jz	.PASS		; --> if yes

	; Indicate failure of SUBTEST 2
	macroCheckpointStack 94h

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	; Display FAILED, but do NOT remove the arrow.
	mov	si,TxtTestCPU
	call	DisplayFailed

	; Display ">> Critical error, diagnostics have been stopped! <<"
	mov	si,TxtTestCPU
	call	DispCritical

	; Halt the CPU
	call	SendCrlfHashToCom1	; Send a CR/LF/hash sequence to COM1, indicating CPU halt.
	cli
	hlt


.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Set DS back to the CS value, because it was changed earlier in this test.
	mov	ax,cs
	mov	ds,ax

	; Display PASSED and remove the arrow.
	mov	si,TxtTestCPU
	call	DisplayPassedA


; ****************************************************************************
; * Checkpoint 16h
; ****************************************************************************
	macroCheckpointStack 16h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Diagnostic ROM checksum".
; *
; * Verify that the 8-bit checksum of Ruud's Diagnostic ROM is 00.
; * Ruud's Diagnostic ROM is 8 KB sized and starts at address FE000 (F000:E000).
; ****************************************************************************
ChecksumROM:

	; Display "Diagnostic ROM checksum" in inverse, with an arrow to the left of it.
	mov	si,TxtChecksumROM
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Preparation.
	mov	ax,0F000h	; Segment = F000
	mov	es,ax
	mov	bx,0E000h	; Starting offset.
	mov	cx,2000h	; 8 KB sized.
	xor	al,al		; Zero our running sum.

.L10:	; Add (sum) the content of every byte in this ROM.
	add	al,[es:bx]
	inc	bx		; Point to next address.
	loop	.L10

	; Result is zero ?
	or	al,al
	jz	.PASS		; --> if yes

.FAIL:	;------------------------
	; Checksum is bad.
	; -----------------------

	macroCheckpointStack 96h

	; Display FAILED, but do NOT remove the arrow.
	mov	si,TxtChecksumROM
	call	DisplayFailed

	; Display ">> Critical error, diagnostics have been stopped! <<"
	mov	si,TxtChecksumROM
	call	DispCritical

	; Halt the CPU
	call	SendCrlfHashToCom1	; Send a CR/LF/hash sequence to COM1, indicating CPU halt.
	cli
	hlt


.PASS:	;------------------------
	; Checksum is good.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtChecksumROM
	call	DisplayPassedA


; ****************************************************************************
; * Checkpoint 18h
; ****************************************************************************
	macroCheckpointStack 18h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "8253 timer channel 0".
; * ON-SCREEN TEST: Displayed is "8253 timer channel 1".
; * ON-SCREEN TEST: Displayed is "8253 timer channel 2".
; ****************************************************************************
CheckTimers:

	; Disable the DMA controller.
	mov	al,4
	out	DMA8237_scr,al		

	; Checkpoint 19h
	macroCheckpointStack 19h

	; Disable the speakers and enable timer 2.
	in	al,PPI8255_B
	or	al,1
	and	al,0FDh
	out	PPI8255_B, al

	; -----------------------------------------------------------------------------
	; Test timer channel 0 -
	; Used by the motherboard BIOS as part of the 'system timer'.
	; -----------------------------------------------------------------------------
	macroCheckpointStack 1Ah

	; Display "8253 timer channel 0" in inverse, with an arrow to the left of it.
	mov	si,Txt8253Ch0
	call	TextToScreenInv
	call	ShowArrow

	; Now test channel 0.
	mov	dx,PIT8253_0
	call	Check8253
	jnc	.CHAN_0_PASS		; --> if no error

.CHAN_0_FAIL:
	macroCheckpointStack 9Ah

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,Txt8253Ch0
	mov	bx,Err8253Ch0
	call	DisplayFailedA

	jmp	.CHAN_0_EXIT		; -->

.CHAN_0_PASS:
	; Display PASSED and remove the arrow.
	mov	si,Txt8253Ch0
	call	DisplayPassedA

.CHAN_0_EXIT:


	; -----------------------------------------------------------------------------
	; Test timer channel 1 -
	; Used by the motherboard BIOS as part of the mechanism that refreshes dynamic RAM.
	; -----------------------------------------------------------------------------
	macroCheckpointStack 1Bh

	; Display "8253 timer channel 1" in inverse, with an arrow to the left of it.
	mov	si,Txt8253Ch1
	call	TextToScreenInv
	call	ShowArrow

	; Now test channel 1.
	mov	dx,PIT8253_1
	call	Check8253
	jnc	.CHAN_1_PASS		; --> if no error

.CHAN_1_FAIL:
	macroCheckpointStack 9Bh

	; Display FAILED, but do NOT remove the arrow.
	mov	si,Txt8253Ch1
	call	DisplayFailed

	; Display ">> Critical error, diagnostics have been stopped! <<"
	mov	si,Txt8253Ch1
	call	DispCritical

	; With a failing timer 1, there won't be any refresh of dynamic RAM on motherboard and card.
	; That compromises a later test of that RAM.
	; So halt the CPU.
	call	SendCrlfHashToCom1	; Send a CR/LF/hash sequence to COM1, indicating CPU halt.
	cli
	hlt

.CHAN_1_PASS:
	; Display PASSED and remove the arrow.
	mov	si,Txt8253Ch1
	call	DisplayPassedA

.CHAN_1_EXIT:


	; -----------------------------------------------------------------------------
	; Test timer channel 2.
	; Part of the motherboard circuity that is involved in generating speaker sounds.
	; -----------------------------------------------------------------------------
.CHAN_2_TEST:
	macroCheckpointStack 1Ch

	; Display "8253 timer channel 2" in inverse, with an arrow to the left of it.
	mov	si,Txt8253Ch2
	call	TextToScreenInv
	call	ShowArrow

	; Now test channel 2.
	mov	dx,PIT8253_2
	call	Check8253
	jnc	.CHAN_2_PASS		; --> if no error

.CHAN_2_FAIL:
	macroCheckpointStack 9Ch

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,Txt8253Ch2
	mov	bx,Err8253Ch2
	call	DisplayFailedA

	jmp	.CHAN_2_EXIT		; -->

.CHAN_2_PASS:
	; Display PASSED and remove the arrow.
	mov	si,Txt8253Ch2
	call	DisplayPassedA

.CHAN_2_EXIT:



; ****************************************************************************
; * Checkpoint 1F
; ****************************************************************************
	macroCheckpointStack 1Fh



; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "8237A DMA controller".
; *
; * Do a read/write test of the first 8 registers of the 8237A DMA controller chip.
; ****************************************************************************
Check8237DMA:

	; Display "8237A DMA controller" in inverse, with an arrow to the left of it.
	mov	si,Txt8237DMA
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Now do the test.

 	cli
	mov	al,0
	out	DMA8237_mc,al		; Stop all DMA activities.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	out	DMA8237_scr,al
	mov	ax,0FF00h
	mov	di,1

.L10:	; Test with value in AX.
	xor	dx,dx
	mov	cx,8

.L20:	; First fill registers 0..7 with the value in AL.
	out	dx,al
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	out	dx,al
	inc	dx
	loop	.L20
	xor	si,si		; SI := zero
	xchg	ah,al
	mov	bx,ax

.L24:	; Now test with the value that was original in AH.
	mov	dx,si
	out	dx,al
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	out	dx,al
	mov	cx,8
	xor	dx,dx
.L28:	cmp	dx,si		; DX = SI?
	je	.S36		; --> if yes
	in	al,dx
	cmp	al,bh		; Same as the original value?
	je	.S32		; --> if yes = OK, continue
	jmp	.FAIL

.S32:	in	al,dx
	cmp	al,bh		; Same as the original value?
	je	.S40		; --> if yes = OK, continue
	jmp	.FAIL

.S36:	in	al,dx
	cmp	al,bl		; Same as the original value?
	jne	.FAIL		; --> if no error

	in	al,dx
	cmp	al,bl		; Same as the original value?
	jne	.FAIL		; --> if no error

.S40:	inc	dx
	loop	.L28
	mov	dx,si
	mov	al,bh
	out	dx,al
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	out	dx,al
	mov	ax,bx
	inc	si
	cmp	si,8		; We tested all eight registers?
	je	.S44		; --> if yes, next phase
	jmp	.L24		; --> next register

.S44:	mov	ax,00FFh
	dec	di		; Did we test with 00FFh already?
	je	.L10		; --> if no test again with new value
	in	al,08h
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	nop			; Delay for I/O.
	in	al,08h
	test	al,0Fh		; Low nibble all zero?
	je	.PASS		; --> if yes

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 9Fh

	; Display FAILED, but do NOT remove the arrow.
	mov	si,Txt8237DMA
	call	DisplayFailed

	; Display ">> Critical error, diagnostics have been stopped! <<"
	mov	si,Txt8237DMA
	call	DispCritical

	; Without DMA, the motherboard RAM cannot be refreshed and therefore I halt the diagnostic.
	call	SendCrlfHashToCom1	; Send a CR/LF/hash sequence to COM1, indicating CPU halt.
	cli
	hlt
	
.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,Txt8237DMA
	call	DisplayPassedA


; ****************************************************************************
; * Checkpoint 22
; ****************************************************************************
	macroCheckpointStack 22h


; ****************************************************************************
; * Configure the 8237 DMA controller.
; *
; * In particular, channel #0 is configured to support the refresh mechanism for dynamic RAM.
; * Note that RAM refreshing will not actually start until we later initialise channel #1 on the 8253 timer chip.
; ****************************************************************************

	; Stop DMA operations on 8237 DMA controller.
	xor	al,al
	out	DMA8237_mc,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O

	; Configure DMA channel #0 (RAM refresh) - Single transfer mode / Address increment / Auto-init / Read transfer
	mov	al,58h
	out	DMA8237_mode,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O
	; Configure DMA channel #0 (RAM refresh) - Word count of FFFF hex (65535).
	mov	al,0FFh
	out	DMA8237_0_wc,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O
	out	DMA8237_0_wc,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O

	; Configure DMA channel #1 - Block verify.
	mov	al,41h
	out	DMA8237_mode,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O

	; Configure DMA channel #2 - Block verify.
	mov	al,42h
	out	DMA8237_mode,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O

	; Configure DMA channel #3 - Block verify.
	mov	al,43h
	out	DMA8237_mode,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O

	; Enable DMA operations on all channels.
	xor	al,al
	out	DMA8237_mask,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O

	; Initialize DMA command register with zero.
	out	DMA8237_scr,al


; ****************************************************************************
; * Checkpoint 24
; ****************************************************************************
	macroCheckpointStack 24h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Hot timer channel 1".
; *
; * Look for a 'hot timer 1'.
; * This check is one that the POST in an IBM 5160 does.
; * Done by seeing if the 8237 DMA controller is seeing a HIGH on its 'DREQ 0' pin (pin 19).
; ****************************************************************************
HotTimer1:

	; Display "Hot timer channel 1" in inverse, with an arrow to the left of it.
	mov	si,TxtHotTimer1
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	in	al,DMA8237_scr		; Status command register
	and	al, 00010000b		; DREQ0 pin is HIGH ?
	jz	.PASS			; --> if no

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0A5h

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtHotTimer1
	mov	bx,ErrHotTimer1
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtHotTimer1
	call	DisplayPassedA


.EXIT:


; ****************************************************************************
; * Checkpoint 26
; ****************************************************************************
	macroCheckpointStack 26h


; ****************************************************************************
; Initialise channel 1 on the 8253 timer chip to produce a negative-going pulse about every 15.1 uS.
; This is done as part of the refresh mechanism for dynamic RAM.
; The other part is suitably configuring channel #0 on the DMA controller, and that was done earlier.
;
; Do this channel 1 initialisation now, a reasonable time before we start testing dynamic RAM.
; This is due to a particular requirement of dynamic RAM.
; ****************************************************************************
InitTimer1:
	mov	al,54h			; Timer 1, LSB only, mode 2, binary counter 16-bit
	out	PIT8253_ctrl,al
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O
	mov	al,12h			; 1.193 MHz / 18 (12H) = 66.3 kHz = 15.1 uS
	out	PIT8253_1,al


; ****************************************************************************
; * Checkpoint 28
; ****************************************************************************
	macroCheckpointStack 28h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "RAM parity error latches".
; *
; * With the two 'RAM parity error' latches disabled, both latches are expected to be in a clear state.
; * There is a problem if either read in a set state (could be the latches, could be the 8255, etc.)
; ****************************************************************************
Check8255Par:

	; Display "RAM parity error latches" in inverse, with an arrow to the left of it.
	mov	si,TxtRamParityErr
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Reset/clear and disable the two 'RAM parity error' latches.
	in	al,PPI8255_B
	or	al,00110000b		; Set 8255 pins PB5 and PB4.
	jmp	short $+2		; delay for I/O
	jmp	short $+2		; delay for I/O
	out	PPI8255_B,al

	; Delay
	xor	cx,cx
	loop	$

	; Neither latch should be indicating a parity error.
	in	al,PPI8255_C
	and	al,11000000b		; Are either bits 7 and 6 on 8255 port C in a set state ?
	jz	.PASS			; --> if no

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0A7h

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtRamParityErr
	mov	bx,Err8255Parity
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtRamParityErr
	call	DisplayPassedA


.EXIT:


; ****************************************************************************
; * Checkpoint 2A
; ****************************************************************************
	macroCheckpointStack 2Ah


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Check first 2 KB of RAM".
; * 
; * Test each address in the first 2 KB of RAM.
; * 
; *  -------------------------------------------------------------
; * At each address:
; * 
; * - Use the five test values of: FFh,00h,55h,AAh,01h (in that order).
; * 
; * - Use ALL (repeat: all) test values.
; *   Do not abort the test of an address if one of the test values results in an error.
; *   For each test value, note the failing bit pattern.
; *   Use the failing bit pattern (if any) from all five test values to create a 'combined' failing bit pattern.
; *   
; *   Note: If during a test, a parity error is indicated, the parity chip in the bank may or may not be faulty.
; *         - If the test value read back differs to what was written, ignore the parity error.
; *         - If the test value read back is the same to what was written, and a parity error indicated, that indicates a faulty parity chip (or parity circuitry).
; *   
; *  -------------------------------------------------------------
; *   
; *  As soon as a test (a five-values test) of an address fails, abort the "Check first 2 KB of RAM", then report details, then halt the CPU.
; *   
; ****************************************************************************
Check2KbRAM:

	; Display "Check first 2 KB of RAM" in inverse, with an arrow to the left of it.
	mov	si,TxtChk2KbRAM
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	xor	dx,dx
	mov	es,dx			; ES := segment of first test RAM address, which is 0000
	mov	cx,0800h		; 2 KB sized block : a byte count required for TESTRAMBLOCK : 800H bytes = 2048 bytes = 2 KB
	call	TESTRAMBLOCK
	jnc	.PASS			; --> if TESTRAMBLOCK indicates no error

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0AAh

	; Display FAILED, but do NOT remove the arrow.
	mov	si,TxtChk2KbRAM
	call	DisplayFailed

	; Display ">> Critical error ...", then display error (address + BEP) details, then halt the CPU.
	mov	si,TxtChk2KbRAM
	jmp	CriticalErrorCond_1

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtChk2KbRAM
	call	DisplayPassedA


; ****************************************************************************
; * Checkpoint 32
; ****************************************************************************
	macroCheckpointStack 32h


; ****************************************************************************
; * ON-SCREEN: Displayed is "Found RAM:".
; * 
; * Find the total amount of conventional RAM.
; * 
; * Done by checking one address within each 16 KB sized block.
; * Start with the 16 KB block directly under the 640K (A0000) address.
; * If no RAM found there, check the next lower block.
; * The 'check the next lower block' step is continued until RAM is found.
; * 
; * Note: RAM is looked for by writing then reading back five test values to the test address.
; *       From that, a 'combined' bit error pattern is returned.
; *       In that bit error pattern, only if more than 4 bits are in error, will it be considered that no RAM exists at the test address.
; * 
; *       I can test this by removing some RAM chips from bank 3 on my IBM 5160 64-256KB motherboard.
; *       With four RAM chips removed from bank 3, the RAM size is still indicated as 256 KB. 
; *       But when I remove a fifth chip, the RAM size is then indicated as 192 KB (i.e. the fourth RAM bank is considered as unpopulated.)
; * 
; ****************************************************************************
SizeRAM:

	; Display "Found RAM:" with an arrow to the left of it.
	mov	si,TxtFoundRAM
	call	TextToScreen
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	mov	ax,9FFFh		; Segment - top of highest possible 16K block of RAM (A0000 = 640K)
	mov	es,ax
.NEWBLK:
	mov	cx,1			; Test only one byte.
	call	TESTRAMBLOCK		; Test first address at ES.   ( Destroys: SI, AX, CX, DX )
	jnc	.EXISTS			; --> if TESTRAMBLOCK reported no bits in error
	; Either TESTRAMBLOCK reported bad data bits, or it reported parity bit error.
	; If it was the parity bit, RAM exists. (TESTRAMBLOCK will only ever indicate a parity error if RAM exists.)
	mov	word ax,[ss:BadDataParity]	; Get the bad data bits (byte) and parity bit indicator (byte).
	cmp	ah,0
	je	.EXISTS			; --> if no bad data bits (i.e. must be a parity chip problem; RAM exists)
	; Bad data bits were found.
	; Count them.
	mov	bl,ah			; BL contains bad bits
	mov	bh,0
	mov	cx,8			; Check 8 bits 
.L10:	rol	bl,1			; Rotate BL through Carry, set?
	jnc	.S20			; --> if no, on to next bit
	inc	bh			; Increase bad bit counter
.S20:	loop	.L10
	; More than 4 bad bits ?
	cmp	bh,4
	jbe	.EXISTS			; --> if not, consider RAM to be present

	; At the test block, there is no RAM, or found was RAM that is too bad (5 or more bad data bits).

	; If we are presently in the first 16 KB block, it is not possible to go lower, so assume that the lower 16K exists.
	; We can assume that because the 'First 2KB RAM' test passed.
	mov	ax,es
	cmp	ax,3FFh			; Are we presently in the first 16 KB block ?
	je	.EXISTS			; --> yes, assume RAM exists

	; Try the next lower 16 KB block.
	sub	ax,0400h		; Subtract 16 KB from segment address.
	mov	es,ax

	jmp	.NEWBLK			; -->

.EXISTS:
	; Either:
	; - No bad data bits (RAM definately present), or 
	; - Less than 5 bad bits (RAM definately present but with some bad RAM chips)
	;
	; At this time, ES points to the highest segment (minus 1) where RAM was found.
	; "RAM was found" = five test values revealed a bad bit count between 0 and 3

	; Save the top-of-RAM segment address for later routines.
	mov	ax,es
	inc	ax			; Round the number up (e.g. 9FFFh --> A000h)(e.g. 03FFh --> 0400h)
	mov	word [ss:SegTopOfRam],ax

	; Set DI to the offset in MDA/CGA video RAM where to display the amount of found RAM.
	; Will be used by our call below to 'DispDecimal_2'.
	mov	si,TxtFoundRAM
	mov	di,11			; 11 chars from the start of "Found RAM:"
	call	CalcScreenOffset	; ( Destroys: nothing )

	; AX presently contains the top-of-RAM segment address (e.g. A000h).
	; Convert that to KB.
	mov	cl,6
	shr	ax,cl			; AX = AX div 64 = number of KB  (e.g. A000h --> 640 dec)

	; Display the amount of found RAM.
	call	AX2DEC			; out: BL = ones, DL = tens, DH = hundreds  in:	AX = number < 1000
	call	DispDecimal_2		;  in: BL = ones, DL = tens, DH = hundreds, DI = offset in MDA/CGA video RAM

	; Remove the arrow.
	mov	si,TxtFoundRAM
	call	RemoveArrow

; Fall through to the next on-screen test.



; ****************************************************************************
; * Checkpoint 35
; ****************************************************************************
	macroCheckpointStack 35h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Testing RAM - Data".
; * 
; * Now test the found RAM, including the first 2 KB.
; * Use the same technique as the 2 KB RAM test.
; * If a test address is found faulty, report details, then halt the CPU.
; ****************************************************************************
TestRamData:

	; Display "Testing RAM - Data" in inverse, with an arrow to the left of it.
	mov	si,TxtEmptyLine12	; Line 12: "                                        "
	call	TextToScreen
	mov	si,TxtTestRamData	; Line 12: "Testing RAM - Data"
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; For the progress dots, calculate the offset into MDA/CGA video RAM.
	; Store in DI.
	mov	si,TxtTestRamData	; Line 12: "Testing RAM - Data"
	mov	al,[cs:si]
	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI (offset in MDA/CGA video RAM) now at start of line on screen.
	add	di,42			; DI now at position where first dot to be displayed.

	; Calculate the number of 16 KB blocks that we will be testing.
	; Put into BX.
	mov	word bx,[ss:SegTopOfRam]	; Get top of RAM as a segment (e.g. A000h corresponds to 640 KB)(e.g. 0400h corresponds to 16 KB).
	mov	cl,10			; 2^10 = 1024
	shr	bx,cl			; Divide by 1024 to get count of 16 KB blocks to test (e.g. A000h corresponds to 40 [28h] blocks).
	;
	; Other preparation.
	xor	ax,ax
	mov	es,ax			; ES := segment of first test RAM address, which is 0000.
	mov	bp,0			; Zero our dot count.
	;
	; Do one dot now.
	mov	ax,8700h+254		; Blinking dot character.
	mov	[ss:di],ax		; Display the dot.
	inc	bp			; Increment our dot counter.

.L50:	; Display the block count-down.
	mov	si,TxtTestRamData
	mov	ax,31			; 31 chars from the start of "Testing RAM - Data"
	mov	dx,bx			; DX := block count-down.
	call	DispDecimal_1		; Display DL in decimal.        { Destroys: nothing }

	; In the bottom right corner, display the address of the 16 KB block that we are about to test.
	mov	dx,0
	call	DispEsDxInBrCorner	; { Destroys: nothing }

	; Test the 16 KB sized block.
	mov	cx,4000h		; 16 KB sized block : a byte count required for TESTRAMBLOCK : 4000H bytes = 16384 bytes = 16 KB
	call	TESTRAMBLOCK		; Test block at ES, sized at CX.   ( Destroys: SI, AX, CX, DX )
	jc	.FAIL			; --> if TESTRAMBLOCK reported an error

	; At this point, the 16 KB block was tested good.
	; Display a progress dot.
	add	di,2			; Next dot position.
	mov	ax,8700h+254		; Blinking dot character.
	mov	[ss:di],ax		; Display the dot.
	inc	bp			; Increment our dot counter.

	; If 11 dots displayed, reset back to one dot.
	cmp	bp,11
	jb	.S10			; --> if less than 11
	mov	ax,0700h+' '		; Space character.
	mov	cx,10			; 10 characters (leave one dot in place).
.L52:	mov	[ss:di],ax		; Display the space.
	sub	di,2			; Go back one character position.
	dec	bp			; Decrement our dot counter.
	loop	.L52

.S10:	; Point to segment of next 16 KB block address.
	mov	ax,es
	add	ax,400h
	mov	es,ax

	; If there are more blocks to do, go do them.
	dec	bx			; Decrement our block counter.
	jnz	.L50			; --> if more blocks to do
	jmp	.PASS			; -->

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0B5h

	; Display FAILED, but do NOT remove the arrow.
	mov	si,TxtTestRamData
	call	DisplayFailed

	; Display ">> Critical error ...", then display error (address + BEP) details, then halt the CPU.
	mov	si,TxtTestRamData
	jmp	CriticalErrorCond_1

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Remove the dots (actually the line).
	mov	si,TxtEmptyLine12
	call	TextToScreen

	; Put "Testing  - Data" back up.
	mov	si,TxtTestRamData
	call	TextToScreen

	; Display PASSED and remove the arrow.
	mov	si,TxtTestRamData
	call	DisplayPassedA

	; Erase the address (shown in various formats) that is currently displayed in the bottom right corner of the screen.
	call	ClearBrCorner

	; The CPU interrupt 2 vector (used for NMI) was altered as part of the RAM testing.
	; Set the vector back to pointing to our NMI handler code for an unexpected NMI.
	call	SetUnexpNmi


; ****************************************************************************
; * Checkpoint 37
; ****************************************************************************
	macroCheckpointStack 37h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Testing RAM - Address".
; * 
; * ( See the following URL for an explantion of an addressing problem:
; *   https://minuszerodegrees.net/memory/Addressing%20problem/Memory%20addressing%20problem.htm )
; * 
; * If a test address is found faulty, report details, then halt the CPU.
; *
; * Divided into two subtests.
; *
; * SUBTEST 1
; *
; * The first subtest uses the same code that IBM uses in the IBM 5160. That way, if the IBM BIOS ROM detected
; * a RAM addressing problem, then so should SUBTEST 1.
; * Done in 16 KB sized blocks, just like what the IBM BIOS ROM does.
; * In this code, the IBM code is routine STGTST_CNT, copied from the source listing of the IBM BIOS ROM for the 5160.
; * However, because the IBM code only address tests within each 16 KB block, that is only a partial addressing test.
; *
; * SUBTEST 2
; *
; * The second subtest expands on the first.
; * For example, in most cases (not all), it will throw an error if a 4164 type RAM chip has been accidentally put into a socket that requires a 41256 type.
; * Divide all RAM into 1 KB blocks. At the first address of each block, write a word that is unique to the block number.
; * Then read back all written words, verifying that each one matches with what was written.
; *
; ****************************************************************************
TestRamAddr:

	; Display "Testing RAM - Address" in inverse, with an arrow to the left of it.
	mov	si,TxtEmptyLine13	; Line 13: "                                        "
	call	TextToScreen
	mov	si,TxtTestRamAddr	; Line 13: "Testing RAM - Address"
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	;----------------
	; SUBTEST 1 OF 2
	; ---------------
.TEST1:	; For the progress dots, calculate the offset into MDA/CGA video RAM.
	; Store in DI.
	mov	si,TxtTestRamAddr	; Line 13: "Testing RAM - Address"
	mov	al,[cs:si]
	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI (offset in MDA/CGA video RAM) now at start of line on screen .
	add	di,48			; DI now at position where first dot to be displayed.
	; Calculate the number of 16 KB blocks we will be testing.
	; Put into BX.
	mov	word bx,[ss:SegTopOfRam]	; Get top of RAM as a segment (e.g. A000h corresponds to 640 KB)(e.g. 0400h corresponds to 16 KB).
	mov	cl,10			; 2^10 = 1024
	shr	bx,cl			; Divide by 1024 to get count of 16 KB blocks to test (e.g. A000h corresponds to 40 [28h] blocks).
	;
	; Other preparation.
	xor	ax,ax
	mov	es,ax			; ES := segment of first test RAM address, which is 0000.  Required by STGTST_CNT
	mov	ds,ax			; DS := segment of first test RAM address, which is 0000.  Required by STGTST_CNT
	mov	bp,0			; Zero our dot count.
	;
	; Do one dot now.
	mov	ax,8700h+254		; Blinking dot character.
	mov	[ss:di],ax		; Display the dot.
	inc	bp			; Increment our dot counter.

.L50:	; Display the block count-down.
	mov	si,TxtTestRamAddr
	mov	ax,31			; 31 chars from the start of "Testing RAM - Address"
	mov	dx,bx			; DX := block count-down.
	call	DispDecimal_1		; Display DL in decimal.        { Destroys: nothing }

	; In the bottom right corner, display the address of the 16 KB block that we are about to test.
	push	ds			; Save DS - what STGTST_CNT relies on.
	mov	ax,cs
	mov	ds,ax			; DispEsDxInBrCorner requires DS pointing to the CS.
	mov	dx,0
	call	DispEsDxInBrCorner	; { Destroys: nothing }
	pop	ds			; Restore DS - what STGTST_CNT relies on.

	; Test the 16 KB sized block. (Using IBM code.)
	push	di
	push	bx
	mov	cx,2000h		; 16 KB sized block : a word count required for STGTST_CNT : 2000H words = 8192 words = 16 KB
	call	STGTST_CNT		; Test block at ES/DS, sized at CX words.   ( Destroys: AX,BX,CX,DX,DI, AND SI )
	pop	bx
	pop	di
	jnz	.FAIL1			; --> if STGTST_CNT reported an error

	; At this point, the 16 KB block was tested good.
	; Display a progress dot.
	add	di,2			; Next dot position.
	mov	ax,8700h+254		; Blinking dot character.
	mov	[ss:di],ax		; Display the dot.
	inc	bp			; Increment our dot counter.

	; If 8 dots displayed, reset back to one dot.
	cmp	bp,8
	jb	.L55			; --> if less than 8
	mov	ax,0700h+' '		; Space character.
	push	cx
	mov	cx,7			; 7 characters (leave one dot in place).
.L52:	mov	[ss:di],ax		; Display the space.
	sub	di,2			; Go back one character position.
	dec	bp			; Decrement our dot counter.
	loop	.L52
	pop	cx

.L55:	; Point to segment of next 16 KB block address.
	mov	ax,es
	add	ax,0400h
	mov	es,ax			; Required by STGTST_CNT.
	mov	ds,ax			; Required by STGTST_CNT.

	; If there are more blocks to do, go do them.
	dec	bx			; Decrement our block counter.
	jnz	.L50			; --> if more blocks to do

	; All blocks done
	mov	ax,cs
	mov	ds,ax			; Restore DS pointing to the CS.
	jmp	.TEST2			; -->

.FAIL1:	mov	ax,cs
	mov	ds,ax			; Restore DS pointing to the CS.
	macroCheckpointStack 0B7h
	jmp	.FAIL			; -->

	;----------------
	; SUBTEST 2 OF 2
	; ---------------
.TEST2:	; Calculate the number of 1 KB blocks that we will be testing.
	; Put into CX.
	mov	word bx,[ss:SegTopOfRam]	; Get top of RAM as a segment (e.g. A000h corresponds to 640 KB)(e.g. 0400h corresponds to 16 KB).
	mov	cl,6			; 2^6 = 64
	shr	bx,cl			; Divide by 64 to get count of 1 KB blocks to test (e.g. A000h corresponds to 640 [0280h] blocks).
	mov	cx,bx			; CX will be the count of 1 KB blocks.
	mov	bp,cx			; Save for the read back phase.
	;
	; Write out the unique numbers.
	xor	ax,ax
	mov	es,ax			; Starting segment is 0.
.L60:	mov	word [es:0],cx		; Write the unique number (which is a block number in a word).
	mov	ax,es
	add	ax,0040h
	mov	es,ax			; Point to segment of next 1 KB block.
	loop	.L60			; loop until all 1K blocks done.
	;
	; Read back the unique numbers, seeing how they compare to what was written.
	xor	ax,ax
	mov	es,ax			; Starting segment is 0.
	mov	cx,bp			; Get back the count of 1 KB blocks
.L70:	mov	word ax,[es:0]		; Read back the unique number (which is a block number).
	cmp	ax,cx			; As expected ?
	jne	.FAIL2			; --> if not
	mov	ax,es
	add	ax,0040h		; Point to segment of next 1 KB block.
	mov	es,ax
	loop	.L70			; loop until all 1K blocks done.
	jmp	.PASS			; -->

.FAIL2:	macroCheckpointStack 0B8h				; Indicate that it was SUBTEST 2 that failed.
	;
	mov	word [ss:BadAddrSegment],es	; Save segment of address in error.
	mov	word [ss:BadAddrOffset],0	; Save  offset of address in error.
	xor	ax,cx				; AX := bit error pattern (at this time, a word).
	; AX is the bit error pattern (a word).
	; If both AH and AL are non-zero, that means that both test bytes (written/read as a word) are in error - choose either.
	; Otherwise, the non-zero register is the bit error pattern.
	cmp	ah,0
	jne	.S10				; --> if AH has a one somewhere in its bit error pattern, use AH
	mov	ah,al				; AL has bit error pattern, move into AH
.S10:	mov	al,0				; Indicate no parity error.
	mov	word [ss:BadDataParity],ax	; High_byte={discovered bit error pattern}, low_byte={no parity bit error occured}

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------
	; Display FAILED, but do NOT remove the arrow.
	mov	si,TxtTestRamAddr
	call	DisplayFailed

	; Display ">> Critical error ...", then display error (address + BEP) details, then halt the CPU.
	mov	si,TxtTestRamAddr
	jmp	CriticalErrorCond_1

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Remove the dots (actually the line).
	mov	si,TxtEmptyLine13
	call	TextToScreen

	; Put "Testing RAM - Address" back up.
	mov	si,TxtTestRamAddr
	call	TextToScreen

	; Display PASSED and remove the arrow.
	mov	si,TxtTestRamAddr
	call	DisplayPassedA

	; Set DS to the CS value, because it was changed earlier in this test.
	mov	ax,cs
	mov	ds,ax

	; Erase the address (shown in various formats) that is currently displayed in the bottom right corner of the screen.
	call	ClearBrCorner

	; The CPU interrupt 2 vector (used for NMI) was altered as part of the RAM testing.
	; Set the vector back to pointing to our NMI handler code for an unexpected NMI.
	call	SetUnexpNmi


; ****************************************************************************
; * Checkpoint 39
; ****************************************************************************
	macroCheckpointStack 39h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Testing RAM - Refresh".
; * 
; * Verify that test values written to RAM are not damaged when they are read back 90 seconds later.
; * 
; * This test is designed to cater for the fact that a fault could result in some RAM banks being refreshed and not others.
; * 
; * SUBTEST 2 of the previous test had written a unique block number to the first word of each 1 KB sized block.
; * We will take advantage of that.
; *    Step 1: Wait 90 seconds.
; *    Step 2: Verify that the unique block numbers written by SUBTEST 2 are still intact.
; *
; *
; *  NOTE: The figure of 90 seconds is based on a motherboard owned by modem7 of the Vintage Computer Forums.
; *        It is an IBM 5160 motherboard of type 256-640KB.
; *        Experimentation shows that with RAM refresh disabled, RAM holds its contents for between 60 and 70 seconds.
; *        90 seconds allows for a buffer, and the fact that someone may have a motherboard that holds contents for a little longer.
; *
; ****************************************************************************
TestRamRefr:

	; Display "Testing RAM - Refresh" in inverse, with an arrow to the left of it.
	mov	si,TxtEmptyLine14	; Line 14: "                                        "
	call	TextToScreen
	mov	si,TxtTestRamRefr	; Line 14: "Testing RAM - Refresh"
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; ---------------------------------------------------
	; 90 second delay, with progress dots being displayed.
	; ---------------------------------------------------
	;
	; For the progress dots, calculate the offset into MDA/CGA video RAM.
	; Store in DI.
	mov	si,TxtTestRamRefr	; Line 14: "Testing RAM - Refresh"
	mov	al,[cs:si]
	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI (offset in MDA/CGA video RAM) now at start of line on screen.
	add	di,48			; DI now at position where first dot to be displayed.
	;
	; Other preparation.
	mov	bp,0			; Zero our dot count.
	mov	bx,90			; We will be waiting 90 seconds.
	;
	; Do one dot now.
	mov	ax,8700h+254		; Blinking dot character.
	mov	[ss:di],ax		; Display the dot.
	inc	bp			; Increment our dot counter.

.L50:	; Display the second count-down.
	mov	si,TxtTestRamRefr
	mov	ax,31			; 31 chars from the start of "Testing RAM - Address"
	mov	dx,bx			; DX := second count-down.
	 call	DispDecimal_1		; Display DL in decimal.        { Destroys: nothing }

	; Wait one second.
	call	OneSecDelay		; { Destroys: CX, DL }

	; Display a progress dot.
	add	di,2			; Next dot position.
	mov	ax,8700h+254		; Blinking dot character.
	mov	[ss:di],ax		; Display the dot.
	inc	bp			; Increment our dot counter.

	; If 8 dots displayed, reset back to one dot.
	cmp	bp,8
	jb	.S70			; --> if less than 8
	mov	ax,0700h+' '		; Space character.
	push	cx
	mov	cx,7			; 7 characters (leave one dot in place)
.L60:	mov	[ss:di],ax		; Display the space.
	sub	di,2			; Go back one character position.
	dec	bp			; Decrement our dot counter.
	loop	.L60
	pop	cx

	; If there are more seconds to wait, go do them.
.S70:	dec	bx			; Decrement our second counter.
	jnz	.L50			; --> if more seconds to do

	; ---------------------------------------------------
	; 90 seconds have passed.
	; Now read back the unique numbers, seeing how they compare to what was written.
	; ---------------------------------------------------
	;
	mov	word bx,[ss:SegTopOfRam]	; Get top of RAM as a segment (e.g. A000h corresponds to 640 KB)(e.g. 0400h corresponds to 16 KB).
	mov	cl,6			; 2^6 = 64
	shr	bx,cl			; Divide by 64 to get count of 1 KB blocks to test (e.g. A000h corresponds to 640 [0280h] blocks).
	mov	cx,bx			; CX will be the count of 1 KB blocks.
	;
	xor	dx,dx
	mov	es,dx			; Starting segment is 0.
.L70:	mov	word ax,[es:0]		; Read back the unique number (which is a block number).
	cmp	ax,cx			; As expected ?
	jne	.FAIL			; --> if not
	mov	ax,es
	add	ax,0040h		; Point to segment of next 1 KB block.
	mov	es,ax
	loop	.L70			; loop until all 1K blocks done.
	jmp	.PASS			; -->

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0B9h

	; Record the failing address.	
	mov	word [ss:BadAddrSegment],es	; Save segment of address in error.
	mov	word [ss:BadAddrOffset],0	; Save  offset of address in error.

	; Display FAILED, but do NOT remove the arrow.
	mov	si,TxtTestRamRefr
	call	DisplayFailed

	; Display ">> Critical error ...", then display address details, then halt the CPU.
	mov	si,TxtTestRamRefr
	jmp	CriticalErrorCond_2

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Remove the dots (actually the line).
	mov	si,TxtEmptyLine14
	call	TextToScreen

	; Put "Testing RAM - Refresh" back up.
	mov	si,TxtTestRamRefr
	call	TextToScreen

	; Display PASSED and remove the arrow.
	mov	si,TxtTestRamRefr
	call	DisplayPassedA

	; Restore ES (normally points to segment address of MDA/CGA video RAM).
	mov	ax,ss
	mov	es,ax


; ****************************************************************************
; * Checkpoint 3A
; ****************************************************************************
	macroCheckpointStack 3Ah


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Testing RAM - Slow Refresh".
; * 
; * Intended to expose 'weak' chips.
; ****************************************************************************
TestRamSlowRefresh:

	; Display "Testing RAM - Slow Refresh" in inverse, with an arrow to the left of it.
	mov	si,TxtEmptyLine15	; Line 15: "                                        "
	call	TextToScreen
	mov	si,TxtTestRamSlowRefr	; Line 15: "Testing RAM - Slow Refresh"
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Set timer 1 to 2 ms
	; 1.193 MHz / 2355 (933H) = 0.506 kHz = 2 ms
	mov	al,74h			; Timer 1, LSB then MSB, mode 2, binary counter 16-bit
	out	PIT8253_ctrl,al
	mov	al,33h
	out	PIT8253_1,al
	jmp	short $+2		; Delay for I/O.
	mov	al,09h
	out	PIT8253_1,al

	; Do count-down set up
	mov	bp,20			; Initial value.

	; Fill the found RAM with 55h.
	mov	dl,55h
	call	FillRamWithByte		; ( Destroys: AX, BX, CX, DI, ES )

	; Wait 10 seconds.
	mov	bx,10
.L33:	mov	si,TxtTestRamSlowRefr
	mov	ax,31			; 31 chars from the start of "Testing RAM - Slow Refresh"
	mov	dx,bp			; DX := on-screen count-down.
	call	DispDecimal_1		; Display DL in decimal.        { Destroys: nothing }
	call	OneSecDelay		; One second delay              { Destroys: CX, DL }
	dec	bp			; Decrement count-down.
	dec	bx			; Decrement second count.
	jnz	.L33			; -->

	; Read back was what written, expecting 55h.
	mov	dl,55h
	call	TestRamWithByte		; ( Destroys: AX, BX, CX, DI, DL, DS, ES )
	jc	.FAIL			; --> bad compare

	; Fill the found RAM with AAh.
	mov	dl,0AAh
	call	FillRamWithByte

	; Wait 10 seconds.
	mov	bx,10
.L66:	mov	si,TxtTestRamSlowRefr
	mov	ax,31			; 31 chars from the start of "Testing RAM - Slow Refresh"
	mov	dx,bp			; DX := on-screen count-down.
	call	DispDecimal_1		; Display DL in decimal.        { Destroys: nothing }
	call	OneSecDelay		; One second delay              { Destroys: CX, DL }
	dec	bp			; Decrement count-down.
	dec	bx			; Decrement second count.
	jnz	.L66			; -->

	; Read back was what written, expecting AAh.
	mov	dl,0AAh
	call	TestRamWithByte
	jc	.FAIL			; --> bad compare

	jmp	.PASS			; -->

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0BAh

	; Display FAILED, but do NOT remove the arrow.
	mov	si,TxtTestRamSlowRefr
	call	DisplayFailed

	; Display ">> Critical error ...", then display error (address + data) details, then halt the CPU.
	mov	si,TxtTestRamSlowRefr
	jmp	CriticalErrorCond_1

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Set DS back to the CS value, because it was changed earlier in this test. (By 'TestRamWithByte')
	mov	ax,cs
	mov	ds,ax

	; Remove the dots (actually the line).
	mov	si,TxtEmptyLine15
	call	TextToScreen

	; Put "Testing  - Slow Refresh" back up.
	mov	si,TxtTestRamSlowRefr
	call	TextToScreen

	; Display PASSED and remove the arrow.
	mov	si,TxtTestRamSlowRefr
	call	DisplayPassedA

	; Erase the address (shown in various formats) that is currently displayed in the bottom right corner of the screen.
	call	ClearBrCorner

	; The CPU interrupt 2 vector (used for NMI) was altered as part of the RAM testing.
	; Set the vector back to pointing to our NMI handler code for an unexpected NMI.
	call	SetUnexpNmi

	; Restore timer 1 back to normal operation.
	mov	al,54h			; Timer 1, LSB only, mode 2, binary counter 16-bit
	out	PIT8253_ctrl,al
	mov	al,12h			; 1.193 MHz / 18 (12H) = 66.3 kHz = 15.1 uS
	out	PIT8253_1,al




; ****************************************************************************
; * Checkpoint 3E
; ****************************************************************************
	macroCheckpointStack 3Eh


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "8259 interrupt controller".
; ****************************************************************************
CheckPIC8259:

	; Display "8259 interrupt controller" in inverse, with an arrow to the left of it.
	mov	si,TxtPIC8259
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Initialise the 8259.
	mov	al,13h			; ICW1 = ICW4 needed, single 8259, call address interval = 8, edge triggered
	out	PIC8259_cmd,al
	jmp	short $+2		; Delay for I/O.
	mov	al,8			; ICW2 = Interrupts's start at interrupt 8
	out	PIC8259_imr,al 
	jmp	short $+2		; Delay for I/O.
	mov	al,9			; ICW4 = buffered, normal EOI, 8086 mode
	out	PIC8259_imr,al
	jmp	short $+2		; Delay for I/O.

	; Do a check of the In-Service Register (ISR).
	mov	al,0Bh			; OCW3 = read ISR on next RD pulse
	out	PIC8259_cmd,al
	jmp	short $+2		; Delay for I/O.
	in	al,PIC8259_cmd		; Should be zero
	and	al,al			; Is it?
	jne	.FAIL			; --> if no

	; Do a check of the Interrupt Enable Register (interrupt mask register).
	out	PIC8259_imr,al		; AL := zero
	jmp	short $+2		; Delay for I/O.
	in	al,PIC8259_imr		; Should be zero
	and	al,al			; Is it?
	jne	.FAIL			; --> if no
	dec	al			; AL := 0FFh
	out	PIC8259_imr,al		;  also disables all IRQs
	jmp	short $+2		; Delay for I/O.
	in	al,PIC8259_imr		; Should be 0FFh
	cmp	al,0FFh			; Is it?
	je	.PASS			; --> if yes 

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0BEh

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtPIC8259
	mov	bx,Err8259PIC
	call	DisplayFailedA

	jmp	.EXIT			; -->


.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtPIC8259
	call	DisplayPassedA

.EXIT:


; ****************************************************************************
; * Checkpoint 40
; ****************************************************************************
	macroCheckpointStack 40h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Hot IRQ interrupts".
; *
; * In the period of about a second, see if an IRQ interrupt occurs.
; * None is expected.
; *
; * Note: In the previous test, all IRQ's were masked out in the 8259.
; ****************************************************************************
CheckHotInterrupts:

	; Display "Hot IRQ interrupts" in inverse, with an arrow to the left of it.
	mov	si,TxtCheckHotIrq
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; IRQ0 through IRQ7 map to CPU interrupts 8 through 15 (0F hex).
	; Point the vectors for CPU interrupts 8 through 15 to our 'IrqHandler' routine.
	xor	ax,ax
	mov	es,ax			; ES := first segment
	mov	di,0020h		; Start at 0000:0020h (vector for CPU interrupt 8)
	mov	cx,0008h		; 8 interrupt vectors (4 bytes each) to set.
.L10:	lea	ax,IrqHandler
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2
	mov	ax,cs
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2
	loop	.L10

	; Clear our interrupt ISR recorder.
	mov	byte [es:IntIsrRecord],0	; 0000:0400 (i.e. in low RAM, which has been tested good)

	; 8259A interrupt controller
	mov	al,0Bh
	out	PIC8259_cmd,al		; OCW3 = read ISR on next RD pulse
	mov	al,0FFh
	out	PIC8259_imr,al		; Disable all IRQ interrupts.

	; CPU - Enable maskable interrupts.
	sti

	; Delay.
	xor	cx,cx
	mov	bl,4
.L20:	loop	.L20
	dec	bl
	jnz	.L20

	; CPU - Disable maskable interrupts.
	cli

	; Did any of the enabled CPU interrupts occur ?
	; None are expected, because the corresponding IRQ interrupts are disabled.
	cmp	byte [es:IntIsrRecord],0
	je	.PASS			; --> if no (none expected)

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0C0h

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtCheckHotIrq
	mov	bx,ErrHotInterrupt
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtCheckHotIrq
	call	DisplayPassedA

.EXIT:


; ****************************************************************************
; * Checkpoint 42
; ****************************************************************************
	macroCheckpointStack 42h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Checking interrupt IRQ0".
; *
; * - IRQ0 is the trigger for a 'system timer' interrupt.
; * - Earlier, the "Hot IRQ interrupts" test had set up the interrupt vector in low RAM.
; ****************************************************************************
CheckINT0:

	; Display "Checking interrupt IRQ0" in inverse, with an arrow to the left of it.
	mov	si,TxtCheckInt0
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Clear our interrupt ISR recorder.
	xor	ax,ax
	mov	es,ax
	mov	byte [es:IntIsrRecord],0	; 0000:0400 (i.e. in low RAM, which has been tested good)

	; 8259 - Enable IRQ0 (used by the system timer) and disable all other IRQ's.
	mov	al,11111110b
	out	PIC8259_imr,al

	; 8253 timer
	mov	al,10h
	out	PIT8253_ctrl,al		; Select timer 0, LSB, mode 0, binary.
	mov	al,0FFh
	out	PIT8253_0,al		; Initial count of FF into timer 0.

	; CPU - Enable maskable interrupts (the ones that we earlier unmasked).
	sti

	; Within a SHORT time period, see if the 8259's ISR showed IRQ0.
	; IRQ0 is not expected.
	mov	cx,10h
.L10:	test byte [es:IntIsrRecord],1	; Bit 0 = IRQ0
	jnz	.FAIL			; --> if IRQ0 occurred
	loop	.L10

	; CPU - Disable maskable interrupts.
	cli

	; Clear our interrupt ISR recorder.
	mov	byte [es:IntIsrRecord],0	; 0000:0400 (i.e. in low RAM, which has been tested good)

	; 8253 timer
	mov	al,10h
	out	PIT8253_ctrl,al		; Select timer 0, LSB, mode 0, binary.
	mov	al,0FFh
	out	PIT8253_0,al		; Initial count of FF into timer 0.

	; CPU - Enable maskable interrupts (the ones that we earlier unmasked).
	sti

	; Within a LONG time period, see if the 8259's ISR showed IRQ0.
	; IRQ0 is expected.
	mov	cl,2Eh
.L20:	test byte [es:IntIsrRecord],1	; Bit 0 = IRQ0
	jnz	.PASS			; --> if IRQ0 occurred
	loop	.L20

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------
	macroCheckpointStack 0C2h

	; 8259 - Disable all IRQ's.
	mov	al,11111111b
	out	PIC8259_imr,al
	cli

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtCheckInt0
	mov	bx,ErrInterrupt0
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; 8259 - Disable all IRQ's.
	mov	al,11111111b
	out	PIC8259_imr,al
	cli

	; 8253 timer - Set it to output a square wave of about 18.2 Hz.
	; ( 1.193160 MHz / FFFFh count = about 18.2 Hz )
	mov	al,36h
	out	PIT8253_ctrl,al		; Select timer 0, LSB/MSB, mode 3, binary.
	mov	al,0
	out	PIT8253_0,al		; Write LSB of 00.
	jmp	short $+2		; Delay for I/O.
	out	PIT8253_0,al		; Write MSB of 00.

	; Display PASSED and remove the arrow.
	mov	si,TxtCheckInt0
	call	DisplayPassedA

.EXIT:


; ****************************************************************************
; * Checkpoint 46
; ****************************************************************************
	macroCheckpointStack 46h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Hot NMI".
; *
; *  Check the non maskable interrupt.
; *
; *  This test is is known to fail if either:
; *  - Math coprocessor (8087 chip) is absent and switch 2 in switch block SW1 is in the wrong position for that (off).
; *  - Math coprocessor (8087 chip) is present and is faulty. 
; ****************************************************************************
CheckNMI:

	; Display "Hot NMI" in inverse, with an arrow to the left of it.
	mov	si,TxtCheckNMI
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; An NMI triggers CPU interrupt 2.
	; Change the CPU interrupt 2 vector to point to our NMI handler code for this test.
	xor	ax,ax
	mov	es,ax			; ES := 0000
	mov	di,0008h		; Offset of vector for CPU interrupt 2. (2 x 4 bytes)
	mov	ax,NmiHandler_1
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2
	mov	ax,cs
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2

	; Clear our NMI recorder/flag.
	mov	byte [es:NmiFlag],0

	; Reset/clear and disable the two 'RAM parity error' latches.
	; Because if either set, that would trigger an NMI.
	in	al,PPI8255_B
	or	al,00110000b		; Done by setting bits 5 and 4 on 8255 port B.
	out	PPI8255_B,al

	; Enable NMI interrupts to reach CPU.
	mov	al,80h
	out	NmiCtrlPort,al

	; Delay
	xor	cx,cx
	mov	bx,4
.L10:	loop	.L10
	dec	bx
	jnz	.L10

	; Disable NMI interrupts from reaching the CPU.
	xor	al,al
	out	NmiCtrlPort,al

	; Did an NMI occur ?
	; We are not expecting one.
	cmp	byte [es:NmiFlag],0
	je	.PASS			; --> if no

.FAIL:	;------------------------
	; An interrupt occurred.
	; -----------------------

	macroCheckpointStack 0C6h

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtCheckNMI
	mov	bx,ErrNMI
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtCheckNMI
	call	DisplayPassedA

	; Set CPU interrupt 2 vector (used for NMI) back to pointing to our NMI handler code for an unexpected NMI.
	call	SetUnexpNmi

.EXIT:


; ****************************************************************************
; * Checkpoint 4E
; ****************************************************************************
	macroCheckpointStack 4Eh


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Keyboard responds to reset".
; *
; * Check if a keyboard is present, by sending it a software reset,
; * then expecting the keyboard to send AA in response.
; ****************************************************************************
CheckKybReset:

	; Display "Keyboard responds to reset" in inverse, with an arrow to the left of it.
	mov	si,TxtChkKeybReset
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Clear our interrupt ISR recorder.
	xor	ax,ax
	mov	es,ax
	mov	byte [es:IntIsrRecord],0	; 0000:0400 (i.e. in low RAM, which has been tested good)

	; 8259 - Enable IRQ1 (the keyboard interrupt) and disable all other IRQ's.
	mov	al,11111101b
	out	PIC8259_imr,al

	; CPU - Enable maskable interrupts.
	sti	

	; Software reset the keyboard.
	; Done by taking the CLK line low for a while.
	mov	al,00001000b		; Clear (bit 7) = low, CLK (bit 6) = low
	out	PPI8255_B,al
	xor	cx,cx
	loop	$			; Main delay.
	mov	al,11001000b		; Clear (bit 7) = high, CLK (bit 6) = high
	out	PPI8255_B,al
	jmp	short $+2		; Small delay.
	jmp	short $+2		; Small delay.
	jmp	short $+2		; Small delay.
	mov	al,01001000b		; Clear (bit 7) = low, CLK (bit 6) = high
	out	PPI8255_B,al

	; Did IRQ1 occur (triggered by the keyboard sending a byte)?
	call	CheckForINT		; Did an interrupt occur ?
	jc	.FAIL			; --> if not
	test byte [es:IntIsrRecord],2	; Was it IRQ1 ?
	jz	.FAIL			; --> if not 

	; IRQ1 occurred.
	in	al,PPI8255_A		; Read byte sent by keyboard.
	cmp	al,0AAh			; Correct code in response to a reset ?
	je	.PASS			; --> if yes

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0CEh

	; Flag to the following test that this test failed.
	mov	byte [es:KybTestResult],1	; 0000:0401 (i.e. in low RAM, which has been tested good)

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtChkKeybReset
	mov	bx,ErrKeybReset
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Flag to the following test that this test passed.
	mov	byte [es:KybTestResult],0	; 0000:0401 (i.e. in low RAM, which has been tested good)

	; Display PASSED and remove the arrow.
	mov	si,TxtChkKeybReset
	call	DisplayPassedA

.EXIT:	; 8259 - Disable all IRQ's.
	mov	al,11111111b
	out	PIC8259_imr,al
	cli

	; Reset the keyboard interface circuitry.
	call	ResetKybInterface	; ( Destroys: AL )


; ****************************************************************************
; * Checkpoint 50
; ****************************************************************************
	macroCheckpointStack 50h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Keyboard stuck key".
; *
; * If the previous test failed:
; *   Do not do this test. Display 'N/A' where PASED or PASSED would go.
; *
; * If the previous test passed:
; *   See if the keyboard is reporting a stuck key.
; *   If not, display PASS.
; *   If so, display FAIL, and display the code for the key next to 'Stuck key found:' 
; *
; ****************************************************************************
CheckKybStuck:

	; Display "Keyboard stuck key" in inverse, with an arrow to the left of it.
	mov	si,TxtChkKeybStuck
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; If the 'Keyboard responds to reset' test failed, then:
	;   - Put N/A in place where Passed/FAILED normally goes; then
	;   - Remove the arrow; then
	;   - Abort this test.
	xor	ax,ax
	mov	es,ax
	cmp	byte [es:KybTestResult],0
	je	.CONT			; --> it passed
	mov	si,TxtChkKeybStuck
	call	TextToScreen		; Undo the inverse text on the original message.
	mov	dx,TxtNA
	call	DispSecondMsg
	mov	si,TxtChkKeybStuck
	call	RemoveArrow
	jmp	.EXIT			; -->

.CONT:	; Reset the keyboard interface circuitry.
	call	ResetKybInterface	; ( Destroys: AL )

	; Clear our interrupt ISR recorder.
	xor	ax,ax
	mov	es,ax
	mov	byte [es:IntIsrRecord],0

	; 8259 - Enable IRQ1 (the keyboard interrupt) and disable all other IRQ's.
	mov	al,11111101b
	out	PIC8259_imr,al

	; Now wait for an interrupt.
	call	CheckForINT		; Did an interrupt occur ?
	jc	.PASS			; --> if no

	; An interrupt occurred.
	; Assume IRQ1, because that it the only IRQ that we have presenty enabled.
	; Read the keyboard byte into AL.
	in	al,PPI8255_A
	push	ax			; Save AL for later

	; Display "Stuck key found:  ".
	mov	si,TxtStuckKey
	call	TextToScreen

	; To the right of that, display the stuck key (in hex) that the keyboard reported.
	pop ax				; Restore byte of stuck key.
	call DisplayALinHex

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0D0h

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtChkKeybStuck
	mov	bx,ErrKeybStuck
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtChkKeybStuck
	call	DisplayPassedA


.EXIT:	; 8259 - Disable all IRQ's.
	mov	al,11111111b
	out	PIC8259_imr,al

	; CPU - Disable maskable interrupts.
	cli

	; Reset the keyboard interface circuitry.
	call	ResetKybInterface	; ( Destroys: AL )


; ****************************************************************************
; * Checkpoint 52
; ****************************************************************************
	macroCheckpointStack 52h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Check floppy controller".
; *
; * Look for and check the Floppy Disk Controller (FDC).
; ****************************************************************************
CheckFDC:
	macroCheckpointStack 52h

	; Display "Check floppy controller" in inverse, with an arrow to the left of it.
	mov	si,TxtChkFDC
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; 8259 - Enable IRQ6 (the FDC interrupt) and disable all other IRQ's.
	mov	al,10111111b
	out	PIC8259_imr,al

	; Attempt to reset the FDC.
	call	FdcReset
	jnc	.PASS			; --> reset successful

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0D2h

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtChkFDC
	mov	bx,ErrFDC
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtChkFDC
	call	DisplayPassedA

.EXIT:


; ****************************************************************************
; * Checkpoint 54
; ****************************************************************************
	macroCheckpointStack 54h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Trying to read a floppy".
; *
; * On floppy drive 0 (A:), see if the first track on a 360K floppy can be read.
; *
; * Only do this if the controller test passed.
; ****************************************************************************
ChkReadFloppy:

	; Display "Trying to read a floppy" in inverse, with an arrow to the left of it.
	mov	si,TxtReadFloppy
	call	TextToScreenInv
	call	ShowArrow

	; Do the delay that we put between on-screen tests.
	call	InterTestDelay

	; Set DS and ES to point to the first segment.
	xor	ax,ax
	mov	ds,ax
	mov	es,ax

	; Get failure count of 'Check floppy controller' test.
	; If non-zero, then:
	;   - Put N/A in place where Passed/FAILED normally goes; then
	;   - Remove the arrow; then
	;   - Abort this test.
	mov	byte al,[ss:ErrFDC]
	cmp	al,0
	je	.CONT			; --> if no controller errors
	mov	si,TxtReadFloppy
	call	TextToScreen		; Undo the inverse text on the original message.
	mov	dx,TxtNA
	call	DispSecondMsg
	mov	si,TxtReadFloppy
	call	RemoveArrow
	jmp	.EXIT			; -->

.CONT:	mov	byte [es:FloppyReadCount],3	; Maximum of 3 read attempts

.L05:	; Clear our interrupt ISR recorder.
	xor	ax,ax
	mov	es,ax
	mov	byte [es:IntIsrRecord],0

	; Floppy drive 0 - Activate the motor
	mov	dx,FdcDor
	mov	al,00011100b	; Motor 0 = 1, /Reset = 1, Drive = 00
	out	dx,al

	; Delay.
	xor	cx,cx
	loop	$

	; 8237 DMA - Program
	mov	al,46h			; Read command for DMA.
	out	DMA8237_cmlff,al	;   clear LSB/MSB flip-flop.
	jmp	short $+2		; Delay for I/O.
	out	DMA8237_mode,al		; Send it to the 8237.

	; Configure things so that the read data will go to the area starting 0000:2000
	; 1. 8237 DMA - Set address where to store the read data = 2000h
	; 2. Set the page register for DMA chan 2 to zero.
	xor	al,al
	out	DMA8237_2_ar,al		; 00h
	jmp	short $+2		; Delay for I/O.
	mov	al,20h			; 20h
	out	DMA8237_2_ar,al
	jmp	short $+2		; Delay for I/O.
	;
	xor	al,al
	out	DmaPageRegCh2,al

	; 8237 DMA - Number of bytes to read = 511 = 01FFh 
	dec	al
	out	DMA8237_2_wc,al		; FF
	inc	al
	inc	al
	out	DMA8237_2_wc,al		; 01

	; Floppy controller uses DMA channel 2.
	; 8237 DMA - Unmask channel 2.
	mov	al,2
	out	DMA8237_mask,al

	; ---------------------------------------------------
	; Floppy drive 0 - Recalibrate
	; Try twice.
	mov	di,2
.L20:	mov	ah,7			; RECALIBRATE command
	call	FdcProgram
	mov	ah,0			; Drive = 0
	call	FdcProgram
	; Now wait for completion (via interrupt IRQ6), then read the result bytes.
	; (Although in the case of a recalibrate command, there are no result bytes.)
	call	FdcIntStatus
	jc	.S55			; --> if no interrupt
	dec	di
	jnz	.L20

	; ---------------------------------------------------
	; Floppy drive 0 - Seek
	mov	ah,0Fh			; SEEK command
	call	FdcProgram
	mov	ah,0			; Head = 0, Drive = 0
	call	FdcProgram
	mov	ah,5			; Cylinder 5
	call	FdcProgram
	; Now wait for completion (via interrupt IRQ6), then read the result bytes.
	; (Although in the case of a seek command, there are no result bytes.)
	call	FdcIntStatus
	jc	.FAIL			; --> if no interrupt

	; Delay
	xor	cx,cx
	loop	$

	; ---------------------------------------------------
	; Floppy drive 0 - Read 9 sectors from head 0 on cylinder 0.
	mov	ah,66h			; READ DATA command: One side, MFM, skip deleted data
	call	FdcProgram
	mov	ah,0			; Head = 0, Drive = 0
	call	FdcProgram
	mov	ah,0			; Cylinder 0
	call	FdcProgram
	mov	ah,0			; Head 0
	call	FdcProgram
	mov	ah,1			; Start at sector 1
	call	FdcProgram
	mov	ah,2			; 512 bytes/sector
	call	FdcProgram
	mov	ah,9			; 9 sectors to transfer
	call	FdcProgram
	mov	ah,2Ah			; Gap length
	call	FdcProgram
	mov	ah,0FFh			; Data length := non-user defined
	call	FdcProgram
	call	CheckForINT
	pushf				; Push flags
	call	FdcReadResults		; Read the result bytes
	jc	.S55
	popf				; Pop flags
	jc	.S55
	xor	ax,ax
	mov	es,ax
	test	byte [es:IntIsrRecord],40h	; Bit 6 = IRQ6
	jnz	.PASS			; --> if IRQ6 happened
.S55:
	dec	byte [es:FloppyReadCount]
	jz	.FAIL
	call	FdcReset
	jmp	.L05			; -->

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------

	macroCheckpointStack 0D4h

	; Display FAILED, update the test's error counter, and remove the arrow.
	mov	si,TxtReadFloppy
	mov	bx,ErrFdcRead
	call	DisplayFailedA

	jmp	.EXIT			; -->

.PASS:	;------------------------
	; Test successful.
	; -----------------------

	; Display PASSED and remove the arrow.
	mov	si,TxtReadFloppy
	call	DisplayPassedA


.EXIT:	; Floppy drive 0 - Stop the motor
	mov	al,00001100b	; Motor 0 = 0, /Reset = 1, Drive = 00
	mov	dx,FdcDor
	out	dx,al

	; Set DS to the CS value, because it was changed earlier in this test.
	mov	ax,cs
	mov	ds,ax


; ****************************************************************************
; * Checkpoint 60
; ****************************************************************************
	macroCheckpointStack 60h


; ****************************************************************************
; * ON-SCREEN TEST: Displayed is "Check ROM at F4000".
; * ON-SCREEN TEST: Displayed is "Check ROM at F6000".
; * ON-SCREEN TEST: Displayed is "Check ROM at F8000".
; * ON-SCREEN TEST: Displayed is "Check ROM at FA000".
; * ON-SCREEN TEST: Displayed is "Check ROM at FC000".
; *
; * Note: Function 'DoCheck8kbRom' will take care of screen updates.
; *
; ****************************************************************************
CheckExtraROMs:

	cli				; CPU - Disable maskable interrupts

	; ---------------------------------------------------
	; F4000
	; ---------------------------------------------------
	mov	si,TxtRomF4000
	push	si
	call	TextToScreenInv		; Display "Check ROM at F4000" in inverse.
	call	ShowArrow		; Put an arrow to the left of that.
	pop	si

	call	InterTestDelay		; Do the delay that we put between on-screen tests.

	mov	bx,ErrRomF4000		; The address of the error count
	mov	di,0F400h		; F400:0000
	call	DoCheck8kbRom		; Check out the ROM.

	macroCheckpointStack 61h

	; ---------------------------------------------------
	; F6000
	; ---------------------------------------------------
	mov	si,TxtRomF6000
	push	si
	call	TextToScreenInv		; Display "Check ROM at F6000" in inverse.
	call	ShowArrow		; Put an arrow to the left of that.
	pop	si

	call	InterTestDelay		; Do the delay that we put between on-screen tests.

	mov	bx,ErrRomF6000		; The address of the error count
	mov	di,0F600h		; F600:0000
	call	DoCheck8kbRom		; Check out the ROM.

	macroCheckpointStack 62h

	; ---------------------------------------------------
	; F8000
	; ---------------------------------------------------
	mov	si,TxtRomF8000
	push	si
	call	TextToScreenInv		; Display "Check ROM at F8000" in inverse.
	call	ShowArrow		; Put an arrow to the left of that.
	pop	si

	call	InterTestDelay		; Do the delay that we put between on-screen tests.

	mov	bx,ErrRomF8000		; The address of the error count
	mov	di,0F800h		; F800:0000
	call	DoCheck8kbRom		; Check out the ROM.

	macroCheckpointStack 63h

	; ---------------------------------------------------
	; FA000
	; ---------------------------------------------------
	mov	si,TxtRomFA000
	push	si
	call	TextToScreenInv		; Display "Check ROM at F8000" in inverse.
	call	ShowArrow		; Put an arrow to the left of that.
	pop	si

	call	InterTestDelay		; Do the delay that we put between on-screen tests.

	mov	bx,ErrRomFA000		; The address of the error count
	mov	di,0FA00h		; FA00:0000
	call	DoCheck8kbRom		; Check out the ROM.

	macroCheckpointStack 64h

	; ---------------------------------------------------
	; FC000
	; ---------------------------------------------------
	mov	si,TxtRomFC000
	push	si
	call	TextToScreenInv		; Display "Check ROM at FC000" in inverse.
	call	ShowArrow		; Put an arrow to the left of that.
	pop	si

	call	InterTestDelay		; Do the delay that we put between on-screen tests.

	mov	bx,ErrRomFC000		; The address of the error count
	mov	di,0FC00h		; FC00:0000
	call	DoCheck8kbRom		; Check out the ROM.


; ****************************************************************************
; * Checkpoint 6A
; ****************************************************************************
	macroCheckpointStack 6Ah


; ****************************************************************************
; * 
; * Display the switch settings.
; * 
; ****************************************************************************
DispSwitches:

	; ------------------------------------------------
	; PC switch blocks SW1 and SW2
	; ------------------------------------------------

	; Draw the empty 'PC SW1' box and 'PC SW2' box.
	mov	si,TxtSwitchPC1
	call	TextToScreen
	mov	si,TxtSwitchPC2
	call	TextToScreen
	mov	si,TxtSwitchPC3
	call	TextToScreen
	mov	si,TxtSwitchPC4
	call	TextToScreen
	mov	si,TxtSwitchPC5
	call	TextToScreen

	; Read SW1 into AL.
	; For a PC, done by setting bit 7 on 8255 port B, then reading port A.
	in	al,PPI8255_B
	or	al,10000000b
	out	PPI8255_B,al
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	in	al,PPI8255_A

	; In the 'PC SW1' box, display the SW1 switch settings.
	mov	si,TxtSwitchPC3
	mov	di,5			; On screen, start at offset 5.
	mov	cx,8			; Show all 8 switches.
	call	SubSwitches

	; Read SW2 into AL.
	;
	; A two-stage process.	
	; For a PC, done by:
	;          Switches 1 to 4:   Set bit 2 on 8255 port B, then read low nibble of port C.
	;                 Switch 5: Clear bit 2 on 8255 port B, then read LSB of port C.
	in	al,PPI8255_B
	or	al,00000100b
	out	PPI8255_B,al		; Set bit 2 on 8255 port B
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	in	al,PPI8255_C
	and	al,00001111b		; Because of later OR'ing.
	mov	bl,al			; BL := {0-0-0-0}{switches 4-3-2-1}
	in	al,PPI8255_B
	and	al,11111011b
	out	PPI8255_B,al		; Clear bit 2 of port B
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	in	al,PPI8255_C		; AL := {?-?-?-?}{?-?-?-switch 5}
	mov	cl,4
	shl	al,cl			; AL := {?-?-?-switch 5}{0-0-0-0}
	or	al,bl			; AL := {?-?-?-switch 5}{switches 4-3-2-1}

	; In the 'PC SW2' box, display the SW2 switch settings.
	mov	si,TxtSwitchPC3
	mov	di,16			; On screen, start at offset 16.
	mov	cx,5			; Show first 5 switches only.
	call	SubSwitches

	; ------------------------------------------------
	; XT switch blocks SW1
	; ------------------------------------------------

	; Draw the empty 'XT SW1' box.
	mov	si,TxtSwitchXT1
	call	TextToScreen
	mov	si,TxtSwitchXT2
	call	TextToScreen
	mov	si,TxtSwitchXT3
	call	TextToScreen
	mov	si,TxtSwitchXT4
	call	TextToScreen
	mov	si,TxtSwitchXT5
	call	TextToScreen

	; Read SW1 into AL.
	;
	; A two-stage process.	
	; For an XT, done by:
	;          Switches 1 to 4: Clear bit 3 on 8255 port B, then read low nibble of port C.
	;          Switches 5 to 8:   Set bit 3 on 8255 port B, then read low nibble of port C.
	in	al,PPI8255_B
	and	al,11110111b
	out	PPI8255_B,al		; Clear bit 3 on port B.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	in	al,PPI8255_C
	and	al,00001111b		; Because of later OR'ing.
	mov	bl,al			; BL := {0-0-0-0}{switches 4-3-2-1}
	in	al,PPI8255_B
	or	al,00001000b
	out	PPI8255_B,al		; Set bit 3 of port B
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	in	al,PPI8255_C		; AL := {?-?-?-?}{switches 8-7-6-5}
	mov	cl,4
	shl	al,cl			; AL := {switches 8-7-6-5}{0-0-0-0}
	or	al,bl			; AL := {switches 8-7-6-5){switches 4-3-2-1}

	; In the 'XT SW1' box, display the SW1 switch settings.
	mov	si,TxtSwitchXT3
	mov	di,12			; On screen, start at offset 12.
	mov	cx,8			; Show all 8 switches.
	call	SubSwitches


; ****************************************************************************
; * Checkpoint 72
; ****************************************************************************
	macroCheckpointStack 72h


; ****************************************************************************
; * 
; * 1. Display "Completed passes:" on-screen.
; * 2. To the right of that, display the count of completed passes.
; * 
; ****************************************************************************
DispPassCount:

	; Display "Completed passes:"
	mov	si,TxtCompletedPasses
	call	TextToScreen

	; Increment the varible containing the 'completed passes' count,
	;    then return the count in decimal form within BL/DL/DH.
	mov	bx,PassCount
	call	IncGetStoredCount

	; Set DI to the offset in MDA/CGA video RAM where the 'completed passes' count is displayed.
	mov	si,TxtCompletedPasses
	mov	di,18			; 18 chars from the start of "Completed passes:"
	call	CalcScreenOffset

	; On-screen, display the count.
	; In: BL = ones, DL = tens, DH = hundreds, DI = offset in MDA/CGA video RAM
	call	DispDecimal_2


; ****************************************************************************
; * Checkpoint 74
; ****************************************************************************
	macroCheckpointStack 74h


; ****************************************************************************
; * 
; * Go back and do it all again.
; * 
; ****************************************************************************
	jmp	DiagLoop




; >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
; * Subroutines
; >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


; ****************************************************************************
; Calculate the offset in MDA/CGA video memory where to place some text.
;
;   INPUTS: - DI is the number of chars to the right of the start of the text.
;           - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: - DI is the offset in MDA/CGA video memory.
;
; DESTROYS: {nothing}
;
; ****************************************************************************
CalcScreenOffset:
	push	ax			; Save AX
	push	cx			; Save CX
	push	si

	shl	di,1			; DI = DI * 2 because of attribute byte
	mov	al,[cs:si]		; Get row
	xor	ah,ah
	mov	ch,160			; = 2 * 80 (character + attribute)
	mul	ch			; AX = start of the line
	add	di,ax			; Save result
	inc	si
	mov	al,[cs:si]		; Get starting column
	xor	ah,ah
	shl	ax,1			; AX = AX * 2 because of attribute
	add	di,ax			; Save end result

	pop	si
	pop	cx			; Restore CX
	pop	ax			; Restore AX

	ret



; ****************************************************************************
; Common routine for checking the channels of the 8253 timer.
; Note: Mode 2 (rate generator) only.
;
;   INPUTS: - DX = I/O address of timer channel. (Either 40h, 41h, or 42h.)
;
;  OUTPUTS: Carry flag, clear (NC) if PASS, set (C) if FAIL.
;
; DESTROYS: AX, BX, CX, DI
;
; ****************************************************************************
Check8253:

	; Create the two counter bits for the 8259 command.
	; Put them into bits 7 and 6 of AH and AL.
	mov	ax,dx			; AX := I/O address of channel to be tested (either 40h, 41h, or 42h).
	and	al,3			; We only need the two LS bits
	ror	al,1
	ror	al,1			; Bit 1..0 -> bits 7..6 = counter
 	mov	ah,al			; Save counter bits (00, or 01, or 10)

	; Configure the target counter.
	or	al,00110100b
	out	PIT8253_ctrl,al		; LSB then MSB, mode 2, Binary
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.

	; Some preparation.
	xor	cx,cx			; CX := 0000

	; Write the counter value of 0000.
	mov	al,cl			; AL := 00
	out	dx,al			; Write an LSB of 00.
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	out	dx,al			; Write an MSB of 00.

	; See if within a certain period of time, the read back value is FFFF.
	mov	di,4
.L10:	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	mov	al,ah			; AL := counter bits (00, or 01, or 10)
	out	PIT8253_ctrl,al
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	in	al,dx
	mov	bl,al			; BL := read back LSB
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	in	al,dx
	mov	bh,al			; BH := read back MSB
	cmp	bx,0FFFFh		; Result is OK?
	je	.S20			; --> if yes 
	loop	.L10			; Another try ->
	dec	di			; Another try?
	jne	.L10			; --> if yes 

	; No, we have an error.
	jmp	.FAIL			; -->

.S20:	; See if within a certain period of time, the read back value is 0000.
	mov	di,4
.L40:	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	mov	al,ah			; AL := counter bits (00, or 01, or 10)
	out	PIT8253_ctrl,al
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	in	al,dx
	mov	bl,al			; BL := read back LSB
	jmp	short $+2		; Delay for I/O.
	jmp	short $+2		; Delay for I/O.
	in	al,dx
	mov	bh,al			; BH := read back MSB
	cmp	bx,0			; Result is OK?
	je	.PASS			; --> if yes 
	loop	.L40			; --> another try
	dec	di			; Another try?
	jnz	.L40			; --> if yes 

.FAIL:	;------------------------
	; An error/fail occurred.
	; -----------------------
	stc				; Indicate FAIL to caller.
	ret


.PASS:	;------------------------
	; Test successful.
	; -----------------------
	clc				; Indicate PASS to caller.
	ret



; ****************************************************************************
; See if an interrupt has occurred.
;
;   INPUTS: {nothing}
;
;  OUTPUTS: Carry flag, clear (NC) = interrupt occurred, set (C) = no interrupt
;
; DESTROYS: BL, CX, ES
;
; ****************************************************************************
CheckForINT:

	; CPU - Enable maskable interrupts (the ones that we earlier unmasked).
	sti

	; ES := 0000
	xor	ax,ax
	mov	es,ax

	; See if an interrupt occurred within a certain period of time.
	mov	bl,4			; Delay amount.
.L10:	xor	cx,cx
	loop	$
	cmp	byte [es:IntIsrRecord],0	; Has there been an interrupt?
	jne	.YES			; --> if yes 
	dec	bl
	jnz	.L10

	; No interrupt occurred.
	cli				; CPU - Disable maskable interrupts.
	stc				; For caller, set carry flag to indicate no interrupt.
	ret

	; An interrupt occurred.
.YES:	cli				; CPU - Disable maskable interrupts.
	clc				; For caller, clear carry flag to indicate an interrupt.
	ret	



; ****************************************************************************
; Display a critical error message on the screen.
;
;   INPUTS: - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES, SI
;
; ****************************************************************************
DispCritical:

	;
	; First, clear the line where ">> Critical error ..." will be displayed, and the following line.
	mov	al,[cs:si]		; Get line of original message
	mov	bl,al			; Save for later
	add	al,1			; Next line
	call	ClearLineY		; Clear screen line in AL. ( Destroys: DI, ES )
	add	al,1			; Next line
	call	ClearLineY		; Clear screen line in AL. ( Destroys: DI, ES )
	;
	; Now display the ">> Critical error ..."
	mov	al,bl			; Get line of original message
	add	al,1			; Next line
	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI = start of line on screen, as offfset in MDA/CGA video RAM
	add	di,4			; Start of critical message
	; Now display the text.
	mov	si,TxtCritical		; " >> Critical error, diagnostics have been stopped! << "
	mov	dh,70h			; Char attribute = inverted
	jmp	TextToScreen3		; Trail ends in a RET  ( Destroys: AX, BX, DI, DX, ES, {SI=SI-2} )


 
; ****************************************************************************
; 1. Display the text "FAILED" against the pointed-to message.
; 2. Remove the arrow against the pointed-to message.
; 3. Increment the associated error count.
; 4. Display the incremented error count to the left of "FAILED".
;
;   INPUTS: - BX points to the varible containing the error count.
;           - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES, SI
;
; ****************************************************************************
DisplayFailedA:

	push	si
	push	bx

	; Undo the inverse text on the pointed-to test text.
	call	TextToScreen		

	; Set DI to to the offset in MDA/CGA video RAM where the error count is to be displayed.
	mov	di,26
	call	CalcScreenOffset	; ( Destroys: nothing )

	; BX points to the varible containing the error count.
	; Increment that error count variable,
	;      then return the count in decimal form within BL/DL/DH.
	pop	bx
	call	IncGetStoredCount

	; Get back our pointer to the test text parameters.
	pop	si
	push	si

	; On-screen, display the incremented error count.
	; In: BL = ones, DL = tens, DH = hundreds, DI = offset in MDA/CGA video RAM
	call	DispDecimal_2

	; Display "FAILED"
	mov	dx,TxtFailed
	mov	bl,70h			; Char attribute = inverted
	call	DispSecondMsg2

	; Remove the arrow.
	pop	si
	jmp	RemoveArrow		; Trail ends in a RET   ( Destroys: AX, BX, CX, DI, DX, ES, SI )



; ****************************************************************************
; Display the text "FAILED" against the pointed-to message.
;
;   INPUTS: - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CH, DI, DX, ES, SI
;
; ****************************************************************************
DisplayFailed:

	; Undo the inverse text on the pointed-to message.
	call	TextToScreen

	; Display "FAILED"
	mov	dx,TxtFailed
	mov	bl,70h			; Char attribute = inverted
	jmp	DispSecondMsg2		; ( Destroys: AX, BX, CH, DI, DX, ES, SI )



; ****************************************************************************
; Display the text "PASSED" against message and remove the arrow.
;
;   INPUTS: - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES, SI
;
; ****************************************************************************
DisplayPassedA:

	push	si

	; Undo the inverse text on the pointed-to message.
	call	TextToScreen		; ( Destroys: AX, BX, CH, DI, DX, ES )

	; Display "PASSED"
	mov	dx,TxtPassed
	call	DispSecondMsg		; ( Destroys: AX, BX, CH, DI, DX, ES, SI )

	; Remove the arrow.
	pop	si
	jmp	RemoveArrow		; Trail ends in a RET   ( Destroys: AX, BX, CX, DI, DX, ES, SI )



; ****************************************************************************
; Display the value in DX at the position specified by SI and DI.
;
;   INPUTS: - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;           - AX = offset from start of the on-screen test text
;           - DX = word to display (in decimal)
;
;  OUTPUTS: {nothing}
;
; DESTROYS: {nothing}
;
; ****************************************************************************
DispDecimal_1:

	push	bx
	push	di
	push	si
	mov	di,ax
	call	CalcScreenOffset	; DI now at position where second count-down to be displayed.
	mov	ax,dx
	call	AX2DEC			; out: BL = ones, DL = tens, DH = hundreds  in:	AX = number < 1000
	call	DispDecimal_2		;  in: BL = ones, DL = tens, DH = hundreds, DI = offset in MDA/CGA video RAM
	pop	si
	pop	di
	pop	bx

	ret



; ****************************************************************************
; Display the decimal value in DH/DL/BL at the specified location in MDA/CGA video RAM.
;
;   INPUTS: - DI = Offset within MDA/CGA video RAM.
;           - BL = ones
;           - DL = tens
;           - DH = hundreds
;           
;  OUTPUTS: {nothing}
;
; DESTROYS: {nothing}
;
; ****************************************************************************
DispDecimal_2:

	push	ax
	push	dx

	cmp	dh,'0'			; Skip hundreds?
	ja	.L10			; --> if no display number

	mov	dh,' '			; Fill hundreds with a space

	cmp	dl,'0'			; Skip tens?
	jne	.L10			; --> if no

	mov	dl,' '			; Fill tens with a space
.L10:
	mov	ah,07h			; Char attribute = normal
	mov	al,dh
	mov	[ss:di],ax		; Hundreds -> screen

	mov	al,dl
	mov	[ss:di+2],ax		; Tens -> screen

	mov	al,bl
	mov	[ss:di+4],ax		; Ones -> screen

	pop	dx
	pop	ax

	ret


 
; ****************************************************************************
; Display a status message to the right of the pointed-to on-screen test text.
;
; On-screen test text example: 'Testing CPU'
;     Status message examples: 'Passed', 'FAILED', and 'N/A   '
;
;   INPUTS: - DX points to the status message to display.
;           - SI points to parameters of the on-screen test text.
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; DESTROYS: DispSecondMsg : AX, BX, CH, DI, DX, ES, SI
;           DispSecondMsg2: AX, BX, CH, DI, DX, ES, SI
;
; ****************************************************************************
DispSecondMsg:

	mov	bl,07h			; Char attribute = normal

DispSecondMsg2:

	; Calculate position on the screen where the status message is to go.
	call	CalcScreenPos		; DI := starting postion of text (offset into MDA/CGA video RAM). ( Destroys: AX, CH, DI )
	add	di,30*2			; The status message starts 30 characters to the right of starting position.

	mov	si,dx			; SI := position on screen where the status message is to go.
	mov	dh,bl			; Char attribute

	jmp	TextToScreen3		; Trail ends in a RET  ( Destroys: AX, BX, DI, DX, ES, {SI=SI-2} )



; ****************************************************************************
; Convert AL to decimal.
;
;   INPUTS: - AL = number
;
;  OUTPUTS: - BL = ones
;           - DL = tens
;           - DH = hundreds
;
; DESTROYS: {nothing}
;
; ****************************************************************************
AL2DEC:

	xor	ah,ah
	; Fall through to AX2DEC



; ****************************************************************************
; Convert AX to decimal.
;
;   INPUTS: - AX = number < 1000
;
;  OUTPUTS: - BL = ones
;           - DL = tens
;           - DH = hundreds
;
; DESTROYS: {nothing}
;
; ****************************************************************************
AX2DEC:

	push	ax

	mov	dx,3030h		; Hundreds and tens
.L20:	cmp	ax,100			; Smaller than 99?
	jb	.L30			; --> if yes

	inc	dh
	sub	ax,100			; Subtract 100
	jmp	.L20			; Another check for hundreds

.L30:	cmp	al,10			; Smaller than 9?
	jb	.S40			; --> if yes

	inc	dl
	sub	al,10			; Subtract 10
	jmp	.L30			; Another check for tens

.S40:	or	al,30h			; Number -> ASCII
	mov	bl,al			; Save AL

	pop	ax

	ret



; ****************************************************************************
; Reset/clear then re-arm the two 'RAM parity error' latches.
;
;   INPUTS: {nothing}
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AL
;
; ****************************************************************************
BounceParityDetection:

	in	al,PPI8255_B
	or	al,00110000b		; Reset/clear by setting bits 5 and 4 on 8255 port B.
	out	PPI8255_B,al
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	nop				; Delay for I/O.
	and	al,11001111b		; Re-arm by clearing bits 5 and 4 on 8255 port B.
	out	PPI8255_B,al
	ret



; ****************************************************************************
; Check out an 8 KB sized ROM and then display the result.
;
;   INPUTS: - DI = start of 8 KB range.
;           - SI = original text.
;
;  OUTPUTS: - {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES, SI
;
; ****************************************************************************
DoCheck8kbRom:

	call	Check8kbRom		; ( Destroys: AX, CX, DX)
	jnc	.PASS			; --> if good
	jmp	DisplayFailedA		; Trail ends in a RET   ( Destroys: AX, BX, CX, DI, DX, ES, SI )
.PASS:	jmp	DisplayPassedA		; Trail ends in a RET   ( Destroys: AX, BX, CX, DI, DX, ES, SI )



; ****************************************************************************
; Check the passed 8 KB sized ROM.
; 
; Primarily, we are looking for the 8-bit checksum being 00. 
;
; However, if that is the only thing we did, empty ROM sockets could pass.
; E.g. Motherboard #1 returns FF reading an empty ROM socket. 8 KB of FF has an 8-bit checksum of 00.
; E.g. Motherboard #1 returns FE reading an empty ROM socket. 8 KB of FE has an 8-bit checksum of 00.
; E.g. Motherboard #1 returns FC reading an empty ROM socket. 8 KB of FC has an 8-bit checksum of 00.
;
; BTW. I have IBM 51xx motherboards that read FC, FE, and FF. Who knows what other variations there are.
;
; Ideally, we want empty ROM sockets to show as FAILED.
;
; Note that identifying an empty ROM socket cannot always be determined by seeing if ALL bytes of the ROM are the same.
; An example is the IBM 62X0819 motherboard ROM (32 KB) for the IBM 5160.
; The second and third 8 KB blocks of that 32 KB ROM contain nothing but CC (also having an 8-bit checksum of 00).
;
; SUMMARY: There is no foolproof way of determining an empty ROM socket.
;
; But a compromise is possible.
;
; Fail the test if either of the following are true:
; Failure criterion #1: The 8-bit checksum is not 00; or
; Failure criterion #2: All bytes are identical, AND, the byte is in [FC to FF].
;
;   INPUTS: - DI = start of 8 KB range (e.g. for F400:0000, passed is F400).
;
;  OUTPUTS: Carry flag, clear (NC) = SUCCESS, set (C) = ERROR
;
; DESTROYS: AX, CX, DX
;
; ****************************************************************************
Check8kbRom:

	push	ds
	push	si

	mov	ds,di		; DS = segment to be tested.
	mov	cx,2000h	; 8 KB of ROM.
	xor	dx,dx		; Zero our byte recorder.
	xor	si,si		; Zero our offset counter.
	xor	ah,ah		; Zero the running checksum.

.L10:	; Read the content of every byte in this ROM.
	lodsb			; Copy contents of [SI] into AL, then increment SI.
	add	ah,al		; Add to the running 8-bit checksum.
	; If first byte, store it.
	cmp	cx,2000h
	jne	.S20
	mov	dl,al		; DL := record of first byte.
.S20:	; Compare the byte read to the first byte read.
	; If different, flag that in DH.
	cmp	al,dl
	je	.S30
	mov	dh,1		; Flag non-identical bytes in content.
.S30:	loop	.L10

	; All 8KB is processed.
	;
	; ------------------------------
	; Failure criterion #1 - 8-bit checksum is not 00.
	cmp	ah,0
	jne	.FAIL		; --> if not 00
	;
	; ------------------------------
	; 8-bit checksum is 00.
	; That is good, but the test can still fail.
	; Failure criterion #2 - All bytes are identical, AND, the byte is in [FC to FF].
	cmp	dh,1
	je	.PASS		; --> if bytes non-identical
	; All bytes are identical.
	; The test fails if the byte is in [FC to FF].
	mov	al,dl
	cmp	dl,0FFh
	je	.FAIL		; --> FF
	cmp	al,0FEh
	je	.FAIL		; --> FE
	cmp	al,0FDh
	je	.FAIL		; --> FD
	cmp	al,0FCh
	je	.FAIL		; --> FC

.PASS:	clc			; Clear carry flag to indicate success.
	jmp	.EXIT		; -->

.FAIL:	stc			; Set carry flag to indicate ERROR.

.EXIT:	pop	si
	pop	ds

	ret



; ****************************************************************************
; Increment, then fetch the pointed-to error count.
;
;   INPUTS: - BX points to the error count variable.
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;
;  OUTPUTS: - BL = ones
;           - DL = tens
;           - DH = hundreds

; DESTROYS: AL
;
; ****************************************************************************
IncGetStoredCount:

	inc	byte [ss:bx]		; Increment the pointed-to count.
	mov	al,[ss:bx]		; Fetch the count.
	jmp	AL2DEC			; Convert it to decimal.   ( Trail ends in a RET )



; ****************************************************************************
; Floppy Disk Controller:
; Reset.
;
;   INPUTS: {nothing}
;
;  OUTPUTS: Carry flag, clear (NC) = SUCCESS, set (C) = ERROR
;
; DESTROYS: ??????????????????
;
; ****************************************************************************
FdcReset:

	xor	ax,ax
	mov	es,ax

	; Up to 5 attempts.
	mov	byte [es:FdcResetCount],5

	; FDC - DOR - enable interrupts and DMA (bit 3 = 1), normal operation (bit 2 = 1)
.L10:	mov	dx,FdcDor		; Digital Output Register (DOR)
	mov	al,00001100b
	out	dx,al

	; Delay
	xor	cx,cx
	loop	$

	; Reset.
	; FDC - DOR - enable interrupts and DMA (bit 3 = 1), reset (bit 2 = 0)
	mov	al,00001000b
	out	dx,al

	; Delay
	loop	$

	; FDC - DOR - enable interrupts and DMA (bit 3 = 1), normal operation (bit 2 = 1)
	mov	al,00001100b
	out	dx,al

	; Clear our interrupt ISR recorder.
	mov	byte [es:IntIsrRecord],0

	; Did IRQ6 happen?
	call	FdcIntStatus
	test	byte [es:IntIsrRecord],40h	; Bit 6 = IRQ6
	jnz	.CONT			; --> if interrupt 6 happened
	dec	byte [es:FdcResetCount]
	jnz	.L10

	stc				; Set carry flag to indicate ERROR.
	ret

.CONT:	; Perform the Specify command so that the FDC de-asserts IRQ6.
	mov	ah,3			; Specify
	call	FdcProgram
	mov	ah,0CFh			; Step rate time: 12 ms.
	call	FdcProgram
	mov	ah,2			; Head load time: 6 ms
	call	FdcProgram
	clc				; Clear carry flag to indicate success.
	ret



; ****************************************************************************
; Floppy Disk Controller:
; Sense interrupt status.
;
;   INPUTS: {nothing}
;
;  OUTPUTS: Carry flag, clear (NC) = SUCCESS, set (C) = ERROR
;
; DESTROYS: ??????????????????
;
; ****************************************************************************
FdcIntStatus:

	; See if an interrupt occur within a certain period of time.
	call	CheckForINT
	jnc	.CONT			; --> if yes
	ret				; otherwise return with carry flag set

.CONT:	mov	ah,8			; Sense interrupt status.
	call	FdcProgram
	jc	.ERROR			; --> if a problem
	;
	call	FdcReadResults		; Read the result bytes.
	jc	.ERROR			; --> if a problem

	clc				; Clear carry flag to indicate SUCCESS.
	ret

.ERROR:	stc				; Set carry flag to indicate ERROR.
	ret



; ****************************************************************************
; Floppy Disk Controller:
; Program command/data register 0 with the passed byte.
;
;   INPUTS: - AH = byte for command/data register 0.
;
;  OUTPUTS: - Carry flag, clear (NC) = SUCCESS, set (C) = ERROR
;
; DESTROYS: ??????????????????
;
; ****************************************************************************
FdcProgram:

; Via bit 6 (DIO), wait for controller to indicate ready to receive commands.
	xor	cx,cx
	mov	dx,FdcMsr	; Main status register (MSR)
	mov	si,4
.L10:	in	al,dx
	test	al,40h		; Bit 6 (DIO)
	jz	.S20		; --> if 0 (ready to receive)
	loop	.L10
	dec	si
	jnz	.L10
	; Timeout - Never came ready.
	jmp	.TIMEOUT

; Via bit 7 (RQM), wait for controller to indicate that CPU allowed to interact with data register. 
.S20:	xor	cx,cx
	mov	si,4
.L20:	in	al,dx
	test	al,80h		; Bit 7 (RQM)
	jnz	.S30		; --> if 1
	loop	.L20
	dec	si
	jnz	.L20
	; Timeout - Never came ready.

.TIMEOUT:
	stc			; Set carry flag to indicate ERROR.
	ret

; Now send the command that was passed in AH.
.S30:	mov	dx,FdcCdr0	; Command/data register 0
	mov	al,ah
	out	dx,al

	clc			; Clear carry flag to indicate SUCCESS.
	ret



; ****************************************************************************
; Floppy Disk Controller:
; Read the result bytes (but no need to store them anywhere).
;
;   INPUTS: {nothing}
;
;  OUTPUTS: Carry flag, clear (NC) = SUCCESS, set (C) = ERROR
;
; DESTROYS: ??????????????????
;
; ****************************************************************************
FdcReadResults:

	xor	cx,cx
	mov	ds,cx		; DS points to first segment

	mov	dx,FdcMsr	; Main status register (MSR)

	; Wait for RQM bit (bit 7) to go HIGH, i.e. CPU allowed to interact with data register
	mov	si,4
.L10:	in	al,dx
	test	al,10000000b
	jnz	.S15		; --> if bit is HIGH
	loop	.L10
	dec	si
	jnz	.L10
	; Timeout - Never came ready.
	jmp	.ERROR		; -->

	; Verify that DIO bit (bit 6) is HIGH, i.e. controller --> CPU
.S15:	in	al,dx
	test	al,01000000b
	jnz	.READY		; --> if bit is HIGH
	jmp	.ERROR		; -->

	; Read the result bytes.
.READY:	mov	bl,7		; 7 result bytes at most to be read.
	xor	si,si
.L40:	mov	dx,FdcCdr0	; Command/data register 0
	in	al,dx
	inc	si		; Point to next storage location.
	;
	; Delay
	mov	cx,0Fh
	loop	$
	;
	mov	dx,FdcMsr	; Main status register (MSR)
	in	al,dx
	test	al,00010000b	; Busy ?
	jz	.EXIT		; --> if no longer busy
	dec	bx
	jnz	.L40

.ERROR:
	stc			; Set carry flag to indicate ERROR.
	ret

.EXIT:
	; Set DS to the CS value, because it was changed earlier in this test.
	mov	ax,cs
	mov	ds,ax

	clc			; Clear carry flag to indicate SUCCESS.	
	ret



; ****************************************************************************
; Handle an interrupt triggered by an IRQ.
; Do that by recording the 8259's ISR.
; ****************************************************************************
IrqHandler:

	; Record the ISR in the variable that we use for that purpose.
	mov	al,0Bh
	out	PIC8259_cmd,al		; OCW3 = read ISR on next RD pulse.
	jmp	short $+2		; Delay for I/O.
	in	al,PIC8259_cmd
	or	byte [es:IntIsrRecord],al

	; 8259 - Send an end-of-interrupt command.
	mov	al,20h
	out	PIC8259_cmd,al

	iret	



; ****************************************************************************
; NMI handler for the "Hot NMI" test.
; ****************************************************************************
NmiHandler_1:

	; Disable NMI interrupts from reaching the CPU.
	xor	ax,ax
	out	NmiCtrlPort,al

	; Flag that an NMI happened.
	mov	es,ax
	mov	byte [es:NmiFlag],1

	iret



; ****************************************************************************
; NMI handler for an unexpected NMI.
;
; Step 1: Send the checkpoint of 99h.
; Step 2: Clear the MDA/CGA screen then display '*** UNEXPECTED NMI ***'.
;
; Note that this code could potentially be called very early, before the stack has been set up.
; Therefore, do not assume that the stack has been set up.
;
; Note that if this handler gets called, it indicates that low RAM must be present (i.e. required for the NMI vector).
;
; ****************************************************************************
NmiHandler_2:

	; CPU - Disable maskable interrupts, and set the direction flag to increment.
	cli
	cld

	; For the following, set up the stack in low RAM.
	xor	ax,ax
	mov	ss,ax
	mov	sp,0100h

	; Send checkpoint 99h
	macroCheckpointStack 99h

	; Clear the MDA/CGA screen.
	mov	ax,ss
	mov	es,ax			; Point ES to segment address of MDA/CGA video RAM.
	mov	ax,0700h+' '		; Attribute + space
	xor	di,di			; Start at first screen position
	mov	cx,4000			; 4000 words.
	rep	stosw			; STOSW: AX-->[ES:DI], then DI=DI+2

	; Go to the second line and write "*** UNEXPECTED NMI ***"
	mov	si,TxtUnexpNmi
	call	TextToScreen

	; Halt the CPU
	cli
	hlt



; ****************************************************************************
; An NMI triggers CPU interrupt 2.
; Point the CPU interrupt 2 vector to our NMI handler for an unexpected NMI.
; ****************************************************************************
SetUnexpNmi:

	xor	ax,ax
	mov	es,ax			; ES := 0000
	mov	di,0008h		; Offset of vector for CPU interrupt 2. (2 x 4 bytes)
	mov	ax,NmiHandler_2
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2
	mov	ax,cs
	stosw				; STOSW: AX-->[ES:DI], then DI=DI+2
	ret



; ****************************************************************************
; Remove the arrow placed at the start of the pointed-to test text.
;
;   INPUTS: - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES, SI
;
; ****************************************************************************
RemoveArrow:

	mov	dl,1
	jmp	SubArrow



; ****************************************************************************
; Display an arrow against the pointed-to test text.
;
;   INPUTS: - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES, SI
;
; ****************************************************************************
ShowArrow:
	xor	dl,dl			; Show arrow

SubArrow:
	call	CalcScreenPos		; DI := starting postion of text (offset into MDA/CGA video RAM). ( Destroys: AX, CH, DI )
	sub	di,4			; Starting position of '->'
	mov	dh,07h			; Char attribute = normal

	or	dl,dl			; Show arrow?
	jnz	.S10			; --> if no 

	mov	si,TxtShowArrow
	jmp	.S20			; -->

.S10:
	mov	si,TxtRemoveArrow
.S20:
	jmp	TextToScreen3		; ( Destroys: AX, BX, DI, DX, ES, SI )

TxtRemoveArrow:
	db '  ', 0

TxtShowArrow:
	db '->', 0



; ****************************************************************************
; Subroutine for outputting a byte to:
;     - the three standard LPT ports; and
;     - the RS-232 serial port at I/O port 3F8h ('COM1'); and
;     - IBM's debug port.
;
; For the serial port, send a CRLF sequence BEFORE the byte, and convert the byte to two ASCII bytes.
;
;   INPUTS: AL contains the byte/code to output.
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, CL, DI, DX
;
; ****************************************************************************
CheckpointStack:

	;--------------------------------------------
	; Output to the standard parallel/LPT ports.
	;--------------------------------------------
	mov	dx,LPT1
	out	dx,al
	mov	dx,LPT2
	out	dx,al
	mov	dx,LPT3			; I/O port 3BCh. Parallel/LPT port on most MDA cards.
	out	dx,al

	;--------------------------------------------
	; Output the byte to IBM AT's debug port.  Rarely works for PC's and XT's.
	;--------------------------------------------
	out	80h,al
	
	;--------------------------------------------
	; Display the byte in the top-right corner of the screen.
	;--------------------------------------------
	call	DispAlTopCorner		; ( Destroys: BX, CL, DI )

	;--------------------------------------------
	; Output the byte to the serial port of 3F8h ('COM1').
	; Start with a CRLF sequence, then the byte.
	;
	; The byte needs to be converted to ASCII.
	; For example, 8Ah would be converted to two bytes: 38h for the '8' followed by 42h for the 'A'.
	;--------------------------------------------

	; Save the byte for later.
	mov	bp,ax

	; Send a CRLF sequence.
	call	SendCrlfToCom1	; ( Destroys: AX, DX )

	; Send the byte as two ASCII bytes.
	mov	ax,bp			; Get the byte to send back into AL.
	call	SendAlToCom1Ascii	; ( Destroys: AX, BP, BX, CL, DX )

	;------------------------
	; Return to caller.
	;------------------------
	ret



; ****************************************************************************
; A critical error occured.
; 1. Display the bad address and data, and also send it to COM1.
; 2. Halt the CPU.
; ****************************************************************************
CriticalErrorCond_1:

	; Display the failing address and data on-screen.
	call	DispBadAddressAndData

	; If COM1 is fitted, send the failing address and bit error pattern (BEP) to COM1.
	; Example: If sent earlier was B8, and the failing address is C000, and BEP is 02, then COM1 monitoring device will see 'B8 C000 02'.
	call	SendBadAddressToCom1
	call	SendBadDataToCom1

	; Now halt the CPU.
	call	SendCrlfHashToCom1	; Send a CR/LF/hash sequence to COM1, indicating CPU halt.
	cli
	hlt


; ****************************************************************************
; A critical error occured.
; 1. Display the bad address, and also send it to COM1.
; 2. Halt the CPU.
; ****************************************************************************
CriticalErrorCond_2:

	; Display the failing address on-screen.
	call	DispBadAddress

	; If COM1 is fitted, send the failing address to COM1.
	; Example: If sent earlier was B9, and the failing address is C000, then COM1 monitoring device will see 'B9 C000'.
	call	SendBadAddressToCom1

	; Now halt the CPU.
	call	SendCrlfHashToCom1	; Send a CR/LF/hash sequence to COM1, indicating CPU halt.
	cli
	hlt


; ****************************************************************************
; Send the failing address to the RS-232 serial port at I/O port 3F8h ('COM1').
; Preceed that with a space character.
;
;   INPUTS: - Variable 'BadAddrSegment' contains the segment of the failing address.
;           - Variable 'BadAddrOffset' contains the offset of the failing address.
;
; REQUIREMENT: For XLATB, DS is set to the CS (where Tbl_ASCII is). This is normally the case in this program.
;
;     OUTPUTS: {nothing}
;
;    DESTROYS: AX, BP, BX, CL, DX
;
; ****************************************************************************
SendBadAddressToCom1:

	; ---------------------------------------
	; Send a space character.
	; ---------------------------------------
	mov	al,' '
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )

	; ---------------------------------------
	; Calculate the absolute address from variables 'BadAddrSegment' and 'BadAddrOffset'.
	; Store the result (eg. 084A3F hex) in the 3-byte variable named 'AbsoluteAddress'.
	; ---------------------------------------
	mov	word ax,[ss:BadAddrSegment]
	mov	word dx,[ss:BadAddrOffset]
	call	CalcAbsFromSegOff	; From AX:DX  ( Destroys: BX, CL )

	; ---------------------------------------
	; Send the failing address.
	; Variable 'AbsoluteAddress' is 3 bytes, e.g. address 084A3F, 6 digits.
	; But this computer is a PC or XT, and so the first digit will always be zero.
	; So in sending the address, do not send the first digit.
	; ---------------------------------------
	;
	; Send the second digit of the six.
	mov	byte al,[ss:AbsoluteAddress+0]
	and	al,0Fh		; AL only has low nibble.
	mov	bx,Tbl_ASCII
	xlatb			; Convert AL into ASCII.
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )
	;
	; Send the third and fourth digits of the six.
	mov	byte al,[ss:AbsoluteAddress+1]
	call	SendAlToCom1Ascii	; ( Destroys: AX, BP, BX, CL, DX )
	;
	; Send the fifth and sixth digits of the six.
	mov	byte al,[ss:AbsoluteAddress+2]
	call	SendAlToCom1Ascii	; ( Destroys: AX, BP, BX, CL, DX )

.EXIT:  ; ---------------------------------------
	; Return to caller.
	; ---------------------------------------
	ret



; ****************************************************************************
; Send the stored bad data bits (Bit Error Pattern) to the RS-232 serial port at I/O port 3F8h ('COM1').
; Preceed that with a space character.
;
;   INPUTS: Variable 'BadDataParity' contains the bad data bits (upper byte) and parity bit indicator (lower byte).
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BP, BX, CL, DX
;
; ****************************************************************************
SendBadDataToCom1:

	; Send a space character.
	mov	al,' '
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )

	; Send the bad data bits.
	mov	word ax,[ss:BadDataParity]	; High byte is the bad data bits (the Bit Error Pattern).
	mov	al,ah
	call	SendAlToCom1Ascii	; ( Destroys: AX, BP, BX, CL, DX )

	; Return to caller.
	ret


; ****************************************************************************
; In ASCII form, send the byte in AL to the RS-232 serial port at I/O port 3F8h ('COM1').
; For example, 8Ah would be converted to two bytes: 38h for the '8' followed by 42h for the 'A'.
;
;   INPUTS: AL contains the byte/code to output.
;
; REQUIREMENT: For XLATB, DS is set to the CS (where Tbl_ASCII is). This is normally the case in this program.
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BP, BX, CL, DX
;
; ****************************************************************************
SendAlToCom1Ascii:

	; See if we earier detected a serial port at I/O port 3F8h ('COM1').
	cmp	byte [ss:Com1Exists],1
	jne	.EXIT		; --> COM1 does not exist

	; Save the passed byte for later.
	mov	bp,ax

	; Send the first byte; the high nibble of the passed byte.
	mov	cl,4
	shr	al,cl		; High nibble to low nibble.
	mov	bx,Tbl_ASCII
	xlatb			; Convert AL into ASCII.
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )

	; Send the second byte; the low nibble of the passed byte.
	mov	ax,bp		; Get the passed byte back.
	and	al,0Fh		; AL only has low nibble of passed AL.
	mov	bx,Tbl_ASCII
	xlatb			; Convert AL into ASCII.
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )

	; Return to caller.
.EXIT	ret


; ****************************************************************************
; In RAW form, send the byte in AL to the RS-232 serial port at I/O port 3F8h ('COM1').
;
;   INPUTS: AL contains the byte/code to output.
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, DX
;
; ****************************************************************************
SendAlToCom1Raw:

	; See if we earier detected a serial port at I/O port 3F8h ('COM1').
	cmp	byte [ss:Com1Exists],1
	jne	.EXIT		; --> COM1 does not exist

	; Save the byte in AL for later.
	mov	ah,al

	; Wait for the COM1 UART to indicate that it is ready for a TX byte.
	; Implement a timeout, just in case.
	mov	dx,COM1_lsr	; Line status register (LSR).
	xor	cx,cx		; Our timeout.
.L10:	in	al,dx
	and	al,00100000b
	jnz	.S10		; --> UART is ready
	dec	cx		; Decrement our timeout.
	jnz	.L10		; If not timed out, see again if UART is ready.
.S10:	; UART is ready, or the timeout occurred.

	; Send the byte.
	mov	al,ah		; Get the byte to send back into AL.
	mov	dx,COM1_tx_rx_dll
	out	dx,al

	; Return to caller.
.EXIT	ret


; ****************************************************************************
; Send a CR/LF/hash sequence to the RS-232 serial port at I/O port 3F8h ('COM1').
;
;   INPUTS: {nothing}
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, DX
;
; ****************************************************************************
SendCrlfHashToCom1:

	; Send a CRLF sequence.
	call	SendCrlfToCom1	; ( Destroys: AX, DX )

	; Send the hash.
	mov	al,'#'
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )

	; Send another CRLF sequence.
	call	SendCrlfToCom1	; ( Destroys: AX, DX )

	; Return to caller.
	ret


; ****************************************************************************
; Send a CR/LF/hash sequence to the RS-232 serial port at I/O port 3F8h ('COM1').
;
;   INPUTS: {nothing}
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, DX
;
; ****************************************************************************
SendCrlfToCom1:

	mov	al,0Dh		; Carriage return (CR)
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )
	mov	al,0Ah		; Line feed (LF)
	call	SendAlToCom1Raw	; ( Destroys: AX, DX )
	ret


; ****************************************************************************
; Subroutine to display a row of switch block settings.
;
;   INPUTS: AL = Contains the switch block settings
;           DI = Place on screen
;           CX = Number of switches in AL to display
; ****************************************************************************
SubSwitches:

	; Set DI to the offset in MDA/CGA video RAM where the switch state is displayed.
	call	CalcScreenOffset

	; Display the switch states.
.L10:	shr	al,1			; Shift right 1 bit. Bit 0 goes into carry flag.
	jc	.OFF			; If carry set, --> bottom row

	; Switch is ON.
	; Place 'X' on top row.
	mov	byte [ss:di],'X'
	jmp	.S20

.OFF:	; Switch is OFF.
	; Place 'X' on bottom row.
	mov	byte [ss:di+160],'X'

.S20:	add	di,2			; Point to next screen position.
	loop	.L10

	ret



; ****************************************************************************
; Display pointed-to text, and display an arrow to the left of it.
;
;   INPUTS: - SI points to parameters of the on-screen text (e.g. 'Testing CPU') to be displayed.
;             Parameters:
;                 1st byte: row
;                 2nd byte: column
;                 The text starts at the 3rd byte
;
;  OUTPUTS: SI points to text starting at the 3rd byte.
;
; DESTROYS: AX, BX, CX, DI, DX, ES
;
; ****************************************************************************
TextToScreenA:

	push	si
	call	TextToScreen	; ( Destroys: AX, BX, CH, DI, DX, ES )
	call	ShowArrow	; ( Destroys: AX, BX, CX, DI, DX, ES, SI }
	pop	si
	ret



; ****************************************************************************
; Display pointed-to text, in inverse.
;
;   INPUTS: - SI points to parameters of the on-screen text (e.g. 'Testing CPU') to be displayed.
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; TextToScreen  destroys:  AX, BX, CH, DI, DX, ES
; TextToScreen2 destroys:  AX, BX, CH, DI, DX, ES
; TextToScreen3 destroys:  AX  BX      DX  DI  ES, {SI=SI-2}
;
; ****************************************************************************
TextToScreenInv:

	mov	dh,70h			; Char attribute = inverse
	jmp	TextToScreen2



; ****************************************************************************
; Display pointed-to text, with preset or custom attribute.
;
;   INPUTS: - SI points to parameters of the on-screen text (e.g. 'Testing CPU') to be displayed.
;             Parameters:
;                1st byte: row
;                2nd byte: column
;                The text starts at the 3rd byte
;
;  OUTPUTS: {nothing}
;
; TextToScreen  destroys:  AX, BX, CH, DI, DX, ES
; TextToScreen2 destroys:  AX, BX, CH, DI, DX, ES
; TextToScreen3 destroys:  AX  BX      DX  DI  ES, {SI=SI-2}
;
; ****************************************************************************
TextToScreen:
	mov	dh,07h			; Char attribute = normal

TextToScreen2:
	call	CalcScreenPos		; DI := starting postion of text (offset into MDA/CGA video RAM). ( Destroys: AX, CH, DI )
					; SI now pointing to text starting at the 3rd byte.
					; ( Destroys: AX, CH, DI )
	
TextToScreen3:
	mov	ah,dh			; Set attribute for character
	; Copy the text to the screen.
	mov	dx,ss			; 
	mov	es,dx			; Point ES to segment address of MDA/CGA video RAM
	mov	bx,si			; Save SI
.L10:
	mov	al,[cs:si]		; Read character
	inc	si
	and	al,al			; End of the text?
	jz	.EXIT			; --> if yes 
	stosw				; Write character plus attribute { STOSW: AX-->[ES:DI], then DI=DI+2 }
	jmp	.L10			; -->
.EXIT:
	mov	si,bx
	sub	si,2			; Restore original SI (if TextToScreen or TextToScreen2 called)

	ret	



; ****************************************************************************
; Calculate the offset into MDA/CGA video RAM where the 
; pointed-to on-screen test text (e.g. 'Testing CPU') is to go.
;
;   INPUTS: - SI points to parameters of the on-screen test text (e.g. 'Testing CPU').
;             Parameters:
;                 1st byte: row
;                 2nd byte: column
;                 The text starts at the 3rd byte
;
;  OUTPUTS: DI is offset into MDA/CGA video RAM.
;
; DESTROYS: AX, CH, DI
;
; ****************************************************************************
CalcScreenPos:

	mov	al,[cs:si]		; Get row.
	inc	si

	xor	ah,ah
	mov	ch,160			; 160 bytes per line (80 chars per line @ 2 bytes each char).
	mul	ch			; AX = AL x 160
	mov	di,ax			; Save result.

	mov	al,[cs:si]		; Get column.
	inc	si

	xor	ah,ah
	shl	ax,1			; Muliply by 2 (because of attribute byte).
	add	di,ax			; DI := place where to copy the text to

	ret



; ****************************************************************************
; Display the word in AX on the screen in hex (e.g. if AX contains C35A hex, display "C35A").
;
;   INPUTS: - AX contains the word to display.
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;           - DI is the offset into that video RAM to write to.
;
;  OUTPUTS: {nothing}
;
; DESTROYS: BX, CL
;
; ****************************************************************************
DisplayAXinHex:

	push	ss
	push	di
	push	ax

	; Do AH.
	xchg	al,ah			; AH into AL
	call	DisplayALinHex		; ( Destroys: BX, CL )

	; Point to next position on screen.
	add	di,4

	; Do AL.
	xchg	al,ah			; Get our original AL back
	call	DisplayALinHex		; ( Destroys: BX, CL )

	; Restore saved registers.
	pop	ax
	pop	di
	pop	ss

	ret



; ****************************************************************************
; Display the byte in AL on the screen in hex (e.g. if AL contains C3 hex, display "C3").
;
;   INPUTS: - AL contains the byte to display.
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;           - DI is the offset into that video RAM to write to.
;
; REQUIREMENT: For XLATB, DS is set to the CS (where Tbl_ASCII is). This is normally the case in this program.
;
;  OUTPUTS: {nothing}
;
; DESTROYS: BX, CL
;
; ****************************************************************************
DisplayALinHex:

	push	ss
	push	di
	push	ax

	; Do high nibble.
	mov	cl,4
	shr	al,cl			; High nibble to low nibble.
	mov	bx,Tbl_ASCII
	xlatb				; Convert AL into ASCII.
	mov	ah,07h			; Char attribute = normal
	mov	[ss:di],ax		; Write to the screen.

	; Point to next position on screen.
	add	di,2

	; Do low nibble.
	pop	ax
	push	ax
	and	al,0fh			; AL only has low nibble of passed AL.
	mov	bx,Tbl_ASCII
	xlatb				; Convert AL into ASCII.
	mov	ah,07h			; Char attribute = normal
	mov	[ss:di],ax		; Write to the screen.

	; Restore saved registers.
	pop	ax
	pop	di
	pop	ss

	ret



; ****************************************************************************
; Test the specified block of RAM.
;
;   INPUTS: - ES is segment of first test RAM address.
;           - CX is size to check (i.e. most possible is FFFF, which is 64K-1).
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;
;  OUTPUTS: - Carry flag, clear (NC) = SUCCESS, set (C) = ERROR
;           - If data or parity error, the segment of the failing address is in variable 'BadAddrSegment'.
;           - If data or parity error, the offset of the failing address is in variable 'BadAddrOffset'.
;           - If data or parity error, the pattern of bad data bits (a byte) and parity bit indicator (a byte) are stored at [ss:BadDataParity]
;
; DESTROYS: SI, AX, CX, DX    (CX via the loop)
;
; ****************************************************************************
TESTRAMBLOCK:

	; Save BX
	push bx

	; Prepare for the actual check
	mov	word [ss:BadAddrSegment],0	; Clear the variable containing: segment of address in error
	mov	word [ss:BadAddrOffset],0	; Clear the variable containing: offset of address in error
	mov	word [ss:BadDataParity],0	; Clear the variable containing: bad data bits (in high byte) and parity bit indicator (in low byte)
	xor	dx,dx				; Clear our record of the bad bits for current address.
	mov	si,dx				; SI = offset of first test RAM address, which is 0000

	; Remove any existing RAM parity error indication that may have been latched earlier.
	call	BounceParityDetection

.NEWADDRESS:
	; Address loop. We are now pointing to next test address.

	; Zero pattern of bad data bits for the current test address.
	xor	dh,dh

	; Get the first test value (of five) into AH
	mov	word bx,TblTestData
	mov	byte ah,[cs:bx]

.NEWVAL:	; Value loop. We are now using a new test value. 

	; Write the current test value (in AH) to the current test address.
	mov	byte es:[si],ah

	; Read the byte back into AL.
	nop				; Delay for I/O.
	mov	byte al,es:[si]

	; Add 
	xor	al,ah			; AL = bad bit pattern for the current test value
	or	dh,al			; Add that to our record of the bad bits for current address.

	; Do next test value for current test address.
	inc	bx
	mov	ah,[cs:bx]
	cmp	bx,TblTestData+5	; 5th check value done?
	jne	.NEWVAL			; --> if no, do same test address with our new test value

	; All five test values done for the current test address.
	; If a problem was detected, jump to the error code.
	cmp	dx,0
	jne	.FAIL

	; No problem at the current test address - on to next address.
	inc	si
	loop	.NEWADDRESS

	; At this point, the complete block tested good from a data perspective.
	; But maybe the parity chip (or circuitry) is faulty.
	in	al,PPI8255_C
	and	al,11000000b		; A RAM parity error (either motherboard or card) ?
	jz	.PASS			; --> if no 
	mov	dl,1			; Temporarily record the parity error.
	mov	si,0			; SI will be final address of block. Point SI to starting address in block.
	; Fall through to FAIL.


.FAIL:	; Either:
	; - A data test of an address failed; or
	; - A data test of the block passed, but a parity related problem was detected.
	;
	; For the caller, store the segment and offset of the address.
	mov	word [ss:BadAddrSegment],es
	mov	word [ss:BadAddrOffset],si
	;
	; For the caller, save bad data bits (in high byte) and parity bit indicator (in low byte).
	mov	word [ss:BadDataParity],dx
	;
	; Set carry flag to indicate ERROR.
	stc
	jmp	.EXIT		; -->

.PASS: 	; Clear carry flag to indicate SUCCESS.
	clc

.EXIT:	pop	bx
	ret



; ****************************************************************************
; There was a failure of a RAM related test (address or data).
; Display details of the address.
;
;   INPUTS: - SI points to LINE,COLUMN,"Check first 2 KB of RAM", or or "Testing RAM ....."
;           - Variable 'BadAddrSegment' contains the segment of the failing address.
;           - Variable 'BadAddrOffset' contains the offset of the failing address.
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES
;
; ****************************************************************************
DispBadAddress:

	push 	si			; Preserve SI.

	mov	al,[cs:si]		; Read line number byte of "Check first 2 KB of RAM" or "Testing RAM ....."
	mov	bl,al			; Save for later.

	; ---------------------------------------
	; Display  ">> Critical error ...".
	; ---------------------------------------
	push	bx
	call	DispCritical		; SI is to point to LINE,COLUMN,"Check first 2 KB of RAM" or "Testing RAM ....."  ( Destroys: AX, BX, CX, DI, DX, ES, SI )
	pop	bx

	; ---------------------------------------
	; Clear the 8 lines after "Critical error ...".
	; ---------------------------------------
	mov	al,bl			; Read line number byte of "Check first 2 KB of RAM" or "Testing RAM ....."
	add	al,1			; Next line.
	mov	cx,8			; 8 lines to clear.
.L10:	add	al,1			; Next line.
	call	ClearLineY		; Clear screen line in AL. ( Destroys: DI, ES )
	loop	.L10

	; ---------------------------------------
	; Display the address.
	; ---------------------------------------
	mov	al,bl			; Read line number byte of "Check first 2 KB of RAM"  or "Testing RAM ....."
	add	al,3			; Desired line.
	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI (offset in MDA/CGA video RAM) is now at start of line on screen.
	mov	bp,di			; Save for later.

	; Display "  Failure at address:     KB  (exactly xxxxx)"
	mov	si,TxtFailAtAddress
	mov	dh,07h			; Char attribute = normal
	call	TextToScreen3		; ( Destroys: AX, BX, DI, DX, ES )

	; Display the failing address as a KB address.
	mov	di,bp			; Back at start of line.
	add	di,44			; Now at position to put KB value.
	mov	word ax,[ss:BadAddrSegment]
	mov	word dx,[ss:BadAddrOffset]
	call	DispSegOffAsKb		; AX:DX  ( Destroys: CL )

	; Display the absolute address figure.
	mov	di,bp			; Back at start of line.
	add	di,78			; Now at "xxxxx" on line.
	mov	word ax,[ss:BadAddrSegment]
	mov	word dx,[ss:BadAddrOffset]
	call	DispSegOffAsAbsolute	; AX:DX  ( Destroys: BX, CL )

	; Display the failing address in the bottom right corner of the screen.
	mov	es,ax
	call	DispEsDxInBrCorner	; ( Destroys: nothing )

	; ---------------------------------------
	; Exit
	; ---------------------------------------
	pop 	si			; Restore SI.
	ret				; Return to caller.



; ****************************************************************************
; There was a failure of a RAM related test (address or data).
; Display address and data details.
;
;   INPUTS: - SI points to LINE,COLUMN,"Check first 2 KB of RAM", or "Testing RAM ....."
;           - Variable 'BadAddrSegment' contains the segment of the failing address.
;           - Variable 'BadAddrOffset' contains the offset of the failing address.
;           - Variable 'BadDataParity' contains the bad data bits (upper byte) and parity bit indicator (lower byte).
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, DX, ES

; ****************************************************************************
DispBadAddressAndData:

	; ---------------------------------------
	; Display the address.
	; ---------------------------------------
	call	DispBadAddress		; ( Destroys: AX, BX, CX, DI, DX, ES )

	; ---------------------------------------
	; Display the data bits.
	; ---------------------------------------

	; Display the "Bad bits: ...' header.
	mov	al,[cs:si]		; Read line number byte of "Check first 2 KB of RAM" or "Testing RAM ....."
	mov	bl,al			; Save for later.
	add	al,5			; Target line for the header.
	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI (offset in MDA/CGA video RAM) is now at start of line on screen.
	mov	si,TxtBadBits		; "  Bad bits:  7 6 5 4 3 2 1 0  P        "
	mov	dh,07h			; Char attribute = normal
	push	bx
	call	TextToScreen3		; ( Destroys: AX, BX, DI, DX, ES )
	pop	bx

	; Display the data bits.
	mov	al,bl			; Read line number byte of "Check first 2 KB of RAM" or "Testing RAM ....."
	add	al,6			; Target line for the data bits.
	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI (offset in MDA/CGA video RAM) is now at start of line on screen.
	add	di,26			; 13th column.
	mov	word ax,[ss:BadDataParity]	; Get the bad data bits (byte) and parity bit indicator (byte).
	mov	bh,ah			; Put bad data bit pattern into BH.
.L20:	mov	cx,8
.L22:	shl	bh,1			; Bit is set?
	jnc	.L24			; --> if not
	mov	ax,7000h+'X'		; 'X' in inverse.
	jmp	.L26
.L24:	mov	ax,0700h+'O'		; 'O' in normal.
.L26:	mov	[ss:di],ax		; Write bit result.
	add	di,2			; Next screen position.
	mov	ax,0700h+' '		; Space
	mov	[ss:di],ax		; Display a space.
	add	di,2			; Next screen position.
	loop	.L22			; If CX <> zero, -> next data bit

	; ---------------------------------------
	; Display the parity bit, but only if there are no bad data bits.
	; ---------------------------------------
	add	di,2			; Next screen position.
	mov	word dx,[ss:BadDataParity]	; Get the bad data bits (byte) and parity bit indicator (byte).
	cmp	dh,0
	jne	.L32			; --> if bad data bits recorded.
	; No bad data bits. Display the parity bit.
	mov	ax,0700h+'O'		; 'O' in normal.
	cmp	dl,1			; Was an error in the parity bit recorded ?
	jne	.L30			; --> if no
.L28:	mov	ax,7000h+'X'		; 'X' in inverse.
.L30:	mov	[ss:di],ax		; Write parity bit result to screen.
	jmp	.L40			; -->
.L32:	; There are bad data bits.
	; Do not display the parity bit.
	; Overwrite the 'P' on the line above with a space.
	sub	di,160
	mov	ax,0700h+' '
	mov	[ss:di],ax
	add	di,160

.L40:	; ---------------------------------------
	; Display the warning.
	; ---------------------------------------
	add	di,12			; Advance 6 screen positions.
	mov	si,TxtRamChipWarning	; "<----- May or may not be RAM chips."
	mov	dh,07h			; Char attribute = normal
	call	TextToScreen3		; ( Destroys: AX, BX, DI, DX, ES )

	; ---------------------------------------
	; Return to caller.
	; ---------------------------------------
	ret


; ****************************************************************************
; Display the passed segment:offset address as an KB one.
;
; E.g. 0400:C000 gets displayed as "64 KB".
;
;     INPUTS: - AX contains the segment of the address.
;             - DX contains the offset of the address.
;             - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;             - DI is the offset into that video RAM to write to.
;
;
;    OUTPUTS: {nothing}
;
;   DESTROYS: CL
;
; ****************************************************************************
DispSegOffAsKb:

	; Save AX and DX, because this subroutine must not destroy them.
	push	ax
	push	dx

	; Display the KB amount.
	mov	cl,10
	shr	dx,cl			; DX = DX/1024  (e.g. C000 --> 0030)
	mov	cl,6
	shr	ax,cl			; AX = AX/64    (e.g. 0400 --> 0010)
	add	ax,dx
	call	AX2DEC			; out: BL = ones, DL = tens, DH = hundreds  in:	AX = number < 1000
	call	DispDecimal_2		;  in: BL = ones, DL = tens, DH = hundreds, DI = offset in MDA/CGA video RAM

	; Restore AX and DX.
	pop	dx
	pop	ax

	ret


; ****************************************************************************
; Display the passed segment:offset address as an absolute one.
;
; E.g. E81C:A67D gets displayed as "F283D".
;
;     INPUTS: - AX contains the segment of the address.
;             - DX contains the offset of the address.
;             - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;             - DI is the offset into that video RAM to write to.
;
; REQUIREMENT: For XLATB, DS is set to the CS (where Tbl_ASCII is). This is normally the case in this program.
;
;    OUTPUTS: {nothing}
;
;   DESTROYS: BX, CL
;
; ****************************************************************************
DispSegOffAsAbsolute:

	; Save AX and DX and DI, because this subroutine must not destroy them.
	push	ax
	push	dx
	push	di

	; Calculate the absolute address from the passed segment and offset address.
	; Store the result in the 3-byte variable named 'AbsoluteAddress'.
	call	CalcAbsFromSegOff	; From AX:DX  ( Destroys: BX, CL )

.S10:	; Display the absolute address.
	; Variable 'AbsoluteAddress' is 3 bytes, e.g. address 084A3F, 6 digits.
	; But this computer is a PC, and so the first digit will always be zero.
	; So in displaying the address, do not display the first digit.
	mov	bx,Tbl_ASCII
	mov	byte al,[ss:AbsoluteAddress+0]
	xlatb				; Convert AL into ASCII.
	mov	ah,07h			; Char attribute = normal
	mov	[ss:di],ax		; Write to the screen.
	;
	add	di,2			; Advance 1 character on-screen.
	;
	mov	byte al,[ss:AbsoluteAddress+1]
	call	DisplayALinHex		; ( Destroys: BX, CL )
	;
	add	di,4			; Advance 2 characters on-screen.
	;
	mov	byte al,[ss:AbsoluteAddress+2]
	call	DisplayALinHex		; ( Destroys: BX, CL )

	; Restore AX and DX and DI.
	pop	di
	pop	dx
	pop	ax

	ret


; ****************************************************************************
; Calculate the absolute address from a passed segment and offset address.
; Store the result in the 3-byte variable named 'AbsoluteAddress'.
;
; E.g. E81C:A67D gets converted to 0F283D.
;
;     INPUTS: - AX contains the segment of the address.
;             - DX contains the offset of the address.
;
;    OUTPUTS: Result placed in our 3-byte variable named 'AbsoluteAddress'
;
;   DESTROYS: BX, CL
;
; ****************************************************************************
CalcAbsFromSegOff:

	; Save AX and DX, because this subroutine must not destroy them.
	push	dx
	push	ax			; Note: Will be popped mid routine.

	; Get a preliminary first byte for the screen. We may need to increment it later.
	; Store in AL.
	mov	cl,12			; 
	shr	ax,cl			; AL: = first byte for screen, E.g. E81C --> E

	; Now calculate the next two bytes, and if required, increment the first byte.
	; Store in DX.
	pop	bx			; BX: = segment
	push	bx
	mov	cl,4			; 
	shl	bx,cl			; E.g. E81C --> 81C0
	add	dx,bx			; Add that to the offset, e.g. 81C0 + A67D = 283D with carry
	jnc	.S10			; --> if no carry
	inc	al			; Increment out first byte, e.g. E --> F

.S10:	; Store the bytes.
	mov	byte [ss:AbsoluteAddress+0],al
	mov	byte [ss:AbsoluteAddress+1],dh
	mov	byte [ss:AbsoluteAddress+2],dl

	; Restore AX and DX.
	pop	ax
	pop	dx

	ret



; ****************************************************************************
; Display the passed segment:offset address in the bottom right corner of the screen.
;
;   INPUTS: - ES contains the segment.
;           - DX contains the offset.
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;
;  OUTPUTS: {nothing}
;
; DESTROYS: {nothing}
;
; ****************************************************************************
DispEsDxInBrCorner:

	; Save registers.
	push	ax
	push	bx
	push	cx
	push	dx
	push	di
	push	ds

	; Display the segment.
	mov	di,BrCornerOffset
	mov	ax,es
	call	DisplayAXinHex		; At address SS:DI, which is in MDA/CGA video RAM.
					; ( Destroys: BX, CL )
	; Display a colon.
	add	di,8			; Advance four character positions to get past the segment.
	mov	ax,0700h+':'
	mov	[ss:di],ax
	add	di,2			; Advance one character position to get past the colon.

	; Display the offset.
	mov	ax,dx
	call	DisplayAXinHex		; At address SS:DI, which is in MDA/CGA video RAM.
	add	di,10			; Advance five character positions to get past the offset.
					; ( Destroys: BX, CL )
	; Display an equals.
	mov	ax,0700h+'='
	mov	[ss:di],ax
	add	di,4			; Advance two character positions to get past the equals.

	; Display the absolute address figure.
	mov	ax,es
	call	DispSegOffAsAbsolute	; AX:DX  ( Destroys: BX, CL )
	add	di,12			; Advance six character positions to get past the absolute address figure.

	; Display an equals.
	mov	ax,0700h+'='
	mov	[ss:di],ax
	add	di,4			; Advance two character positions to get past the equals.

	; Display a KB address (calculated from the segment:offset in AX:DX).
	; E.g. 0400:C000 = 10000 = 64 KB, so display '64 KB'.
	mov	ax,es
	call	DispSegOffAsKb		; AX:DX  ( Destroys: CL )
	add	di,8			; Advance four character positions to get past the KB figure.
	mov	ax,0700h+'K'
	mov	[ss:di],ax
	add	di,2			; Advance a character position.
	mov	ax,0700h+'B'
	mov	[ss:di],ax

	; Restore registers.
	pop	ds
	pop	di
	pop	dx
	pop	cx
	pop	bx
	pop	ax

	ret


; ****************************************************************************
; Erase the address (shown in various formats) that is currently displayed in the bottom right corner of the screen.
;
;   INPUTS: - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;
;  OUTPUTS: {nothing}
;
; DESTROYS: DI
;
; ****************************************************************************
ClearBrCorner:

	push	ax
	push	cx

	mov	cx,26		; 26 words to erase.
	mov	di,BrCornerOffset
	mov	ax,0700h+' '
.L10:	mov	[ss:di],ax
	inc 	di
	inc 	di
	loop	.L10

	pop	cx
	pop	ax

	ret



; ****************************************************************************
; Display the passed byte, in hex, at the top right corner of the screen.
;
;   INPUTS: - AL contains the byte.
;           - SS points to the base of MDA or CGA video RAM, as applicable. (We also store some variables there.)
;
;  OUTPUTS: {nothing}
;
; DESTROYS: BX, CL, DI
;
; ****************************************************************************
DispAlTopCorner:

	mov	di,150
	call	DisplayALinHex		; At address SS:DI, which is in MDA/CGA video RAM.
					; ( Destroys: BX, CL )
	ret



; ****************************************************************************
; On-screen, clear the specified line number.
;
;   INPUTS: - AL is the line number (1 to 25).
;
;  OUTPUTS: {nothing}
;
; DESTROYS: DI, ES
;
; ****************************************************************************
ClearLineY:

	push	ax
	push	bx
	push	cx
	push	dx
	push	si

	xor	ah,ah
	mov	ch,160
	mul	ch
	mov	di,ax			; DI = start of line on screen
	mov	si,TxtClearLine+2
	mov	dh,07h			; Char attribute = normal
	call	TextToScreen3		; ( Destroys: AX, BX, DI, DX, ES )

	pop	si
	pop	dx
	pop	cx
	pop	bx
	pop	ax
	ret



; ****************************************************************************
; Inter-test delay.
;
;   INPUTS: {nothing}
;
;  OUTPUTS: {nothing}
;
; DESTROYS: CX, DL
;
; ****************************************************************************
InterTestDelay:

	mov	dl,2		; About half a second assuming 4.77 MHz.
	sub	cx,cx
.L10:	loop	.L10
	dec	dl
	jnz	.L10
	ret



; ****************************************************************************
; One second delay.
; Assumption: 8088 running at 4.77 MHz.
;
;   INPUTS: {nothing}
;
;  OUTPUTS: {nothing}
;
; DESTROYS: CX, DL
;
; ****************************************************************************
OneSecDelay:

	mov	dl,4
	sub	cx,cx
.L10:	loop	.L10
	dec	dl
	jnz	.L10
	ret



; ****************************************************************************
; Fill found RAM with the byte passed in DL.
;
;   INPUTS: - DL contains the byte.
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AX, BX, CX, DI, ES
;
; ****************************************************************************
FillRamWithByte:

	; Calculate the number of 16 KB blocks to fill.
	; Put into BX.
	mov	word bx,[ss:SegTopOfRam]	; Get top of RAM as a segment (e.g. A000h corresponds to 640 KB)(e.g. 0400h corresponds to 16 KB).
	mov	cl,10			; 2^10 = 1024
	shr	bx,cl			; Divide by 1024 to get count of 16 KB blocks to test (e.g. A000h corresponds to 40 [28h] blocks).

	; Starting segment is 0000.
	xor	ax,ax
	mov	es,ax

.L10:	; Write the specified byte into the 16 KB sized block at segment in ES.
	mov	ah,dl
	mov	al,dl
	mov	di,0
	mov	cx,2000h		; 8K words = 16 KB
	rep	stosw			; STOSW: AX-->[ES:DI], then DI=DI+2

	; Point to segment of next 16 KB block address.
	mov	ax,es
	add	ax,0400h
	mov	es,ax

	; If there are more blocks to fill, go do them.
	dec	bx			; Decrement our block counter.
	jnz	.L10			; --> if more blocks to do

	ret



; ****************************************************************************
; Read all addresses of found RAM, expecting to read the byte passed in DL.
; If all bytes are read as expected, pass, otherwise fail.
;
;   INPUTS: - DL contains the byte.
;
;  OUTPUTS: - Carry flag, clear (NC) = SUCCESS, set (C) = ERROR
;           - If data or parity error, the segment of the failing address is in variable 'BadAddrSegment'.
;           - If data or parity error, the offset of the failing address is in variable 'BadAddrOffset'.
;           - If data or parity error, the pattern of bad data bits (a byte) and parity bit indicator (a byte) are stored at [ss:BadDataParity]
;
; DESTROYS: AX, BX, CX, DI, DL, ES
;
; ****************************************************************************
TestRamWithByte:

	; Calculate the number of 16 KB blocks to read.
	; Put into BX.
	mov	word bx,[ss:SegTopOfRam]	; Get top of RAM as a segment (e.g. A000h corresponds to 640 KB)(e.g. 0400h corresponds to 16 KB).
	mov	cl,10			; 2^10 = 1024
	shr	bx,cl			; Divide by 1024 to get count of 16 KB blocks to test (e.g. A000h corresponds to 40 [28h] blocks).

	; Starting segment is 0000.
	xor	ax,ax
	mov	es,ax

.L10:	; In the bottom right corner, display the address of the 16 KB block that we are about to test.
	push	dx
	mov	dx,0
	call	DispEsDxInBrCorner	; { Destroys: nothing }
	pop	dx			; get back DL (the byte to look for).
	;
	; Read the 16 KB sized block at segment in ES.
	mov	si,0
.L15:	mov	al,byte [es:si]
	cmp	al,dl
	je	.S10			; --> compare good
	; Failure
	; For the caller, store the segment and offset of the address, and the bad data bits.
	mov	word [ss:BadAddrSegment],es
	mov	word [ss:BadAddrOffset],si
	xor	al,dl
	mov	ah,0
	xchg	al,ah			; bad bit pattern into AH, 0 into AL.
	mov	word [ss:BadDataParity],ax
	stc				; Set carry flag to indicate ERROR.
	ret

.S10:	inc	si
	cmp	si,4000h
	jb	.L15			; --> not all 16 KB done yet

	; Point to segment of next 16 KB block address.
	mov	ax,es
	add	ax,0400h
	mov	es,ax

	; If there are more blocks to read, go do them.
	dec	bx			; Decrement our block counter.
	jnz	.L10			; --> if more blocks to do

	; Erase the address in the bottom right corner.
	call	ClearBrCorner

 	; Clear carry flag to indicate SUCCESS.
	clc

	ret



; ****************************************************************************
; Reset the keyboard interface circuitry on the motherboard.
;   - This clears the 74LS322 shift register. 
;   - This clears IRQ1 (hardware interrupt 1 request).
;
; Done by positive-pulsing the PB7 pin on the 8255.
;
;   INPUTS: {nothing}
;
;  OUTPUTS: {nothing}
;
; DESTROYS: AL
;
; ****************************************************************************
ResetKybInterface:

	mov	al,11001000b		; Clear (bit 7) = high, CLK (bit 6) = high
	out	PPI8255_B,al
	jmp	short $+2		; Small delay.
	jmp	short $+2		; Small delay.
	jmp	short $+2		; Small delay.
	mov	al,01001000b		; Clear (bit 7) = low, CLK (bit 6) = high
	out	PPI8255_B,al
	ret



; ****************************************************************************
; THIS SUBROUTINE PERFORMS A READ/WRITE STORAGE TEST ON A BLOCK
;      OF STORAGE.
; ENTRY REQUIREMENTS:
;       ES = ADDRESS OF STORAGE SEGMENT BEING TESTED
;       DS = ADDRESS OF STORAGE SEGMENT BEING TESTED
;       CX = WORD COUNT OF STORAGE BLOCK TO BE TESTED
; EXIT PARAMETERS:
;       ZERO FLAG = 0 IF STORAGE ERROR (DATA COMPARE OR PARITY
;       CHECK. AL=O DENOTES A PARITY CHECK. ELSE AL=XOR'ED
;       BIT PATTERN OF THE EXPECTED DATA PATTERN VS THE ACTUAL
;       DATA READ.
; AX,BX,CX,DX,DI, AND SI ARE ALL DESTROYED.
; ****************************************************************************
PORT_B	EQU 061H	; PORT B READ/WRITE DIAGNOSTIC REGISTER
PORT_C	EQU 062H	; 8255 PORT C ADDR
STGTST_CNT:
	MOV BX,CX	; SAVE WORD COUNT OF BLOCK TO TEST
	CLD		; SET DIR FLAG TO INCREMENT
	SUB DI,DI	; SET DI=OFFSET 0 REL TO ES REG
	SUB AX,AX	; SETUP FOR O->FF PATTERN TEST
.C2_1:
	MOV [DI],AL	; ON FIRST BYTE
	MOV AL,[DI] 
	XOR AL,AH	; O.K.?
	JNZ .COMPERR
	; GO ERROR IF NOT <--- changed
	INC AH
	MOV AL,AH
	JNZ .C2_1	; LOOP TILL WRAP THROUGH FF
	MOV AX,055AAH 	; GET INITIAL DATA PATTERN TO WRITE
	MOV DX,AX 	; SET INITIAL COMPARE PATTERN.
	REP STOSW 	; FILL STORAGE LOCATIONS IN BLOCK
	IN AL,PORT_B
	OR AL,030H	; TOGGLE PARITY CHECK LATCHES
	OUT PORT_B,AL
	NOP
	AND AL,0CFH
	OUT PORT_B,AL
;
	DEC DI		; POINT TO LAST WORD JUST WRITTEN
	DEC DI
	STD		; SET DIR FLAG TO GO BACKWARDS
	MOV SI,DI	; INITIALIZE DESTINATION POINTER
	MOV CX,BX	; SETUP WORD COUNT FOR LOOP
.C3:			;       INNER TEST LOOP
	LODSW	 	; READ OLD TEST WORD FROM STORAGE	; { LODSW: [DS:SI]-->AX, then SI=SI+2 }
	XOR AX,DX	; DATA READ AS EXPECTED ?
	JNE .COMPERR	; NO - GO TO ERROR ROUTINE
	MOV AX,0AA55H	; GET NEXT DATA PATTERN TO WRITE
	STOSW		; WRITE INTO LOCATION JUST READ		; { STOSW: AX-->[ES:DI], then DI=DI+2 }
	LOOP .C3	; DECREMENT WORD COUNT AND LOOP
;
	CLD		; SET DIR FLAG TO GO FORWARD
	INC DI		; SET POINTER TO BEG LOCATION
	INC DI
	MOV SI,DI	; INITIALIZE DESTINATION POINTER
	MOV CX,BX	; SETUP WORD COUNT FOR LOOP
	MOV DX,AX	; SETUP COMPARE PATTERN OF "0AA55H"
.C4:			;       INNER TEST LOOP
	LODSW		; READ OLD TEST WORD FROM STORAGE	; { LODSW: [DS:SI]-->AX, then SI=SI+2 }
	XOR AX,DX	; DATA READ AS EXPECTED?
	JNE .COMPERR	; NO - GO TO ERROR ROUTINE
	MOV AX,0FFFFH	; GET NEXT DATA PATTERN TO WRITE
	STOSW		; WRITE INTO LOCATION JUST READ		; { STOSW: AX-->[ES:DI], then DI=DI+2 }
	LOOP .C4	; DECREMENT WORD COUNT AND LOOP
;
	DEC DI		; POINT TO LAST WORD JUST WRITTEN
	DEC DI
	STD		; SET DIR FLAG TO GO BACKWARDS
	MOV SI,DI	; INITIALIZE DESTINATION POINTER
	MOV CX,BX	; SETUP WORD COUNT FOR LOOP
	MOV DX,AX	; SETUP COMPARE PATTERN "0FFFFH"
.C5:			;       INNER TEST LOOP
	LODSW		; READ OLD TEST WORD FROM STORAGE	; { LODSW: [DS:SI]-->AX, then SI=SI+2 }
	XOR AX,DX	; DATA READ AS EXPECTED?
	JNE .COMPERR	; NO - GO TO ERROR ROUTINE
	MOV AX,00101H	; GET NEXT DATA PATTERN TO WRITE
	STOSW		; WRITE INTO LOCATION JUST READ		; { STOSW: AX-->[ES:DI], then DI=DI+2 }
	LOOP .C5	; DECREMENT WORD COUNT AND LOOP
;
	CLD		; SET DIR FLAG TO GO FORWARD
	INC DI		; SET POINTER TO BEG LOCATION
	INC DI
	MOV SI,DI	; INITIALIZE DESTINATION POINTER
	MOV CX,BX	; SETUP WORD COUNT FOR LOOP
	MOV DX,AX	; SETUP COMPARE PATTERN "00101H"
.C6:			;       INNER TEST LOOP
	LODSW		; READ OLD TEST WORD FROM STORAGE	; { LODSW: [DS:SI]-->AX, then SI=SI+2 }
	XOR AX,DX	; DATA READ AS EXPECTED ?
	JNE .COMPERR	; NO - GO TO ERROR ROUTINE
	STOSW		; WRITE ZERO INTO LOCATION READ		; { STOSW: AX-->[ES:DI], then DI=DI+2 }
	LOOP .C6	; DECREMENT WORD COUNT AND LOOP
	DEC DI		; POINT TO LAST WORD JUST WRITTEN
	DEC DI
	STD		; SET DIR FLAG TO GO BACKWARDS
	MOV SI,DI	; INITIALIZE DESTINATION POINTER
	MOV CX,BX	; SETUP WORD COUNT FOR LOOP
	MOV DX,AX	; SETUP COMPARE PATTERN "00000H"
.C6X:
	LODSW		; VERIFY MEMORY IS ZERO.		; { LODSW: [DS:SI]-->AX, then SI=SI+2 }
	XOR AX,DX	; DATA READ AS EXPECTED ?
	JNE .COMPERR	; NO - GO TO ERROR ROUTINE
	LOOP .C6X	; DECREMENT WORD COUNT AND LOOP
;
	IN AL,PORT_C	; DID A PARITY ERROR OCCUR ?
	AND AL,0C0H	; ZERO FLAG WILL BE OFF, IF PARITY ERROR
	jnz .PARERR	; --> if parity error

	; RAM tested okay.
	; Zero flag is set.
	MOV AL,0	; AL=O DATA COMPARE OK
	CLD		; SET DIRECTION FLAG TO INC
	RET

.PARERR:
	; Data comparison good, but a parity error is indicated.
	;
	mov	word [ss:BadAddrSegment],ds	; Save segment of address in error (LODSW uses DS:SI)
	mov	word [ss:BadAddrOffset],si	; Save  offset of address in error (LODSW uses DS:SI)
	mov	word [ss:BadDataParity],1	; Save: high_byte:{bit error pattern = 00}, low_byte={parity bit error}
	jmp	.FAILEXIT

.COMPERR:
	; A data comparison failed.
	;
	mov	word [ss:BadAddrSegment],ds	; Save segment of address in error (LODSW uses DS:SI)
	mov	word [ss:BadAddrOffset],si	; Save  offset of address in error (LODSW uses DS:SI)
	;
	; AX is the bit error pattern (a word).
	; If both AH and AL are non-zero, that means that both test bytes (written/read as a word) are in error - choose either.
	; Otherwise, the non-zero register is the bit error pattern.
	cmp	ah,0
	jne	.S10				; --> if AH has a one somewhere in its bit error pattern, use AH
	mov	ah,al				; AL has bit error pattern, move into AH
.S10:	mov	al,0				; Indicate no parity error.
	mov	word [ss:BadDataParity],ax	; High_byte:{bit error pattern}, low_byte={no parity bit error}
	;
	; Get bit error pattern into AL for caller.
	; Because AL is what this IBM subroutine returns the bit error pattern in.
	mov	al,ah
	;
.FAILEXIT:
	cld					; Set direction flag to increment (because STD was used earlier).
	;
	; Clear ZF to indicate error to caller, but do not change AL in doing so.
	mov	bx,1
	or	bx,bx

	ret



; ****************************************************************************
; * Data
; ****************************************************************************

; Tables with data to initialise the MDA and CGA screen
Tbl_dataMDA:
	db 061h, 050h, 052h, 00Fh, 019h, 006h, 019h, 019h
	db 002h, 00Dh, 00Bh, 00Ch, 000h, 000h, 000h, 000h
Tbl_dataCGA:
	db 071h, 050h, 05Ah, 00Ah, 01Fh, 006h, 019h, 01Ch
	db 002h, 007h, 006h, 007h, 000h, 000h, 000h, 000h

TblTestData:
	db 0FFh, 0, 055h, 0AAh, 1	; The 1 is required - odd (not even) to cater for most parity chip failures.

Tbl_ASCII:
	db '0123456789ABCDEF'

TxtTitle:
	db  "Ruud's Diagnostic ROM for PC/XT     Version: 4.8  ("
	db __DATE__		; Compiled date (YYYY-MM-DD)
	db ')', 0

TxtUnexpNmi:
	db 2, 2, '*** UNEXPECTED NMI ***', 0

TxtTestCPU:
	db  2,  2, 'Testing CPU           ', 0

TxtChecksumROM:
	db  3,  2, 'Diagnostic ROM checksum', 0

Txt8253Ch0:
	db  4,  2, '8253 timer channel 0', 0

Txt8253Ch1:
	db  5,  2, '8253 timer channel 1', 0

Txt8253Ch2:
	db  6,  2, '8253 timer channel 2', 0

Txt8237DMA:
	db  7,  2, '8237A DMA controller', 0
	
TxtHotTimer1:
	db  8,  2, 'Hot timer channel 1', 0

TxtRamParityErr:
	db  9,  2, 'RAM parity error latches', 0

TxtChk2KbRAM:
	db 10,  2, 'Check first 2 KB of RAM', 0

TxtFoundRAM:
	db 11,  2, 'Found RAM:     KB', 0

TxtEmptyLine12:
	db 12,  0, '                                        ', 0

TxtEmptyLine13:
	db 13,  0, '                                        ', 0

TxtEmptyLine14:
	db 14,  0, '                                        ', 0

TxtEmptyLine15:
	db 15,  0, '                                        ', 0

TxtClearLine:
	db '                                                                                ', 0

TxtTestRamData:
	db 12,  2, 'Testing RAM - Data', 0

TxtTestRamAddr:
	db 13,  2, 'Testing RAM - Address', 0

TxtTestRamRefr:
	db 14,  2, 'Testing RAM - Refresh', 0

TxtTestRamSlowRefr:
	db 15,  2, 'Testing RAM - Slow Refresh', 0

TxtPIC8259:
	db 16,  2, '8259 interrupt controller', 0

TxtCheckHotIrq:
	db 17,  2, 'Hot IRQ interrupts', 0

TxtCheckInt0:
	db 18,  2, 'Checking interrupt IRQ0', 0

TxtCheckNMI:
	db 19,  2, 'Hot NMI', 0

TxtChkKeybReset:
	db 20,  2, 'Keyboard responds to reset', 0

TxtChkKeybStuck:
	db 21,  2, 'Keyboard stuck key', 0

TxtChkFDC:
	db 22,  2, 'Check floppy controller', 0

TxtReadFloppy:
	db 23,  2, 'Trying to read a floppy', 0


TxtRomF4000:
	db  2, 42, 'Check ROM at F4000', 0

TxtRomF6000:
	db  3, 42, 'Check ROM at F6000', 0

TxtRomF8000:
	db  4, 42, 'Check ROM at F8000', 0

TxtRomFA000:
	db  5, 42, 'Check ROM at FA000', 0

TxtRomFC000:
	db  6, 42, 'Check ROM at FC000', 0


TxtSwitchPC1:
	db 9, 45, '    '
	db 201, 205, 'PC SW1', 205, 187,
	db ' '
	db 201, 205, 'PC SW2', 205, 187, 0

TxtSwitchPC2:
	db 10, 45, '    ', 186, '12345678', 186
	db ' ', 186, '12345---', 186, 0

TxtSwitchPC3:
	db 11, 45, ' ON ', 186, '        ', 186
	db ' ', 186, '        ', 186, '', 0

TxtSwitchPC4:
	db 12, 45, '    ', 186, '        ', 186
	db ' ', 186, '        ', 186, '', 0

TxtSwitchPC5:
	db 13, 45, '    '
	db 200, 205, 205, 205, 205, 205, 205, 205, 205, 188,
	db ' '
	db 200, 205, 205, 205, 205, 205, 205, 205, 205, 188, 0

TxtSwitchXT1:
	db 14, 44, '           '
	db 201, 205, 'XT SW1', 205, 187, 0

TxtSwitchXT2:
	db 15, 44, '           ', 186, '12345678', 186, '       ', 0

TxtSwitchXT3:
	db 16, 44, '        ON ', 186, '        ', 186, '       ', 0

TxtSwitchXT4:
	db 17, 44, '           ', 186, '        ', 186, '       ', 0

TxtSwitchXT5:
	db 18, 44, '           '
	db 200, 205, 205, 205, 205, 205, 205, 205, 205, 188, 0

TxtCompletedPasses:
	db 20, 42, 'Completed passes:', 0

TxtStuckKey:
	db 21, 42, ' Stuck key found:  ', 0

TxtFailed:
	db 'FAILED', 0

TxtPassed:
	db 'Passed', 0

TxtNA:
	db 'N/A   ', 0		; Important: three spaces at end.

TxtCritical:
	db ' >> Critical error, diagnostics have been stopped! << ', 0

TxtVrtf: db	'Test of video RAM failed. Aborting.  <-----', 0

TxtFailAtAddress:
	db	'  Failure at address:     KB  (exactly xxxxx [hex])', 0

TxtBadBits:
	db	'  Bad bits:  7 6 5 4 3 2 1 0  P      ', 0

TxtRamChipWarning:
	db	'<----- May or may not be RAM chips.', 0

;------------------------------------------------------------------------------


	setloc	0FFF0h

;------------------------------------------------------------------------------
; Power-On Entry Point
;------------------------------------------------------------------------------
PowerOn:
	jmp	0F000h:ColdStart

 
S_FFF5:
	db __DATE__		; Assembled date (YYYY-MM-DD)

	db 0			; checksum
	
 

%ifdef TEMP
 mov al,%1
 call CheckpointStack
%endif





