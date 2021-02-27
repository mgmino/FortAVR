; STK500 constants list, from AVRDUDE
#define STK_OK              0x10
#define STK_FAILED          0x11  ; Not used
#define STK_UNKNOWN         0x12  ; Not used
#define STK_NODEVICE        0x13  ; Not used
#define STK_INSYNC          0x14  ; ' '
#define STK_NOSYNC          0x15  ; Not used
#define ADC_CHANNEL_ERROR   0x16  ; Not used
#define ADC_MEASURE_OK      0x17  ; Not used
#define PWM_CHANNEL_ERROR   0x18  ; Not used
#define PWM_ADJUST_OK       0x19  ; Not used
#define CRC_EOP             0x20  ; 'SPACE'
#define STK_GET_SYNC        0x30  ; '0'
#define STK_GET_SIGN_ON     0x31  ; '1'
#define STK_SET_PARAMETER   0x40  ; '@'
#define STK_GET_PARAMETER   0x41  ; 'A'
#define STK_SET_DEVICE      0x42  ; 'B'
#define STK_SET_DEVICE_EXT  0x45  ; 'E'
#define STK_ENTER_PROGMODE  0x50  ; 'P'
#define STK_LEAVE_PROGMODE  0x51  ; 'Q'
#define STK_CHIP_ERASE      0x52  ; 'R'
#define STK_CHECK_AUTOINC   0x53  ; 'S'
#define STK_LOAD_ADDRESS    0x55  ; 'U'
#define STK_UNIVERSAL       0x56  ; 'V'
#define STK_PROG_FLASH      0x60  ; '`'
#define STK_PROG_DATA       0x61  ; 'a'
#define STK_PROG_FUSE       0x62  ; 'b'
#define STK_PROG_LOCK       0x63  ; 'c'
#define STK_PROG_PAGE       0x64  ; 'd'
#define STK_PROG_FUSE_EXT   0x65  ; 'e'
#define STK_READ_FLASH      0x70  ; 'p'
#define STK_READ_DATA       0x71  ; 'q'
#define STK_READ_FUSE       0x72  ; 'r'
#define STK_READ_LOCK       0x73  ; 's'
#define STK_READ_PAGE       0x74  ; 't'
#define STK_READ_SIGN       0x75  ; 'u'
#define STK_READ_OSCCAL     0x76  ; 'v'
#define STK_READ_FUSE_EXT   0x77  ; 'w'
#define STK_READ_OSCCAL_EXT 0x78  ; 'x'
; ******[ STK parameter constants ]******
#define PARAM_BUILD_NUMBER_LOW	0x80
#define PARAM_BUILD_NUMBER_HIGH	0x81
#define PARAM_HW_VER		0x90
#define PARAM_SW_MAJOR		0x91
#define PARAM_SW_MINOR		0x92
#define PARAM_VTARGET		0x94
#define PARAM_VADJUST		0x95
#define PARAM_OSC_PSCALE	0x96
#define PARAM_OSC_CMATCH	0x97
#define PARAM_SCK_DURATION	0x98
#define PARAM_TOPCARD_DETECT	0x9A
#define PARAM_STATUS			0x9C
#define PARAM_DATA				0x9D
#define PARAM_RESET_POLARITY	0x9E
#define PARAM_CONTROLLER_INIT	0x9F

;www.tuxgraphics.org/electronics/200705/article07052.shtml
;	.set	BOOT_ONLY=1

.ifdef BOOT_ONLY
	.nolist
	.include "../inc/m328Pdef.inc"
	.list
	.def zerol = r2
	;--- high registers (16 - 31) ---
	.def tmp0 = r16	;cemit chr $10
	.def tmp1 = r17
	.def tmp2 = r18
	.def tmp3 = r19
	.def spmcmd = r20
	.def TOS  = r24
.endif

.def addrl = r4
.def addrh = r5
.def bycnt = r6
.equ bootBuf = $800

; entry point of 512 byte boot loader
; initiated by a hardware reset with D13 (LED) pulled high (470 ohm) %%% experimenting
; low: 1.5v (.3Vcc); high: 3.0v (.6Vcc)

; <cmd> <eop>         <response>
;            <insync>

; <cmd> <??>
;            <nosync>

	.org	$3f00
bootldr:
	in_		r10,MCUSR	;MCU Status Register (save reset source)
	clr		zerol
;	SP=RAMEND done by hardware reset
	out		MCUSR,zerol	;MCUSR = 0
	sbrs	r10,EXTRF	;hardware reset?
	rjmp	exboot		;n: run xcode at appl vector
;	....				;y:
	sbis	PINB,PINB5	;y: PB5 (D13: LED) high?
	rjmp	exboot		;n: run xcode at appl vector
;	....				;y: run STK500 emulator
;	rcall	wds8sec	;set watchdog to trigger after 8 seconds
	rcall	cinit_boot

; set LED as output: PB5, SCK, D13; DDRB: $04
	sbi		DDRB,5	;LED D13 portB-b5

; command: <STK_CMD> <parm> <CRC_EOP>
; response: <STK_INSYNC> <value> <STK_OK>
; if <CRC_EOP> not received: <STK_NOSYNC>
boot_loop:
	rcall	getch
	rcall	commands
	ldi		tmp0,STK_OK
	rcall	putch	;send ok response
	rjmp	boot_loop

readFuse:		;STK_PROG_FUSE_EXT (e)
	rcall	verifyEOP
	ldi		zl,0	;LFUSE
	rcall	fusout
	ldi		zl,3	;HFUSE
	rcall	fusout
	ldi		zl,2	;EFUSE
fusout:
	clr		zh
	ldi		spmcmd, (1<<BLBSET) | (1<<SELFPRGEN)
	out		SPMCSR,spmcmd	;Store Program Memory Control and Status Register
	lpm
	mov		tmp0,r0
	rjmp	putch

readSign:		;STK_READ_SIGN (u) ATmega328p
	rcall	verifyEOP
	ldi		tmp0,$1e	; 30 SIGNATURE_0
	rcall	putch
	ldi		tmp0,$95	;149 SIGNATURE_1
	rcall	putch
	ldi		tmp0,$0f	; 15 SIGNATURE_2
	rjmp	putch

universal:		;STK_UNIVERSAL (V), noop this function
	ldi		tmp1,4	;ignore four parms
	rcall	ignoreParms
	ldi		tmp0,0	;byte4_out
	rjmp	putch

loadAddr:		;STK_LOAD_ADDRESS (U)
	rcall	getch	;low addr byte
	mov		addrl,tmp0
	rcall	getch	;high addr byte
	cpi		tmp0,high(bootldr)*2	;attempt to address bootloader?
	brsh	exboot	;y: exit
	mov		addrh,tmp0
	rjmp	verifyEOP
	
readPage:		;STK_READ_PAGE (t)
	rcall	getLen
	cpi		tmp0,'F'	;read flash?
	brne	exboot		;n: exit
	rcall	verifyEOP
	movw	Z,ADDRL
rdPage1:	;read data into buffer
	lpm		tmp0,Z+
	rcall	putch	;send data
	dec		bycnt
	brne	rdPage1
	ret

commands:
	cpi		tmp0,'X'
	breq	exboot	;y: exit bootloader
	cpi		tmp0,STK_SET_DEVICE	;(B), noop
	ldi		tmp1,20	;ignore 20 parms
	breq	ignoreParms
	cpi		tmp0,STK_SET_DEVICE_EXT	;(E), noop
	ldi		tmp1,5	;ignore 5 parms
	breq	ignoreParms
	cpi		tmp0,STK_UNIVERSAL	;(V), noop
	breq	universal
	cpi		tmp0,STK_GET_PARAMETER	;(A)
	breq	getInfo
	cpi		tmp0,STK_LOAD_ADDRESS	;(U)
	breq	loadAddr
	cpi		tmp0,STK_READ_PAGE	;(t) flash
	breq	readPage
	cpi		tmp0,STK_READ_SIGN	;(u)
	breq	readSign
	cpi		tmp0,STK_READ_FUSE_EXT	;(e)
	breq	readFuse
	cpi		tmp0,STK_PROG_PAGE	;(d) flash
	breq	progPage
	cpi		tmp0,':'	;Intel HEX data record
	breq	progPage
	cpi		tmp0,STK_LEAVE_PROGMODE	;(Q)
	brne	verifyEOP	;noop remaining commands
exboot:
	jmp		0

wds8sec:						;0 (16ms), 2 (64ms), 4 (.25sec), 5 (.5sec)
	ldi		tmp0,(1<<WDE) | $21	;6 (1sec), 7 (2sec); $20 (4sec), $21 (8sec)
wdset:	 ;**ENTRY 4**
	wdr			;reset watchdog
	ldi		tmp1,(1<<WDCE) | (1<<WDE)
	sts		WDTCSR,tmp1	;enable change prescaler
	sts		WDTCSR,tmp0	;($60) prescaler value
	ret

ignoreParms:
	rcall	getch
	dec		tmp1
	brne	ignoreParms
verifyEOP:
	rcall	getch	;chr after cmd
	cpi		tmp0,CRC_EOP
	brne	nosync
putack:
	ldi		tmp0,STK_INSYNC
putch:	;output byte (tmp0)
	lds		xl,UCSR0A	;USART ctrl/stat regA
	sbrs	xl,UDRE0	;USART Data Register empty?
	rjmp	putch		;n: wait
	sts		UDR0,tmp0	;y: read char from USART data reg
	ret

nosync:
	ldi		tmp0,STK_NOSYNC
	rcall	putch
	pop		tmp0
	pop		tmp0	;remove return address
;	**** flush incoming characters
	rjmp	boot_loop


; *** commands ***

getInfo:		;STK_GET_PARAMETER (A)
	rcall	getch
	mov		tmp1,tmp0
	rcall	verifyEOP
	ldi		tmp0,0
	cpi		tmp1,$82	;minor SW version?
	breq	putch		;y: OPTIBOOT_MINVER
	ldi		tmp0,4
	cpi		tmp1,$81	;major SW version?
	breq	putch		;y: OPTIBOOT_MAJVER
	ldi		tmp0,3		;generic reply
	rjmp	putch

getLen:
	rcall	getch	;high len byte
	cpi		tmp0,0	;len > 255?
	brne	exboot	;y: exit bootloader
	rcall	getch	;low len byte
	mov		bycnt,tmp0	;byte count
;	rjmp	getch	;get mem type
getch:
	ldi		tmp0,30	;[1]
getch1:
	dec		tmp0	;-1-
	brne	getch1	;-2- 3*30= 90 cycles
	adiw	TOS,1	;[2]cycles
	brne	getch2	;[2] cycles
	sbi		PINB,5	;toggle	LED D13 portB-b5
;	each processor cycle with 16MHz clock: 1/16MHz = 62.5ns
;	62.5ns * 65536: 4.1 ms; 9 cycles: 36.9 ms
;	10+90= 100 cycles: .410 sec
getch2:
	lds		tmp0,UCSR0A	;[2] read USART Control and Status Register A
	sbrs	tmp0,RXC0	;[1] USART Receive Complete?
	rjmp	getch		;[2]
;	sbrs	tmp0,FE0	;framing error?
	wdr					;n: reset watchdog
	lds		tmp0,UDR0	;read chr from USART I/O Data Register
	ret

progPage:		;STK_PROG_PAGE (d)
	rcall	getLen	;two bytes, first byte s/b zero; bycnt= page size
	cpi		tmp0,'F'	;read flash?
	brne	exboot		;n: exit
	mov		tmp1,bycnt
	ldi		xl,low(bootBuf)
	ldi		xh,high(bootBuf)
	movw	Z,X

pgPage1:	;;;read data into bootBuf (f>b)
	rcall	getch	;read data
	st		Z+,tmp0
	dec		tmp1	;decrease byte count
	brne	pgPage1
	rcall	verifyEOP

;;; erase page - move before read data to speed operation (*f*)
pgErase:	;**ENTRY 1**
	movw	Z,ADDRL	;page load address
	ldi		spmcmd, (1<<PGERS) | (1<<SELFPRGEN)
	rcall	do_spm	;erase page
	ldi		spmcmd,(1<<RWWSRE) | (1<<SELFPRGEN)
	rcall	do_spm	;re-enable RWW section

;;; transfer cells from bootBuf to flash writeBuffer (>fb)
pgPage2:	;**ENTRY 2**
	lsr		bycnt	;count in cells
	movw	Z,ADDRL	;page load address
pgPage3:
	ld		r0,X+
	ld		r1,X+
	ldi		spmcmd,(1<<SELFPRGEN)
	rcall	do_spm	;store cell in flash buffer
	adiw	Z,2		;inc flash ptr one cell
	dec		bycnt	;next cell
	brne	pgPage3

;;; Program flash page from writeBuffer (fb>)
	movw	Z,ADDRL	;page load address
	ldi		spmcmd,(1<<PGWRT) | (1<<SELFPRGEN)
	rcall	do_spm	;execute Page Write
	ldi		spmcmd,(1<<RWWSRE) | (1<<SELFPRGEN)
;	rjmp	do_spm	;re-enable RWW section

do_spm:		;spmcmd determines SPM action	 **ENTRY 3**
	in		tmp1,SPMCSR
	sbrc	tmp1,SELFPRGEN	;previous SPM complete?
	rjmp	do_spm		;n: wait
	in		tmp1,SREG	;save Status Register
	cli					;disable interrupts
wait_eep:
	sbic	EECR,EEPE	;EEPROM write in progress?
	rjmp	wait_eep	;y: wait
	out		SPMCSR,spmcmd	;y: SPM timed sequence
	spm
	out		SREG,tmp1	;restore status register (and interrupts)
bootret:
	ret

cinit_boot:
	ldi		tmp0,25		;baud: 9600 (103), 19200 (51), 38.4k (25), 76.8k (12)
	sts		UBRR0L,tmp0	;USART ($c4) baud rate
	sts		UBRR0H,zerol;USART ($c5) baud rate
;	ldi		xl,U2X0		;double speed (U2X0)
;	out		UCSR0A,xl	;USART ($c0) ctrl/stat regA
	ldi		tmp0,(1<<TXEN0)|(1<<RXEN0)	;enable TX and RX
	sts		UCSR0B,tmp0	;USART ($c1) ctrl/stat regB
	ldi		tmp0,$06	;8 data bits ($6), no parity, 1 stop bit
	sts		UCSR0C,tmp0	;USART ($c2) ctrl/stat regC
	ret

;	$3f00 - $3fbd (189 words; 378 bytes) 67 words; 134 bytes remain
;	process Intel Hex formatted records
;	:llaaaatt[dd...]cc
;	ll - length (number of dd's)
;	aaaa - address of first dd
;	tt - record type (00: data, 01: end of file)
;	dd - data byte
;	cc - checksum of all hex digits mod 256, two's complement
;	:02 0000 01 0000FD
;	:00 0000 01 FF
;	:06 0000 00 CCAABBFFEEDDFF

hexrec:
	clr		tmp1	;init checksum
	rcall	hxbyt2	;length (ll)
	mov		tmp2,tmp0
	rcall	hxbyt2	;high address (aa)
	cp		tmp0,addrh	;load address match?
	brne	hxerr	;n:
	rcall	hxbyt2	;low address (aa)
	mov		xl,tmp0	;save address offset
	eor		tmp0,addrl	;load address b7 match?
	brmi	hxerr	;n:
	andi	xl,$7f	;remove b7
	rcall	hxbyt2	;record type (tt)
	tst		tmp0	;data record type?
	brne	hxerr	;n
	ldi		xh,high(bootBuf)
hxrec4:
	rcall	hxbyt2	;data byte (dd)
	st		X+,tmp0
	dec		tmp2	;len counter
	brne	hxrec4
	rcall	hxbyt2	;checksum (cc)
	tst		tmp1	;checksum zero?
	brne	hxerr	;n:
	rjmp	putack	;y: send ack

hxbyt2:
	rcall	hxnib2
	swap	tmp0	;xchg nibbles
	mov		tmp3,tmp0
	rcall	hxnib2
	or		tmp0,tmp3
	add		tmp1,tmp0	;checksum
hxrtn2:
	ret

hxnib2:
	rcall	getch
	subi	tmp0,'0'
	cpi		tmp0,10	;digit < 10?
	brlo	hxrtn2	;y:
	subi	tmp0,'A' -'9' -1
	cpi		tmp0,10	;adjusted digit < 10?
	brlo	hxerr	;y:
	cpi		tmp0,16	;adjusted digit < 16?
	brlo	hxrtn2	;y:
hxerr:
	rjmp	nosync

;	$3f00 - $3fe7 (231 words; 462 bytes) 25 words; 50 bytes remain

