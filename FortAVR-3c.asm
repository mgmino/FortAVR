; TITLE FortAVR v3c 2020.06.17
;-UdotR_ - overflow if field too small
;-numberq - optimize
;-increase baud rate
;-smart words - exe/run
;-flush out boot section - add intel hex pgm
; https://sites.google.com/site/avrasmintro/home

; avrasm2 -fI -o "$(OutputFileName).hex"  -m "$(OutputFileName).map"  -l "$(OutputFileName).lss"  -S "$(OutputFileName).tmp"  -W+ie -i($IncludeFile) -d "$(OutputDir)/$(OutputFileName).obj"  "$(EntryFile)" 

.nolist
	.include "inc/m328Pdef.inc"
	.include "inc/avr-macros-v31.inc"
	.equ	loader	=	1
.list

;; stat attributes
.equ	cmpsbt	=	7	;compile mode bit (vs interpret)
.equ	tsksbt	=	6	;multitask mode
.equ	urxsbt	=	5	;sftUART: receive data ready (RDR)
.equ	utxsbt	=	4	;sftUART: transmitting

.equ	zzzzz2	=	3	;reserved
.equ	mncsbt	=	2	;multi unitID, no crc mode
.equ	qitsbt	=	1	;non interactive quiet mode
.equ	flgsbt	=	0	;general purpose flag (used by ?num)

.equ	tibmax	=	80	;input line - maximum bytes

;; fuses - http://www.engbedded.com/fusecalc/
; default: E:05, H:da, L:ff; FortAVR v3:  LK:ef E:fd H:d6 L:ff
; 1: unprogrammed; 0: programmed

;--lock:  $3f; no SPM/LPM restrictions; $2f SPM restrict boot
;	--11 ---- BLB1 (Lock Bit Protection Mode1) No restrictions SPM/LPM access Boot Loader section
;	---- 11-- BLB0 (Lock Bit Protection Mode0) No restrictions SPM/LPM access Application section
;	---- --11 LB (Lock bits) no memory lock features enabled

;--efuse: BODLEVEL2:1, BODLEVEL1:0, BODLEVEL0:1 -- $fd ($05)
;	Brown Out Reset: 5: 2.7v +/-0.2v; 6: 1.8v; 4: 4.3v; 7: disabled
;	top five bits are reserved, so $05 = $fd for 2.7v trigger

;--hfuse: $d6
;	1--- ---- RSTDISBL (External Reset Disable) PC6 reset enabled
;	-1-- ---- DWEN (debugWIRE Enable) disabled
;	--0- ---- SPIEN (SPI Enable) enabled
;	---1 ---- WDTON (WatchDog Timer Always On) disabled
;	---- 0--- EESAVE (EEPROM preserved) yes
;	---- -11- BOOTSZ Boot (size) Start: 00: $3800, 01: $3c00, 10: $3e00, 11:: $3f00
;	---- ---0 BOOTRST Reset Vector -> 0:: boot loader, 1:: $0000

;--lfuse: $ff
;	low power crystal oscillator 16MHz; 14CK + 65ms Startup
;	1--- ---- CKDIV8 (Divide clock by 8) disabled
;	-1-- ---- CKOUT (Clock output) disabled
;	--11 ---- SUT (Select start-up time) 65ms delay
;	---- 1111 CKSEL (select Clock source) Low Power Crystal Oscillator

;--Device signature = 0x1e950f
; sudo avrdude -p m328p -c dapa -P /dev/parport0 -U hfuse:w:0xd8:m

; sudo avrdude -p m328p -c dapa -P /dev/parport0 -U flash:w:mForth328.hex:i
; sudo avrdude -p m328p -c usbasp -t
; sudo avrdude -p m328p -c usbasp -U flash:w:FortAVR-3a.hex:i
; sudo avrdude -p m328p -c usbasp -U lfuse:w:0xff:m -U hfuse:w:0xD6:m -U efuse:w:0xfd:m

;vocab -> last defined word nfa

;; word attributes (31 byte max word len)
.equ	nowbit	=	7	;immediate (now) word bit
.equ	cmpbit	=	6	;compile only word bit
;.equ	clnbit	=	5	;colon word bit (vs code word)

;lfa -> previously defined word nfa; zero: end of vocab
;nfa (name field addr) -> [aaa# ####] WORD - a: word attributes, #: word length
;cfa (code field [byte] addr) -> [icode or xcode]
;kfa (kode field [cell] addr) -> [icode or xcode]

;--- low registers (0 - 15) ---
;	r1:r0 used by SPM in flash write
.def spml = r0	;store Program Memory (SPM)
.def spmh = r1
.def ZERO  = r2
.def zerol = r2
.def zeroh = r3
; r4
; r5 count usart receive errors
.def ubtcnt = r6	;UART bit counter
.def ubuf	= r7	;UART char buffer
.def urxchr	= r8	;UART received character
; r10 boot reset source
; r11 rst_ source
; r12 count in ?num
.def crc8 = r13
.def TIC = r14
.def ticl = r14	;systic - system 1ms timer (TMR2)
.def tich = r15
;--- high registers (16 - 31) ---
.def tmp0 = r16	;cemit chr $10
.def tmp1 = r17
.def tmp2 = r18
.def tmp3 = r19
.def spmcmd = r20	;SPMCSR: Store Program Memory Control and Status Register
.def stat = r21
.def nxt  = r22
.def nxth = r23
.def TOS  = r24
.def tosl = r24	;$18
.def tosh = r25
; r26 (low) r27: X
; r28 (low) r29: Y - data stack pointer
; r30 (low) r31: Z ;$1f

;; software uart (TMR0, 8bit)

;; open (TMR1, 16bit)

;; systic - system timer (TMR2, 8bit)
;	prescaler   *250 ($b1) $1fff
;	1024:128us   16ms (7) 131.1sec
;	 256: 32us    4ms (6) 32.8sec
;	 128: 16us    2ms (5) 16.4sec
;***  64:  4us    1ms (4) 8.2sec
;	  32:  2us  500us (3) 4.1sec
;	   8: .5us  125us (2) 1.02sec
;	   1: 63ns 15.6us (1) 128ms

; 24*60*60=86,400; 24*60*40= 57,600 ($e100) = 225 ($e1) * 256; 1.5 second unit

;; Memory allocation for ATmega328P, byte (word) addresses
;	Flash memory (bytes)
; $0		reset and interrupt vectors, RWW section
; $100 ($80) - initial values for variables

.equ	ICOD	=	$0d00		;Interpreted CODe
; $d00 ($680) - start of flash (icode) dictionary
; --- icode, RWW section
; ftop	next available flash (icode) dictionary
; --- icode, RWW section

.equ	XCOD	=	$4000	;eXecuted CODe
; $4000 ($2000) - icode/xcode boundary
; --- xtra xcode, RWW section
; $5e00 ($2f00) - 3.2c: boundary moved $3000 -> $2f00
; --- core xcode (forth vocab), RWW section

; $7e00 ($3f00) - boot loader entry
; --- boot loader xcode, NRWW section
; $7fff ($3fff)

.equ	FILBYTE	=	$00		;fill byte for word boundary
.equ	PAGESIZEB = PAGESIZE*2 ;page size: 64 cells, 128 bytes
.equ	radmsk = $03	;high byte mask for relative address


;	memory mapping
; $0000 - $00ff : registers (.25k bytes)
; $0100 - $08ff : SRAM (2k bytes)
; $0900 - $0cff : EEPROM (1k bytes)
; $0d00 - $7fff : Flash (top 28.75k bytes)
; $8000 - $ffff : memory mirrored (32k bytes)

; ============ INTERRUPT VECTORS ================
	.cseg
	.org	0
	jmp rst_	; Reset
;IRQ0; INT0: EIMSK-b0 (PD2)
	push	tmp0
	rjmp	ext_irq0	;software uart2
;IRQ1; INT1: EIMSK-b1 (PD3)
	push	tmp0
	rjmp	en_task1	;enable task1, set active bit

;PCINT0 Pin Change Interrupt; PCMSK0 (PBx)
	push	tmp0
	rcall	qirq
;	rjmp	en_task3	;enable task3, set active bit
;PCINT1 Pin Change Interrupt; PCMSK1 (PCx)
	push	tmp0
	rcall	qirq
;	rjmp	en_task4	;enable task4, set active bit
;PCINT2 Pin Change Interrupt; PCMSK2 (PDx)
	push	tmp0
	rjmp	en_task2	;enable task2, set active bit

;WDT Watchdog Timer; WDIE: WDTCS-b6
	jmp		rst_

;Timer2 Compare Match A; OCIE2A: TIMSK2-b1
	push	tmp0
	rjmp	isystic
;Timer2 Compare Match B; OCIE2B: TIMSK2-b2 ($10)
	push	tmp0
	rcall	qirq
;Timer2 Overflow; OCIE2A: TIMSK2-b0
	push	tmp0
	rcall	qirq

;Timer1 Capture Event; ICIE1: TIMSK1-b5
	push	tmp0
	rcall	qirq
;Timer1 Compare Match A; OCIE1A: TIMSK1-b1
	push	tmp0
	rcall	qirq
;Timer1 Compare Match B; OCIE1B: TIMSK1-b2 ($18)
	push	tmp0
	rcall	qirq
;Timer1 Overflow; TOIE1: TIMSK1-b0
	push	tmp0
	rcall	qirq

;Timer0 Compare A; OCIE0A: TIMSK0-b1
	push	tmp0
	rcall	qirq
;Timer0 Compare B; OCIE0B: TIMSK0-b2
	push	tmp0
	rcall	qirq
;Timer0 Overflow; TOIE0: TIMSK0-b0 ($20)
	push	tmp0
	rjmp	tim0_ovf	;software uart2

;SPI Transfer Complete
	push	tmp0
	rcall	qirq

;USART, RX Complete; RXCI0: UCSR0B-b7
	push	tmp0
	rcall	qirq
;USART, UDRE Data Register Empty; UDRE0: UCSR0B-b5
	push	tmp0
	rcall	qirq
;USART, TX Complete TXCIE0: UCSR0B-b6 ($28)
	cbi		PORTC,0	;disable RS485 driver (PC0)
	reti

;ADC Conversion Complete; ADIE: ADCSRA-b3
	push	tmp0
	rcall	qirq
;EEPROM Ready; EERIE: EECR-b3
	push	tmp0
	rcall	qirq
;Analog Comparator; ACIE: ACSR-b3
	push	tmp0
	rcall	qirq
;TWI 2-wire Serial Interface; TWIE: TWCR-b0 ($30)
	push	tmp0
	rcall	qirq
;SPM_RDY Store Program Memory Ready; SPMIE: SPMCSR-b7
	push	tmp0
	rcall	qirq

.if (pc != $34)
	.error " *** Interrupt vector table corrupt"
.endif
; ============ INTERRUPT HANDLERS ================

qirq:		;interrupt catch all
;;	SREG not saved since instructions dont affect SREG
;	push	tmp0
;	rcall	qirq
	pop		tmp0	;discard high byte of calling return address
	pop		tmp0	;get interrupt vector address (low byte)
	sts		irqid,tmp0	;save interrupt ID
	pop		tmp0	;restore tmp0
	reti		;return from int

isystic:			;[4] TMR2 calls every ms
;	push    tmp0	;[2] save xl
;	rjmp	isystic	;[2]
	in      tmp0,SREG	;[1] save CPU flags
	inc		ticl	;[1]
	brne	int_ret1;[1/2+7] 19 clock ticks
	inc		tich	;[1]
	rjmp	int_ret1;restore CPU flags and tmp0

en_task1:		;enable task1, set active bit
;	push
;	rjmp	en_task1
	in      tmp0,SREG	;save CPU flags
	push	tmp0
	lds		tmp0,tsk1kfa+1	;task1 cfa high byte	
	ori		tmp0,$4	;set task active bit6
	sts		tsk1kfa+1,tmp0
int_ret:	;restore CPU flags and tmp0
	pop		tmp0	;[2]
int_ret1:
	out		SREG,tmp0;[1] restore CPU flags
	pop		tmp0	;[2] restore tmp0
	reti			;[4]

en_task2:		;enable task2, set active bit
;	push
;	rjmp	en_task2
	in      tmp0,SREG	;save CPU flags
	push	tmp0
	lds		tmp0,tsk2kfa+1	;task1 cfa high byte	
	ori		tmp0,$4	;set task active bit6
	sts		tsk2kfa+1,tmp0
	rjmp	int_ret	;restore CPU flags and tmp0

;;; additional interrupt code after RAM initialization

;	RAM memory (bytes)
;	$0		CPU and I/O registers
;	$100	UP0 - Variables ('BOOT)

	.org	$80	;byte address $100, copy to ram on boot
; ---	initialized variables ---
label	varinit
label	base	;radix base for numeric I/O
.equ uid = base+1	;unit ID
	.db		10, 'A'	;BASE, uid
label	context	;search vocabulary
	.dw		forth	;CONTEXT vocab (search)
label	current	;new word vocabulary
	.dw		vam		;CURRENT vocab (search)
label	eepdp	;ptr next available memory in EEPROM dict
	.dw		etop
label	flhdp	;ptr next available memory in FLASH dict
	.dw		ftop
label	ramdp	;ptr next available memory in RAM dict
	.dw		dtop
label	vptr	;ptr next available memory in RAM variable space
	.dw		vp0
label	forth	;ptr last nfa in FORTH vocab
	.dw		topforth
label	vee		;ptr last nfa in VEE vocab
	.dw		topvee
label	vam		;ptr last nfa in VAM vocab
	.dw		topvam
label	vas		;ptr last nfa in VAS vocab
	.dw		topvas
label	dp		;ptr current dict space ptr
	.dw		ramdp
label	erradr	;error handler
	.dw		errmsg*2	;default error message display

label	sysclk
	.dw		0	;system clock, 1.5sec count since midnight
	.dw		0	;systsk (1.5sec) next systic target
	.dw		0	;next scheduled chore cfa (b15 set if active)
	.dw		0	;next scheduled chore clock target

label	reserve
.equ taskID = reserve+1
	.db		0,$80	;taskID set inactive
label	tsklst
	.dw		0	;task4 kfa
	.dw		0	;task3 kfa	;;high byte = 0: empty task
label	tsk2kfa
	.dw		0	;task2 kfa	;;high byte b7 (b15): task waiting
label	tsk1kfa
	.dw		0	;task1 kfa	;;high byte b6 (b14): task ready
label	tskwait	;task waiting
	.dw		0	;task4; systic target
	.dw		0	;task3
	.dw		0	;task2
	.dw		0	;task1
.if (((pc | $7) & $ff) == $ff)	;task list cant cross high/low byte boundary
	.error " Task list address error"
.endif

; ---	uninitialized variables?? ---
label	tmp		;byte used in: parse_ [nxtib_], fnd_ [swd_]; cell: vector in DS?
	.dw		0		;tmp
label	toin	;char ptr while parsing term input
.equ ntib = toin+1	;terminal input buffer (TIB) length
	.db		0, 0	;>IN, #TIB
label	ttib	;address of terminal input buffer {{load from flash only}}
	.dw		tib		;'TIB
label	hld		;pointer to build numeric output string in PAD
	.dw		0		;HLD

label	dtop	;next available memory in ram dictionary
.equ	varsize = dtop - varinit	;size of initialized variables

;					$162	Initial PAD
.equ	vp0		=	$780	;top of variable space
.equ	sp0		=	$7f8	;start of data_stack; grows down (60 cells)
.equ	anonwd	=	$7fa	;anonymous word cfa
.equ	cmpcnt	=	$7fc	;data stack depth during compile
.equ	xvar	=	$7fd	;X variable word
.equ	irqid	=	$7ff	;interrupt ID
.equ	buf		=	$800	;start of page buffer (128 bytes)
.equ	tib		=	$880	;terminal input buffer (TIB, 80 bytes)
.equ	rp0		=	$8ff	;start of return_stack; grows down (24 cells)
;					$8ff	End of RAM memory

.equ	EEPRM	=	$900	;first byte of EEPROM (1k)
.equ	tboot	=	$900	;ptr to addr of powerup instructions
.equ	etop	=	EEPRM+40	;first byte in EEPROM dictionary



; ============ interrupt code ================
	.org	$100	;RWW (read while write)

;	38400 baud (26usec):
.equ	bit15	=	78	;39 usec, 1.5bit
.equ	bit10	=	52	;26 usec, 1.0bit

;.equ	PORTC	= 0x08
;.equ	PIND	= 0x09
;.equ	DDRD	= 0x0a
;.equ	PORTD	= 0x0b
;.equ	EIMSK	= 0x1d
;.equ	TCCR0A	= 0x24
;.equ	TCNT0	= 0x26
;.equ	OCR0A	= 0x27
;.equ	MCUCR	= 0x35
;.equ	TIMSK0	= 0x6e	;memory mapped
;.equ	EICRA	= 0x69	;memory mapped


;	software uart2; TX: PD4 (4), RX: PD2 (2), RS485 enable: PC0 (14)
;*	External Interrupt Routine 0
;		receive uppercase U ($55); 8N1 or 7E1
;--+ S +-1-+ 0 +-1-+ 0 +-1-+ 0 +-1-+ P +-S-+
;  |-1 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8
;  +---+ . +-.-+ . +-.-+ . +-.-+ . +-.-+ .
;		receive uppercase U ($55); 7N1
;--+ S +-1-+ 0 +-1-+ 0 +-1-+ 0 +-1-+-S-+
;  | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7   8
;  +---+ . +-.-+ . +-.-+ . +-.-+ .   .

ext_irq0:	;falling edge of next received character start bit
;	push	tmp0		;save tmp0 register
;	rjmp	ext_irq0
	in		tmp0,SREG
	push	tmp0		;save MCU status register
	out		TCNT0,zerol	;clear Timer/Counter 0
	mov		ubtcnt,zerol;erase bit counter (7bit)
;	dec		ubtcnt		;set bit counter to $ff (8bit)
	cbi		EIMSK,INT1	;disable rx INT1 (PD2)
	ldi		tmp0,(1<<OCIE0A);$2
	sts		TIMSK0,tmp0	;enable Oc0A interrupt
	clr		ubuf		;clear rx buffer

	ldi		tmp0,bit15
	out		OCR0A,tmp0	;wait time to sample lsb (skip start bit)
	rjmp	int_ret


;*	Timer/Counter 0 Overflow Interrupt
tim0_ovf:	;RX: sample next data bit of received character
			;TX: set state of next transmitted character data bit
;	push	tmp0		;save tmp0 register
;	rjmp	tim0_ovf
	in		tmp0,SREG
	push	tmp0		;save MCU status register

	out		TCNT0,zerol	;clear Timer/Counter 0
	ldi		tmp0,bit10
	out		OCR0A,tmp0

	inc		ubtcnt		;increment bit_counter
	sbrs	stat,utxsbt	;transmit mode?
	rjmp	rx_bit		;n: receive mode
;	---					;y: transmit char
	sbrc	ubtcnt,3	;;stop bit (bit counter >7)?
	rjmp	tx_stop		;y: send stop bit
;	---
	sbrc	ubuf,0		;n: buffer LSB 1?
	sbi		PORTD,PD4	;y: transmit mark (1) bit (idle)
;	---
	sbrs	ubuf,0		;n: buffer LSB 0?
	cbi		PORTD,PD4	;y: transmit space (0) bit (break)
;	---
	lsr		ubuf		;shift buffer right
	rjmp	int_ret		;restore CPU flags and tmp0

tx_stop:
	sbi		PORTD,PD4	;transmit stop-bit: mark (1) bit (idle)
	sbrs	ubtcnt,0	;stop-bit sent (bit counter =9)?
	rjmp	int_ret		;n: exit
;	---					;y: transmit complete
	cbr		stat,(1<<utxsbt)	;clear transmit-flag
	cbi		PORTC,PC1	;disable RS485 driver (C1)
	rjmp	set_rx		;enable rx

rx_stop:
;	verify stop bit??
	sbr		stat,(1<<urxsbt)	;set read data ready (RDR)
	mov		urxchr,ubuf	;save received character
set_rx:
	sbi		EIMSK,INT1	;enable rx INT1 (PD2)
	sts		TIMSK0,zerol;disable Oc0A interrupt
	rjmp	int_ret

rx_bit:
	sbrc	ubtcnt,3;stop bit (bit counter =8)?
	breq	rx_stop	;y:
;	---
	sec				;n: set carry
	sbis	PIND,2	;sample PD2 low?  *** sample rx input ***
	clc				;y: clear carry
	ror		ubuf	;shift into rx buffer
	rjmp	int_ret	;restore CPU flags and tmp0


; ============ ICODE SECTION ================
	.org	ICOD/2	;RWW (read while write)

;	*** tail of VAM vocabulary ***
.SET _link		=	0	;init tail null link; VAM vocabulary
;=>	WHO	(--)	display name of vocabulary
	colon	3,"who"
who_:
	blit	vamtxt*2
typnfa:
	exec	count_
	slit	$1f
	exec	and_
	exec	type_
	cend

;->	C01	(n --)	count down
;::	begin dup . 1- dur ;
	colon	3,"c01"
cntdown:			;BEGIN
	exec	dup_
	exec	dot_
	exec	less1_	;1-
	qkgot	cntdown	;DUR
	exec	drop_
	rjump	who_
;	cend

;->	CS1	(n --)	case example
;::	CASE[ 'a ]EQ '= [ 'C ]LS '<  ELS '# ;CASE
	colon	3,"cs1"
	case	c9		;CASE[
c1:	caseq	"a",c2	;]EQ
	slit	'='
c2:	casls	"C",c3	;]LS
	slit	'<'
c3:	cased			;ELS
	slit	'#'
c9:	cend			;;CASE
	exec	emit_
	cend

;->	C03	(n --)
;::	bk: begin dup . 1- dup not ?exit again words bk; drop ;
	colon	3,"c03"
	bcall	bk99	;BK:
bk1:
	exec	dup_
	exec	dot_
	exec	less1_	;1-
	exec	dup_
	exec	not_
	exec	do_qexit;?EXIT
	jump	bk1		;AGAIN
	exec	words_	;should not exec
bk99:				;BK;
	exec	drop_
	cend

;->	C04	(n --)	count down FOR ... NEXT
;::	for i . next ;
	colon	3,"c04"
	exec	tor_	;FOR
t04:
	exec	rat_	;I
	exec	dot_
	rnext	t04
	cend

;->	C05	(limit start --)	count down DO ... LOOP
;::	c05 do i . loop ;
	colon	3,"c05"
	exec	dtor_	;DO
t05:
	exec	rat_	;I
	exec	dot_
	rloop	t05
	cend

;->	.KY	(--)	dot-key ~ display ASCII value of key
;::	.ky begin key dup . $1b = until who ;
	colon	3,".ky"
dotkey:
	exec	key_
	exec	dup_
	exec	dot_
	lit		$1b
	exec	equal_
	qdgof	dotkey
	rjump	who_
;	cend
.equ	topvam	=	_link*2	;head of VAM vocab

;	*** tail of VEE vocabulary ***
.SET _link		=	0	;init tail null link; VEE vocabulary
;=>	WHO	(--)	display name of VEE vocabulary
	colon	3,"who"
	blit	veetxt*2
	jump	typnfa
.equ	topvee	=	_link*2	;head of VEE vocab

;	*** tail of VAS vocabulary ***
.SET _link		=	0	;init tail null link; VAS vocabulary
;=>	WHO	(--)	display name of VAS vocabulary
	colon	3,"who"
	blit	vastxt*2
	jump	typnfa
.equ	topvas	=	_link*2	;head of VAS vocab

ftop:	;beginning of flash icode dictionary

;============ XCODE SECTION ================
	.org	$2f00	;RWW (read while write)

do_unlop:	;unlop (remove index (i) and limit (i') from return stack)
	pop		zh
	pop		zl	;Z=return addr
	pop		xh
	pop		xl	;remove index
	pop		xh
	pop		xl	;remove limit
	ijmp	;return Z->pc

do_fex:		;forex (for-next exit) set i to one
	pop		zh
	pop		zl	;Z=return addr
	pop		xh
	pop		xl	;remove index
	ldi		xl,1
	push	xl
	push	zerol	;replace index= 1
	ijmp	;return Z->pc

do_leav:	;leave (do-loop exit) set i and i' to zero
	in		xl,SPL
	in		xh,SPH
	adiw	X,3		;point to index (skip return addr)
	st		X+,zerol	;set index=0
	st		X+,zerol
	st		X+,zerol	;set limit = 0 = index
	st		X+,zerol
	ret

;	*** tail of FORTH vocabulary ***
.SET _link		=	0	;init tail null link; FORTH vocabulary

;=>	FORTH	(--)	set FORTH vocab as context (search) vocab
.equ	forthtxt	=	pc+1	;FORTH cstring
	code	5 +(1<<nowbit),"forth"
	literal	forth
savContxt:
	rcall	context_
	rjmp	store_

;=>	VAM	(--)	set VAM (RAM) vocab as context (search) vocab
.equ	vamtxt	=	pc+1	;VAM cstring
	code	3 +(1<<nowbit),"vam"
	literal	vam
	rjmp	savContxt

;=>	VEE	(--)	set vee (EEPROM) vocab as context (search) vocab
.equ	veetxt	=	pc+1	;vee cstring
	code	3 +(1<<nowbit),"vee"
	literal	vee
	rjmp	savContxt

;=>	VAS	(--)	set VAS (flash) vocab as context (search) vocab
.equ	vastxt	=	pc+1	;VAS cstring
	code	3 +(1<<nowbit),"vas"
	literal	vas
	rjmp	savContxt

;=>	HEX	(--)	use 16 as radix for numeric conversions
	code	3,"hex"
	ldi		tmp0,16
	rjmp	setbase


;=>	DEC	(--)	decimal ~ use ten as radix for numeric conversions
	code	3,"dec"
	ldi		tmp0,10
setbase:
	sts		base,tmp0
	ret

;=>	WDR	(--)	WatchDog Timer Reset
	code	3,"wdr"
	wdr
	ret

;=>	WDS	(u --)	WatchDog Timer Set (enable/disable)
;  bit codes: WDP3 (b5), WDE (b3), WDP2 (b2), WDP1 (b1), WDP0 (b0)
;	$0 (off), $8 (16ms), $a (64ms), $c (.25sec), $d (.5sec), $e (1sec), $f (2sec); $28 (4sec), $29 (8sec)
	code	3,"wds"
	in		tmp0,SREG	;save interrupt status (b7)
	cli
	wdr			;reset watchdog
	out		MCUSR,zerol	;clear WDRF
	lds		xl,WDTCSR
	ori		xl,(1<<WDCE) | (1<<WDE)
	sts		WDTCSR,xl	;watchdog change enable, hdw resets in 4 clock cycles
;	ori		tosl,(1<<WDE) | $6: 1sec, $7: 2sec; $20: 4sec, $21: 8sec, 16ms: $0 ;WDE: b3
	sts		WDTCSR,tosl	;set prescaler value (timeout)
	sbrc	tmp0,7	;were interrupts enabled?
	sei			;y: enable interrupts
;	...
	rjmp	drop2


;=>	CLI	(--)	clear global interrupts
	code	3,"cli"
	cli
	ret

;=>	SEI	(--)	set global interrupts
	code	3,"sei"
	sei
	ret

;=>	DIP	(a b c -- b c)	drop third stack item
;	ROT DROP ;
	code	3,"dip"
	rjmp	dip_

;=>	SWUP	(a b -- b a a)	swap dup ~ swap top two stack items, duplicate top item
;	SWAP DUP ;
	code	4,"swup"
	ld		xl,Y+
	ld		xh,Y	;save a
	st		Y,tosh
	st		-Y,tosl;TOS-1= b
	st		-Y,xh
	st		-Y,xl	;push a
	movw	TOS,X	;TOS= a
	ret

;=>	OVAP	(a b -- a a b)	over swap ~ push second stack item as third item
;	OVER SWAP ;
	code	4,"ovap"
	ld		xl,Y
	ldd		xh,Y+1	;get a
	st		-Y,xh
	st		-Y,xl	;push a
	ret

;=>	SNIP	(a b c -- c)	drop first and second items below top of stack
;	SWAP DROP SWAP DROP ;
	code	4,"snip"
	adiw	Y,4
	ret

;=>	RAP	(a b c -- b a c)	Rotate swAP ~ swap TOS-1 and TOS-2
;	ROT SWAP ;
	code	3,"rap"
	ld		tmp2,Y+
	ld		tmp3,Y+	;TOS= b
	ld		tmp0,Y+
	ld		tmp1,Y+	;TMP0= a
	rjmp	rot1

;=>	4US	(u --)	four microsecond systic delay ~ 4 microsecond systic delay
;	max: 262 ms; 1 ms (#250)
	code	3,"4us"
	lds		tmp0,TCNT2	;$b2
	add		tosl,tmp0
	adc		tosh,ticl	;target systic
hus0:
	mov		tmp0,tosh	;target systic high byte
	sub		tmp0,ticl
	cpi		tmp0,-1	;systic = target +1 (high byte)?
	breq	hus9	;y: exit delay
	tst		tmp0	;systic = target (high byte)?
	brne	hus0	;n: continue delay
	lds		tmp0,TCNT2	;$b2
	cp		tmp0,tosl	;systic >= target (low byte)?
	brlt	hus0	;n: continue delay
hus9:
	rjmp	drop2	;y: exit delay

;=>	MS	(u --)	millisecond systic delay
	code	2,"MS"
	add		tosl,ticl
	adc		tosh,tich	;target systic
cus0:
	rcall	pause_
	mov		tmp0,tosh	;target systic high byte
	sub		tmp0,tich
	cpi		tmp0,-1	;systic = target +1 (high byte)?
	breq	cus9	;y: exit delay
	tst		tmp0	;systic = target (high byte)?
	brne	cus0	;n: continue delay
	cp		ticl,tosl	;systic >= target (low byte)?
	brlt	cus0	;n: continue delay
cus9:
	rjmp	drop2	;y: exit delay

;=>	TWAIT	(u --)	task wait ~ reschedule current task (1 millisecond units)
	code	5,"twait"
	add		tosl,ticl
	adc		tosh,tich
	ldi		xh,high(tskwait)	
	ldi		xl,low(tskwait)
	lds		tmp0,taskID	;current taskID
	andi	tmp0,$3	;mask low index bits
	lsl		tmp0	;2x index
	add		xl,tmp0	;point to task timer
	st		X+,tosl	;save target systic value
	st		X,tosh
	sbiw	X,8
	ld		tmp0,X	;task cfa (high byte)
	ori		tmp0,$80;set task wait bit7
	st		X,tmp0	;replace task cfa
	rjmp	drop2

;=>	ROT	(a b c -- b c a)	rotate ~ rotate third item to top
	code	3,"rot"
rot_:
	movw	TMP0,TOS;save c
	ld		tmp2,Y+
	ld		tmp3,Y+	;TMP2= b
	ld		tosl,Y+
	ld		tosh,Y+	;TOS= a
rot1:
	st		-Y,tmp3
	st		-Y,tmp2	;TOS-2= b
	st		-Y,tmp1
	st		-Y,tmp0	;TOS-1= c
	ret

;=>	-ROT	(a b c -- c a b)	minus-rotate ~ rotate top item to third
;	ROT ROT ;
	code	4,"-rot"
negrot_:
	movw	TMP2,TOS;save c
	ld		tosl,Y+
	ld		tosh,Y+	;TOS= b
	ld		tmp0,Y+
	ld		tmp1,Y+	;TMP0= a
	rjmp	rot1
;	st		-Y,tmp3
;	st		-Y,tmp2	;TOS-2= c
;	st		-Y,tmp1
;	st		-Y,tmp0	;TOS-1= a
;	ret


;=>	[	(--)	left-bracket ~ start text interpreter
	code	1 +(1<<nowbit),"["
lbrak_:
	andi	stat,~(1<<cmpsbt)	;clear compile mode (interpret)
	ret

;=>	>LO	(c -- c')	lower case ~ change character to lower case
	code	3,">lo"
	cpi		tosl,'A'
	brlo	lower1
	cpi		tosl,'Z'+1
	brsh	lower1
	ori		tosl,$60
lower1:
	ret

;=>	FLIP	(u -- u')	exchange upper and lower bytes of cell
	code	4,"flip"
flip_:
	mov		xl,tosl
	mov		tosl,tosh
	mov		tosh,xl
	ret

;=>	NIB	(u -- u')	nibble ~ exchange upper and lower four bits of byte
	code	3,"nib"
	swap	tosl
	swap	tosh
	ret


; pim: (port index mask)	upper byte (port index); lower byte (port mask)

;=>	LED	( -- pim)	LED on port PB5 (D13)
;:: $B 5 bit pim cons: led
; osink led pm! \ init led low (sinking) OFF
; ohigh led pm! \ init led high (sourcing) ON
	code	3,"led"
	varable	$020	; addr: 0 (port B); mask: b5
;	ret

;				PINx	DDRx	PORTx
;	0 port B	$23		$24		$25
;	3 port C	$26		$27		$28
;	6 port D	$29		$2A		$2B
; PINx input: 1 enables pullups; output: 1 toggles out
; DDRx 1 sets output
; PORTx input: read; output: 1 drives high
; $B 0 bit pim cons: PB0
; $C 1 bit pim cons: PC1
; $D 2 bit pim cons: PD2

;=>	PIM	(port mask -- pim)	create port index-mask from port# and mask
	code	3,"pim"
	mov		tmp0,tosl	;save mask
	rcall	drop2
	subi	tosl,11	;port# (B:11, C:12, D:13)
	mov		tosh,tosl
	add		tosh,tosl	;port# doubled
	add		tosh,tosl	;port# tripled; B:0, C:3, D:6
	mov		tosl,tmp0	;port mask in upper byte
	ret

;=>	P@	(pim -- n)	port-store ~ read digital input port (mask-addr)
	code	2,"p@"
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	clr		zh
	ld		tosh,Z	;read port
	and		tosl,tosh	;port mask (tosl)
	clr		tosh
	ret

;=>	P?!	( t|f pim --)	set (source) [if flag] or clear (sink) mask bits at port (index-mask)
	code	3,"p?!"
	mov		tmp0,tosl	;save port mask
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	clr		zh
	rcall	drop2	;(t|f)
pset0:
	ldd		xl,Z+2	;read PORTx
	or		xl,tmp0	;set output(s) sourcing
	sbrs	tosl,0	;output flag set?
pset1:
	eor		xl,tmp0	;n: set mask output(s) sinking
;	....
pset2:
	std		Z+2,xl	;set PORTx
drop2:
	loadtos		;drop data stack item
	ret

;=>	PC!	( pim --)	port-clear-store ~ clear (sink) mask bits at port address (mask-addr)
	code	3,"pc!"
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	clr		zh
	mov		tmp0,tosl	;save port mask
	ldd		xl,Z+2	;read PORTx config
	or		xl,tmp0	;set mask output(s) sourcing
	rjmp	pset1

;=>	PS!	( pim --)	port-set-store ~ set (source) mask bits at port address (mask-addr)
	code	3,"ps!"
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	clr		zh
	ldd		xl,Z+2	;read PORTx config
	or		xl,tosl	;set mask output(s) sourcing
	rjmp	pset2

;=>	PT!	( pim --)	port-toggle ~ toggle mask bits at port address (mask-addr)
	code	3,"pt!"
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	clr		zh
	st		Z,tosl	;toggle mask bits
	rjmp	drop2

;=>	PM!	( mode pim --)	port-mode-store ~ set port mode
;	tristate upon reset
;	PINx	Input Pins; output toggle (1)
;	DDRx	Data Direction Register: input (0); output (1)
;	PORTx	Data Register: input pullups (1); output (0/1)
	code	3,"pm!"
	mov		tmp0,tosl	;save port mask
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	clr		zh
	rcall	drop2	;(mode)

; bit 1: input (0)/output (1) mode
	ldd		xl,Z+1	;read DDRx - data direction reg
	or		xl,tmp0	;set mask bits
	sbrs	tosl,1	;output mode?
	eor		xl,tmp0	;n: clear input bit(s)
;	....
	std		Z+1,xl	;set DDRx
	sbrc	tosl,1	;output mode?
	rjmp	pset0	;y: set sink/source
;	....
; bit 0: pullup (input) enable; set initial output state
	ldd		xl,Z+2	;read PORTx
	or		xl,tmp0	;enable masked pullups
	sbrs	tosl,0	;pullup mode?
	eor		xl,tmp0	;n: disable masked pullups
;	....
	std		Z+2,xl	;set PORTx
	rjmp	drop2

;=>	IFLOT	( -- mode)	input-float ~ floating input port mode
	code	5,"iflot"
	varable	$00
;	ret

;=>	IPULL	( -- mode)	input-pullup ~ pullup resistor input port mode
	code	5,"ipull"
	varable	$01
;	ret

;=>	OSINK	( -- mode)	output-sink ~ low (sinking) output port mode
	code	5,"osink"
	varable	$02
;	ret

;=>	OHIGH	( -- mode)	output-high ~ high (sourcing) output port mode
	code	5,"ohigh"
	varable	$03
;	ret


;=>	PLM	(f pim -- u | f)	wait rising (f=1)/falling (f=0) edge; count (u .625us) til falling/rising edge; max 41ms
;	
;	 f=1 +-------+       -----+   u   +-----
;	_____|   u   |____    f=0 |_______|
	code	3,"plm"
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	mov		tmp0,tosl	;save port mask
	rcall	drop2
	mov		tmp2,tosl	;save rising/falling flag
pulse:				;** leading edge **
	clr		zh		;Z= port address
	movw	TOS,ZERO	;** leading edge **
pulse1:		 		;10 cycles = .625 usec
	ld		xl,Z	;read port {2}
	and		xl,tmp0	;port mask {1}
	sbrc	tmp2,0	;waiting for rising edge? {1}
	eor		xl,tmp0	;y: invert {1}
;	....
	breq	pulse2	;{1}
	adiw	TOS,1	;{2}
	brne	pulse1	;{2}
	ret		;timeout error {65536 * .625 us = 40.96 ms}

pulse2:
	movw	TOS,ZERO	;** trailing edge **
pulse3:				;10 cycles = .625 usec
	ld		xl,Z	;read port {2}
	and		xl,tmp0	;port mask {1}
	sbrc	tmp2,0	;waiting on falling edge? {1}
	eor		xl,tmp0	;y: invert {1}
;	....
	brne	pulse9	;{1}
	adiw	TOS,1	;{2}
	brne	pulse3	;{2}
;	----	timeout error (false)
pulse9:
	ret

;=>	PL8	(n pim -- data | f)	compare 8 pulses to threshold (n); n>0: positive pulse, else negative pulse, max threshold: 20ms
;pulse width; 0: 24usec, 1: 70usec (DHT11)
	code	3,"pl8"
	mov		zl,tosh	;PINx index
	subi	zl,-$23	;PINx address
	mov		tmp0,tosl	;save port mask
	rcall	drop2
	cpi		tosh,0x80
	rol		tmp2	;save rising/falling flag (bit0)
	rcall	abs_
	movw	SPML,TOS	;save threshold
	ldi		tmp3,8	;bits to collect
	clr		tmp1	;results register
pl8:
	rcall	pulse
	breq	pl8rtn	;timeout error if zero
	cp		spml,tosl
	cpc		spmh,tosh	;set cy if large pulse (> threshold)
	rol		tmp1	;msb is first pulse
	dec		tmp3	;end of bits to collect?
	brne	pl8		;n: measure next pulse (bit)
	mov		tosl,tmp1 ;y: move result to stack
	clr		tosh
pl8rtn:
	ret

;=>	SLP	(u --)	sleep ~ set sleep mode and sleep
;	Idle(0), ADC Noise Reduction(1), Power-down(2)
;	Power-save(3), Standby(6), External Standby(7)
	code	3,"slp"
	out		SMCR,tosl
	ldi		tosl,(1<<SE) ;enable sleep mode
	out		MCUCR,tosl
	sleep	;Put MCU in sleep mode
	rjmp	drop2

;=>	FB@	(id -- u)	fuse-byte ~ read indicated (id) fuse byte
;	0: LFUSE; 3: HFUSE; 2: EFUSE; 1: LOCKB
;	80: SIGN1; $81: OSC CAL; $82: SIGN2; $84: SIGN3
; : fuse? 4 0 do i fb@ . loop '; emit $85 $80 do i fb@ . loop ;
.equ	SIGRD	= 5	; Signature Row Read
	code	3,"fb@"
	movw	Z,TOS
	ldi		spmcmd, (1<<BLBSET) | (1<<SELFPRGEN)
	sbrc	tosl,7	;read signature bits?
	ldi		spmcmd, (1<<SIGRD) | (1<<SELFPRGEN)	;y:change command
;	....
	out		SPMCSR,spmcmd	;Store Program Memory Control and Status Register
	andi	zl,$07	;max addr of 7
	lpm
	mov		tosl,r0
	clr		tosh
	ret


EPwrite:	;write tosl= EEPROM byte; Z= mapped addr
	rcall	EPwait		;wait for EEPROM ready, adj mapped addr
	out		eedr,tosl	;set Data Reg
EPwrit2:
	sbi		eecr,eempe	;set Master Write Enable
	sbi		eecr,eepe	;set Write Enable to start write
return4:
	ret

EPwait:	;wait for previous write, set real addr (from Z= mapped addr)
	push	zh	;save mapped addr
	subi	zh,high(EEPRM)	;adjust mapped to real adr: $09 -> $00
EPwait2:
	sbic	eecr,eepe	;previous write complete?
	rjmp	EPwait2		;EEPROM Control Register (Write Enable)
	out		eearh,zh	;set Addr Reg
	out		eearl,zl
	pop		zh	;retrieve mapped addr
	ret

atz:	;fetch cell at Z; return in X; Z= Z+2
	cpi		zh,high(ICOD)	;byte addr > $dff (flash)
	brsh	r_fsh
	cpi		zh,high(EEPRM)	;byte addr > $8ff (EEPROM)
	brsh	EPread
	ld		xl,Z+	;read SRAM cell
	ld		xh,Z+
	ret

r_fsh:		;read flash cell
	lpm		xl,Z+
	lpm		xh,Z+
	ret

EPread:		;read EEPROM cell; Z= mapped cell addr; X= (Z)
	rcall	rceep	;read first (low) byte; adjust addr
	adiw	Z,1
	rcall	EPwait	;wait for EEPROM ready, adj mapped addr
	sbi     eecr,eere	;set Read Enable
	in      xh,eedr	;read Data Reg
	adiw	Z,1
	ret		;Z= Z+2, X= (cell)

rcfsh:		;read flash byte
;	andi	zh,$7f	;mask top bit for mirror
	lpm		xl,Z
	ret

rceep:		;read EEPROM byte; Z= mapped addr -> xl=(Z)
	rcall	EPwait	;wait for EEPROM ready, adj mapped addr
	sbi     eecr,eere	;set Read Enable
	in      xl,eedr	;read Data Reg, Z=hdw addr
	ret

spcatz:		;used by .m
	mov		tmp2,zl
	andi	tmp2,$3	;space every 4 bytes
	brne	catz
	rcall	bldot_
catz:	;fetch byte at Z; return in X
	clr		xh
catz1:
	cpi		zh,high(ICOD)	;byte addr > $dff (flash)?
	brsh	rcfsh	;y: read flash byte
	cpi		zh,high(EEPRM)	;byte addr > $8ff (EEPROM)?
	brsh	rceep	;y: read EEPROM byte
	ld		xl,Z	;read SRAM byte
	ret

;=>	DRWAP	(a b c -- b a)	drop-swap ~ drop top stack item, swap top two items
;	DROP SWAP
	code	5,"drwap"
	ldd		tosl,Y+2
	ldd		tosh,Y+3;tos= a
dip_:
	ld		xl,Y+
	ld		xh,Y+	;pop b
dip1:
	st		Y,xl
	std		Y+1,xh	;TOS-1= b
	ret

;=>	DIWAP	(a b c -- c b)	dip-swap ~ drop third stack item, swap top two items
;	DIP SWAP
	code	5,"diwap"
	movw	X,TOS	;save c
	ld		tosl,Y+
	ld		tosh,Y+	;pop b into TOS
	rjmp	dip1	;TOS-1= c

;=>	NIWAP	(a b c -- c a)	nip-swap ~ drop second stack item, swap top two items
;	NIP SWAP
	code	5,"niwap"
	movw	X,TOS	;save c
	adiw	Y,2		;drop b
	ld		tosl,Y
	ldd		tosh,Y+1;tos= a
	rjmp	dip1	;TOS-1= c

;=>	:[	(--)	colon-bracket ~ start compiling anonymous word
;PAD+18: cfa (PAD+20)
;PAD+20: icode
	code	2,":["
	rcall	pad_	;(here+64)
	adiw	TOS,18	;(here+82) leaves 18 bytes for pad
	movw	X,TOS	;X=anonymous word dictionary pointer
	adiw	TOS,2	;TOS=anonymous word cfa
	st		X+,tosl
	st		X,tosh	;dictionary pointer=anonymous word cfa
	sts		anonwd,tosl
	sts		anonwd+1,tosh	;save anonymous word cfa
	sbiw	TOS,2	;TOS=anonymous word dictionary pointer
	rjmp	colon5
	
;=>	];	(--)	bracket-semicolon- ~ end compiling anonymous word and execute it
	code	2 +(1<<nowbit) +(1<<cmpbit),"];"
	rcall	semicolon_	;insert return, end compile, check stack depth
	lds		zl,anonwd
	lds		zh,anonwd+1	;Z=anonymous word cfa
	rjmp	lopadr		;interprete anonymous word

;------------------- COMPILATION WORDS -------------------

;=>	,	(u --)	comma ~ compile TOS into code dictionary
;::	HERE DUP CELL+ DP ! ! ;
	code	1,","
comma_:
	rcall	here_
	rcall	dup_	;(u dp@ dp@)
	adiw	TOS,2	;(u dp@ dp@+2)
	rcall	savdp	;increment dp 1 cell
;	...
	rcall	curentat;current vocab pointer
	movw	X,TOS
	rcall	drop_1
	cpi		xh,high(ICOD)	;flash dp?
	brlo	comma2	;n: put u in prev dict cell (RAM or EEPROM)
	ldi		tosh,high(buf)	;y: translate flash addr to buffer
	andi	tosl,$7f
	ori		tosl,low(buf)	;*** eliminate if buffer 256 byte boundary
comma2:
	rjmp	store_	;put u in prev dict cell

;; ***************** Control Structures *************************** $3554
;	BEGIN ... AGAIN (infinite loop)
;	BEGIN ... <t|f> UNTIL (loop until T, consume flag)
;	BEGIN ... <t|f> TIL (loop until T, keep flag)
;	BEGIN ... <t|f> DURIG (loop while T, consume flag)
;	BEGIN ... <t|f> DUR (loop while T, keep flag)
;	BEGIN ... <t|f> WHILE ... REPEAT (loop while T, exit at WHILE if F)
;	BEGIN ... <t|f> WHL ... REPEAT
;	<t|f> IF ... THEN (consume flag)
;	<t|f> IF ... ELSE ... THEN
;	<t|f> IFS ... ELSE <t|f> IF ... ELSE <t|f> IF ... ELSE ... THENS
;	<t|f> ?IF ... THEN (keep flag)
;	<count> FOR ... NEXT
;	<count> FOR ..{first time}.. AFT ..{not first time}.. THEN ... NEXT
;	BEGIN ..{first time}.. AFT ..{not first time}.. THEN ... AGAIN
;	BEGIN ..{first time}.. AFT ..{not first time}.. THEN ... <t|f> UNTIL
;	BEGIN ..{first time}.. AFT ..{not first time}.. THEN ... <t|f> WHILE ... REPEAT
;	<limit> <start> DO ... I ... LEAVE ... LOP
;	<limit> <start> DO ... <inc> +LOOP (+LP) ... unsigned index
;	<limit> <start> DO ... <inc> -LOOP (-LP) ... unsigned index
;	<limit> <start> ?DO ... LOOP (skip to loop if index > limit)
;	UNLOP removes limit/index
;	<cas?> CASE[ <cas1> ]EQ ... [ <cas2> ]LS ...  ELS ... ;CASE (given/when)
;	7xbb 

;	bref: byte address to branch to
;	Arev: byte address to fix/resolve
;	Arev-cod: byte address to fix/resolve followed by opcode

;	: <MARK ( -- bref ) HERE ;
;	: <FIX ( bref -- ) , ;
;	: >MARK ( -- A cod ) HERE 0 , ;
;	: >FIX ( A cod -- ) HERE SWAP ! ;

;	: FOR ( -- bref ; count -- R: -- count ) COMPILE >R <MARK ; IMMEDIATE
;	: DO ( -- bref ; limit count -- R: -- limit count ) COMPILE 2>R <MARK ; IMMEDIATE
;	: BEGIN ( -- bref ; -- ) <MARK ; IMMEDIATE
;	: IF ( -- A $50 ; f/t -- ) COMPILE ?branch >MARK ; IMMEDIATE
;	: IF? ( -- A $58 ; f/t -- ) COMPILE ?branch >MARK ; IMMEDIATE
;	: IFS ( -- 0 A $50 ; f/t -- ) COMPILE ?branch >MARK ; IMMEDIATE
;	: AHEAD ( -- A $60 ; -- ) COMPILE branch >MARK ; IMMEDIATE

;	: AFT ( bref1 -- bref2 A $60 ; -- ) DROP [COMPILE] AHEAD [COMPILE] BEGIN SWAP ; IMMEDIATE
;	: WHILE ( bref -- A $50 bref ; f/t -- ) [COMPILE] IF SWAP ; IMMEDIATE
;	: ELSE ( A cod -- A $60 ; -- ) [COMPILE] AHEAD SWAP [COMPILE] THEN ; IMMEDIATE
;	: WHEN ( bref A -- bref A bref ; f/t -- ) [COMPILE] IF OVER ; IMMEDIATE

;	: UNTIL ( bref -- ; f/t -- ) COMPILE ?branch <FIX ; IMMEDIATE
;	: NEXT ( bref -- ; R: count -- count-1 ) COMPILE next <FIX ; IMMEDIATE
;	: LOOP ( bref -- ; R: limit count -- limit count-1 ) COMPILE doloop <FIX ; IMMEDIATE
;	: +LOOP ( bref -- ; n -- R: limit count -- limit count+n ) COMPILE doplop <FIX ; IMMEDIATE
;	: AGAIN ( bref -- ; -- ) COMPILE branch <FIX ; IMMEDIATE
;	: REPEAT ( A cod bref -- ; -- ) [COMPILE] AGAIN >FIX ; IMMEDIATE
;	: THEN ( A cod -- ; -- ) >FIX ; IMMEDIATE

;=>	(+loop)	(inc -- ; R: index limit -- index' limit)	+loop execution code
;::	R>  R> 1 + R>  2DUP XOR
;::	IF R@  SWAP >R  SWAP >R  >R  DROP  EXIT
;::	THEN 2DROP  R> DROP  >R ;
do_plp:
	pop		xh
	pop		xl	;discard rtn addr
	pop		zh
	pop		zl	;fetch index
	pop		xh
	pop		xl	;fetch limit
	add		zl,tosl
	adc		zh,tosh	;inc index
	bst		tosh,7	;set T if negative index
	ld		tosl,Y+
	ld		tosh,Y+	;pop inc from data stack
	cp		zl,xl
	cpc		zh,xh	;index ?? limit?
	brtc	do_plp1	;positive increment?
	brge	do_plp2	;n: -inc and index >= limit -> continue looping
do_exit:			;y: end loop
	subi	nxt,$fe	;skip loop addr (add 2 bytes)
	sbci	nxth,$ff
	rjmp	lopnxt

do_plp1:	;y: positive increment
	brge	do_exit	;+inc and index >= limit => exit (skip loop branch)
do_plp2:
	push	xl
	push	xh	;restore limit
	push	zl
	push	zh	;restore index
	rjmp	lopnxt	;continue loop (next branch)


do_lit:		;put inline literal on dstk
	rcall	dup_	;save TOS
	movw	Z,NXT
	rcall	atz
	movw	TOS,X
	movw	NXT,Z
	ret

;=>	BEGIN	(-- bref)	start loop structure
;::	HERE ; IMMED
	code	5 +(1<<nowbit) +(1<<cmpbit),"begin"
begin_:
	rjmp	here_	;reference aref

;=>	FOR	(-- bref; limit --)	start FOR-NEXT loop in colon definition
;::	COMPILE >R HERE ; IMMED
	code	3 +(1<<nowbit) +(1<<cmpbit),"for"
	literal	tor_		;runtime index on rtn stack
for2:
	rcall	comma_	;tor -> code dict 
	rjmp	here_	;reference aref

;=>	DO	(-- bref ; limit count --)	start DO-LOOP structure in colon definition
;::	COMPILE dodo HERE ; IMMED
	code	2 +(1<<nowbit) +(1<<cmpbit),"do"
	literal	dtor_	;runtime index on rtn stack
	rjmp	for2

;=>	NEXT	(bref --)	terminate FOR-NEXT loop if index zero
;::	COMPILE next , ; IMMED
	code	4 +(1<<nowbit) +(1<<cmpbit),"next"
	ldi		tmp0,$6c	;rnext - relative NEXT runtime
	rjmp	relcmd

;=>	LOOP	(bref --)	continue DO-LOOP if limit < index +1
;::	COMPILE dolop , ; IMMED
	code	4 +(1<<nowbit) +(1<<cmpbit),"loop"
	ldi		tmp0,$68	;rloop - relative LOOP runtime
	rjmp	relcmd

;=>	2>R	(d -- R: -- d)	double-to-r ~ push double data stack to return stack
;::	BEGIN SWAP R> CELL+ DUP >R SWAP >R SWAP >R >R ;
	code	3 +(1<<cmpbit),"2>r"
dtor_:
	pop		zh
	pop		zl	;Z=return addr
	movw	X,TOS	;save high cell from parm stk
	rcall	drop2	;get low cell from parm stk
	push	tosl
	push	tosh;push dl on rtn stk
	push	xl
	push	xh	;push dh on rtn stk
	rcall	drop2
	ijmp	;return Z->pc

;=>	2R>	(-- d R:d --)	double-r-from ~ double pop return stack to data stack
	code	3 +(1<<cmpbit),"2r>"
	pop		zh
	pop		zl	;Z=return addr
	pop		xh
	pop		xl	;fetch dh from rtn stk
	rcall	dup_
	pop		tosh
	pop		tosl;fetch dl from rtn stk
	rcall	dup_
	movw	TOS,X	;move dh to TOS
	ijmp	;return Z->pc

;=>	2R@	(-- d R:d -- d)	double-r-fetch ~ double copy return stack to data stack
	code	3 +(1<<cmpbit),"2r@"
	pop		zh
	pop		zl	;Z=return addr
	pop		xh
	pop		xl	;fetch dh from rtn stk
	rcall	dup_
	pop		tosh
	pop		tosl;fetch dl from rtn stk
	rcall	dup_
	push	tosl	;put dl back on rtn stk
	push	tosh
	movw	TOS,X	;move dh to TOS
	push	xl	;put dh back on rtn stk
	push	xh
	ijmp	;return Z->pc

;->	$,"	(--)	compile literal string up to next " 
;::	34 WORD COUNT ALIGNED CP ! ;
;	code	3 +(1<<cmpbit),'$', ',', '"'
STRCQ:
	literal	'"'
	rcall	word_	;move string to code dictionary
	rcall	dup_
	rcall	cat_	;string length (bytes)
	rcall	twosl_	;cell length
	rcall	tor_	;set loop index
STRCQ1:
	rcall	dup_
	rcall	at_
	rcall	comma_
	adiw	TOS,2	;inc pointer to next cell
	donext	STRCQ1	;repeat til index is zero
	rjmp	drop_1

;=>	+LOOP	(bref --)	plus-loop ~ continue DO-LOOP if limit > index +inc
;::	COMPILE doplp , ; IMMED
	code	5 +(1<<nowbit) +(1<<cmpbit),"+loop"
	literal	do_plp
next2:
	rcall	comma_
;	rjmp	again_	;loop to aref
byjmp:
	rcall	twosl_	;2/ byte addr -> cell addr
	ori		tosh,$80	;$80: jump - to cell addr
	rjmp	comma_		;abs branch to bref (BEGIN)

;->	/LOOP	(bref --)	slash-loop ~ unsigned continue DO-LOOP if limit > index +inc
;::	COMPILE doplp , ; IMMED
;	code	5 +(1<<nowbit) +(1<<cmpbit),"/loop"
;	literal	do_slp
;	rjmp	next2	;call doplp; loop to aref

; *** ?leave *******

;=>	LEAVE	(--)	terminate DO-LOOP
;::	COMPILE doleav , ; IMMED
	code	5 +(1<<nowbit) +(1<<cmpbit),"leave"
	literal	do_leav
	rjmp	comma_

;=>	UNLOP	(--)	unloop ~ remove index and limit from return stack
;::	COMPILE doleav , ; IMMED
	code	5 +(1<<nowbit) +(1<<cmpbit),"unlop"
	literal	do_unlop
	rjmp	comma_

;=>	FOREX	(--)	for-exit ~ terminate FOR ... NEXT
;::	COMPILE dofex , ; IMMED
	code	5 +(1<<nowbit) +(1<<cmpbit),"forex"
	literal	do_fex
	rjmp	comma_

;=>	DFAR:	($ --)	xxx ~ indirect execution of existing word (ram dict)
	code	5 +(1<<nowbit) +(1<<cmpbit),"dfar:"
	rcall	tick_	;parse next word in TIB with blank delim
	rcall	here_	;(cfa adr) free space in dictionary
	adiw	TOS,2	;(cfa dp@+2)
	rcall	head_	;link word in dictionary
	rjmp	byjmp	;compile byte addr as jmp

;=>	UNTIL	(bref -- ; t|f --)	end BEGIN-UNTIL loop; continue until true
;::	COMPILE ?branch , ; IMMED
	code	5 +(1<<nowbit) +(1<<cmpbit),"until"
	ldi		tmp0,$50	;qdgof - relative branch if false, drop flag
relcmd:
	rcall	here_
	rcall	sub_	;relative bytes
	sbiw	TOS,2	;adjust inner interpreter next cell
	rcall	twosl_	;2/ relative cells
	andi	tosh,radmsk	;mask for relative opcode
	or		tosh,tmp0	;add opcode
	rjmp	comma_		;qbranch to aref (BEGIN)

;=>	TIL	(bref -- ; t|f -- t|f)	end BEGIN-TIL loop; continue until true, retain flag
	code	3 +(1<<nowbit) +(1<<cmpbit),"til"
	ldi		tmp0,$58	;qkgof - relative branch if false, keep flag
	rjmp	relcmd

;=>	DURIG	(bref -- ; t|f --)	end BEGIN-DURIG loop; continue while true
	code	5 +(1<<nowbit) +(1<<cmpbit),"durig"
	ldi		tmp0,$40	;qdgot - relative branch if true, drop flag
	rjmp	relcmd

;=>	DUR	(bref -- ; t|f -- t|f)	end BEGIN-DUR loop; continue while true, retain flag
	code	3 +(1<<nowbit) +(1<<cmpbit),"dur"
	ldi		tmp0,$48	;qkgot - relative branch if true, keep flag
	rjmp	relcmd

;=>	AGAIN	(bref -- ; --)	end BEGIN-AGAIN loop
	code	5 +(1<<nowbit) +(1<<cmpbit),"again"
again_:
	ldi		tmp0,$60	;rjump - relative branch
	rjmp	relcmd

;=>	IFS	(-- 0 Arev $50)	begin f IFS ... ELSE f IF ... ELSE f IF ... ELSE ... THENS structure
;::	0 LIT COMPILE ?branch HERE 0 , ; IMMED
	code	3 +(1<<nowbit) +(1<<cmpbit),"ifs"
	rcall	zeros
	rjmp	if_

;=>	{	(-- Arev )	begin called block structure BK:
	code	1 +(1<<nowbit) +(1<<cmpbit),"{"
block_:
	rcall	begin_	;(dp@)
	rjmp	dallot2

;=>	}	(Arev --)	terminate called block structure BK;
	code	1 +(1<<nowbit) +(1<<cmpbit),"}"
bkend_:
	ldi		tmp0,$44	;bcall - block inline call
	rjmp	then2

;=>	IF	(-- Arev $50 ; t|f --)	begin conditional branch structure; drop flag
;::	COMPILE ?branch HERE 0 , ; IMMED
	code	2 +(1<<nowbit) +(1<<cmpbit),"if"
if_:
	rcall	begin_	;(dp@)
	rcall	dup_
	ldi		tosh,$50;qdgof - relative branch if false, drop flag
dallot2:		;allot Arev (ELSE/THEN)
	literal	2
	rjmp	allot_

;=>	?IF	(-- Arev $58 ; t|f -- t|f)	question-if ~ begin conditional branch structure; keep flag
	code	3 +(1<<nowbit) +(1<<cmpbit),"?if"
	rcall	begin_	;(dp@)
	rcall	dup_
	ldi		tosh,$58	;qkgof - relative branch if false, keep flag
	rjmp	dallot2		;allot Arev (ELSE/THEN)

;=>	ALLOT	(n --)	save n bytes in dictionary space
;::	dp +! ;
	code	5 +(1<<nowbit),"allot"
allot_:
	rcall	dp_		;ptr to next dict space
	rjmp	pstore_ ;add n to DP

;=>	ALLOC	(n --)	allocate ~ reserve n bytes in variable space
;::	- vp +! ;
	code	5 +(1<<nowbit),"alloc"
	rcall	neg_
	rcall	vp_		;ptr to last variable space
	rjmp	pstore_ ;add n to VP

;=>	REPEAT	(Arev cod bref --)	end BEGIN-WHILE-REPEAT loop
;::	COMPILE AGAIN HERE SWAP ! ; IMMED
	code	6 +(1<<nowbit) +(1<<cmpbit),"repeat"
	rcall	again_	;branch to bref (BEGIN)
	rjmp	then_	;resolve branch to Arev (WHILE)

;=>	THENS	(0 Arev cod ... Arev cod --)	end IFS ... THENS structure
	code	5 +(1<<nowbit) +(1<<cmpbit),"thens"
thens_:
	rcall	then_
	adiw	TOS,0	;zero marker?
	brne	thens_	;n: resolve next IF
	rjmp	drop_	;y: toss zero marker

;=>	THEN	(Arev cod --)	end conditional branch structure
;::	HERE SWAP ! ; IMMED
	code	4 +(1<<nowbit) +(1<<cmpbit),"then"
then_:		;--- resolve Arev-cod code on data stack
	mov		tmp0,tosh	;branch ref type (cod)
	rcall	drop_	;(Arev)
then2:		;--- resolve Arev-cod tmp0=code
	rcall	dup_	;(Arev Arev)
	rcall	here_	;(Arev Arev dp@)
	rcall	swap_	;(Arev dp@ Arev)
	rcall	sub_	;(Arev dp@-Arev)
	sbiw	TOS,2	;adjust inner interpreter next cell
	rcall	twosl_	;(Arev (dp@-Arev-2)/2)
	andi	tosh,radmsk	;mask for relative opcode
	or		tosh,tmp0	;mark ?/branch cell
	rcall	swap_	;((dp@-Arev-2)/2 Arev)
	rjmp	store_	;resolve ?/branch to Arev

;=>	ELSE	(Arev1 $50 -- Arev2 $60)	start false clause in IF-ELSE-THEN
;::	[COMPILE] AHEAD SWAP [COMPILE] THEN ; IMMED
	code	4 +(1<<nowbit) +(1<<cmpbit),"else"
	rcall	ahead_	;allot Arev2 (THEN)
	rcall	dswap_	;(Arev2 cod Arev1 cod)
	rjmp	then_	;resolve branch to Arev1 (IF)

;=>	AFT	(bref1 -- bref2 Arev $60)	skip AFT-THEN first time FOR-AFT-THEN-NEXT loop first time through
;::	DROP [COMPILE] AHEAD [COMPILE] BEGIN SWAP ; IMMED
	code	3 +(1<<nowbit) +(1<<cmpbit),"aft"
	rcall	drop_	;discard bref1
	rcall	ahead_	;allot Arev (THEN)
	rcall	begin_	;(Arev cod bref2) reference bref2 (NEXT/AGAIN/UNTIL/REPEAT)
	rjmp	negrot_	;(bref2 Arev cod)

;->	AHEAD	(-- Arev $60)	compile a forward branch instruction
;::	COMPILE branch HERE 0 , ; IMMED
;	code	5 +(1<<nowbit) +(1<<cmpbit),"ahead"
ahead_:
	rcall	begin_	;(dp@)
	rcall	dup_	;(dp@ dp@)
	ldi		tosh,$60;(dp@ $60) rjump - relative branch
	rjmp	dallot2	;allot Arev (ELSE/THEN)

;=>	EXIT	(--)	terminate BK: or colon word
	code	4 +(1<<nowbit) +(1<<cmpbit),"exit"
	rjmp	zcomma
	
;=>	?EXIT	(-- ; t|f --)	question-exit ~ terminate BK: or colon word if true
	code	5 +(1<<nowbit) +(1<<cmpbit),"?exit"
	literal	do_qexit
	rjmp	comma_

;=>	CASE[	(-- 0 Arev0; test -- test)	case-begin ~ start of case structure
	code	5 +(1<<nowbit) +(1<<cmpbit),"case["
	rcall	zeros	;(0)
	rcall	begin_	;(0 Arev)
	rcall	dallot2	;allot Arev
	literal	do_case
	rcall	comma_
	rjmp	lbrak_	;stop compiling

;=>	]EQ	(0 Arev0 [op-tst Arev] Tst -- 0 Arev0 op-Tst Arev1; test -- test)	end-equal ~ element of CASE structure
	code	3 +(1<<nowbit) +(1<<cmpbit),"]eq"
	ldi		tosh,$70;cas2= opcode:b1
eqcase:
	ldd		tmp0,Y+2;opcode of previous case
	sbrc	tmp0,4	;not first case ($70/$78)?
	rcall	fixcas	;y: resolve previous case
;	....
	rjmp	here_	;(op-Tst Arev1)

fixcas:		;resolve op-tst Arev
	rcall	negrot_	;(Arev0 op-Tst op-tst Arev)
	rcall	tuck_	;(Arev0 op-Tst Arev op-tst Arev)
	rcall	here_	;(Arev0 op-Tst Arev op-tst Arev dp@)
	rcall	swap_	;(Arev0 op-Tst Arev op-tst dp@ Arev)
	rcall	sub_	;(Arev0 op-Tst Arev op-tst dp@-Arev)
	rcall	twosl_	;(Arev0 op-Tst Arev op-tst (dp@-Arev)/2)
	andi	tosl,$07;mask relative address
	mov		tmp0,tosl
	rcall	drop_	;(Arev0 op-Tst Arev op-tst)
	add		tosh,tmp0
	rcall	swap_	;(Arev0 op-Tst opbr-tst Arev)
	rcall	store_	;resolve previous case
	rjmp	rbrak_	;(Arev0 op-Tst) start compiling

;=>	]LS	(0 Arev0 [op-tst Arev] Tst -- 0 Arev0 op-Tst Arev1; test -- test)	CASE lower/same element
	code	3 +(1<<nowbit) +(1<<cmpbit),"]ls"
	ldi		tosh,$78;cas2= opcode:b1
	rjmp	eqcase

;=>	ELS	(0 Arev0 [op-tst Arev] -- 0 Arev0 op-0 Arev1; test -- test)	else case ~ CASE default element
	code	3 +(1<<nowbit) +(1<<cmpbit),"els"
	literal	$7800	;default case
	rjmp	eqcase

;=>	;CASE	(0 Arev0 op-tst Arev -- ; test -- test)	end-case ~ structure
	code	5 +(1<<nowbit) +(1<<cmpbit),";case"
	rcall	store_	;(0 Arev0) resolve Arev branch
	rcall	bkend_	;(0) resolve CASE Arev
	rjmp	comma_	;put block exit in dictionary

;=>	COMP:	($ --)	compile ~ (don't execute) immediate word
;	Compile next (immediate) word into dictionary
	code	5 +(1<<nowbit) +(1<<cmpbit),"comp:"
	rcall	tick_	;parse next word in TIB with blank delim
bytcomma:
	rcall	twosl_	;2/ byte addr -> cell addr
	rjmp	comma_

;=>	$"	(-- ; -- adr)	string-quote ~ compile inline string literal, adr of string at runtime
;::	COMPILE $"| $," ; IMMED
	codez	2 +(1<<nowbit) +(1<<cmpbit), '$', '"'
	literal	do_str
	rjmp	dotq2

do_str:
	rcall	dup_
do_str1:
	movw	TOS,NXT
	rcall	at_
	mov		tmp0,tosl	;count
	movw	TOS,NXT		;str addr on dstk
	subi	tmp0,$fe	;count+2
	add		nxt,tmp0
	adc		nxth,zerol	;NXT + count +2
	andi	nxt,$fe		;cell align
	ret

;=>	."	(-- ; <tout: string>)	dot-quote ~ compile inline string literal, typed out at run time
	codez	2 +(1<<nowbit) +(1<<cmpbit), '.', '"'
;::	COMPILE ."| $," ; IMMED
	literal	do_dotq
dotq2:
	rcall	comma_
	rjmp	STRCQ

do_dotq:
	rcall	do_str
	rjmp	dotmsg

;=>	.M	(adr --)	dot-memory ~ formatted display 128 bytes in hex and ascii
;::	: MEM (addr cnt --)
;::	BASE @ >R HEX 16 /
;::	FOR CR 16 2DUP dm+ ROT ROT 2 SPACES _TYPE NUF? NOT WHILE
;::	NEXT ELSE R> DROP THEN DROP  R> BASE ! ;
	code	2,".m"
	movw	Z,TOS
	andi	zl,$f0	;start on even address
	ldi		tmp1,8	;8 rows of 16 bytes
mem01:
	rcall	cr_
	rcall	dothex4	;disp address
mem02:
	rcall	spcatz	;Z=addr; xl=contents
	rcall	dothex2
	rcall	bldot_
	subi	zl,$ff	;inc address
	brhs	mem02
	subi	zl,16
mem03:
	rcall	spcatz	;Z=addr; xl=contents
	mov		tosl,xl
	rcall	tochar_	;dot if non-print chr
	mov		tmp0,tosl
	rcall	cemit	;disp ascii
	subi	zl,$ff	;inc address
	brhs	mem03
	brbs	0,mem04	;carry set?
	inc		zh		;n: inc address high byte
mem04:
	dec		tmp1
	brne	mem01
drop_1:
	rjmp	drop_

;=>	.HX	(n --)	dot-hex ~ display contents of data stack in hex (four hex digits)
;::	: .X HEX . ;
	code	3,".hx"
dothex_:
	rcall	tos2z
dothex4:
	mov		xl,zh
	rcall	dothex2
	mov		xl,zl
dothex2:
	push	xl		;save hex value
	mov		tmp0,xl
	swap	tmp0	;high nibble first
	rcall	dothex
	pop		tmp0	;low nibble
dothex:
	andi	tmp0,$0f
	subi	tmp0,-'0'
	cpi		tmp0,'9'+1
	brlo	dthx9
	subi	tmp0,'9'-'@'
dthx9:
	rjmp	cemit

;=>	]	(--)	right-bracket ~ start compiling words in input stream
	code	1,"]"
rbrak_:
	ori		stat,(1<<cmpsbt)	;set compile mode
	ret

;=>	:>E	(-- ; <string>)	colon-to-eeprom ~ start new definition in EEPROM for next word in TIB
;::	BL WORD $,n doLIT doLIST  call, ] ;
	code	3,":>e"
	literal	eepdp	;EEPROM dictionary free space ptr
	rjmp	colon1

;=>	:>F	(-- ; <string>)	colon-to-flash ~ start new definition in flash for next word in TIB
;::	BL WORD $,n doLIT doLIST  call, ] ;
	code	3,":>f"
	literal	flhdp	;flash dictionary free space ptr
	rjmp	colon1

;->	HED	(nfa --)	head ~ build colon dictionary head from nfa (dp@+2)
;	WORD (blword) put nf from TIB in dictionary; insert in links
;	code	3,"hed"
head_:
	rcall	dup_	;(nfa nfa)
	rcall	cat_	;(nfa nfb) name field byte zero?
	ldi		tmp1,$06;error# zero length word
	qjump	syserr	;y: error $06 (nfa)
	rcall	qword_	;redefinition? (nfa)
;	----	copy nf from RAM dict to selected dictionary
	rcall	here_	;(nfa dp@)
	adiw	TOS,2	;(nfa dp@+2)
	rcall	tuck_	;(dp@+2 nfa dp@+2)
	rcall	cmv_	;(dp@+2) copy nf to dict
;	----			;dict nfa= dp@+2
	rcall	last_	;(nfa nfa') last CURRENT word nfa'
	rcall	comma_	;save last CURRENT vocab nfa in lfa
	rcall	here_	;(nfa dp@) points to name field
	rcall	curentat;(nfa dp@ vadr)
	rcall	store_	;save new nfa in CURRENT vocab
;	---		move dp@ to end of name field
	rcall	cat_	;(nfb) name field byte
	adiw	TOS,2
	andi	tosl,$1e;cell align and mask word attributes
	rjmp	allot_	;increment dp@ by aligned name field size

;; FORTH compiler

;=>	:	(-- ; <string>)	colon ~ start new definition in RAM for next word in TIB
;::	BL WORD $,n doLIT doLIST  call, ] ;
	code	1,":"
	literal	ramdp	;RAM dictionary free space ptr
colon1:
	rcall	colon5
	rjmp	creat_	;parse word, create head (lf/nf) in dict

colon5:
	literal	dp
	rcall	store_	;save ptr to dictionary pointer
	sts		cmpcnt,yl	;save stack location in compile counter; tested by semicolon
	rjmp	rbrak_	;start compilation

;=>	IMMED	(--)	immediate ~ make last compiled word an immediate word
;::	LAST @ $80 OR LAST !
	code	5,"immed"
	rcall	last_	;last word nfa
	rcall	cat_
	ori		tosl,(1<<nowbit)
	rcall	last_
	rjmp	cstore_

;=>	;	(--)	semicolon ~ terminate colon definition
;::	COMPILE EXIT [COMPILE] [ OVERT ; IMMED
	code	1 +(1<<nowbit) +(1<<cmpbit),';'
semicolon_:
	andi	stat,~(1<<cmpsbt)	;clear compile mode (interpret)
	lds		tmp0,cmpcnt	;save stack location
	ldi		tmp1,$07;error# compile stack inbalance
	sub		tmp0,yl	;compile stack location: start = end?
	breq	zcomma	;y: put return code in dict
	rjmp	syserr	;n: compilation error (tmp0=stack delta)

;=>	CWD	(cstr --)	compile word ~ compile cstr to code dictionary as word or literal
	code	3,"cwd"
cwd_:
	rcall	swd_	;srch vocabs for word (cfa nfa | cstr F)
	rcall	qdup_	;defined?
	qjump	cwd1	;n: see if number
	rcall	cat_	;y: (cfa nfa@)
	andi	tosl,(1<<nowbit) ;immediate word?
	qjump	bytcomma	;n: compile cfa
	rjmp	exe_	;y: execute word

cwd1:		;(cstr) check if number?
	rcall	numberq_;(n|d T | cstr F)
	ldi		tmp1,$81;error# compile undefined word
	qjump	syserr	;n: error $81 (cstr)
	sbrc	stat,flgsbt	;y: compile as literal; double number?
	rcall	lit_	;y: compile part of double literal
;	....
	rjmp	lit_	;compile literal

;=>	WD?	(cstr -- cstr)	word-question ~ warning message if word exists (not unique)
	code	3,"wd?"
qword_:
	rcall	dup_	;(cstr cstr)
	rcall	swd_	;?name exists (cstr [cfa nfa | cstr F])
	qjump	drop_	;n: return with (cstr)
	rcall	doMSG	;y: (cstr cfa) redefinitions are OK
	.db		7," reDef>"	;but warn user
	rcall	drop_	;(cstr) drop cfa
	rcall	dup_	;(cstr cstr)
	rjmp	dotmsg	;(cstr)

;; Defining words

;=>	LIT	(u --; -- u)	literal ~ compile TOS to code dictionary as integer literal
;::	COMPILE doLIT , ; IMMED
; -8192 ($e000) to -1 ($ffff) to $00 to 8191 ($1fff)
	code	3 +(1<<nowbit),"lit"
lit_:
	mov		xh,tosh
	subi	xh,$e0	;add $20 so $e0/$f0 -> $00/01
	andi	xh,$c0	;short lit?
	brne	lit1	;n: save long literal
	ori		tosh,$c0
	rjmp	comma_

lit1:		;long literal
	literal	do_lit
x2comma:
	rcall	comma_	;store do_lit
	rjmp	comma_	;store long literal

;=>	VP	(-- adr)	variable-pointer ~ pointer (adr) to available variable space
	code	2,"vp"
vp_:
	varable	vptr
;	ret

;=>	CONS:	($ n -- ; -- n)	constant ~ create constant
;::	CON CREAT , DOES> @ ;
	code	5,"cons:"
con_:
	rcall	creat_	;parse word, create head (lf/nf) in dict
	rcall	lit1	;put do_lit and n in dict
zcomma:				;put return code in dict
	rcall	zeros
	rjmp	comma_	;end of list

;=>	VARB:	($ n -- ; -- adr)	variable ~ create new variable uninitialized with n bytes
	code	5,"varb:"
	rcall	neg_
	rcall	vp_		;ptr to last variable space
	rcall	pstore_ ;add -n to VP (move ptr back)
	rcall	vp_
	rcall	at_		;address in variable space
	rjmp	con_	;create constant with address

;=>	CRAT:	($ -- ; --)	create ~ make new dictionary entry
	code	5,"crat:"
creat_:
	rcall	blword	;parse next TIB word name (- nfa)
head__:
	rjmp	head_	;create head (lf/nf) in dict

;->	DOES>	(-- ; -- adr)	DOES> - compile run time action
;	code	5 +(1<<nowbit),"does>"
;	rcall	here_	;parse next TIB word name (- nfa)
;	rjmp	comma	;create head (lf/nf) in dict

;	%%% to be implemented
;	: CREATE        ( just like the original, except for the NOP )
;	TOKEN $,n OVERT
;	doLIT doLIST call,
;	COMPILE NOP COMPILE doVAR ;

;	: does>         ( helping function )
;	LAST @ NAME> DUP
;	doLIT branch SWAP 4 + ! DUP
;	HERE SWAP 6 + !
;	COMPILE doLIT
;	8 + ,
;	COMPILE branch ;

;	: DOES>     ( here it is! )
;	doLIT does> ,
;	doLIT doLIT ,
;	HERE 6 + ,
;	doLIT , ,
;	doLIT EXIT ,
;	; IMMEDIATE


;=>	VALU:	($ n -- ; -- n)	value ~ create new variable initialized with n
	code	5,"valu:"
	rcall	creat_	;parse next TIB word and create head (lf/nf) in dict
	rcall	makvar	;allot cell to var, init to n
	rcall	vp_
	rcall	at_		;address in variable space
	literal	do_val
x2coma0:
	rcall	x2comma	;put do-val and parm in dict
	rjmp	zcomma	;put return code in dict

do_val:
	rcall	do_lit
	rjmp	at_		;contents of value

;; Device dependent I/O

;>>	set zero flag
qkey:
	clz			;clear zero flag
	in_		tmp0,UCSR0A	;USART Control and Status Register A
	sbrc	tmp0,RXC0	;USART Receive Complete?
	sez		;y: z=1, char received
;	....
	ret		;n: z=0

key:
	rcall	pause_
	rcall	qkey	;input chr?
	brne	key		;n: wait
	andi	tmp0,$1c;frame, overrun or parity error?
	breq	key1	;n: skip counter
	inc		r5		;USART error counter
;	rjmp	key		;%%% ignore character if error %%%
key1:
	in_		tmp0,UDR0	;y: USART I/O Data Register
crc8tmp0:
	eor		crc8,tmp0	;CRC8= CRC8 ^ data
	rjmp	crc8calc

;bits sent lsb first, reverse polynomial, top coefficient (x8) inactive
;CRC-8- Dallas/Maxim iButton polynomial x8+x5+x4+1 (1 0011 0001) $31 -> 1000 1100 -> $8C
;CRC-8-AUTOSAR polynomial x8+x5+x3+x2+x+1 $2F -> $F4

;=>	CRC	(n -- n)	Cyclic Redundancy Check ~ compute crc
	code	3,"crc"
crc8calc_:
	eor		crc8,tosl	;CRC8= CRC8 ^ data
crc8calc:
	ldi		xl,7	;7bit transmission
crc01:
	bst		crc8,0	;save b0
	lsr		crc8	;CRC8 >>= 1
	brtc	crc02	;b0 clear?
	ldi		xh,$F4	;n: poly coefficients
	eor		crc8,xh	;n: CRC8 = (CRC8 >> 1) ^ 0x8C
crc02:
	dec		xl		;processed all bits?
	brne	crc01	;n: next bit
	ret

qesc:
	rcall	qkey
	brne	qesc1
	rcall	key
	cpi		tmp0,$1b	;ESC char?
qesc1:
	ret		;y: z=1 if ESC

;=>	EMIT	(c --)	send character c to serial port
	code	4,"emit"
emit_:
	mov		tmp0,tosl
	rcall	cemit
	rjmp	drop2

;>>	send chr in tmp0 via serial port
cemit:
	rcall	crc8tmp0	;compute CRC8
cemit1:
;	push	tmp0	;save tmp0 (chr)
;	rcall	pause_
;	pop		tmp0	;restore tmp0 (chr)
	in_		xl,UCSR0A	;USART ctrl/stat regA
	sbrs	xl,UDRE0	;USART Data Register empty?
	rjmp	cemit1	;n: wait
;	....
	sbi		PORTC,0	;turn on RS485 driver (portC0)
	out_	UDR0,tmp0	;y: USART data reg
	ret

;=>	BD!	(n --)	set-baud ~ initialize USART0; n= baud rate /100; e.g. 192
	code	3,"bd!"
	literal	10000	;baud conversion constant
	rcall	swap_
	rcall	slash_	;divide constant by baud rate
	mov		xl,tosl
	subi	xl,1
	rcall	drop_
;	UBRR= 1,000,000/baudRate - 1
;	UBRR (2x)= 2000000/baudRate - 1

;	sUART receive character, tmp0= character
;=>	key2	( -- chr )	initialize software UART
	code	4,"key2"
rx_chr:
	savetos
	rcall	pause_
	sbrs	stat,urxsbt	;chr received?
	rjmp	rx_chr	;n: wait for chr
;	....			;y: get chr
	clr		tosh
	mov		tosl,urxchr
	andi	stat,~(1<<urxsbt)	;clear received data bit
	ret

;=>	TUCK	(a b -- b a b)	copy first (top) stack item below second stack item
;	SWAP OVER
	code	4,"tuck"
tuck_:
	movw	X,TOS	;x2
	ld		xl,Y+
	ld		xh,Y	;X= x1
	st		Y,tosh
	st		-Y,tosl	;tos-2= x2
	st		-Y,xh
	st		-Y,xl	;tos-1= x1
	ret

;=>	NIP	(a b -- b)	drop first item below top of stack
;	SWAP DROP
	code	3,"nip"
nip_:
	adiw	Y,2
	ret


;=>	CLR	(--)	clear ~ reset data stack
	code	3,"clr"
clr_:
	ldi		yl,low(SP0)
	ldi		yh,high(SP0)	;reset data stack ptr
	ret

do_defer:
	rcall	do_lit
	rjmp	atexec_

;=>	RUN	(cfa --)	execute icode at cfa
	code	3,"run"
	ori		nxth,$80;mark icode byte addr
	push	nxt		;save byte return address
	push	nxth
	rcall	tos2z
	rjmp	lopadr	;Z= icode byte address

; ------- cooperative multitasker ---------
; taskID:	r~~~ ~~##
;			r: a task is ready (0), no ready tasks (1)
;			##: current task#
; tsklst:
;	task1 kfa	:: active task if b6 set
;	task2 kfa	:: timed task if b7 set
;	task3 kfa	:: max time $7eff ms (29439 -> 30.1sec)
;	task4 kfa
;	if top byte of kfa in task list is zero: empty slot
; tskwait:
;	task1 timed systic target
;	task2 timed systic target
;	task3 timed systic target
;	task4 timed systic target

; WARN: tsklst address must not be close to 256 bit boundary
; +TASK	(cfa -- t|f)	add-task ~ to queue; false if queue full
; -TASK	(cfa -- t|f)	remove-task ~ delete task from queue; false if not found
; TWAIT (ms --)			task wait ~ reschedule current task (1 millisecond units)
; key and key? do not call pause
; tasker 	NOWRAP	WRAP
; bit 7:	clear	set
; systic:	$7fff	$7fff
;			#####	.^^^.
;	$7f01	.end.	begin	$7fff
;			.^^^.	#####
;	$0101	begin	.end.	$7dff
;			#####	.^^^.
; systic:	$0000	$0000
; 1ms * 256 ($100) window = 256ms max pause interval
; 1ms * 65024 ($fe00) = 65sec max wait interval

systsk:		;system task increments 1.5sec clock and reschedules itself
;	pop		xl
;	pop		xl		;remove systsk return addr; %%% rtn from pause
	ld		tmp1,-X	;clock high byte
	ld		tmp0,-X	;clock low byte
	inc		tmp0	;wrap clock low byte?
	st		X+,tmp0
	brne	systsk3	;n: reschedule task
	inc		tmp1	;y: increment clock high byte
	cpi		tmp1,$e1;midnight (57,600 1.5sec)?
	brne	systsk2	;n: continue count
	ldi		tmp1,0	;y: reset clock
systsk2:
	st		X,tmp1
systsk3:	; === reschedule system task ===
	adiw	X,1	;X= next systic target
	ldi		zl,low(1500)	;execute systsk in 1500 ms
	add		zl,ticl
	st		X+,zl	;save target systic low byte value
	ldi		zl,high(1500)
	adc		zl,tich
	st		X+,zl	;save target systic high byte value
;	=== check for next scheduled chore ===
	ld		zl,X+	;next scheduled cfa low byte
	ld		zh,X+	;next scheduled cfa high byte
	sbrs	zh,7	;active chore (b7 set)?
	ret				;n: exit
;	....			;y: active chore
	ld		tmp2,X+	;next clock target low byte
	cp		tmp2,tmp0
	ld		tmp2,X	;next clock target high byte
	cpc		tmp2,tmp1	;next clock target?
	breq	exe1	;y: execute schedule task cfa (Z)
;	....			;n: not active task
	ret				;check if next task ready

;=>	PAUSE	(--)	switch to next waiting task
	code	5,"pause"
;	changes: X, Z, tmp0, 
pause_:
	wdr				;watchdog reset
	sbrs	stat,tsksbt	;multitask mode?
	ret		;n: exit

;	....	;y: execute next active task
	ldi		zh,high(systsk)	
	ldi		zl,low(systsk)	;system 1.5sec task address
	ldi		xh,high(sysclk+3)	
	ldi		xl,low(sysclk+3)	;1.5sec systic target
	rcall	pause3	;exe 1.5sec task if scheduled

	lds		tmp0,taskID
	sbrc	tmp0,7	;ready task (b7 clear)?
	ret				;n: exit
;	....			;y: a task is ready
	ldi		xh,high(tsklst)	
	ldi		xl,low(tsklst)
	inc		tmp0	;index next task
	andi	tmp0,$3	;mask low index bits ($4 -> $0)
	sts		taskID,tmp0
	lsl		tmp0	;2x index
	add		xl,tmp0	;point to next task in list
	ld		zl,X+	;read task cfa
	ld		zh,X	;Z=task cfa
	sbrc	zh,7	;timed task (b7 set)?
	rjmp	pause2	;y: check if time expired
;	....			;n: not timed task
	sbrc	zh,6	;active task (b6 set)?
	rjmp	pause5	;y: call cfa (Z) of task word
;	....			;n: not active task
pause9:
	ret

pause2:		;*** timed wait ***
	adiw	X,8		;task systic target
pause3:
	ld		tmp0,X	;systic target high byte
	sub		tmp0,tich
	cpi		tmp0,-1	;systic = target +1 (high byte)?
	breq	exe1	;y: call cfa of task word (Z)
	tst		tmp0	;systic = target (high byte)?
	brne	pause9	;n: exit
	ld		tmp0,-X	;systic target low byte
	cp		ticl,tmp0	;systic >= target (low byte)?
	brlo	pause9	;n: exit
pause5:		;y: execute icode/xcode at kfa (Z)
	andi	zh,$3f	;remove kfa timed b7, active b6
	cpi		zh,high(XCOD)/2	;below xcode?
	brge	exe3	;y: exe kfa xcode
	lsl		zl
	rol		zh	;kfa -> cfa
	rjmp	lopadr	;y: exe cfa icode


;=>	?EXEC	(adr --)	execute cfa stored at adr if not zero
	code	5,"?exec"
atexec_:
	rcall	at_		;get cfa
	mov		xl,tosl
	or		xl,tosh	;cfa zero?
	brne	exe_	;n: execute code @ cfa
	rjmp	drop_	;y: discard adr

;=>	EXE	(cfa --)	execute ~ run icode/xcode at cfa
	code	3,"exe"
exe_:
;	rcall	pause_	;outer interpreter, check sysclk
	rcall	tos2z	;Z= (TOS)
exe1:		;execute icode/xcode at Z
	cpi		zh,high(XCOD)	;cfa below xcode?
	brlo	lopadr	;y: icode cfa
exe2:	;n: execute xcode, Z= cfa
	asr		zh
	ror		zl	;cfa -> kfa
exe3:
	ijmp	;return Z->pc

do_case:
	movw	Z,NXT
do_cas1:
	rcall	atz		;fetch X= icode cell
	cpi		xl,$ff	;ELS?
	breq	lopadr	;y: else case
	cp		xl,tosl	;test byte?
	breq	lopadr	;y: same case
	sbrc	xh,3	;LS[ ($78)?
	brlo	lopadr	;y: less case
;	....
	andi	xh,$07	;mask offset
	tst		xh		;last case (zero offset)?
	breq	lop99	;y: end of case block
	lsl		xh		;convert # cells to bytes
	add		zl,xh
	adc		zh,zerol;add offset to Z
	rjmp	do_cas1	;next case

;=>	IWD	(cstr --)	interpret word ~ else try to convert to literal
;::	NAME? ?DUP
;::	IF @ $40 AND
;::	  ABORT" compile ONLY" EXECUTE EXIT
;::	THEN 'NUMBER @EXECUTE IF EXIT THEN THROW ;
	code	3,"iwd"
iwd_:	; ===== OUTER INTERPRETER =====
	rcall	swd_	;srch vocabs for word
	mov		xl,tosl	;(cfa nfa | cstr F)
	or		xl,tosh	;word found?
	brne	iwd1	;y: process word
	rcall	drop_	;n: (cstr)
	rcall	numberq_	;(cstr) number?
	ldi		tmp1,$82;error# not number (or in vocab)
	qjump	syserr	;n: error $82 (cstr)
	ret				;y: put number on dstk

iwd1:		;y: (cfa nfa)
	rcall	at_		;(cfa nfa@) get word type/len byte
	ldi		tmp1,$85;error# compile word during interpret
	sbrc	tosl,cmpbit	;compile word?
	rjmp	syserr	;y: error $03 (cfa)
;	....			;n: interpret word
	rcall	drop_	;(cfa)
	rjmp	exe_	;execute icode/xcode at cfa

do_qexit:
	pop		xh	;discard rtn addr
	pop		xl	;discard rtn addr
	or		tosh,tosl
	rcall	drop_
	breq	lopnxt
lop99:	;--------- end of vector list ($0000)
	pop		zh		;previous ptr
	pop		zl		;previous ptr
	tst		zh		;icode marker?
	brmi	lopadr	;y: previous vector list
	ijmp	;return Z->pc, EXIT INNER INTERPRETER

lop8C:
	sbrc	zh,6	;b6: branch?
	rjmp	loplit	;n: convert literal
;	....
exekfa:
	cpi		zh,$80 | high(XCOD)/2	;kfa < xcode boundary?
	brlo	lop8x	;y: branch to icode
;	andi	zh,$7f	;top bit set so address wrap
	ijmp	;return Z->pc, branch to xcode cell address

lop0x:	;--------- call Z= kfa ($00)
	ori		nxth,$80;mark icode byte addr
	push	nxt		;save byte return address
	push	nxth
lop8x:	;--------- branch Z= kfa ($80)
	lsl		zl
	rol		zh		;*2 kfa -> cfa

; ===== INNER INTERPRETER LOOP =====
lopadr:	;--------- Z= icode cfa
;	push	zl
;	push	zh
;	rcall	pause_	;inner interpreter, check sysclk
;	pop		zh
;	pop		zl
	andi	zh,$7f	;mask icode type bit
	rcall	atz		;X= icode cell
	movw	NXT,Z	;save byte ptr to next icode cell
	movw	Z,X		;Z= X= current icode cell
	wdr			;reset watchdog
	sbrc	zh,7	;branch/short literal?
	rjmp	lop8C	;y: ($80/$c0)
;	....
	sbrc	zh,6	;$4x:$7x opcodes?
	rjmp	loprel	;y: relative opcode
;	....			;n: direct opcode
	or		xl,xh	;end of icode list?
	breq	lop99	;y: previous list
lopcall:			;$00 call opcode
	cpi		zh,high(XCOD)/2	;kfa < xcode boundary
	brlo	lop0x	;y: call icode word ---- ($00)
	icall			;n: call xcode word ---- ($00)
lopnxt:	;------------ next --------------
	movw	Z,NXT	;byte ptr to next icode cell
	rjmp	lopadr	;continue INNER INTERPRETER

;	$0000 - $08ff	icode in SRAM
;	$0900 - $0cff	icode in EEPROM
;	$0d00 - $3fff	icode in flash (ICOD)
;	$4000 - $7fff	xcode in flash, 29 cycles (1.8 usec) (XCOD)

;;;	00kk kkkk kkkk kkkk - call icode/xcode at kfa
;	$00: exec - call icode cell addr < XCOD boundary ($4000)
;	$00: exec - call xcode cell addr > XCOD boundary ($4000)
;;;	01cc cc## #### #### - relative jump # of cells
;	relative branch +/- 511 cells (lower 10 bits, +/- 2**9
;	$40: qdgot - relative branch if true, drop flag
;	$44: bcall - block inline call
;	$48: qkgot - relative branch if true, keep flag
;	$4c:
;	$50: qdgof - relative branch if false, drop flag
;	$54:
;	$58: qkgof - relative branch if false, keep flag
;	$5c:
;	$60: rjump - relative branch (noop: $6000)
;	$64: rexec - relative call
;	$68: rloop - relative LOOP runtime
;	$6c: rnext - relative NEXT runtime
;	$70: casof - CASE EQ[
;	$74:
;	$78: casgf - CASE LS[
;	$7c:
;;;	10kk kkkk kkkk kkkk - jump to icode/xcode at kfa
;	$80: jump - to icode cell addr < XCOD boundary ($4000)
;	$80: jump - to xcode cell addr > XCOD boundary ($4000)
;;;	11## #### #### #### - push short literal on data stack
;	$c0: slit - short literal (8,191 max, -8192 min)

loplit:	;------------ short literal ($c0)
		; -8192 ($2000) to -1 ($3fff) to 8191 ($1fff)
		; 26 cycles, 1.6 usec
	savetos
	movw	TOS,Z
	sbrs	tosh,5	;b5: negative literal?
	andi	tosh,$3f;n: positive lit, clear top bits
;	....
	rjmp	lopnxt

loprel:	;------------------ z= cell addr ($40-$7x)
	sbrc	zh,5	;$60/$70 opcode?
	rjmp	lop67
;	....
	sbrc	zh,2	;$5c/$54/$4c/$44 opcode?
	rjmp	lopfcf4	;y:
;	....
	movw	X,TOS	;$50/$58/$40/$48 opcodes
	sbrs	zh,3	;$40 qdgot, $50 qdgof?
	rcall	drop_	;y: consume flag
;	....
		;------------------ qdjmp/qqjmp ($40/$50) z= cell addr
	or		xl,xh	;true condition?
	sbrs	zh,4	;$40 qdgot, $48 qqgot?
	breq	lopnxt	;y: next icode word
;	....
	sbrc	zh,4	;$50 qdgof, $58 qqgof?
	brne	lopnxt	;y: next icode word
;	....
lop60:	;------------------ rjump ($60) relative branch
	clt		;clear T flag
lop64:	;------------------ rexec ($64) relative call
	andi	zh,radmsk	;isolate relative address $3ff
	sbrc	zh,1	;negative relative address?
	ori		zh,~radmsk	;y: extend bits
;	....
lopinc:
	lsl		zl
	rol		zh		;*2 cell addr -> byte addr
	add		zl,nxt
	adc		zh,nxth	;add relative address to next address
	brtc	lopadr	;jmp to byte address if branch $60
	lsr		zl		;call $64
	ror		zh		;convert to cell address
	rjmp	lopcall

lop6064:	;------------------ rjump ($60) or rexec ($64)
	sbrs	zh,2	;$60 or $64 command?
	rjmp	lop60	;n: relative branch
;	....
	set		;set T flag
	rjmp	lop64	;n: relative call
;	....
lop67:	;------------------ z= cell addr ($60-$7x)
	sbrc	zh,4	;$6x command?
	rjmp	lop99	;n: end of a case block; EQ[ ($70) or LS[ ($78)
;	....
		;------------------ $6x
	sbrs	zh,3	;$60/$64 command?
	rjmp	lop6064	;y: relative branch ($60) or call ($64)
;	....
	sbrs	zh,2	;$6c/$68 command?
	rjmp	lop68	;$68 LOOP
;	....
		;------------------ $6c NEXT (rnext)
	pop		xh
	pop		xl	;fetch index
	sbiw	X,1	;decrement index, zero?
	breq	lopnxt	;y: exit loop	%%% signed or unsigned?
	push	xl
	push	xh	;restore index
	rjmp	lop60	;loop back to FOR

lop68:	;------------------ ($68) LOOP (rloop)
	pop		xh
	pop		xl	;fetch index
	pop		tmp3
	pop		tmp2	;fetch limit
	adiw	X,1		;inc index
	cp		xl,tmp2
	cpc		xh,tmp3	;index (X) > limit {tmp2}?
	brge	lopnxt	;y: exit loop
	push	tmp2
	push	tmp3	;restore limit
	push	xl
	push	xh	;restore index
	rjmp	lop60	;loop back to DO

lopfcf4:	;------------------ $5c/$54/$4c/$44 command
;	sbrs	zh,4	;$4c/$44 command?
;	rjmp	lop5c54	;n: $5c/$54
;	....
		;------------------ $4c/$44
;	sbrs	zh,3	;$4c command?
;	rjmp	lop4c	;y:
;	....
		;------------------ $44 bcall - block inline call
	andi	zh,radmsk	;isolate relative address $3ff
	lsl		zl
	rol		zh		;*2 cell addr -> byte addr
	add		zl,nxt
	adc		zh,nxth	;add relative address to next address
	ori		zh,$80	;mark icode byte addr
	push	zl
	push	zh		;save byte return address
	rjmp	lopnxt

;=>	REP	(adr --)	REPLAY - repeat execution of addressed word until console input
	code	3,"rep"
	movw	Z,TOS
rep1:
	push	zh
	push	zl		;save adr
	rcall	exe1
	rcall	qesc	;ESC entered?
	pop		zl
	pop		zh		;restore adr
	brne	rep1	;n: continue
	rjmp	drop_

;=>	@	(adr -- u)	fetch ~ cell (u) from adr
	code	1,"@"
at_:
	movw	Z,TOS
atztos:
	rcall	atz
	movw	TOS,X
	ret

;=>	!	(n adr --)	store cell (n) at adr
	code	1,"!"
store_:
	rcall	tos2z
	cpi		zh,high(ICOD)	;byte addr > $cff
	brsh	ramerr	;y: error (adr)
	cpi		zh,high(EEPRM)	;byte addr > $8ff (EEPROM)
	brsh	w_eep
	st		Z+,tosl	;write SRAM cell
	st		Z,tosh
	rjmp	drop_

w_eep:		;write EEPROM cell
	rcall	EPwrite	;write tosl=byte in Z= mapped addr
	adiw	Z,1		;Z= adr+1
	rcall	EPwait	;wait for EEPROM ready, adj mapped addr
	out		eedr,tosh	;set Data Reg
	rcall	EPwrit2
	rjmp	drop_

ramerr:
	ldi		tmp1,$41;error# not a ram address
	rjmp	syserr	;y: error (adr)

;=>	C!	(b adr --)	C-store ~ store byte (b) at address (adr)
	code	2,"c!"
cstore_:
	rcall	tos2z
	cpi		zh,high(ICOD)	;byte addr > $cff
	brsh	ramerr	;error (adr)
	cpi		zh,high(EEPRM)	;byte addr > $8ff (EEPROM)
	brsh	wceep
	st		Z,tosl	;write SRAM byte
	rjmp	drop_

wceep:		;write EEPROM byte
	rcall	EPwrite
	rjmp	drop_	;*rb-range*


;=>	2R>	(-- d R: d --)	double-R-from ~ pop double from return stack to data stack
	code	3 +(1<<cmpbit),"2r>"
drfrom_:
	savetos
	pop		zh
	pop		zl	;Z=return addr
	pop		xh
	pop		xl	;X= dl
	st		-Y,xh
	st		-Y,xl	;push X= dl
	rjmp	rfrom2

;=>	R>	(-- n R: n --)	R-from ~ pop return stack to data stack
	code	2 +(1<<cmpbit),"r>"
rfrom_:
	savetos
	pop		zh
	pop		zl	;Z=return addr
rfrom2:
	pop		tosh
	pop		tosl	;TOS from rtn stk
	ijmp	;return Z->pc

;=>	I	(-- n R: n -- n)	copy top of return stack to data stack
	code	1 +(1<<cmpbit),"i"
rat_:	;*** switch to I' code, add J code
	savetos
	pop		zh
	pop		zl	;Z=return addr
	pop		tosh
	pop		tosl
	push	tosl
	push	tosh
	ijmp	;return Z->pc

;=>	>R	(n -- R: -- n)	to-r ~ push data stack to the return stack
	code	2 +(1<<cmpbit),">r"
tor_:
	pop		zh
	pop		zl	;Z=return addr
	push	tosl
	push	tosh	;save hl order on rtn stk
	rcall	drop_
	ijmp	;return Z->pc

;=>	>R@	(n -- n R: -- n)	to-r-fetch ~ copy data stack to the return stack
	code	3 +(1<<cmpbit),">r@"
torat_:
	pop		zh
	pop		zl	;Z=return addr
	push	tosl
	push	tosh	;save hl order on rtn stk
	ijmp	;return Z->pc

;=>	SP@	(-- a)	push current data stack pointer
	code	3,"sp@"
spat_:
	rcall	dup_
	movw	TOS,Y
	ret

;=>	DUP	(a -- a a)	duplicate - push first stack item
	code	3,"dup"
dup_:
	savetos
	ret

;=>	2SWAP	(a b c d -- c d a b)	double-swap ~ exchange top two double stack items
	code	5,"2swap"
dswap_:
	ld		xl,Y+	;n3
	ld		xh,Y+
	ld		zl,Y+	;n2
	ld		zh,Y+
	ld		tmp0,Y+	;n1
	ld		tmp1,Y
	st		Y,xh	;n3
	st		-Y,xl
	st		-Y,tosh	;n4
	st		-Y,tosl
	st		-Y,tmp1	;n1
	st		-Y,tmp0
	movw	TOS,Z	;n2
	ret

;=>	SWAP	(a b -- b a)	exchange top two stack items
	code	4,"swap"
swap_:
	movw	X,TOS
	ld		tosl,Y+
	ld		tosh,Y	;pop TOS-1= TOS
	st		Y,xh
	st		-Y,xl	;push TOS=TOS-1
	ret

;=>	WHILE	(bref -- Arev $50 bref ; t|f --)	conditional branch from BEGIN-WHILE-REPEAT loop
;::	COMPILE IF ROT ; IMMED
	code	5 +(1<<nowbit) +(1<<cmpbit),"while"
	rcall	if_		;(bref Arev $50) allot Arev (REPEAT)
rotjmp:
	rjmp	rot_	;(Arev $50 bref)

;=>	D0<	(d -- t|f)	double-zero-less-than - is double negative?
	code	3,"d0<"
dzless_:
	adiw	Y,2	;remove lower cell of double
	rjmp	zless_

;=>	0<	(n -- t|f)	zero-less-than ~ true if n is negative
	code	2,"0<"
zless_:
	tst		tosh	;n high bit set (negative)?
	movw	TOS,ZERO
	brge	zless1	;n: false
	sbiw	TOS,1
zless1:
	ret

;=>	OVER	(a b -- a b a)	push second stack item
;	1 S@ ; 
	code	4,"over"
over_:
	savetos
	ldd		tosl,Y+2
	ldd		tosh,Y+3
	ret

;=>	UM+	(u1 u2 -- udsum)	u-m-plus ~ add two unsigned single numbers and return double sum
	code	3,"um+"
umplus_:
	ld		xl,Y+
	ld		xh,Y+	;pop u1 from stk
	add		tosl,xl
	adc		tosh,xh
	savetos
	clr		tosh
	clr		tosl
	rol		tosl
	ret


;->	doLIT	(-- n)	push inline literal code constant
;	code	5 +(1<<cmpbit),"doLIT"	;Kernel-interpreter
doLIT:
	savetos
	pop		zh
	pop		zl	;Z=return addr
	rdflash tosl,tosh
	ror		zh
	ror		zl	;2/ byte addr -> cell addr
	ijmp	;return to addr after literal, Z->pc

;->	doVAR	(-- a)	run time routine for VARIABLE and CREATE
;	code	COMPO+5,"doVAR"
doVAR:
	savetos
	pop		zh
	pop		zl	;Z=return addr
	rdflash tosl,tosh
	ret			;rtn to VAR calling routine

;->	next	(--)	run time code for single index loop
;	code	4 +(1<<cmpbit),"next"	;Kernel-interpreter
doNXT:
	pop		zh	;rtn cell addr
	pop		zl
	pop		xh	;fetch index
	pop		xl
	sbiw	X,1	;dec index, zero?
	brge	doNXT1	;n: continue loop
	adiw	Z,1	;y: end loop
	ijmp	;return to addr after branch Z->pc

doNXT1:	
	push	xl	;push index back
	push	xh	
BRAN1:
	rdflash	xl,xh
	movw	Z,X
	ijmp	;return to branch addr Z->pc

;->	?branch	(t|f --)	branch if flag is zero
;	code	COMPO+7,"?branch"	;Kernel-interpreter
QBRAN:
	pop		zh
	pop		zl	;Z=return addr
	or		tosh,tosl
	rcall	drop_
	breq	BRAN1
	adiw	Z,1	;skip next branch addr cell
	ijmp	;return Z->pc


;; ============= System and user variables ==================

;=>	BASE	(-- adr)	location of radix for numeric I/O
	code	4,"base"
base_:
	varable	base
;	ret

;->	tmp	(-- adr)	temporary storage location used in parse and find
;	code	3,"tmp"
tmp_:
	varable	tmp
;	ret

;=>	>IN	(-- adr)	character pointer while parsing TIB
	code	3,">in"
toin_:
	varable	toin
;	ret

;=>	#TIB	(-- adr)	address of number of char in terminal input buffer (TIB)
	code	4,"#tib"
ntib_:
	varable	ntib
;	ret

;=>	'TIB	(-- adr)	address of terminal input buffer (TIB)
	code	4,"'tib"
ttib_:
	varable	ttib
;	ret

;=>	HLD	(-- adr)	hold ~ variable with addr of next free char in numeric output string at pad
	code	3,"hld"
hld_:
	varable	hld
;	ret

;=>	CONTEXT	(-- vadr)	address of search vocabulary
	code	7,"context"
context_:
	varable	context
;	ret

;=>	CURRENT	(-- vadr)	address of vocabulary to add words to
	code	7,"current"
current_:
	varable	current
;	ret

;->	CURR@	(-- nfa)	address of top nf of current vocabulary
;	code	5,"curr@"
curentat:
	rcall	current_
	rjmp	at_	;current vocab

;=>	DP	(-- adr)	dictionary pointer ~ address of free dictionary space
	code	2,"dp"
dp_:
	literal	dp	;dictionary pointer
	rjmp	at_	;current dp

;=>	LAST	(-- nfa)	point to nfa of last word in current vocab
;::	CURRENT @ @ ;
	code	4,"last"
last_:
	rcall	curentat	;(vfa) current vocab address
	rjmp	at_	;nfa of last word in current vocab

;=>	DEF	(--)	definitions ~ set current (def) vocab as context (search) vocab
;::	CONTEXT @ CURRENT ! ;
	code	3,"def"
	rcall	context_
	rcall	at_
	rcall	current_
	rjmp	store_

;; ================ Common functions ====================


;=>	2*	(n -- 2n)	two-star ~ multiply TOS by cell size in bytes
	code	2,"2*"
twox_:
	lsl		tosl
	rol		tosh
	ret

;=>	2/	(n -- n/2)	two-slash ~ divide TOS by two (cell size in bytes)
	code	2,"2/"
twosl_:
	asr		tosh
	ror		tosl
	ret

;=>	ALIGN	(adr -- adr')	align address to cell boundary
	code	5,"align"
align_:
	adiw	TOS,1
	andi	tosl,$fe	;clear lsb
return:
	ret

;=>	BL	(-- #32)	blank ~ put blank character ($20) on TOS
	code	2,"bl"
bl_:
	savetos
	ldi		tosl,' '
	clr		tosh
	ret

;=>	?DUP	(n -- n n | 0)	question-dup ~ duplicate TOS if not zero
	code	4,"?dup"
qdup_:
	adiw	TOS,0	;zero?
	breq	return	;y: leave zero
	rjmp	dup_	;n: dup non zero

;??	: TUCK (n1 n2 -- n2 n1 n2) SWAP OVER ;
;??	: NIP (n1 n2 -- n2) SWAP DROP ;
;??	: UMIN (u u -- u) 2DUP U< IF   BEGIN DROP ;
;??	: UMAX (u u -- u) 2DUP U< UNTIL THEN SWAP DROP ;
;??	: 2>R (n1 n2 --) (R: -- n1 n2)
;		SWAP R>  SWAP >R  SWAP >R  >R ; COMPILE-ONLY
;??	: 2R> (-- n1 n2) (R: n1 n2 --)
;		R>  R> SWAP  R> SWAP  >R  SWAP ; COMPILE-ONLY

;=>	3DROP	(a b c --)	three-drop ~ discard three items on stack
	code	5,"3drop"
tdrop_:
	adiw	Y,4		;remove n1 and n2
	rjmp	drop_	;remove n3

;=>	2DROP	(a b --)	double-drop ~ discard two items on stack
	code	5,"2drop"
ddrop_:
	adiw	Y,2	;remove b
	rjmp	drop_	;remove a

;=>	2DUP	(a b -- a b a b)	double-dup ~ duplicate top two items
	code	4,"2dup"
ddup_:
	rcall	over_	; (n1 n2 n1)
	rjmp	over_	; (n1 n2 n1 n2)
;	ld		tmp2,Y
;	ldd		tmp3,Y+1;get a
;	mov		TMP0,TOS;get b
;	rjmp	rot1	;push a then b	%%% make substitution

;=>	+	(n1 n2 -- n1+n2)	plus ~ add top two items
	code	1,"+"
plus_:
	ld		xl,Y+
	ld		xh,Y+	;X= n2
	add		tosl,xl
	adc		tosh,xh
	ret

;=>	COM	(n -- n')	complement ~ one's complement of TOS
	code	3,"com"
com_:
	com		tosl
	com		tosh	;ones complement
	ret

;=>	NEG	(n -- -n)	negate ~ two's complement of TOS
;::	COM 1+ ;
	code	3,"neg"
neg_:
	com		tosl
	com		tosh	;ones complement
	adiw	TOS,1	;twos complement
	ret

;=>	DNEG	(d -- -d)	double-negate ~ two's complement of top double
;::	COM >R COM 1 UM+ R> + ;
	code	4,"dneg"
dnegate_:
	rcall	com_
	rcall	tor_
	rcall	com_
	literal	1
	rcall	umplus_	;UM+ rtn double
	rcall	rfrom_
	rjmp	plus_

;=>	-	(n1 n2 -- n1-n2)	minus ~ subtraction
	code	1,"-"
sub_:
	ld		xl,Y+
	ld		xh,Y+	;X= n1
	sub		xl,tosl
	sbc		xh,tosh	;n1 (X)- n2 (Y)
	movw	TOS,X
	ret

;=>	ABS	(n -- n')	absolute-value ~ of n
	code	3,"abs"
abs_:
	tst		tosh	;minus?
	brmi	neg_	;y: invert
	ret

;=>	=	(n1 n2 -- t|f)	equal ~ true n1 = n2
;	(1 1 -- -1) (1 3 -- 0)
	code	1,"="
equal_:
	ld		xl,Y+
	ld		xh,Y+	;X= n1
	sub		tosl,xl
	sbc		tosh,xh
	brne	zero1
	sbiw	TOS,1	;0->false
	ret


;=>	FGET:	($ -- )	forget ~ purge word including later defined words
;::	BL WORD SWD		; recover dictionary space of last colon def
;	?DUP IF ERROR THEN NIP DUP 2- @ CURRENT @ ! 2- DP ! ;
	code	5,"fget:"
	rcall	blword	;parse next word in TIB with blank delim
	rcall	current_;(cstr current-adr)
	rcall	forthq	;word found in nonForth vocab?	(cfa nfa | cstr F)
	rcall	qdup_	;(cfa nfa nfa | cstr F)
	ldi		tmp1,$83;error# word not found
	qjump	syserr	;n: error $83 (cstr)
	adiw	Y,2		;NIP (nfa)
	rcall	dup_	;(nfa nfa)
	sbiw	TOS,2	;(nfa lfa)
	rcall	at_		;(nfa nfa') of previous word
	rcall	curentat;(nfa nfa' vocab-adr) current vocab
	rcall	store_	;current vocab points to previous word
	sbiw	TOS,2	;(lfa) top of the selected word definition
savdp:
	rcall	dp_		;(lfa dp)
	rjmp	store_	;reset dictionary (of last colon def) pointer

;=>	ESC?	(-- t|f)	escape-question ~ true if ESC key, else false
	code	4,"esc?"
qesc_:
	rcall	qesc	;ESC (z=1)?
	breq	true
zeros:	;push zero [false]
	savetos
zero1:
;	movw	TOS,ZERO	;does not set zero flag
	clr		tosl
	clr		tosh
	ret			;zero flag set by clr

;=>	KEY?	(-- c t|f)	key-question ~ input character and true, or false if no input
	code	4,"key?"
qkey_:
	savetos
	in_		xl,UCSR0A	;USART Control and Status Register A
	sbrs	xl,RXC0		;USART Receive Complete?
	rjmp	zero1		;n: no serial chr
;	....				;y: chr received
	andi	xl,$1c	;frame, overrun or parity error?
	breq	qkey2	;n: skip counter
	inc		r5		;USART error counter
;	rjmp	zero1	;%%% ignore character if error %%%
qkey2:
	clr		tosh
	in_		tosl,UDR0	;USART I/O Data Register
	rcall	crc8calc_	;compute CRC8 (tosl)
true:	;push -1 [true]
	savetos
true1:
	ldi		tosl,-1
	ldi		tosh,-1
	ret

;=>	C@	(adr -- u)	C-fetch ~ byte to data stack
	code	2,"c@"
cat_:
	movw	Z,TOS
	rcall	catz
	movw	TOS,X
	ret

;=>	U<	(u1 u2 -- t|f)	u-less-than ~ unsigned compare: true if u1 < u2
;	(1 3 -- -1) (1 -3 -- -1) (3 1 -- 0) (-3 1 -- 0)
	code	2,"u<"
uless_:
	ld		xl,Y+
	ld		xh,Y+	;X= u2
	cp		xl,tosl
	cpc		xh,tosh	;u1 < u2?
	brlo	true1	;y: true
	rjmp	zero1

;signed values: brge (greater or equal) and brlt (less than)
;unsigned values: brsh (same or higher) and brlo (lower)

;=>	<	(n1 n2 -- t|f)	less-than ~ signed compare: true if n1 < n2
;	(1 3 -- -1) (1 -3 -- 0) (3 1 -- 0) (-3 1 -- -1)
	code	1,"<"
less_:
	ld		xl,Y+
	ld		xh,Y+	;X= n2
	cp		xl,tosl
	cpc		xh,tosh	;n1 < n2?
	brlt	true1	;y: true
	rjmp	zero1

;	S>D	(n -- d)	single-to-double ~ convert signed single to double
;::	DUP <0 IF -1 ELSE 0 THEN ;
	code	3,"s>d"
s2d_:
	rcall	dup_	;(n n)
	tst		tosh	;(n n) negative?
	brmi	true1	;(n -1)
	rjmp	zero1	;(n 0)

;=>	RANGE	(ul uh u -- t|f)	true if u within range of ul and uh (ul < u < uh)
;	(1 5 5 -- 0) (1 5 3 -- -1) (1 5 1 -- 0) (1 5 -3 -- 0) (-3 5 -2 -- -1)
;::	DUP ROT < -ROT < AND ;
	code	5,"range"
	rcall	dup_	;(ul uh u u)
	rcall	rot_	;((ul u u uh)
	rcall	less_	;(ul u u<uh)
	rcall	negrot_	;(u<uh ul u)
	rcall	less_	;(u<uh ul<u)
	rjmp	and_

;; ============== Division =======================
; remainder carries the sign of the divisor
; quotient is rounded to its arithmetic floor (greatest integer less than or equal to)
; Dividend	divisor	remainder	quotient
;	10			7		3			1
;	-10			7		4			-2
;	10			-7		-4			-2
;	-10			-7		-3			1

;=>	UM/MOD	(ud u -- ur uq)	u-m-divide-mod ~ unsigned divide of double by single; return remainder and quotient
	code	6,"um/mod"
ummod_:
	movw	TMP0,TOS	;save TMP0= u
	ld		tmp2,Y+
	ld		tmp3,Y+	;pop udh in TMP2
	ld		tosl,Y+
	ld		tosh,Y+	;pop udl in TOS
;	---
	ldi		xl,16	;set loop counter
ummod1:
	clr		xh
	lsl		tosl
	rol		tosh
	rol		tmp2
	rol		tmp3	;ud shifted left on bit
	rol		xh		;xh has shifted out bit
;	try subtracting divisor
	cp		tmp2,tmp0
	cpc		tmp3,tmp1
	cpc		xh,zerol
	brcs	ummod3
;	dividend is large enough
;	do subtraction for real and set lowest bit
	inc		tosl
	sub		tmp2,tmp0
	sbc		tmp3,tmp1
ummod3:
	dec		xl
	brne	ummod1
;	---
	st		-Y,tmp3
	st		-Y,tmp2	;push remainder
	ret    ;quotient in TOS

;=>	M/MOD	(d n -- r q)	m-divide-mod ~ signed floored divide of double by single; return remainder and quotient
	code	5,"m/mod"
mslmod_:
	rcall	dup_
	rcall	zless_
	rcall	dup_
	rcall	tor_
	qjump	mslmod1
	rcall	neg_
	rcall	tor_
	rcall	dnegate_
	rcall	rfrom_
mslmod1:	
	rcall	tor_
	rcall	dup_
	rcall	zless_
	qjump	mslmod2
	rcall	rat_
	rcall	plus_
mslmod2:	
	rcall	rfrom_
	rcall	ummod_
	rcall	rfrom_
	qjump	ret3
	rcall	swap_
	rcall	neg_
	rjmp	swap_

;=>	/MOD	(n n -- r q)	divide-mod ~ signed divide; return remainder and quotient
	code	4,"/mod"
slmod_:
	rcall	over_
	rcall	zless_
	rcall	swap_
	rjmp	mslmod_

;=>	%	(n n -- r)	modulo ~ signed divide; return remainder only
	code	1,"%"
	rcall	slmod_
	rjmp	drop_

;=>	/	(n n -- q)	divide ~ signed divide; return quotient only
	code	1,"/"
slash_:
	rcall	slmod_
	rjmp	nip_

;; Multiply

;=>	UM*	(u1 u2 -- ud)	u-m-star ~ unsigned multiply, return double product
	code	3,"um*"
umstar_:
	movw	TMP0,TOS	;save u2
	rcall	drop_
	mul		tosl,tmp0	;u2l*u1l
	movw	Z,R0
	clr		tmp2
	clr		tmp3
	mul		tosh,tmp0	;u2l*u1h
	add		zh,r0
	adc		tmp2,r1
	adc		tmp3,zerol
	mul		tosl,tmp1	;u2h*u1l
	add		zh,r0
	adc		tmp2,r1
	adc		tmp3,zerol
	mul		tosh,tmp1	;u2h*u1h
	add		tmp2,r0
	adc		tmp3,r1
	movw	TOS,Z
	savetos
	movw	TOS,TMP2
	ret

;=>	*	(n1 n2 -- n3)	times ~ signed multiply; return single product
	code	1,"*"
times_:
	rcall	mstar_
	rjmp	drop_

;=>	*/MOD	(n1 n2 n3 -- r q)	star-slash-mod ~ multiply n1 and n2; divide by n3; return mod and quotient
	code	5,"*/mod"
SSMOD:
	rcall	tor_
	rcall	mstar_
	rcall	rfrom_
	rjmp	mslmod_

;=>	*/	(n1 n2 n3 -- q)	star-slash ~ multiply n1 by n2; divide by n3; return quotient only
	code	2,"*/"
STASL:
	rcall	SSMOD
	rjmp	nip_

;; Miscellaneous

;->	>PR	(c -- c')	to-printable ~ non-printing character returned as backtick or caret
;	code	3,">pr"
tochar_:
	cpi		tosl,' ';ctrl char?
	brlo	tochar2	;y: caret
	sbrc	tosl,7	;chr > $7f?
	ldi		tosl,'`';y: backtick
;	....
ret3:
	ret
;	...
tochar2:
	ldi		tosl,'^';y: caret
	ret

;=>	DEPTH	(-- n)	number of cells on data stack
;:: SP@ S0 SWAP - 2 / ;
	code	5,"depth"
depth_:
	literal	SP0-4	;top data stack less 2 cells
	rcall	spat_	;sp@
	rcall	sub_
	rjmp	twosl_	;(topsp - sp) /2

;=>	S@	(nth -- n)	push nth stack item (TOS = 0)
	code	2,"s@"
pick_:
	lsl		tosl	;(2n)
	add		tosl,yl
	adc		tosh,yh	;nth stack address
	rjmp	at_

;	movw	X,TOS
;	ld		tosl,X+
;	ld		tosh,X
;	ret

;=>	ROP	(a b c -- c b a)	Rotate Opposite swaP ~ swap TOS and TOS-2
;	-ROT SWAP ;
	code	3,"rop"
rop_:
	movw	TMP2,TOS;save c
	ld		tmp0,Y+
	ld		tmp1,Y+	;TMP0= b
	ld		tosl,Y+
	ld		tosh,Y+	;TOS= a
	rjmp	rot1

;; Memory access

;=>	+!	(n adr --)	plus-store ~ add n to contents of cell at adr
;::	SWAP OVER @ + SWAP ! ;
	code	2,"+!"
pstore_:
	rcall	tuck_	;(adr n adr)
	rcall	at_
	rcall	plus_	;(adr n+adr@)
pstore1:
	rcall	swap_	;(n+adr@ adr)
	rjmp	store_

;=>	++!	(adr --)	plus-plus-store ~ increment contents of cell at adr
;::	DUP @ 1+ SWAP ! ;
	code	3,"++!"
	rcall	dup_	;(adr adr)
	rcall	at_
	adiw	TOS,1	;(adr adr@+1)
	rjmp	pstore1

;=>	--!	(adr --)	minus-minus-store ~ decrement contents of cell at adr
;::	DUP @ 1- SWAP ! ;
	code	3,"--!"
	rcall	dup_	;(adr adr)
	rcall	at_
	sbiw	TOS,1	;(adr adr@-1)
	rjmp	pstore1

;=>	XCH	(n1 adr -- n2)	exchange ~ contents of cell at adr (n2) with n1
;:: DUP @ -ROT ! ;
	code	3,"xch"
	rcall	dup_	;(n1 adr adr)
	rcall	at_		;(n1 adr n2)
	rcall	negrot_	;(n2 n1 adr)
	rjmp	store_	;(n2)
;	movw	Z,TOS
;	loadtos
;	xch		Z,tosl	;not on 328p
;	adiw	Z,1
;	xch		Z,tosh	;not on 328p
;	ret

;=>	+C!	(n adr --)	plus-c-store ~ add n to contents of byte at adr
;::	SWAP OVER C@ + SWAP C! ;
	code	3,"+c!"
pcstore_:
	rcall	tuck_	;(adr n adr)
	rcall	cat_	;(adr n adr@)
	rcall	plus_	;(adr n+adr@)
	rcall	swap_	;(n+adr@ adr)
	rjmp	cstore_

;=>	INC	(adr -- n)	increment byte at adr and retrieve it
;::	DUP C@ 1 + TUCK SWAP C! ;
	code	3,"inc"
	movw	Z,TOS
	ld		tosl,Z	;retrieve byte
	inc		tosl	;increment byte
	st		Z,tosl	;store byte
	ldi		tosh,0 
	ret

;=>	COUNT	(adr -- adr+1 n)	return count byte of counted string and add 1 to byte address
;::	DUP 1+ SWAP C@ ;
	code	5,"count"
count_:
	rcall	dup_
	adiw	TOS,1
	rcall	swap_
	rjmp	cat_

;=>	HERE	(-- adr)	address of next free byte in current dictionary
;::	DP @ ;
	code	4,"here"
here_:
	rcall	dp_
	rjmp	at_

ramhere:
	literal	ramdp	;ptr free memory in RAM dictionary
	rjmp	at_		;first free byte in RAM dictionary

;=>	PAD	(-- adr)	address of buffer area above dictionary
;::	HERE 64 + ;
	code	3,"pad"
pad_:
	rcall	ramhere	;first free byte in RAM dictionary
	literal	64
	rjmp	plus_	;64 bytes above free memory

;=>	TIB	(-- adr)	address of terminal input buffer
;::	'TIB @ ;
	code	3,"tib"
tib_:
	rcall	ttib_
	rjmp	at_

;=>	CMV	(cstr adr --)	cstring-move ~ copy counted string (cstr) to adr
	code	3,"cmv"
cmv_:
	rcall	over_	;(cstr adr cstr)
	rcall	cat_
	adiw	TOS,2	;(cstr adr cnt+2)
	andi	tosl,$fe	;cell align
	rjmp	cmove_

;=>	UPMOV	(adr1 adr2 u --)	uppercase-move ~ copy u bytes from adr1 to adr2 uppercase
	code	5,"upmov"
	set			;set T bit
	rjmp	cmov0

;=>	CMOV>	(adr1 adr2 u --)	move-byte-forward ~ copy u bytes from adr1 to adr2
;::	for >r count r@ c! r> 1+ next ddrop ;
	code	5,"cmov>"
cmove_:
	clt			;clear T bit; no case change
cmov0:
	rcall	tor_	;save #bytes to move (u)
	rjmp	cmov3
cmov1:
	rcall	tor_	;save adr2 (adr1)
	rcall	count_	;(adr1+1 byte)
	brtc	cmov2	;T clear (no case change)?
	rcall	upc_	;n: uppercase
cmov2:				;y: no case change
	rcall	rat_	;(adr1+1 byte adr2)
	rcall	cstore_
	rcall	rfrom_	;(adr1+1 adr2)
	adiw	TOS,1	;(adr1+1 adr2+1)
cmov3:
	donext	cmov1
	rjmp	ddrop_	;%exit%

;=>	<CMOV	(fr-adr to-adr cnt --)	reverse-byte-move ~ copy cnt bytes from adr1 to adr2
;::	>r@ + swap i + swap r> for >r 1- dup c@ r> 1- tuck c! next ddrop ; [19]
	code	5,"<cmov"
	rcall	torat_	;(fr-adr to-adr R: cnt)
	rcall	plus_
	rcall	swap_
	rcall	rat_
	rcall	plus_
	rcall	swap_	;(fr-adr+cnt to-adr+cnt R: cnt)
cmovr:
	rcall	tor_	;(fr-adr' R: cnt to-adr')
	sbiw	TOS,1
	rcall	dup_	;(fr-adr' fr-adr' R: cnt to-adr')
	rcall	cat_	;(fr-adr' byte R: cnt to-adr')
	rcall	rfrom_
	sbiw	TOS,1	;(fr-adr' byte to-adr' R: cnt)
	rcall	tuck_	;(fr-adr' to-adr' byte to-adr' R: cnt)
	rcall	cstore_	;(fr-adr' to-adr' R: cnt)
	donext	cmovr
	rjmp	ddrop_	;%exit%

;=>	>UP	(c -- c')	convert character to UPper case
	code	3,">up"
upc_:
	cpi		tosl,'a'
	brlo	upcRtn
	cpi		tosl,'z'+1
	brsh	upcRtn
	andi	tosl,$5f
upcRtn:
	ret

tickcomp:
	literal	tick_ | $8000	;set top bit for inner interpreter
	rjmp	comma_	;compile tick call in dictionary


;; Numeric output, single precision

;->	DIGIT	(u -- c)	convert digit u to a character
;::	9 OVER < 7 AND + 48 + ;
;	code	5,"digit"
digit_:
	cpi		tosl,10
	brlo	digit1
	adiw	TOS,7
digit1:
	adiw	TOS,'0'
ret2:
	ret

;->	EXTRACT	(d radix -- d' c)	extract least significant digit from d
;::	0 SWAP UM/MOD SWAP DIGIT ;
;	code	7,"EXTRACT"
extract_:
	rcall	dummod_	;(udqot urem)
	rjmp	digit_	;(udqot c) convert digit to ASCII character
	
;=>	<#	(--)	bracket-number ~ initiate numeric output process in PAD
;::	PAD HLD ! ;
	code	2,"<#"
lesharp_:
	rcall	pad_	;(adr1) pad addr
	rcall	hld_	;(adr1 adr2) hold variable addr
	rjmp	store_	;save pad addr in hold variable

;=>	HOLD	(c --)	insert character into numeric output string
;::	HLD @ 1 - DUP HLD ! C! ;
	code	4,"hold"
hold_:
	rcall	hld_	;(c vadr) ptr to last char in pad
	rcall	at_		;(c adr) adr of last char in pad
	sbiw	TOS,1	;(c adr-1) adr of next free char in pad
	rcall	dup_	;(c adr-1 adr-1)
	rcall	hld_	;(c adr-1 adr-1 vadr) hold variable addr
	rcall	store_	;(c adr-1) update hld variable
	rjmp	cstore_	;put c in pad

basat:	;(-- radix) put current radix on stack
	rcall	base_	;(base-adr)
	rjmp	cat_	;(radix)

;=>	#	(ud -- ud')	number ~ extract digit from u, append digit to output string
;::	BASE @ EXTRACT HOLD ;
	code	1,"#"
sharp_:
	rcall	basat
	rcall	extract_
	rjmp	hold_

;=>	#S	(ud -- d0)	numbers ~ convert u until all digits are added to output string
;::	BEGIN # 2DUP WHILE REPEAT ;
	code	2,"#s"
sharpS_:
	rcall	sharp_	;(d')
	ld		xl,Y
	ldd		xh,Y+1
 	or		xl,xh	;d' zero?
	or		xl,tosl
 	or		xl,tosh	;d' zero?
	brne	sharpS_	;n: cont proc digits
	ret		;y: conversion complete

;=>	SIGN	(d --)	add minus sign to numeric output string if d negative
;::	D0< IF 45 HOLD THEN ;
	code	4,"sign"
sign_:
	rcall	dzless_	;(t|f) is double negative?
	qjump	ret2	;n: exit
	literal	'-'		;y: insert minus sign in pad
	rjmp	hold_

;=>	#>	(d -- adr len)	number-bracket ~ prepare output string to be TYPE'd
;::	2DROP HLD @ PAD OVER - ;
	code	2,"#>"
sharpgt_:
	rcall	ddrop_	;() discard conversion remainder
	rcall	hld_	;(vadr) variable: hold
	rcall	at_		;(adr1) addr of last char in pad
	rcall	pad_	;(adr1 adr2) addr of first char in pad
	rcall	over_	;(adr1 adr2 adr1)
	rjmp	sub_	;(adr1 len)

;=>	str	(d -- adr len)	string ~ convert signed double to numeric string
;::	2DUP 2>R	(save a copy for sign)
;::	DABS		(absolute of n)
;::	<# #S		(convert all digits)
;::	2R> SIGN	(add sign from n)
;::	#> ;		(return string addr and length)
	code	3,"str"
str_:
	rcall	ddup_	;(d d)
	rcall	dtor_	;(d; R:d) save d on return stack
	rcall	dabs_	;(d'; R:d) absolute (unsigned) double
	rcall	lesharp_;(ud; R:d) start number conversion
	rcall	sharpS_	;(d0; R:d) convert each digit
	rcall	drfrom_	;(d0 d) restore d
	rcall	sign_	;(d0; R:d) add sign if negative and decimal radix
	rjmp	sharpgt_;(adr len) addr and len of converted string

;; Numeric input, single precision $3baa

;=>	DGIT?	(c radix -- u t|f)	digit-question ~ convert char to numeric; flag indicates success
;::	>R 48 - 9 OVER <
;::	IF 7 - DUP 10 < OR THEN DUP R> U< ;
	code	5,"dgit?"
dgitq_:
	rcall	tor_	;save radix
	subi	tosl,'0'
	cpi		tosl,10	;digit < 10?
	brlo	dgitq1	;y:
	subi	tosl,'A' -'9' -1	;seven
	cpi		tosl,10	;adjusted digit < 10?
	brge	dgitq1	;n: char between 9 and A (:;<=>@?=)
	ldi		tosh,$ff	;y: make a big number to fail
dgitq1:
	rcall	dup_
	rcall	rfrom_	;fetch radix
	rjmp	uless_

;=>	NUM?	(cstr -- n|d T | cstr F)	number-question ~ convert string to integer in current radix, result flag
;::	BASE @ >R  0 OVER COUNT (txtaddr 0 radix cnt)
;::	OVER C@ $24 =
;::	IF HEX SWAP 1 + SWAP 1 - THEN (txtaddr 0 radix' cnt-)
;::	OVER C@ $2d = >R (a 0 b n)
;::	SWAP R@ - SWAP R@ + (a 0 b" n") ?DUP
;::	IF 1 - (a 0 b n)
;::	 FOR DUP >R C@ BASE @ DIGIT?
;::	  WHILE SWAP BASE @ * +  R> 1 +
;::	 NEXT DROP R@ (b ?sign) IF NEGATE THEN SWAP
;::	  ELSE R> R> (b index) 2DROP (digit number) 2DROP 0
;::	  THEN DUP
;::	THEN R> (n ?sign) 2DROP R> BASE ! ;
	code	4,"num?"

;::	base @ >r 0 0 dov count 0 do dup >r c@ >up base @ dgit?
;::	if >r base @ dum* r> um+ r> 1+
;::	else r> 3drop drop 0 leave then loop drop dip r> base ! ;
numberq_:
	rcall	basat	;(cstr radix) get previous radix
	rcall	tor_	;(cstr; R:radix) save previous radix [R1]
	andi	stat,~(1<<flgsbt)	;clear double number flag
	rcall	zeros	;(cstr 0; R:radix)
	rcall	zeros	;(cstr d0; R:radix) initialize double zero
	rcall	tov_	;(cstr d0 cstr; R:radix)
	rcall	count_	;(cstr d0 cstr+1 cnt; R:radix)
	mov		r12,tosl;save cnt
	rcall	drop_	;(cstr d0 cstr+1; R:radix) remove cnt
	movw	Z,TOS	;(cstr d0 cstr+1; R:radix)
	rcall	catz	;xl= chr from cstr+1
;	---
	cpi		xl,'$'	;hex number?
	breq	numhex	;y: change to hex radix
	cpi		xl,'#'	;dec number?
	breq	numdec	;y: change to decimal radix
	cpi		xl,$27	;'x ASCII conversion?
 	breq	numasc	;y: convert next char
	cpi		xl,'%'	;bin number?
	breq	numbin	;y: change to binary radix
	clt			;clear T bit assuming positive
	cpi		xl,'-'	;negative number?
	breq	numneg	;y: set T bit, change to decimal radix, dcr char
;	----
numq3:		;(cstr d cstr'; R:radix)
	rcall	dup_	;(cstr d cstr' cstr'; R:radix)
	rcall	tor_	;(cstr d cstr; R:radix cstr') save cstr' [R2]
	rcall	cat_	;(cstr d c; R:radix cstr') get next digit
	cpi		tosl,'.';dot char?	;%%% add d input for ,/-: %%%%%
	breq	numdot	;y: double number
	rcall	upc_	;(cstr d c'; R:radix cstr) upper case digit
	rcall	basat	;(cstr d c' radix'; R:radix cstr) get current radix
	rcall	dgitq_	;(cstr d digit t|f; R:radix cstr') c' -> digit?
	qjump	numq6	;n: error, non-digit found, quit
;	---
	rcall	tor_	;(cstr d; R:radix cstr' digit) y: save digit [R3]
	rcall	basat	;(cstr d radix; R:radix cstr' digit) get current radix
	rcall	dumstar_;(cstr d*radix; R:radix cstr' digit) mult by radix
	rcall	rfrom_	;(cstr d*radix digit; R:radix cstr') retrieve digit [R3]
	rcall	dumplus_;(cstr d*radix+digit; R:radix cstr') add digit
numq4:
	rcall	rfrom_	;(cstr d' cstr'; R:radix) retrieve cstr' [R2]
numq5:
	adiw	TOS,1	;(cstr d' cstr+1; R:radix)
	dec		r12		;decrement cnt; digits remain?
	brne	numq3	;y: repeat for next digit
	rcall	drop_	;n: all digits processed (cstr d'; R:radix)
	brtc	numq7	;T clear (negative flag)? y: skip neg
	rcall	dnegate_;(cstr -d'; R:radix) n: negate negative number
numq7:
	rcall	dip_	;(d'; R:radix) remove cstr
	sbrs	stat,flgsbt	;double number?
	rcall	drop_	;n: convert to single (n; R:radix)
;	----			;y: keep as double
numq8:
	rcall	true	;(d|n T; R:radix) success
numq9:
	rcall	rfrom_	;(d|n T radix) get previous radix [R1]
	rcall	base_	;(d|n T radix base-adr)
	rjmp	cstore_	;(d|n T) restore radix; EXIT

numq6:		;(cstr n digit; R:radix cstr') error, non-digit found
	rcall	rfrom_	;(cstr d digit cstr; R:radix) get saved cstr [R2]
	rcall	tdrop_	;(cstr n; R:radix)
	movw	TOS,ZERO;(cstr F; R:radix) false
	rjmp	numq9

numdot:		;(cstr d c; R:radix cstr)
	ori		stat,(1<<flgsbt)	;set double number flag
	rcall	drop_	;(cstr d; R:radix cstr) drop dot char
	rjmp	numq4	;skip dot, point to next digit

numneg:		;(cstr d0 cstr+1; R:radix)
	set		;set T bit indicating negative
numdec:		;(cstr d0 cstr+1; R:radix)
	ldi		tmp0,10	;set decimal radix
	rjmp	numbase	;skip prefix, point to next digit

numasc:		;(cstr d0 cstr+1; R:radix)
	rcall	tdrop_	;(cstr; R:radix)
	adiw	TOS,2	;(cstr+2; R:radix)
	rcall	cat_	;(u; R:radix) ASCII value of char
	rjmp	numq8	;add true, cleanup, exit

numhex:		;(cstr d0 cstr+1; R:radix)
	ldi		tmp0,16	;set hex radix
	rjmp	numbase	;skip prefix, point to next digit

numbin:		;(cstr d0 cstr+1; R:radix)
	ldi		tmp0,2	;set binary radix
numbase:
	sts		base,tmp0
	rjmp	numq5	;skip prefix, point to next digit

;; Basic I/O

;=>	KEY	(-- c)	wait for and return input character
	code	3,"key"
key_:
	rcall	qkey_
	rcall	pause_
	qjump	key_
	ret

;=>	.BL	(--)	dot-blank ~ send blank character to output device
;::	BL EMIT ;
	code	3,".bl"
bldot_:
	ldi		tmp0,' '
	rjmp	cemit

;=>	.#CHR	(n chr --)	dot-sharp-characters ~ send n characters to output device
	code	5,".#chr"
chars_:
	mov		tmp0,tosl	;chr
	rcall	drop_
chars1:
	rcall	cemit	;send (tmp0)
	sbiw	TOS,1
	brne	chars1
	rjmp	drop_

nbl_:
	rcall	bl_
	rjmp	chars_

;=>	.TYPE	(adr cnt --)	type ~ output cnt characters from adr
;::	FOR AFT DUP C@ EMIT 1 + THEN NEXT DROP ;
	code	5,".type"
type_:
	rcall	tor_
	rjmp	type2
;	---
type1:
	rcall	count_
	rcall	tochar_
	rcall	emit_
type2:
	donext	type1
	rjmp	drop_

;=>	.CR	(--)	carriage return ~ output carriage return and line feed
	code	3,".cr"
cr_:
	ldi		tmp0,$d	;carriage return
	rcall	cemit
	ldi		tmp0,$a	;line feed
	rjmp	cemit

;->	$"|	(-- a)	do$ - run time routine compiled by $"; Return address of compiled string
;	code	COMPO+3,'$'
;	.db		2,'"','|'
doSTR:
	rcall	rfrom_	;(rtnadr)
	rcall	rfrom_	;(rtnadr Cstr)
	rcall	dup_
	rcall	dup_	;(rtnadr Cstr Cstr Cstr)
	movw	Z,TOS
	rdflash	tosl,tosh
	clr		tosh	;(rtnadr Cstr Cstr cnt)
	rcall	twosl_
	rcall	plus_	;(rtnadr Cstr Cstr+cnt/2)
	adiw	TOS,1	;(rtnadr Cstr Cstr+cnt/2+1)
	rcall	tor_	;addr after cstr on rtnstk
	rcall	swap_	;(Cstr rtnadr)
	rcall	tor_	;restore doSTR rtnadr
	rjmp	twox_	;Cstr -> cstr (cstr)

;->	."|	(--)	run time routine of ."; Output compiled string
;	code	COMPO+3,'.'
;	.db		'"','|'
doMSG:
	rcall	doSTR	;(cadr)
dotmsg:		;(cstr --)
	rcall	count_	;(adr n)
;	clr		tosh	;limit string length to 255 char	
	rjmp	type_

;=>	D.R	(d col --)	double-dot-r ~ display double in field of n columns; right justified
	code	3,"d.r"
	rcall	tor_	;(d; R:col) save column
	rjmp	dotR1

;=>	.R	(n col --)	dot-r ~ display integer in field of n columns; right justified
;::	>R str			(convert n to a number string)
;::	R> OVER - SPACES(print leading spaces)
;::	TYPE ;			(print number in +n column format)
	code	2,".r"
	rcall	tor_	;(n; R:col) save column
	rcall	s2d_	;(ud; R:col) make single -> double
dotR1:
	rcall	str_	;(adr len; R:col) convert signed double to numeric string
dotR2:
	rcall	rfrom_ ;(adr len col) retrieve column
	rcall	over_	;(adr len col len)
	rcall	sub_	;(adr len col-len)
	rcall	nbl_	;(adr len) output calculated spaces
	rjmp	type_

;=>	U.R	(u col --)	u-dot-r ~ display unsigned integer in field of n columns; right justified
;::	>R				(save column number)
;::	<# #S #> R>		(convert unsigned number)
;::	OVER - SPACES	(print leading spaces)
;::	TYPE ;			(print number in +n columns)
	code	3,"u.r"
UdotR_:
	rcall	tor_	;(u; R:col) save column
	rcall	zeros	;(ud; R:col) make single -> double
	rcall	lesharp_;(ud; R:col)
	rcall	sharpS_	;(d0; R:col)
	rcall	sharpgt_;(adr len)
	rjmp	dotR2

;=>	U.	(u --)	u-dot ~ display unsigned integer in free format
;::	0 <# #S #>	(convert unsigned double)
;::	SPACES		(print leading space)
;::	TYPE ;		(print number)
	code	2,"u."
udot_:
	rcall	zeros	;(ud) make single -> double
uddot_:
	rcall	lesharp_;(ud)
	rcall	sharpS_	;(d0)
	rcall	sharpgt_;(adr len)
udot2:
	rcall	qdstk_	;(adr len) check stack overflow
	rcall	bldot_	;(adr len) output one space
	rjmp	type_

;=>	.	(n --)	dot ~ display integer in free format, followed by a space
;::	BASE @ 10 XOR		(if not in decimal mode)
;::	IF U. EXIT THEN		(print unsigned number)
;::	str SPACE TYPE ;	(print signed number if decimal)
	code	1,"."
dot_:
	lds		xl,base	;(n)
	cpi		xl,10	;decimal?
	brne	udot_	;n: unsigned display
	rcall	s2d_	;make single -> double
dot2:
	rcall	str_	;y: signed display
	rjmp	udot2

;=>	D.	(d --)	"double dot" - display double in free format, followed by a space
;::	TUCK DABS <#  #S ROT SIGN  #>  TYPE SPACE ;
	code	2,"d."
	lds		xl,base	;(n)
	cpi		xl,10	;decimal?
	brne	uddot_	;n: unsigned display
	rjmp	dot2

;=>	UD.	(d --)	display unsigned double in free format, followed by a space
	code	3,"ud."
	rjmp	uddot_

;=>	?	(adr --)	display contents of cell
;::	@ . ;
	code	1,"?"
	rcall	at_
	rjmp	dot_

;; --------------------- Parsing ----------------------------

;=>	PARSE	(adr1 cnt1 c -- adr2 cnt2 delta ; <string>)	scan string (adr cnt) delimited by c; return found string (adr2 cnt2) and its offset
;::	tmp !  OVER >R  DUP (adr1 cnt1 cnt1)
;::	IF 1 -  tmp @ BL =
;::	 IF b u' 'skip'
;::	  FOR BL OVER C@ - 0< NOT  WHILE 1 +
;::	  NEXT ( b) R> DROP 0 DUP EXIT all delim
;::	    THEN  R>
;::	 THEN OVER SWAP b' b' u' 'scan'
;::	 FOR tmp @ OVER C@ -  tmp @ BL =
;::	  IF 0< THEN WHILE 1 +
;::	 NEXT DUP >R  ELSE R> DROP DUP 1 + >R
;::	     THEN OVER -  R>  R> - EXIT
;::	THEN (adr1 cnt1) OVER R> - ;
	code	5,"parse"
parse_:
	rcall	tmp_	;(adr1 cnt1 delim tmp_)
	rcall	cstore_	;save delim
	rcall	over_	;(adr1 cnt1 adr1)
	rcall	tor_	;save adr1
	rcall	dup_	;(adr1 cnt1 cnt1)
	qjump	parse8	;string len zero?
	sbiw	TOS,1	;n: (adr1 cnt1-1)
	rcall	tmp_	;(adr1 cnt1-1 tmp_)
	rcall	cat_	;(adr1 cnt1-1 delim)
	rcall	bl_
	rcall	equal_	;delim is blank?
	qjump	parse3	;n: don't skip leading blanks
	rcall	tor_	;y: index= cnt1-1 (adr1)
parslp1:		;---- skip leading blanks (adr)
	rcall	bl_
	rcall	over_	;(adr blk adr)
	rcall	cat_	;(adr blk adr@)
	rcall	sub_	;(adr blk-adr@)
	rcall	zless_	;0> blk or ctrl chr?
	rcall	com_
	qjump	parse2	;non blank chr
	adiw	TOS,1	;(adr+1)
	donext	parslp1	;skip next blank
	rcall	rfrom_	;(adr adr1)
	rcall	drop_
	rcall	zeros	;(adr 0)
	rjmp	dup_	;(adr 0 0) all blanks %exit%

parse2:
	rcall	rfrom_	;remaining chr cnt (adr cnt)
parse3:
	rcall	over_	;(adr cnt adr)
	rcall	swap_	;(adr adr cnt)
;	rcall	ovap_	;(adr adr cnt) %%%% replace over/swap
	rcall	tor_	;index= cnt (adr adr)
parslp2:
	rcall	tmp_
	rcall	cat_	;(adr adr delim)
	rcall	over_	;(adr adr delim adr)
	rcall	cat_	;(adr adr delim adr@)
	rcall	sub_	;(adr adr delim-adr@)
	rcall	tmp_
	rcall	cat_	;(adr adr delim-adr@ delim)
	rcall	bl_		;(adr adr delim-adr@ delim blk)
	rcall	equal_	;delim = blk?
	qjump	parse5	;n:
	rcall	zless_	;y: include ctrl chr
parse5:
	qjump	parse6	;delim found?
	adiw	TOS,1	;n: (adr adr+1)
	donext	parslp2	;next string chr?
	rcall	dup_	;n: (adr adr+1 adr+1)
	rcall	tor_	;(adr adr+1)
	rjmp	parse7

parse6:
	rcall	rfrom_	;(adr adr adr1)
	rcall	drop_	;(adr adr)
	rcall	dup_	;(adr adr adr)
	adiw	TOS,1	;(adr adr adr+1)
	rcall	tor_	;(adr adr)
parse7:
	rcall	over_	;(adr adr adr)
	rcall	sub_	;(adr cnt)
	rcall	rfrom_	;(adr cnt adr+1)
	rcall	rfrom_	;(adr cnt adr+1 adr1)
	rjmp	sub_	;(adr cnt offset) %exit%

parse8:		;zero length string
	rcall	over_	;(adr1 cnt1 adr1)
	rcall	rfrom_	;(adr1 cnt1 adr1 adr)
	rjmp	sub_	;(adr1 cnt1 0) %exit%

;=>	NXTIB	(c -- adr cnt ; <string>)	next TIB word ~ scan TIB, return string (adr cnt) delimited by c
;::	>R TIB >IN @ + #TIB @ >IN @ - R> parse >IN +! ;
	code	5,"nxtib"
nxtib_:
	rcall	tor_	;save delim
	rcall	tib_	;TIB addr
	rcall	toin_
	rcall	cat_	;(tib >in)
	rcall	plus_	;unparsed TIB pointer: badr= tib+ >in
	rcall	ntib_
	rcall	cat_	;(badr #tib)
	rcall	toin_
	rcall	cat_	;(badr #tib >in)
	rcall	sub_	;unparsed TIB len: bcnt= #tib- >in
	rcall	rfrom_	;restore delim (badr bcnt delim)
	rcall	parse_	;(adr cnt delta)
	rcall	toin_
	rjmp	pcstore_;add delta to >IN (adr cnt)

;=>	.(	($ --)	dot-paren ~ output following string up to next parenthesis 
	code	2 +(1<<nowbit),".("
	literal	')'
	rcall	nxtib_
	rjmp	type_

;=>	(	($ --)	paren ~ ignore following text till next parenthesis
	code	1 +(1<<nowbit),"("
paren_:
	literal	')'
skipTIB:
	rcall	nxtib_
	rjmp	ddrop_

;=>	\	($ --)	backslash ~ ignore following text till end of line
	code	1 +(1<<nowbit), '\\'
	rcall	zeros	;set delimiter character to zero
	rjmp	skipTIB

;=>	WORD:	($ c -- nfa)	parse word delim by c from TIB, copy to RAM dict; nfa of future word?
;::	PARSE HERE PACK$ ;
	code	5,"word:"
word_:
	rcall	nxtib_	;rtn ptr to word string (adr cnt)
word1:
	rcall	herep2	;skip link field (adr cnt dict+2)
	rcall	ddup_	;(adr cnt dict+2 cnt dict+2)
	rcall	cstore_	;store word length in dict
	rcall	ddup_	;(adr cnt dict+2 cnt dict+2)
	rcall	plus_	;(adr cnt dict+2 dict+cnt+2)
	adiw	TOS,1	;(adr cnt dict+2 dict+cnt+3)
	rcall	zeros
	rcall	swap_	;(adr cnt dict+2 0 dict+cnt+3)
	rcall	cstore_	;store zero at end of string
	adiw	TOS,1	;(adr cnt dict+3)
	rcall	swap_	;(adr dict+3 cnt)
	rcall	cmove_	;copy nf to dict
herep2:
	rcall	ramhere	;RAM dict free space
	adiw	TOS,2	;skip lfa, point to nfa
	ret

blword:		;BL WORD with length capped
	rcall	bl_
	rcall	nxtib_	;rtn ptr to word string (adr cnt)
	literal	31		;max word length
	rcall	min_	;(adr cnt)
	rjmp	word1

tos2z:
	movw	Z,TOS
	rjmp	drop_

;=>	DROP	(n --)	discard top stack item
	code	4,"drop"
drop_:
;	loadtos
	ld		tosl,Y+	;pop l
	ld		tosh,Y+	;pop h
	ret

;=>	EVAL	(--)	evaluate ~ interpret/compile words in TIB
;::	BEGIN BL WORD DUP C@
;::	WHILE 'EVAL @EXECUTE ?STACK
;::	REPEAT DROP PROMPT ;
	code	4,"eval"
eval_:
	rcall	qdstk_	;check data stack over/under flow
	rcall	blword	;parse next word in TIB to dict
	movw	Z,TOS	;(nfa)
	ld		xl,Z	;next TIB char
	tst		xl		;input stream empty?
	breq	drop_	;y: drop cstr, %exit%
	sbrc	stat,cmpsbt	;compile mode?
	rjmp	eval2	;y: compile
;	....
	rcall	iwd_	;n: interpret cstr
	rjmp	eval_

eval2:
	rcall	cwd_	;compile cstr
	rjmp	eval_
	
;->	DSTK?	(--)	abort if data stack over/underflow
;	code	5,"DSTK?"
qdstk_:
	rcall	depth_
	tst		tosh
	brne	qdstk1	;underflow if minus, overflow if nonzero
	cpi		tosl,60	;60 cells -> overflow?
	brlt	drop_	;n: ok %exit%
qdstk1:
	ldi		tmp1,$42;error# data stack under/overflow
	rjmp	syserr	;y: error $42 (stk-depth)

;=>	QUERY	(--)	accept input stream to terminal input buffer
;::	TIB 80 'EXPECT @EXEC #TIB !  DROP 0 >IN ! ;
	code	5,"query"
query_:
	rcall	tib_	;(tibaddr)
	literal	tibmax	;(tibaddr tibmax)
	clr		crc8	;init crc (%%% vary for security)
	rcall	tin_	;fill TIB (tibaddr cnt)
query2:		
	rcall	ntib_	;(adr cnt ntib-addr)
	rcall	cstore_	;save #TIB length (addr)
	rcall	drop_	;(~)
ztoin:
	sts		toin,zerol	;zero TIB ptr (>IN)
	ret

;; ================= Dictionary search =======================

;=>	'	($ -- cfa)	tick ~ search content/context/forth vocabularies for next word in input stream
	code	1 +(1<<nowbit),"'"
	sbrc	stat,cmpsbt	;compile mode?
	rjmp	tickcomp;y: compile special case
;	....			;n: 
tick_:
	rcall	blword	;parse next word in TIB with blank delim
	rcall	swd_	;?defined (cfa nfa | cstr F)
	ldi		tmp1,$84;error# word not defined
	qjump	syserr	;n: error (cstr)
	ret				;y: cfa


;=>	[']	($ --; -- cfa)	bracket-tick ~ make literal of cfa of next word
;::	[ ' <word> ] LIT ;
	code	3 +(1<<nowbit),"[']"
	rcall	tick_
	rjmp	lit_

;=>	N>CFA	(nfa -- cfa)	nfa-to-cfa ~ code address of name address (nfa)
;::	CELL- CELL- @ ;
	code	5,"n>cfa"
nameto_:
	rcall	count_	;get ptr to string and string cnt
	andi	tosl,$1f	;mask word attributes
	rcall	plus_
	rjmp	align_

;->	find	(cstr nfa0 -- cfa nfa | cstr F)	search vocabulary (nfa0) for counted string (cstr); return cfa and nfa if success
;::	SWAP              va a
;::	DUP C@ 2 / tmp !  va a  get cell count
;::	DUP @ >R          va a  count byte & 1st char
;::	CELL+ SWAP        a' va
;::	BEGIN @ DUP       a' na na
;::	 IF DUP @ [ =MASK ] LITERAL AND  R@ XOR ignore lexicon bits
;::	  IF CELL+ -1 ELSE CELL+ tmp @ SAME? THEN
;::	 ELSE R> DROP EXIT
;::	 THEN
;::	WHILE CELL- CELL- a' la
;::	REPEAT R> DROP SWAP DROP CELL-  DUP NAME> SWAP ;
	code	3,"fnd"
fnd_:
	rcall	swap_	;(nfa cstr)
	rcall	dup_	;(nfa cstr cstr)
;	rcall	swup_	;(nfa cstr cstr) %%%% replace swap/dup
	rcall	cat_	;(nfa cstr cnt)
	andi	tosl,$1f;mask word attributes
	rcall	tmp_
	rcall	cstore_	;temp= word length

	rcall	dup_	;(nfa cstr cstr)
	rcall	at_		;(nfa cstr chrcnt)
	rcall	tor_	;(nfa cstr; R: chrcnt)
	adiw	TOS,2	;(nfa cstr+2; R: chrcnt)
	rcall	swap_	;(cstr+2 nfa; R: chrcnt)
find1:
	rcall	dup_	;(cstr+2 nfa nfa)
	qjump	find6	;nfa == 0? y:end of vocab
	rcall	dup_	;(cstr+2 nfa nfa)
	rcall	at_		;dict word len/firstChr
	andi	tosl,$1f	;mask word attributes
	rcall	rat_	;(cstr+2 nfa chrcnt' chrcnt)
	rcall	xor_	;len/firstChar match?
	qjump	find2	;y: compare rest
	adiw	TOS,2
	rjmp	find3	;(cstr+2 nfa+2)

find2:
	adiw	TOS,2	;(cstr+2 nfa+2)
	rcall	tmp_
	rcall	cat_	;(cstr+2 nfa+2 len)
	rcall	sameq	;rest of dict word match?
	qjump	find9	;y:
find3:
	sbiw	TOS,4	;(cstr+2 nfa-2); lfa=nfa-2
	rcall	at_		;(cstr+2 nfa'); get next nfa
	rjmp	find1

find6:		;(cstr+2 0; R: chrcnt); end of vocab, no match
	rcall	rfrom_	;(cstr+2 0 chrcnt)
	rcall	drop_	;(cstr+2 0)
	rcall	swap_	;(0 cstr+2)
;	rcall	drwap_	;(0 cstr+2) %%%% replace drop/swap
	sbiw	TOS,2
	rjmp	swap_	;(cstr F)

find9:		;(cstr+2 nfa+2; R: chrcnt) found match
	rcall	rfrom_	;(cstr+2 nfa+2 chrcnt)
	rcall	drop_	;(cstr+2 nfa+2)
	rcall	swap_	;(nfa+2 cstr+2)
;	rcall	drwap_	;(nfa+2 cstr+2) %%%% replace drop/swap
	rcall	drop_
	sbiw	TOS,2	;(nfa)
	rcall	dup_
	rcall	nameto_	;nfa -> cfa
	rjmp	swap_	;(cfa nfa)

;=>	SWD	(cstr -- cfa nfa | cstr F)	search word ~ search current, context, forth vocabs for cstr
;::	BEGIN R>  CELL+ DUP >R @ ?DUP
;::	WHILE find ?DUP
;::	UNTIL R> DROP EXIT THEN R> DROP  0 ;
	code	3,"swd"
swd_:
	rcall	current_;(cstr current-adr)
	rcall	forthq	;word found in vocab?
	brne	swdret	;y: return, z=0
	rcall	drop_
	rcall	context_;(cstr context-adr)
	rcall	forthq	;word found in vocab?
	brne	swdret	;y: return, z=0
	ldi		tosh,high(forth)
	ldi		tosl,low(forth)	;(cstr forth-adr)
swd2:
	rcall	at_		;nfa of vocab first word
	adiw	TOS,1	;nfa = $ffff (erased flash)?
	breq	swdret	;y: z=1, not found
	sbiw	TOS,1	;restore TOS
	rjmp	fnd_	;n: srch vocab

forthq:		;(cstr vadr -- cfa nfa nfa | cstr F)
	rcall	at_		;(cstr vadr@)
	literal	forth
	rcall	over_	;(cstr vadr@ forth:vadr vadr@)
	rcall	xor_	;forth vocab?
	qjump	zero1	;y: skip srch; z=1 (cstr F)
	rcall	swd2	;srch vocab
	mov		tmp0,tosl	;(cfa nfa | cstr F)
	or		tmp0,tosh	;word found in vocab?
swdret:
return3:
	ret				;y: z=0; n: z=1

;->	SAME?	(adr1 adr2 cnt -- adr1 adr2 t|f)
;	Compare cnt bytes in two strings; 0 if same
;::	FOR AFT OVER R@ CELLS + @
;::	  OVER R@ CELLS + @ - ?DUP
;::	IF R> DROP EXIT THEN THEN
;::	NEXT 0 ;
;	code	5,"SAME?"
sameq:
	rcall	twosl_	;cnt/2
	rcall	tor_	;save cnt/2
same1:
	donext	same2
	rjmp	zeros	;strings match
same2:
	rcall	over_	;(adr1 adr2 adr1)
	rcall	rat_	;(adr1 adr2 adr1 cnt/2)
	rcall	twox_	;(adr1 adr2 adr1 cnt)
	rcall	plus_	;(adr1 adr2 adr1+cnt)
	rcall	at_		;next buffer cell (adr1)
	rcall	over_	;(adr1 adr2 c1)
	rcall	rat_
	rcall	twox_	;double
	rcall	plus_
	rcall	at_		;next dict cell (adr2)
	rcall	sub_	;cells match?
	rcall	qdup_
	qjump	same1	;y: compare next cell
	rcall	rfrom_
drop01:
	rjmp	drop_

hxcel:
	rcall	hxbyt	;get msb of <addr>
	mov		zh,zl
hxbyt:
	rcall	hxnib
	swap	tmp0	;xchg nibbles
	mov		zl,tmp0
	rcall	hxnib
	or		zl,tmp0
hexrtn:
	ret

hxnib:		;convert ASCII hex character to binary nibble
	rcall	key
	subi	tmp0,'0'
	cpi		tmp0,10	;digit < 10?
	brlo	hexrtn	;y: valid digit
	subi	tmp0,'A' -'9' -1
	cpi		tmp0,10	;adjusted digit < 10?
	brlo	hxnib9	;y: invalid digit
	cpi		tmp0,16	;adjusted digit < 16?
	brlo	hexrtn	;y: valid digit
hxnib9:
	ldi		tmp1,$04;error# invalid hex digit
	rjmp	syserr	;n: error $04 (tmp0: invalid digit)

qcrc8:
	mov		tmp0,crc8	;save current crc
	rcall	hxbyt	;get msg crc
	cp		tmp0,zl	;crc match?
	breq	hexrtn	;y: exit
	rcall	key		;n: eat unitID
	ldi		tmp1,$05;error# invalid crc
	rjmp	syserr	;n: error $05 (tmp0: in crc; zl: msg crc)

backsp:				;(addr eob ptr)
	rcall	tov_	;(addr eob ptr addr)
	rcall	over_	;(addr eob ptr addr ptr)
	rcall	xor_	;(addr eob flag) ptr == addr?
	qjump	tin1	;y: at beginning of buffer
	sbiw	TOS,1	;n: back up ptr
	rjmp	tin1	;n: (addr eob ptr-1)
	
;=>	TIN	(adr maxcnt -- adr cnt)	terminal-input ~ accept chr to buffer addr (TIB), return chr count
;::	OVER + OVER
;::	BEGIN 2DUP XOR
;::	WHILE KEY DUP BL - 95 U<
;::	  IF TAP ELSE 'TAP @EXECUTE THEN
;::	REPEAT DROP OVER - ;
	code	3,"tin"
tin_:		;(adr maxcnt)
	rcall	over_
	rcall	plus_
	rcall	over_	;(adr adr+maxcnt adr)
tin1:		;(adr adr+maxcnt ptr)
	rcall	ddup_	;(adr adr+maxcnt adr adr+maxcnt adr)
	rcall	xor_	;(adr adr+maxcnt ptr t|f) adr+maxcnt = ptr?
	qjump	tinqit	;y: quit
	rcall	key	;n: terminal input
;	rcall tolower if convert uppercase input
	cpi		tmp0,$a	;LF: line feed?
	breq	tinqit	;y: quit
	cpi		tmp0,$1b;ESC: escape?
	breq	dstkrst	;y: quit, reset data stack
	cpi		tmp0,$08;BS: backspace?
	breq	backsp
	cpi		tmp0,$12;ctrl-R: repeat?
	breq	repeatib
	cpi		tmp0,$0d;CR: carriage return?
	breq	tin8	;y: exit (process TIB)
tin2:			;(addr eob ptr)
;	sbrc	stat,ecosbt	;echo mode?
;	rcall	cemit	;y: echo chr
	movw	Z,TOS	;(addr eob ptr)
	st		Z+,tmp0	;save chr in buffer *****
	movw	TOS,Z	;(addr eob ptr+1)
	rjmp	tin1

;	Half Duplex Message Protocol *** status register: R21
;	NID - No ID Interactive Mode; qitsbt=0, mncsbt=0; 0 #21 c!
;	MID - Multi ID Interactive Mode; qitsbt=0, mncsbt=1; 4 #21 c!
;	MCD - Multi ID Command Mode; qitsbt=1, mncsbt=1; 3 #21 c!
;	MCR - Multi ID CRC Mode; qitsbt=1, mncsbt=0; 2 #21 c!

muid:		;multi unitID (MID or MCD or MCR) mode
	sbrs	stat,mncsbt	;multi unitID (MCR) mode?
	rcall	qcrc8	;y: check crc
;	....
	rcall	key		;tmp0= unitID
	lds		xl,uid	;y: fetch local xl= unitID
	cp		tmp0,xl	;unitID match?
	breq	tin9	;y: eval TIB
	cpi		tmp0,'@';unitID global match?
	breq	tin9	;y: eval TIB
tinqit:		;quit tin, no TIB eval
	rcall	tdrop_	;clear TIN parms
	rjmp	quit_	;resets rtn stk so rtn addr ignored

tin8:		;CR encountered
	mov		tmp1,stat
	andi	tmp1,(1<<qitsbt) | (1<<mncsbt)	;multi unitID mode?
	brne	muid	;y: process multi unitID cmd
tin9:		;exit TIN, eval TIB (adr adr+maxcnt adr+cnt)
	adiw	Y,2		;NIP (adr adr+cnt)
	rcall	over_	;(adr adr+cnt adr)
	rjmp	sub_	;(adr cnt)

;	$03 iwd - compile word during interpret (cfa)
;	$04 invalid hex digit (tmp0: invalid digit)
;	$05 invalid crc (tmp0: in crc; zl: msg crc)
;	$06 hed - null input; (nfa) zero length word
;	$07	semicolon - compile stack inbalance
; system  (number) error codes
;	$41 ! and c! - (adr) not a ram address
;	$42 (stk-depth) data stack under/overflow
; system (cstr) error codes
;	$81	cdw - compile undefined word
;	$82	iwd - interpret undefined word
;	$83 fget: - word not found
;	$84 tick (') - word not found
syserr:			; system error handler tmp1= error_code
	literal	erradr
	rcall	atexec_	;zero handler?
	rjmp	dstkrst	;y: reset data stack

errmsg:
	sbrc	stat,qitsbt	;msg quiet mode?
	rjmp	dstkrst	;y: skip error info
;	....			;n display error info
	mov		xl,tmp1
	rcall	dothex2	;display error code
	rcall	doMSG
	.db		3,"::>"
	sbrc	tmp1,7	;error (cstr)?
	rcall	dotmsg	;y: output cstr
;	....
	sbrc	tmp1,6	;error (n)?
	rcall	dothex_	;y: output n
;	....
	rcall	cr_
	rcall	toin_	;addr of >IN
	rcall	cat_	;(>IN)
	adiw	TOS,4	;include stack depth/#base/unitID/prompt chr
	rcall	nbl_	;output >IN spaces
	ldi		tmp0,'^'	;caret
	rcall	cemit
dstkrst:
	rcall	clr_	;reset data stack
	rjmp	quit_

repeatib:	;repeat TIB (ctrl-R)
			;(adr adr+maxcnt adr)
	rcall	tin9	;(adr cnt)
	rcall	query2	;save cnt in ntib (--)
repeatib2:
	rcall	eval_	;parse words in TIB *****
	rcall	ztoin	;reset TIB pointer (IN>)
	rcall	qesc	;escape key?
	brne	repeatib2
	rjmp	quit_

;=>	BOV	(--)	buffer-over ~ repeat evaluation of TIB until ESC
	code	3,"bov"
	rcall	qesc
	breq	quit_
	rjmp	ztoin	;zero >IN

;=>	QUIT	(--)	reset return stack pointer and start text outer interpreter
	code	4,"quit"
quit_:
	ldi		xl,low(RP0)
	out_	SPL,xl
	ldi		xh,high(RP0)
	out_	SPH,xh	;reset rtn stk ptr
	literal	tib		;(tibaddr)
	rcall	ttib_	;(tibptr tibaddr)
	rcall	store_	;init TIB ptr
	andi	stat,~(1<<cmpsbt)	;clear compile mode (interpret)
	mov		xl,stat
	andi	xl,(1<<qitsbt) | (1<<mncsbt)	;msg NID No ID Interactive Mode?
	breq	quit3	;y: verbose outer interpreter
;	----			;n: also MID when not addressed

mqtlop:		;message (MCD/MCR) loop --- QUIET OUTER INTERPRETER ---
	rcall	query_	;get terminal input in TIB *****
	cpi		tmp0,'@';[over_ and sub_ preserve tmp0] unitID global match?
	breq	quit4	;y: silent mode, multi IDs can't talk together
	clr		crc8	;n: init crc for response (%%% vary for security)
	rcall	eval_	;parse words in TIB *****
	mov		tmp1,crc8	;save current crc
;	****	send tail of MCD/MCR response
	ldi		tmp0,$a	;line feed
	rcall	cemit	;send LF, response message
	mov		xl,tmp1	;xl= saved crc8
	sbrs	stat,mncsbt	;multi unitID (MCR) mode?
	rcall	dothex2	;y: send crc
;	....
	lds		tmp0,uid	;fetch local unitID
	rcall	cemit	;send unitID
quit2:
	sbrc	stat,qitsbt	;msg quiet mode?
	rjmp	mqtlop	;y: quiet outer interpreter
;	....	n: verbose outer interpreter

quit3:		;--- VERBOSE OUTER INTERPRETER ---
	rcall	cr_
	rcall	depth_
	rcall	udot_	;display stack depth
	ldi		tmp0,'#';decimal id
	lds		xl,base
	sbrc	xl,4	;hex base?
	ldi		tmp0,'$';y: hex id
;	....
	sbrc	stat,cmpsbt	;compile mode?
	ldi		tmp0,'+';y: compile id
;	....
	rcall	cemit	;output #/$/+
	lds		tmp0,uid	;fetch local unitID
	sbrc	stat,mncsbt	;multi interactive mode (MID)?
	rcall	cemit	;y: echo unitID
;	....
	ldi		tmp0,'}';prompt chr
	rcall	cemit
	rcall	query_	;get terminal input in TIB *****
	rcall	doMSG
	.db		3," ->"
quit4:
	rcall	eval_	;parse words in TIB *****
	rjmp	quit2	;continue --- OUTER INTERPRETER

;=>	DFER:	($ --)	defer ~ indirect execution of word (non ram dict)
;::	BL WORD DUP SWD IF 
	code	5,"dfer:"
	rcall	blword	;parse next TIB word name (- nfa)
	rcall	dup_	;(cstr cstr)
	rcall	swd_	;?defined (cfa nfa | cstr F)
	breq	defer1	;word not found
	rcall	swap_	;use $0 for cfa of undefined word
defer1:
	rcall	drop_	;(cstr cfa|0)
	rcall	makvar	;allot cell to var, init to cfa|0
	rcall	head__	;create head (lf/nf) in dict for cstr
	rcall	vp_
	rcall	at_		;address in variable space
	literal	do_defer
	rjmp	x2coma0	;vector var literal

; ============ FortAVR Initialization ================
;=>	RST	(--)	reset ~ hilevel cold start sequence
	code	3,"rst"
rst_:
	cbi		PORTC,0	;disable RS485 driver (C0)
	in_		r11,MCUSR	;MCU Status Register (save reset source)
	clr		zerol
	clr		zeroh		;init zero reg
	out_	MCUSR,zerol	;clear/reset flags
	ldi		xl,low(RP0)	;init return_stack pointer
	out_	SPL,xl
	ldi		xh,high(RP0)
	out_	SPH,xh
	ldi		stat,0	;interpret, interactive (NID), non-echo mode
	rcall	clr_	;init data stack pointer
;	--> version letters: a b c d e  f g h i j  k l m n o  p
	ldi		tmp1,$32;morse IMII (version 3c)
	rcall	ledcode	;debug checkpoint

;	rcall	wds8sec	;set watchdog to 8 sec
	ldi		xl,$4	;systic 62.5ns clk *64=  4us tick
	sts		TCCR2B,xl	;TIM2 control register B $b1
	ldi		xl,250	;TIM2 250*4us= 1ms
	sts		OCR2A,xl	;TIM2 Output Compare Register A $b3
	ldi		xl,$2	;clear TIM2 on Compare Match (CTC) mode,
	sts		TCCR2A,xl	;TIM2 control register A $b0
;	ldi		xl,$2	;enable TIM2 compare match A interrupt enable
	sts		TIMSK2,xl	;TIM2 interrupt mask register $70

;init vars from flash (Z) -> ram (X)
	ldi		zl,low(varinit)	;Z: source (flash)
	ldi		zh,high(varinit)
	ldi		xl,low($100)	;X: destination (ram)
	ldi		xh,high($100)
	ldi		tmp0,varsize	;bytes to copy to (initialize) ram
rst1:
	lpm		tmp1,Z+	;read flash init value (Z)
	st		X+,tmp1	;store (Z) in (X)
	dec		tmp0
	brne	rst1

;init serial port, display signon
	ldi		xl,25	;baud: 38.4k (25), 76.8k (12), 9600 (103), 19200 (51)
	rcall	cinit	;init serial I/O
	rcall	cr_
	rcall	doMSG
	.db		11,"FortAVR v3c"
	rcall	cr_

;	sei			;set global interrupts
	literal	tboot	;application addr ('BOOT)
	sbis	PINB,PINB5		;PB5 (D13: LED) high?	*** move before signon ?? ***
	rcall	atexec_	;n: exec tboot
;	....			;y: skip tboot
	rjmp	quit_	;start interpretation


makvar:		;(n--) allot cell to var, init to n
	rcall	vp_		;ptr to last variable space
	rcall	at_		;(n vp@)
	sbiw	TOS,2	;(	n vp@-2) allot one cell
	rcall	dup_	;(n vp@-2 vp@-2)
	rcall	vp_		;(n vp@-2 vp@-2 vp)
	rcall	store_	;update var space pointer
	rjmp	store_	;init value to n


;=>	IS:	($ adr --)	set adr as indirect execution address of DFER: word
;=>	IS:	($ n --)	set value of variable (VARB:) to n
	code	3,"is:"
	rcall	tick_	;parse next TIB word name (n|adr cfa)
	adiw	TOS,2	;(n|adr cfa+2) address of vector
	rcall	at_		;(n|adr cfa+2@) vector
	rjmp	store_	;save n in variable | update vector with new exec address

;=>	TO:	($ n --)	set n as value of RAM constant (CONS:)
	code	3,"to:"
	rcall	tick_	;cfa of deferred word
	adiw	TOS,2	;(adr cfa+2) address of vector
	rjmp	store_	;update vector with new exec address


cinit:
	out_	UBRR0L,xl	;USART baud rate
	out_	UBRR0H,zerol;USART baud rate
;	ldi		xl,U2X0		;double speed (U2X0)
;	out_	UCSR0A,xl	;USART ctrl/stat regA
	ldi		xl,(1<<TXEN0)|(1<<RXEN0)|(1<<TXCIE0)	;enable TXCIE, TX and RX
	out_	UCSR0B,xl	;USART ctrl/stat regB
;	ldi		xl,$34		;async,odd parity ($30),7 data ($4),1 stop bit
	ldi		xl,$06		;async,no parity ($0),8 data ($6),1 stop bit
	out_	UCSR0C,xl	;USART ctrl/stat regC
	cbi		PORTC,0		;disable RS485 driver (A0, portC-b0)
	sbi		DDRC,0		;set RS485 driver as output (A0, portC-b0)
	ret

;=>	XOR	(u1 u2 -- u3)	bitwise exclusive OR
	code	3,"xor"
xor_:
	ld		xl,Y+
	ld		xh,Y+	;pop u2 from stk
	eor		tosl,xl
	eor		tosh,xh
	ret

;=>	AND	(u1 u2 -- u3)	bitwise AND
	code	3,"and"
and_:
	ld		xl,Y+
	ld		xh,Y+	;pop u2 from stk
	and		tosl,xl
	and		tosh,xh
	ret

;=>	OR	(u1 u2 -- u3)	bitwise inclusive OR
	code	2,"or"
or_:
	ld		xl,Y+
	ld		xh,Y+	;pop u2 from stk
	or		tosl,xl
	or		tosh,xh
	ret

;=>	NAND	(u1 u2 -- u3)	bitwise AND complemented
	code	4,"nand"
	rcall	and_
onecom:
	com		tosl
	com		tosh	;ones complement
	ret

;=>	NOR	(u1 u2 -- u3)	bitwise inclusive OR complemented
	code	3,"nor"
	rcall	or_
	rjmp	onecom

;=>	BIT	(n -- u)	set nth bit of u
	code	3,"bit"
	mov		xl,tosl	;n bit
	ldi		tosl,1
	clr		tosh
	rjmp	lshift0

;=>	<<0	(u n -- u')	shift-left-zero ~ left shift n bits; zero fill
	code	3,"<<0"
	mov		xl,tosl	;number of bits to shift
	ld		tosl,Y+
	ld		tosh,Y+	;pop u from stk
lshift0:
	andi	xl,$f	;zero or mult of 16?
	breq	lshift2
lshift1:
	lsl		tosl
	rol		tosh
	dec		xl
	brne	lshift1
lshift2:
	ret

;=>	0>>	(u n -- u')	zero-shift-right ~ right shift n bits; zero fill
	code	3,"0>>"
	mov		xl,tosl	;number of bits to shift
	ld		tosl,Y+
	ld		tosh,Y+	;pop u from stk
	andi	xl,$f	;zero or mult of 16?
	breq	rshift2
rshift1:
	lsr		tosh
	ror		tosl
	dec		xl
	brne	rshift1
rshift2:
	ret

;=>	<<1	(u n -- u')	shift-left-one ~ left shift n bits; one fill
	code	3,"<<1"
	mov		xl,tosl	;number of bits to shift
	ld		tosl,Y+
	ld		tosh,Y+	;pop u from stk
	andi	xl,$f	;zero or mult of 16?
	breq	sl1f2
sl1f1:
	sec				;one fill
	rol		tosl
	rol		tosh
	dec		xl
	brne	sl1f1
sl1f2:
	ret

;=>	1>>	(u n -- u')	one-shift-right ~ right shift n bits; one fill
	code	3,"1>>"
	mov		xl,tosl	;number of bits to shift
	ld		tosl,Y+
	ld		tosh,Y+	;pop u from stk
	andi	xl,$f	;zero or mult of 16?
	breq	sr1f2
sr1f1:
	sec				;one fill
	ror		tosh
	ror		tosl
	dec		xl
	brne	sr1f1
sr1f2:
	ret

;=>	<<<	(u n -- u')	bitwise left rotate
	code	3,"<<<"
	mov		xl,tosl	;number of bits to shift
	ld		tosl,Y+
	ld		tosh,Y+	;pop u from stk
	andi	xl,$f	;zero or mult of 16?
	breq	lrot2
	clc			;clear carry bit
	sbrc	tosh,7	;b7 clear?
	sec			;n: set carry bit = b7
;	....
lrot1:
	rol		tosl
	rol		tosh
	dec		xl
	brne	lrot1
lrot2:
	ret

;; ============= Tools =========================

;=>	.S	(--)	dot-s ~ display contents of data stack
;::	DEPTH >R FOR R@ S@ . NEXT ." <;" ;
	code	2,".s"
	rcall	doMSG
	.db		5," Dat>"
	rcall	depth_	;(#items) stack depth
	rcall	tor_	;(R: #items) start count down loop
	rjmp	dots2	;skip first pass
dots1:
	rcall	rat_	;(#items'; R: #items') get index (n)
	rcall	pick_	;(item; R: #items') retrieve nth stack number
	rcall	dot_	;(R: #items') display contents
dots2:
	donext	dots1	;(R: #items-1) decrement stack count
dots3:
	rcall	doMSG
	.db		3," <;"
	ret

;=>	.RS	(--)	dot-return-stack ~ display contents
	code	3,".rs"
	rcall	doMSG
	.db		5," Rtn>"
	rcall	dup_
	ldi		xl,low(RP0)
	ldi		xh,high(RP0)	;top of return stack
	in_		zl,SPL
	in_		zh,SPH	;current return stack position
	sub		xl,zl
	sbc		xh,zh	;return stack depth
dotrs2:
	ld		tosh,Z+
	ld		tosl,Z+
	rcall	dot_
	sbiw	X,2
	brne	dotrs2
	rcall	drop_
	rjmp	dots3

;=>	.NW	(--)	dot-new-words ~ display words in current vocabulary
;::	CURRENT @ @ WORDS' ;
	code	3,".nw"
	rcall	current_
	rjmp	words0

;=>	.SW	(--)	dot-search-words ~ display words with cfa in context vocabulary
;::	context @ @ begin ?dup while dup cr
;::	icnt $1f and 2dup itype + dup u. 2/ u. 2- fetch repeat ;
	code	3,".sw"
words_:
	rcall	context_
words0:	
	rcall	at_	;(vadr)
	rcall	at_	;(nfa)
words1:	
	rcall	qdup_	;end of list?
	qjump	return
	rcall	dup_	;(nfa nfa)
	rcall	cr_
	rcall	count_	;(nfa nfa+1 cnt)
	andi	tosl,$1f	;mask word type bits
	rcall	ddup_
	rcall	type_	;(nfa nfa+1 cnt')
	rcall	plus_	;(nfa cfa)
	rcall	align_
	rcall	dup_	;(nfa cfa cfa)
	rcall	udot_	;disp byte addr
	rcall	twosl_
	rcall	udot_	;disp cell addr
	sbiw	TOS,2	;(lfa)
	rcall	at_		;(lfa@) next nfa
	rjmp	words1

;=>	MS	(u --)	milliseconds ~ delay u milliseconds
	code	2,"ms"
;	each processor cycle with 16MHz clock: 1/16MHz = 62.5ns
;	1000 / .250 = 4000 -(6*.25) = 3998
ms_:
	ldi		xh,high(3998)	;1 cycle
	ldi		xl,low(3998)	;1 cycle
msec2:	;each inner loop is 158 * .25us = 39.5us
	sbiw	X,1		;2 cycles
	brne	msec2	;2 cycles -> 4 * 62.5ns = .250us
	sbiw	TOS,1	;2 cycles
	brne	ms_	;2 cycles
	rjmp	drop_

;=>	+TASK	(cfa -- task#|f)	plus-task ~ add task to queue; false if queue full; task# if success
	code	5,"+task"
	ldi		zh,high(tsklst)	
	ldi		zl,low(tsklst)+1	;task1 cfa msb
	ldi		tmp0,4	;=== check four task slots ===
adtsk0:
	ld		xh,Z	;xl= task cfa high byte
	tst		xh		;task slot empty?
	breq	adtsk1	;y: add new task
	adiw	Z,2		;n: point to next task cfa msb
	dec		tmp0	;last slot?
	brne	adtsk0	;n: check next slot
	rjmp	adtsk9	;y: exit zero (false) queue full

adtsk1:		;(Z)= task slot adr+2; (TOS)= task cfa
	asr		tosh
	ror		tosl	;cfa -> kfa
	st		Z,tosh
	st		-Z,tosl	;put task kfa in (tmp0) task slot
	ldi		xh,3	;taskID: clear empty task bit (b7), next task0
adtsk2:
	sts		taskID,xh
adtsk9:
	mov		tosl,tmp0	;task# or zero (false) if queue full/cfa not found
	ldi		tosh,0
	ret

;=>	-TASK	(cfa|taskID -- t|f)	minus-task ~ delete task from queue; false if not found
; taskID: (0) current task; (1-4) identified task
	code	5,"-task"
	ldi		zh,high(tsklst)	
	ldi		zl,low(tsklst)
	tst		tosh	;taskID?
	breq	rmtsk2	;y: delete indicated task
	asr		tosh
	ror		tosl	;n: cfa -> kfa
	ldi		tmp0,4	;=== find kfa task, check four task slots ===
rmtsk0:
	ld		xl,Z+	;read task kfa
	ld		xh,Z+
	andi	xh,$3f	;remove kfa timer bit7 and active bit6
	cp		tosl,xl
	cpc		tosh,xh	;task kfas equal?
	breq	rmtsk5	;y: remove task
	dec		tmp0	;n: last slot?
	brne	rmtsk0	;n: check next slot
	rjmp	adtsk9	;y: kfa not found (rtn false)

rmtsk2:		;=== remove task with (tosl) ID ===
	dec		tosl	;current task?
	brpl	rmtsk3	;n: use taskID on data stack
	lds		tosl,taskID	;load current taskID
rmtsk3:
	andi	tosl,$3	;mask low index bits
	lsl		tosl	;2x index
	add		zl,tosl	;task cfa (low byte)
	adiw	Z,2		;point past high byte
rmtsk5:		;remove task
	st		-Z,zerol	;empty task slot
;	check if any tasks in queue
	ldi		zh,high(tsklst)	
	ldi		zl,low(tsklst)+1
	ldi		tmp0,4	;check four task slots
rmtsk6:
	ld		xh,Z	;xl= task cfa high byte
	tst		xh		;task slot empty?
	brne	adtsk9	;n: exit (true)
	adiw	Z,2		;point to next task cfa high byte
	dec		tmp0	;last slot?
	brne	rmtsk6	;n: check next slot
	ldi		tmp0,$ff;y: empty task queue code
	ldi		xh,$80	;no active tasks
	rjmp	adtsk2	;set empty task bit (b7)


;=>	QUS	(u --)	quarter microsecond delay ~ u 0.25 microseconds; 16 ms max
;	max: 16.38 ms; 4 ms short int
	code	3,"qus"
qus_:	;each cycle is 62.5ns
	sbiw	TOS,1	;2 cycles
	brne	qus_	;2 cycles -> 4 * 62.5ns = .250us
	rjmp	drop_

;=>	Z>X	(z adr -- x)	call code at cell adr with Z=TOS, X -> TOS
	code	3,"z>x"
	ldi		xl,low(z2x)
	ldi		xh,high(z2x)
	push	xl		;(z adr)
	push	xh	;local return addr on stack
	rcall	tos2z	;(z)
;	ror		zh
;	ror		zl	;2/ byte addr -> cell addr
	push	zl
	push	zh	;subroutine at adr
	movw	Z,TOS
	ret		;call subroutine at adr
z2x:	
	movw	TOS,X	;(x)
	ret	

; @@@@@ stand alone words @@@@@

;=>	MIN	(n1 n2 -- n1|n2)	minimum ~ smaller of two signed integers
;::	2DUP SWAP < IF SWAP THEN DROP ;
	code	3,"min"
min_:
	ld		xl,Y+
	ld		xh,Y+	;X= n1
	cp		xl,tosl
	cpc		xh,tosh	;n1 < n2 ?
	brlt	max1	;y: n1
	ret				;n: n2

;=>	MAX	(n1 n2 -- n1|n2)	maximum ~ greater of two signed integers
;::	2DUP < IF SWAP THEN DROP ;
	code	3,"max"
	ld		xl,Y+
	ld		xh,Y+	;X= n1
	cp		tosl,xl
	cpc		tosh,xh	;n2 >= n1 ?
	brge	rtn2	;y: n2
max1:
	movw	TOS,X	;n1
rtn2:
	ret

;=>	2OVER	(d1 d2 -- d1 d2 d1)	double-over ~ copy second double on TOS
;::	2SWAP 2DUP ;
	code	5,"2over"
dover_:
	rcall	ddup_	;(d1 d2 d2)
	ldd		tosl,Y+6;d1-by2
	ldd		tosh,Y+7;d1-by3
	ldd		xl,Y+8	;d1-by0
	st		Y,xl	;d1-by0
	ldd		xl,Y+9	;d1-by1
	std		Y+1,xl	;d1-by1
	ret

;=>	0=	(u -- t|f)	zero-equals ~ TOS equal zero?
	code	2,"0="
zequal_:
not_:
	or		tosl,tosh	;TOS=0?
	breq	less1_	;y: set TOS true (-1)
	rjmp	zero1	;n: set TOS false (0)	

;=>	NOT	(u -- t|f)	if false (zero) -> true else false
	code	3,"not"
	rjmp	zequal_
	
;=>	1+	(n -- n+1)	one-plus ~ add 1 to number
	code	2,"1+"
	adiw	TOS,1
	ret

;=>	1-	(n -- n-1)	one-minus ~ subtract 1 from number
	code	2,"1-"
less1_:
	sbiw	TOS,1
	ret

;=>	2+	(n -- n+2)	two-plus ~ add cell size in bytes to address (CELL+)
	code	2,"2+"
	adiw	TOS,2
	ret

;=>	2-	(n -- n-2)	two-minus ~ subtract cell size in bytes from address (CELL-)
	code	2,"2-"
	sbiw	TOS,2
	ret

;=>	>	(n1 n2 -- t|f)	greater-than ~ compare two signed integers
	code	1,">"
	ld		xl,Y+	;n1
	cp		tosl,xl
	ld		xl,Y+
	cpc		tosh,xl	;n2 (TOS) > n1 (X)?
greater1:
	movw	TOS,ZERO	;TOS=0
	brlt	less1_		;TOS=-1 (true)
	ret


;=>	M*	(n n -- d)	m-times ~ signed multiply; Return double product
	code	2,"m*"
mstar_:
	rcall	ddup_
	rcall	xor_
	rcall	zless_
	rcall	tor_
	rcall	abs_
	rcall	swap_
	rcall	abs_
	rcall	umstar_
	rcall	rfrom_
	qjump	ret3
dnegatej:
	rjmp	dnegate_

;=>	DUM+	(d1 n2 -- d-sum)	double-u-m-plus ~ add double and single number; return double sum
;::	0 D+ ;
	code	4,"dum+"
dumplus_:
	rcall	zeros	;(d1 d2)	
	rjmp	dplus_

;=>	D+	(d1 d2 -- d3)	double-plus ~ add double signed integers
	code	2,"d+"
dplus_:
	rcall	dload	;load d1 (low) and d2 in tmp registers
	add		tmp0,spml;add by0
	adc		tmp1,spmh;add by1
	adc		tosl,tmp2;add by2
	adc		tosh,tmp3;add by3
	st		-Y,tmp1	;push by1
	st		-Y,tmp0	;push by0
	ret

dload:	;pop d1 (low) and d2 in tmp registers
	ld		tmp0,Y+	;pop d2-by0
	ld		tmp1,Y+	;pop d2-by1
	ld		tmp2,Y+	;pop d1-by2
	ld		tmp3,Y+	;pop d1-by3
	ld		spml,Y+	;pop d1-by0
	ld		spmh,Y+	;pop d1-by1
ret05:
	ret

;=>	DABS	(d -- d')	double-absolute ~ double absolute value
;::	DUP 0< IF DNEG THEN ;
	code	4,"dabs"
dabs_:
	tst		tosh	;(d) negative?
	brmi	dnegatej;y: make positive
	ret		;n: leave alone

;=>	D>	(d1 d2 -- t|f)	double-greater-than ~ compare signed doubles
	code	2,"d>"
dgreat_:
	rcall	dload	;pop d1 (low) and d2 in tmp registers
	cp		tmp0,spml;>by0
	cpc		tmp1,spmh;>by1
	cpc		tosl,tmp2;>by2
	cpc		tosh,tmp3;>by3
	rjmp	greater1

;=>	D<	(d1 d2 -- t|f)	double-less-than ~ compare signed doubles
;::	SWAP D> ;
	code	2,"d<"
	rcall	dswap_
	rjmp	dgreat_

;=>	D=	(d1 d2 -- t|f)	double-equals ~ compare signed doubles
;::	D- D0= ;
	code	2,"d="
	rcall	dminus_
	rjmp	dzequal_

;=>	D0=	(d -- t|f)	double-zero-equals ~ double equal zero?
;::	D- D0= ;
	code	3,"d0="
dzequal_:
	or		tosl,tosh	;d-by2 || d-by3
	mov		xl,tosl
	rcall	drop_
	or		tosl,xl	;result || d-b0
	rjmp	zequal_	;result || d-b1; set flag

;=>	D-	(d1 d2 -- d3)	double-minus ~ subtract double signed integers
;::	DNEG D+ ;
	code	2,"d-"
dminus_:
	rcall	dnegate_
	rjmp	dplus_

;=>	DMAX	(d1 d2 -- d1|d2)	double-maximum ~ greater of two double signed integers
;::	2OVER 2OVER D> IF ELSE 2SWAP THEN DDROP ;
	code	4,"dmax"
	rcall	dover_	;(d1 d2 d1)
	rcall	dover_	;(d1 d2 d1 d2)
	rcall	dgreat_	;(d1 d2 d1 t|f)
	qjump	dmin1	;d1>d2?
	rjmp	ddrop_	;y: drop minimum double

;=>	DMIN	(d1 d2 -- d1|d2)	double-minimum ~ lesser of two double signed integers
;::	2OVER 2OVER D> IF ELSE 2SWAP THEN DDROP ;
	code	4,"dmin"
	rcall	dover_	;(d1 d2 d1)
	rcall	dover_	;(d1 d2 d1 d2)
	rcall	dgreat_	;(d1 d2 d1 t|f)
	qjump	dmin2	;d1>d2?
dmin1:
	rcall	dswap_	;y: swap d1 and d2
dmin2:
	rjmp	ddrop_	;n: drop minimum double

;=>	DUM/MOD	(ud u -- d-qot u-rem)	double-u-m-divide-mod ~ unsigned divide of double by single; return mod and double quotient
; (1 1 2) -> (8000 0 1); (1 1 3) -> (5555 0 2)
	code	7,"dum/mod"
dummod_:
	movw	Z,TOS	;save Z= u (divisor)
	ld		tmp2,Y+
	ld		tmp3,Y+
	ld		tmp0,Y+
	ld		tmp1,Y+	;pop ud (dividend)
	movw	TOS,ZERO	;zero reml
	movw	spml,ZERO	;zero remh (remainder)
;	---
	ldi		xl,33	;set loop counter
dummod1:
	clc		;clear carry bit
dummod2:
	rol		tmp0
	rol		tmp1
	rol		tmp2
	rol		tmp3	;udqot shifted left one bit
	dec		xl	;last digit shifted?
	breq	dummod9	;y: exit
	rol		tosl	;n: shift remainder
	rol		tosh
	rol		spml
	rol		spmh	;udrem shifted left one bit
; test subtracting divisor
	cp		tosl,zl
	cpc		tosh,zh
	cpc		spml,zerol
	cpc		spmh,zerol	;negative result from compare?
	brcs	dummod1	;y: next digit, cy=1
;do subtraction for real and set lowest bit
	sub		tosl,zl
	sbc		tosh,zh
	sbc		spml,zerol
	sbc		spmh,zerol	;udrem shifted left one bit
	sec		;set carry
	rjmp	dummod2	;cy=1
;	---
dummod9:	;%%% change coding to use rot1 %%%
	st		-Y,tmp1
	st		-Y,tmp0
	st		-Y,tmp3
	st		-Y,tmp2	;push double quotient
	ret		;remainder in TOS, upper bytes s/b zero w/single divisor

;=>	DUM*	(ud u -- ud')	double-u-m-times ~ unsigned double*single multiply; return double product
;::	DUP >R UM* DROP 0 -ROT SWAP R> UM* D+ ;
; (51 51 64) -> (1fa4 1fa4)
	code	4,"dum*"
dumstar_:
	movw	X,TOS	;(udl udh u) save u
	rcall	umstar_	;(udl ud2l ud2h) u times upper cell
	movw	TOS,ZERO	;(udl ud2l 0) discard upper cell of result
	rcall	rop_	;(0 ud2l udl) lower result cell shifted left
	rcall	dup_	;(0 ud2l udl udl)
	movw	TOS,X	;(0 ud2l udl u) restore u
	rcall	umstar_	;(0 ud2l udl*u) u times lower cell
	rjmp	dplus_

;=>	DU>	(d1 d2 -- t|f)	double-u-greater-than ~ compare unsigned doubles
	code	3,"du>"
	rcall	dload	;pop d1 (low) and d2 in tmp registers
	cp		tmp0,spml;>by0
	cpc		tmp1,spmh;>by1
	cpc		tosl,tmp2;>by2
	cpc		tosh,tmp3;>by3
	movw	TOS,ZERO	;TOS=0
	brlo	dumret
	sbiw	TOS,1
dumret:
	ret

;=>	2@	(adr -- d)	double-fetch ~ d from adr
;::	DUP 2+ @ @ ;

;=>	2!	(d adr --)	double-store ~ d at adr
;::	SWAP OVER 2+ ! ! ;

;=>	TOV	(a b c -- a b c a)	third-over ~ push third stack item
;	2 S@ ;
	code	3,"tov"
tov_:
	savetos
	ldd		tosl,Y+4
	ldd		tosh,Y+5
	ret

;-----++++++++----- FLASH memory programming -------+++++++++-------

;=>	F>B	(fadr --)	flash-to-buffer ~ transfer page (128 bytes) from Flash to RAM buffer
	code	3,"f>b"
	rcall	tos2z	;flash addr
	ldi		xl,low(buf)
	ldi		xh,high(buf)
	ldi		tmp0,128 ;init loop variable
movflash:	;move tmp0 flash bytes from (Z) to (X)
	lpm		r0,Z+	;read flash byte
	st		X+,r0	;store in buffer
	dec		tmp0
	brne	movflash
	ret

;=>	FB?	(fadr -- t|f)	check flash for erase
	code	3,"fb?"
	ldi		xl,low(buf)
	ldi		xh,high(buf)
	movw	Z,TOS	;flash addr
	clr		tosl
	clr		tosh	;init false exit
	ldi		tmp0,128 ;init loop variable
fqd1:
	lpm		r0,Z+	;read flash byte
	ld		r1,X+	;get buffer byte
	and		r0,r1
	cp		r0,r1	;any programmed zero bits?
	brne	ret4	;y: exit false
	dec		tmp0	;n: keep checking
	brne	fqd1
	subi	TOS,1	;exit true
ret4:
	ret

;=>	FX!	(fadr --)	flash-erase-store ~ erase page (64 cells) of flash memory at fadr
;**todo** check if erase required (flag set) ***
	code	3,"fx!"
peras:
	rcall	tos2z	;flash address
	ldi		spmcmd,(1<<PGERS) | (1<<SELFPRGEN)
	rcall	do_spm	;erase page
; re-enable the RWW section
	ldi		spmcmd,(1<<RWWSRE) | (1<<SELFPRGEN)
	rjmp	do_spm

;=>	>FB	(fadr --)	to-flash-buffer ~ copy RAM buffer to Flash writeBuffer
	code	3,">fb"
	movw	Z,TOS	;flash page load address
	ldi		xl,low(buf)
	ldi		xh,high(buf)
	ldi		tmp0,64 ;cells to transfer
tofbuf:
	ld		r0,X+
	ld		r1,X+
	ldi		spmcmd,(1<<SELFPRGEN)
	rcall	do_spm	;store cell in flash buffer
	adiw	Z,2		;inc flash ptr one cell
	dec		tmp0	;next cell
	brne	tofbuf
	rjmp	drop_

;=>	FB!	(fadr --)	flash-buffer-store ~ write flash page (fadr) from writeBuffer
	code	3,"fb!"
	rcall	tos2z	;flash page addr
wbpgm:
	ldi		spmcmd,(1<<PGWRT) | (1<<SELFPRGEN)
	rcall	do_spm	;execute Page Write
	ldi		spmcmd,(1<<RWWSRE) | (1<<SELFPRGEN)
	rjmp	do_spm	;re-enable the RWW section

pagbuf:
	varable	buf
;	ret

;=>	WHO	(--)	display name of FORTH vocabulary
	code	3,"who"
	literal	forthtxt*2
	rcall	count_
	andi	tosl,$1f
	rjmp	type_

;=>	F01	(fadr --)	write Flash Buffer - flash boot test
	code	3,"f01"
	mov		addrl,tosl
	mov		addrh,tosh	;flash page addr
	ldi		xl,128	;count in bytes
	mov		bycnt,xl
	ldi		xl,low(bootBuf)
	ldi		xh,high(bootBuf)
	rjmp	pgPage2

;	sUART transmit character, tmp0= character
;		transmit uppercase U ($55); 8N1 or 7E1
;--+ S +-1-+ 0 +-1-+ 0 +-1-+ 0 +-1-+ P +-S-+
; -1   0   1   2   3   4   5   6   7   8   9
;  +---+   +---+   +---+   +---+   +---+
;		transmit uppercase U ($55); 7N1
;--+ S +-1-+ 0 +-1-+ 0 +-1-+ 0 +-1-+-S-+
;  0   1   2   3   4   5   6   7   8   9
;  +---+   +---+   +---+   +---+

;=>	EMIT2	(chr --)	send chr via software UART
	code	5,"emit2"
	mov		ubuf,tosl	;copy transmit-data to buffer
tx_chr:
	cbi		EIMSK,INT1	;disable rx INT1 (PD2)
	sbr		stat,(1<<utxsbt)	;set transmit flag
	mov		ubtcnt,zerol;erase bit-counter (7bit)
;	dec		ubtcnt		;set bit counter to $ff (8bit)
	
	ldi		tmp0,bit10
	out		OCR0A,tmp0	;set timer for one bit width
	ldi		tmp0,(1<<OCIE0A);$2
	sts		TIMSK0,tmp0	;enable Oc0A interrupt
	cbi		PORTD,PD4	;clear output (start-bit)
	sbi		PORTC,PC1	;enable RS485 driver (C1)
drop02:
	rjmp	drop_

;=>	KEY2?	( -- t|f)	key-two-question ~ input character from software UART?
	code	5,"key2?"
	sbrs	stat,urxsbt	;chr received?
	rjmp	zeros
;	---
	rjmp	true

;=>	S2INI	( --)	serial-two-init ~ initialize software UART
	code	5,"s2ini"
;	::: initialize Timer/Counter0 - transmit and receive	
	ldi		tmp0,(1<<COM0A1)|(1<<COM0A0)	;$c0
	out		TCCR0A,tmp0	;nonPWM mode, set OC0A on compare match
	ldi		tmp0,(1<<CS01)	;$2
	out		TCCR0B,tmp0	;set clk/8, 16MHz/8 = 0.5us
	sts		TIMSK0,zerol;disable Oc0A interrupt

;	::: initialize Interrupt0 (PD2)	- receive
;	ldi		tmp0,(1<<ISC01)	;$2
	sts		EICRA,tmp0	;falling edge INT0 -> interrupt (PD2)
	sbi		EIMSK,INT1	;enable rx INT1 (PD2)

	cbi		PORTC,1		;disable RS485 driver (A1, portC-b1)
	sbi		DDRC,1		;set RS485 driver as output (A1, portC-b1)
	out		PORTD,zerol	;Set port D as input with pullups
	sbi		DDRD,DDD4	;except PD4 output (tx)

	cbr		stat,(1<<urxsbt)|(1<<utxsbt)	;clear status bits
	ret


; ----- push (post-dec); pop (pre-inc)
; empty <- SP (after a call)
; rtn-h
; rtn-l <- SP (after ret)
;  i -h
;  i -l
;  i'-h (SP + 5)
;  i'-l
;  j -h (SP + 7)
;  j -l

;=>	I'	(-- n1 R: n1 n2 -- n1 n2)	copy second return stack cell to data stack
;::	2 r@ ;
	code	2 +(1<<cmpbit),"i'"
	in_		zl,SPL
	in_		zh,SPH
	adiw	Z,5
	rjmp	readstk0

;=>	J	(-- n1 R: n1 n2 n3 -- n1 n2 n3)	copy third return stack cell to data stack
;::	3 r@ ;
	code	1 +(1<<cmpbit),"j"
	in_		zl,SPL
	in_		zh,SPH
	adiw	Z,7
readstk0:
	savetos
	rjmp	readstk

;=>	R@	(nth -- n)	r-fetch ~ copy nth cell of return stack (i=1, i'=2, j=3)
	code	2 +(1<<cmpbit),"r@"
	lsl		tosl
	rol		tosh	;2* cells -> bytes
	in_		zl,SPL
	in_		zh,SPH
	add		zl,tosl
	adc		zh,tosh
	adiw	Z,1
readstk:
	ld		tosh,Z+	;read return stack item high byte first
	ld		tosl,Z+
	ret

;=>	DROP0	(n -- 0)	drop-zero ~ replace top of stack with zero
	code	5,"drop0"
drop0_:
	movw	TOS,ZERO
	ret

;=>	RDROP	(-- R: n --)	r-drop ~ remove item from return stack
	code	5 +(1<<cmpbit),"rdrop"
	pop		zh
	pop		zl	;Z=return addr
	pop		xl
	pop		xl	;discard return stack item 
	ijmp	;return Z->pc

;=>	0>	(n -- t|f)	zero-greater-than ~ true if n > 0
	code	2,"0>"
	tst		tosh	;n high bit set (negative)?
	brmi	drop0_	;y: false n < 0
	rjmp	znot_

;=>	0<>	(n -- t|f)	zero-not-equal ~ true if n <> 0
	code	3,"0<>"
znot_:
	or		tosl,tosh	;n = 0?
	breq	znot1		;y: false
truex:
	ldi		tosh,-1		;n: true n > 0
	ldi		tosl,-1
znot1:
	ret		

;=>	<>	(n1 n2 -- t|f)	not-equal ~ true if n1 <> n2
	code	2,"<>"
	ld		xl,Y+
	ld		xh,Y+	;X= n1
	sub		tosl,xl
	sbc		tosh,xh
	brne	truex
	ret

;=>	>=	(n1 n2 -- t|f)	greater-than-equals ~ true if n1 >= n2
	code	2,">="
	ld		xl,Y+
	ld		xh,Y+	;X= n1
	sub		xl,tosl
	sbc		xh,tosh	;n1 >= n2?
	brge	truex	;y: true (-1)
	rjmp	drop0_	;n: false (0)

;=>	<=	(n1 n2 -- t|f)	less-than-equals ~ true if n1 <= n2
	code	2,"<="
	ld		xl,Y+
	ld		xh,Y+	;X= n1
	sub		xl,tosl
	sbc		xh,tosh	;n1 <= n2?
	brlt	truex	;y: true (-1)
	breq	truex	;y: true (-1)
	rjmp	drop0_	;n: false (0)

;=>	X!	(n --)	x-store ~ save n in variable X
	code	2,"x!"
	sts		xvar,tosl
	sts		xvar+1,tosh
	rjmp	drop_

;=>	X@	(-- n)	x-at ~ retrieve value of variable X
	code	2,"x@"
	savetos
	lds		tosl,xvar
	lds		tosh,xvar+1
	ret

;=>	.LD	(u --)	dot-LED ~ flash 8-bit binary code on LED
	code	3,".ld"
	mov		tmp1,tosl
	rcall	drop_
ledcode:
	sbi		DDRB,5	;set LED as output; D13 portB-b5
	ldi		tmp2,8	;number of flashes
lcode2:
	ldi		tmp0,50	;zero delay of 100ms
	sbrc	tmp1,7	;top bit one?
	ldi		tmp0,250;y: one delay of 500ms
;	....			;n: zero delay of 100ms
	sbi		PORTB,5	;LED on
	rcall	ms2
	cbi		PORTB,5	;LED off
	ldi		tmp0,250;between bit delay of 500ms
	rcall	ms2
	lsl		tmp1	;shift bits left
	dec		tmp2	;all bits flashed?
	brne	lcode2	;n: continue flashing
	cbi		DDRB,5	;set LED as (floating) input
	ret			;n: return Z->pc

;	delay 2*tmp0 milliseconds
;	2000 / .250 = 8000 -(5*.25) = 7999
ms2:
	ldi		xh,high(7999)	;1 cycle
	ldi		xl,low(7999)	;1 cycle
ms2L:	;each inner loop is 158 * .25us = 39.5us
	sbiw	X,1		;2 cycles
	brne	ms2L	;2 cycles -> 4 * 62.5ns = .250us
	dec		tmp0	;1 cycle
	brne	ms2		;2 cycles
	ret

;slowest conversion rate: ADPS=128 -> 125kHz -> 8us * 25 -> 200us
;temp mv (314): $c7c8; 1.1v ref (225): $c74e; 0vdc (0): $c74f; ADC1: $c741
;=>	A@	(u--n)	analog-at ~ analog input
	code	2,"a@"
;REFS: voltage reference (b7:b6): external capacitor at AREF pin ($40)
;ADLAR: adjust left (b5): right ($00), left ($20)
;MUX: analog input select (b3:b0); channels 0-7, temp($8), 1.1v ($e), 0v ($f)
    sts ADMUX,tosl	;init ADC, select channel ($7c)

;ADEN[7]: ADC Enable; turn on ADC
;ADSC[6]: ADC Start Conversion 
;ADATE[5]: ADC Auto Trigger Enable: start conversion on positive edge of selected trigger
;ADIF[4]: ADC Interrupt Flag
;ADIE[3]: ADC Interrupt Enable
;ADPS[2:0]: ADC Prescaler Select Bits; x2 ($0), 500kHz/32 ($5), 125kHz/128 ($7)
	sts ADCSRA,tosh	;enable ADC, set prescaling factor ($7a)

;	ldi		tosh,0		;62.5ns *6 * 256= 96us
adcWait:
    lds		tosl,ADCSRA	;[2]Observe the ADIF flag, it gets set by hardware when ADC conversion completes
;	dec		tosh		;[1]
    sbrs	tosl,4		;[1]conversion complete?
    rjmp	adcWait		;n: wait
;	brne	adcWait		;[2]n: wait if not timed out
;	....				;y: data ready
    ori		tosl,0b00010000	;set flag to signal 'ready-to-be-cleared' by hardware
    sts		ADCSRA,tosl
	lds		tosl,ADCL	;must read ADCL first ($78)
	lds		tosh,ADCH	; ($79)
	ret


.equ	topforth	=	_link*2	;head of FORTH vocab


.if (pc > $3eff)	;	$3e12 - $3eff empty: 237 words, 474 bytes v3a
	.error " *** Overwriting boot sector"
.endif


.if	loader
	.include "mgmBoot.asm"
.else
;	.equ	do_spm	= $3e93
wds8sec:						;0 (16ms), 2 (64ms), 4 (.25sec), 5 (.5sec)
	ldi		tmp0,(1<<WDE) | $21	;6 (1sec), 7 (2sec); $20 (4sec), $21 (8sec)
wdset:
	ldi		tmp1,(1<<WDCE) | (1<<WDE)
	sts		WDTCSR,tmp1	;enable change prescaler
	sts		WDTCSR,tmp0	;($60) prescaler value
	ret
.endif


