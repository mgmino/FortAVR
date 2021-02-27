; -------------- avr macros v1.3 -------------
.macro loadtos
	ld tosl,Y+	;pop l
	ld tosh,Y+	;pop h
.endm

.macro savetos
	st -Y,tosh	;push h
	st -Y,tosl	;push l
.endm

.macro in_
.if (@1 < $40)
	in @0,@1
.else
	lds @0,@1
.endif
.endm

.macro out_
.if (@0 < $40)
	out @0,@1
.else
	sts @0,@1
.endif
.endm

.macro rdflash
	lsl zl
	rol zh		;2* cell addr -> byte addr
	lpm @0,Z+	;pop l
	lpm @1,Z+	;pop h
.endm

;	Compile a code definition header
.macro	code				;;LEX,NAME
.if (pc < XCOD/2)
	.error " code def location error"
	__LINE__
.endif
	.dw		_LINK*2			;;link pointer
	.SET _LINK	=	pc		;;link points to prev name field
.if (@0 & 1)				;;odd number of characters
	.db		@0,@1
.else						;;even number of characters
	.db		@0,@1,FILBYTE
.endif
.if ((((@0 & $f) +2) >> 1) != (pc - _LINK))
	.error " code word length error"
	__LINE__
.endif
.endm

.macro	codez				;;LEX,NAME,SPECIAL-CHAR
.if (pc < XCOD/2)
	.error " codez def location error"
	__LINE__
.endif
	.dw		_LINK*2			;;link pointer
	.SET _LINK	=	pc		;;link points to prev name field
.if (@0 & 1)				;;odd number of characters
	.db		@0,@1,@2
.else						;;even number of characters
	.db		@0,@1,@2,FILBYTE
.endif
.if ((((@0 & $f) +2) >> 1) != (pc - _LINK))
	.error " codez word length error"
.endif
.endm

;	Compile a colon definition header
.macro	colon				;;LEX,NAME
.if (pc >= XCOD/2)
	.error " colon def location error"
	__LINE__
.endif
	.dw		_LINK*2			;;link pointer
	.SET _LINK	=	pc		;;link points to prev name field
.if (@0 & 1)				;;odd number of characters
	.db		@0,@1
.else						;;even number of characters
	.db		@0,@1,FILBYTE
.endif
.if ((((@0 & $f) +2) >> 1) != (pc - _LINK))
	.error " colon word length error"
.endif
.endm

.macro	literal			;;CONSTANT
	rcall	doLIT
	.dw		@0
.endm

.macro	qjump			;;ADDRESS ### recode 33 instances ###
	rcall	QBRAN
	.dw		@0
.endm

.macro	varable		;;VAR_ADDRESS
	rcall	doVAR
	.dw		@0
;	.dw		0
.endm

.macro	donext		;;ADDRESS ### recode 9 instances ###
	rcall	doNXT
	.dw		@0
.endm

.macro	label
.equ	@0	= pc*2
.endm


;; colon words

.macro	exec
	.dw		@0	;execute colon/code word ( -- )
.endm

.macro	slit
; -8192 ($2000) to 8191 ($1fff)
	.dw		$c000 +@0	;short literal ( -- n )
.endm

.macro	blit
	.dw		do_lit	;call ( -- )
	.dw		@0	;big literal ( -- n )
.endm

.macro	lit
.if (@0 > $3fff)	;;long literal
	.dw		do_lit	;call ( -- )
	.dw		@0	;literal ( -- n )
.else				;;short literal
	.dw		$c000 +@0	;short literal ( -- n )
.endif
.endm

.macro	jump
	.dw		@0 +$8000	;branch ( -- )
.endm

.macro	qdgot	;relative branch if true, drop flag
	.dw		((@0 -pc -1) & $3ff) + $4000
.endm

.macro	qdgof	;relative branch if false, drop flag
	.dw		((@0 -pc -1) & $3ff) + $5000
.endm

.macro	qkgot	;relative branch if true, keep flag
	.dw		((@0 -pc -1) & $3ff) + $4800
.endm

.macro	qkgof	;relative branch if false, keep flag
	.dw		((@0 -pc -1) & $3ff) + $5800
.endm

.macro	rjump	;relative branch
	.dw		((@0 -pc -1) & $3ff) + $6000
.endm

.macro	rexec	;relative call
	.dw		((@0 -pc -1) & $3ff) + $6400
.endm

.macro	rloop	;relative LOOP runtime
	.dw		((@0 -pc -1) & $3ff) + $6800
.endm

.macro	rnext	;relative NEXT runtime
	.dw		((@0 -pc -1) & $3ff) + $6c00
.endm

.macro	bcall	;relative BK: call
	.dw		((@0 -pc -1) & $3ff) + $4400
.endm

.macro	case	;(b -- b)CASE[
	.dw		((@0 -pc -1) & $3ff) + $4400
	.dw		do_case +$8000
.endm

.macro	caseq	;case ]EQ element
	.db		@0, ((@1 -pc -1) & $7) + $70
.endm

.macro	casls	;case ]LS element
	.db		@0, ((@1 -pc -1) & $7) + $78
.endm

.macro	cased	;case ELS element
	.dw		$70ff
.endm

.macro	cend
	.dw		0	;end of colon instructions
.endm