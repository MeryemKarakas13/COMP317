;
; DEFUSE_THE_BOMB.asm
;
; Created: 13.05.2021 19:04:59
; Author : asus
;

;SEVEN SEGMENT LETTERS:
; 1-A	 2-b	3-C		4-d		5-E		6-F		7-g		8-H
; 9-I	 10-J	11-L	12-n	13-o	14-P	15-q	16-r
; 17-S	 18-t	19-U	0(20)-Y

; Program size in memory (bytes): 5251 bytes
; Needed data memory size (bytes): 30 bytes
; Number registers in use: 26

;; Reset interrupt vector
.cseg
.org $0000
	jmp RESET

.org    $012
    jmp    TIM1_OVF         ; Timer1 overflow Handle

.org $0014
	jmp TIMER0				; Timer0 compare match

.org    $01A
	jmp	RXC_INT			    ; $01A UART Rx complete


;; Define temporary register
.def tmp		= R16
.def Delay0		= R17
.def Delay1		= R18
.def minute		= R19		; timer minute digit
.def secTens	= R20		; timer tens of seconds
.def secOnes	= R21		; timer ones of seconds
.def butIn		= R22		; to read input from port B for second Phase
.def found1		= R23		; first founded letter in password
.def found2		= R24		; second founded letter in password
.def found3		= R25		; third founded letter in password
.def found4		= R26		; fourth founded letter in password
.def x_SW		= R27       ; step width
.def RECCHAR	= R28		; received character
.def tmp2       = R29

.def RECFLAG	=R2			; new data received
.def numOfFounded = R3		; founded letter counter
.def pw1		= R4		; first letter of password
.def pw2		= R5		; second letter of password
.def pw3		= R6		; third letter of password
.def pw4		= R7		; fourth letter of password
.def XL_Tmp		= R8        ; low  part temporary register
.def XH_Tmp		= R9        ; high part temporary register
.def XL_LUT		= R10       ; number of LUT-Element (low byte)
.def XH_LUT		= R11       ; number of LUT-Element (high byte)  
.def slower		= R12       ; to adjust seconds decrement
.def forOnce	= R13       ; to be aware of which letters are founded in password

.equ tdot		= 0b10000000		; dot after ones for sev seg display

.equ	BAUD_RATE	=9600
.equ	CPU_CLOCK	=4000000
.equ	UBRRVAL		=(CPU_CLOCK /(16*BAUD_RATE))-1

.equ     Xtal		= 4000000				; system clock frequency
.equ     prescaler	= 1						; timer0 prescaler
.equ     N_samples	= 128					; Number of samples in lookup table
.equ     Fck		= Xtal/prescaler        ; timer0 working frequency
.equ	 SW_500		= 15					; LUT step width

	
RESET:
	;; Initialize stack pointer
	ldi		tmp, LOW(RAMEND)		;load the end of SRAM's low byte to "tmp"
	out		SPL, tmp				;Initialize stack pointers low byte
	ldi		tmp, HIGH(RAMEND)		;load the end of SRAM's high byte to "tmp"
	out		SPH, tmp				;Initialize stack pointers high byte

	;; Initialize PortA[2:0] as an output
	ldi		tmp, $07
	out		DDRA, tmp

	;; Initialize PortC[7:0] as an output
	ldi		tmp, $FF
	out		DDRC, tmp

	;make PORTD as an input-output port
	ldi		tmp, $0F
	out		DDRD, tmp
	
	clr		tmp2
	
init_Timers_USART:
	in		butIn, PIND ;check button press
	inc		tmp2
	cpi		butIn, $80
	brne	init_Timers_USART
	clr		RECFLAG
	ldi		tmp, low(UBRRVAL)	; set the baud rate
	out		UBRRL, tmp
	ldi		tmp, high(UBRRVAL)	; set the baud rate
	out		UBRRH, tmp  
	ldi		tmp, (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0)
	out		UCSRC, tmp			; set 8-bit comm.
	sbi		UCSRB, TXEN			; enable serial transmitter
    sbi		UCSRB, RXEN			; enable serial receiver
    sbi		UCSRB, RXCIE		; enable receiver interrupts

    clr		XL_LUT
    clr		XH_LUT				; Set table pointer to 0x0000
    ldi		X_SW, SW_500		; set the step width

	;Timers configurations
	ldi		tmp,(1<<WGM10)+(1<<COM1B1)
    out		TCCR1A,tmp                   ; 8 bit PWM non-inverting (Fck/510)
    ldi		tmp,(1<<CS10)
    out		TCCR1B,tmp                   ; prescaler = 1
    ldi		tmp, (1<<CS02)|(1<<CS00)
	out		TCCR0, tmp
	ldi		tmp, 1<<TOV0
	out		TIFR, tmp
	ldi		tmp, (1<<OCIE0)|(1<<TOIE1)
	out		TIMSK, tmp
	ldi		tmp, 98
	out		OCR0, tmp
	sei

generate_password:
	ldi		minute, 1			; Set the timer for 1.30 minutes
	ldi		secTens, 3
	ldi		secOnes, 0
	clr		slower
	clr		forOnce
	clr		numOfFounded
	mov		tmp, tmp2			;to make random password
	rcall	generate_pw			; generate password letters
	mov		pw1, tmp
	rcall	generate_pw
	mov		pw2, tmp
	rcall	generate_pw
	mov		pw3, tmp
	rcall	generate_pw
	mov		pw4, tmp


PHASE1:
	tst		RECFLAG
	breq	no_data
	rcall	convert_ASCII		; convert ascii code of input to the our encoding for letters
	rcall	CONTROL_INPUT		; copmare input with password, buzz morse code if matches
	mov		tmp, numOfFounded	
	cpi		tmp,4				; whether all letters are founded
	brsh	PHASE2
	clr		RECFLAG

no_data:
	;; Display No#0
	ldi		tmp, (1<<0)		    ; Make bit#0 one, others zero
	out		PORTA, tmp
	mov		tmp, secOnes
	rcall	sev_seg_display 
	out		PORTC, tmp
	rcall	DelayS

	;; Display No#1
	ldi		tmp, (1<<1)			; Make bit#1 one, others zero
	out		PORTA, tmp
	mov		tmp, secTens
	rcall	sev_seg_display 
	out		PORTC, tmp
	rcall	DelayS

	;; Display No#2
	ldi		tmp, (1<<2)			; Make bit#2 one, others zero
	out		PORTA, tmp
	mov		tmp, minute
	rcall	sev_seg_display
	ori		tmp, tdot
	out		PORTC, tmp
	rcall	DelayS

	rjmp PHASE1

PHASE2:
	ldi		tmp, 0
	out		TCCR0, tmp			; stop Timer0
	ldi		tmp, $0F
	out		DDRA, tmp
	cbi		UCSRB, TXEN			; disable serial transmitter
    cbi		UCSRB, RXEN			; disable serial receiver
    cbi		UCSRB, RXCIE		; disable receiver interrupts
	ldi		tmp, $00			; port B all inputs
	out		DDRB, tmp
	ldi		r19, 4
	loop:
		in		butIn, PINB		; check button press
		cpi		butIn, $01		; user tries current sevseg order for password
		brne	comp
		release: in butIn, PINB
		cpi		butIn, $00
		brne	release
		cpi		r19,0			; whether the 5 attempts are over or not
		brne	go
		rcall   BOM				; if wrong passwords are entered 3 times
		go:
		cp		found1,pw1
		brne	decr
		cp		found2,pw2
		brne	decr
		cp		found3,pw3
		brne	decr
		cp		found4,pw4
		brne	decr
		mov		RECCHAR, pw1	; when true password is entered
		rcall	MORSE
		mov		RECCHAR, pw2
		rcall	MORSE
		mov		RECCHAR, pw3
		rcall	MORSE
		mov		RECCHAR, pw4
		rcall	MORSE
		rcall	FINISH			
		decr:	
			dec		r19				; decrement the right to try
			sbi		DDRD,PD4		; Set pin PD4 as output
			rcall	delay
			rcall	delay
			rcall	delay
			rcall	delay
			rcall	delay
			rcall	delay
			rcall	delay
			rcall	delay
			rcall	delay
			rcall	delay
			cbi		DDRD,PD4		; to silence the buzzer
		comp:
		cpi		butIn, $80
		breq	change12			; swap first an second letters
		cpi		butIn, $40
		breq	change23			; swap second an third letters
		cpi		butIn, $20
		breq	change34			; swap third an fourth letters
		cpi		butIn, $10
		brne	diss_pasw
		release4: in butIn, PINB
		cpi		butIn, $00
		brne	release4
		mov		tmp, found1			; swap first an fourth letters
		mov		found1, found4
		mov		found4, tmp
		rjmp	diss_pasw

change12:
		release1: in butIn, PINB
		cpi		butIn, $00
		brne	release1
		mov		tmp, found1
		mov		found1, found2
		mov		found2, tmp
		rjmp	diss_pasw
change23:
		release2: in butIn, PINB
		cpi		butIn, $00
		brne	release2
		mov		tmp, found2
		mov		found2, found3
		mov		found3, tmp
		rjmp	diss_pasw
change34:
		release3: in butIn, PINB
		cpi		butIn, $00
		brne	release3
		mov		tmp, found3
		mov		found3, found4
		mov		found4, tmp

diss_pasw:
		;; Display first letter
		ldi		tmp, (1<<0)     ; Make bit#0 one, others zero
		out		PORTA, tmp
		mov		tmp, found4
		rcall	sev_seg_letters 
		out		PORTC, tmp
		rcall	DelayS

		;; Display second letter
		ldi		tmp, (1<<1)     ; Make bit#1 one, others zero
		out		PORTA, tmp
		mov		tmp, found3
		rcall	sev_seg_letters
		out		PORTC, tmp
		rcall	DelayS

		;; Display third letter
		ldi		tmp, (1<<2)     ; Make bit#2 one, others zero
		out		PORTA, tmp
		mov		tmp, found2
		rcall	sev_seg_letters
		out		PORTC, tmp
		rcall	DelayS

		;; Display fourth letter
		ldi		tmp, (1<<3)     ; Make bit#3 one, others zero
		out		PORTA, tmp
		mov		tmp, found1
		rcall	sev_seg_letters
		out		PORTC, tmp
		rcall	DelayS

		rjmp loop
FINISH:							; display "donE" on sev seg
		;; Display e
		ldi		tmp, (1<<0)		; Make bit#0 one, others zero
		out		PORTA, tmp
		ldi		tmp, 5
		rcall	sev_seg_letters 
		out		PORTC, tmp
		rcall	DelayS

		;; Display n
		ldi		tmp, (1<<1)     ; Make bit#1 one, others zero
		out		PORTA, tmp
		ldi		tmp, 12
		rcall	sev_seg_letters
		out		PORTC, tmp
		rcall	DelayS

		;; Display o
		ldi		tmp, (1<<2)     ; Make bit#2 one, others zero
		out		PORTA, tmp
		ldi		tmp, 13
		rcall	sev_seg_letters
		out		PORTC, tmp
		rcall	DelayS

		;; Display d
		ldi		tmp, (1<<3)     ; Make bit#3 one, others zero
		out		PORTA, tmp
		ldi		tmp, 4
		rcall	sev_seg_letters
		out		PORTC, tmp
		rcall	DelayS

		rjmp	FINISH
		ret
	

TIMER0:
	push	tmp
	in		tmp, SREG
	push	tmp
	ldi		tmp, $00
	out		TCNT0, tmp			;reset timer counter
	ldi		tmp,40
	cp		tmp,slower			;to make time changes in each 1 second
	brne	return
	ldi		tmp,0
	mov		slower, tmp
	cpi		secOnes, 0			;whether time is up
	brne	start_decrement
	cpi		secTens, 0
	brne	start_decrement
	cpi		minute, 0
	brne	start_decrement
	rcall	BOM					;when time is over before finding letters
start_decrement:
	dec		secOnes
	cpi		secOnes, $FF
	brne	return
	ldi		secOnes, 9
	dec		secTens
	cpi		secTens, $FF
	brne	return
	ldi		secTens, 5
	dec		minute
	cpi		minute, $FF
	brne	return
	ldi		minute, 9
return:
	inc		slower
	pop		tmp
	out		SREG, tmp
	pop		tmp
	reti

TIM1_OVF:
   push		tmp    					; Store temporary register
   in		tmp,SREG
   push		tmp						; Store status register
   push		ZL
   push		ZH						; Store Z-Pointer
   push		r0						; Store R0 Register

   mov		XL_Tmp,XL_LUT			; The current OCR value is stored
   mov		XH_Tmp,XH_LUT			; at this position in the LUT
   add		XL_LUT,x_SW
   clr		tmp						; (tmp is cleared, but not the carry flag)
   adc		XH_LUT,tmp				; Refresh pointer for the next sample


   ldi		tmp,0x7f
   and		XL_Tmp,tmp              ; module 128 (samples number sine table)

   ldi		ZL,low(sine_tbl*2)		; Program memory is organized as words
   ldi		ZH,high(sine_tbl*2)		; so, a "label" is a word count 
   add		ZL,XL_Tmp
   clr		tmp
   adc		ZH,tmp                  ; Z is a pointer to the correct
                                    ; sine_tbl value

   lpm								; read the OCR value
   out		OCR1BL,r0               ; send the current value to PWM

   pop		r0                      ; Restore R0 Register
   pop		ZH
   pop		ZL                      ; Restore Z-Pointer
   pop		tmp
   out		SREG,tmp                ; Restore SREG
   pop		tmp                     ; Restore temporary register;
   reti

BOM:
	sbi		DDRD,PD4				; Set pin PD4 as output
	loop1:
		nop
		rjmp	loop1
	ret

RXC_INT:
	push	tmp2					; save TEMP register
    in		tmp2, SREG				; save status register
    push	tmp2
        
    in		RECCHAR, UDR			; read UART receive data
	ser		tmp2
    mov		RECFLAG, tmp2			; set received flag register
                
    pop		tmp2
    out		SREG, tmp2
    pop		tmp2
	reti

;;Convert letters to the corresponding pw encoding
CONVERT_ASCII:
	cpi		RECCHAR, $41
	breq	convert_A
	cpi		RECCHAR, $61
	breq	convert_A
	jmp		compare_B
	convert_A:
		ldi		RECCHAR, 1
		ret
	compare_B:
		cpi		RECCHAR, $42
		breq	convert_B
		cpi		RECCHAR, $62
		breq	convert_B
		jmp		compare_C
	convert_B:
		ldi		RECCHAR, 2
		ret
	compare_C:
		cpi		RECCHAR, $43
		breq	convert_C
		cpi		RECCHAR, $63
		breq	convert_C
		jmp		compare_D
	convert_C:
		ldi		RECCHAR, 3
		ret
	compare_D:
		cpi		RECCHAR, $44
		breq	convert_D
		cpi		RECCHAR, $64
		breq	convert_D
		jmp		compare_E
	convert_D:
		ldi		RECCHAR, 4
		ret
	compare_E:
		cpi		RECCHAR, $45
		breq	convert_E
		cpi		RECCHAR, $65
		breq	convert_E
		jmp		compare_F
	convert_E:	
	    ldi		RECCHAR, 5
		ret
	compare_F:
		cpi		RECCHAR, $46
		breq	convert_F
		cpi		RECCHAR, $66
		breq	convert_F
		jmp		compare_G
	convert_F:	
		ldi		RECCHAR, 6
		ret
	compare_G:
		cpi		RECCHAR, $47
		breq	convert_G
		cpi		RECCHAR, $67
		breq	convert_G
		jmp		compare_H
	convert_G:
		ldi		RECCHAR, 7
		ret
	compare_H:
		cpi		RECCHAR, $48
		breq	convert_H
		cpi		RECCHAR, $68
		breq	convert_H
		jmp		compare_I
	convert_H:
		ldi		RECCHAR, 8
		ret
	compare_I:
		cpi		RECCHAR, $49
		breq	convert_I
		cpi		RECCHAR, $69
		breq	convert_I
		jmp		compare_J
	convert_I:
		ldi		RECCHAR, 9
		ret
	compare_J:
		cpi		RECCHAR, $4A
		breq	convert_J
		cpi		RECCHAR, $6A
		breq	convert_J
		jmp		compare_L
	convert_J:
		ldi		RECCHAR, 10
		ret
	compare_L:
		cpi		RECCHAR, $4C
		breq	convert_L
		cpi		RECCHAR, $6C
		breq	convert_L
		jmp		compare_N
	convert_L:
		ldi		RECCHAR, 11
		ret
	compare_N:
		cpi		RECCHAR, $4E
		breq	convert_N
		cpi		RECCHAR, $6E
		breq	convert_N
		jmp		compare_O
	convert_N:
		ldi		RECCHAR, 12
		ret
	compare_O:
		cpi		RECCHAR, $4F
		breq	convert_O
		cpi		RECCHAR, $6F
		breq	convert_O
		jmp		compare_P
	convert_O:
		ldi		RECCHAR, 13
		ret
	compare_P:
		cpi		RECCHAR, $50
		breq	convert_P
		cpi		RECCHAR, $70
		breq	convert_P
		jmp		compare_Q
	convert_P:
		ldi		RECCHAR, 14
		ret
	compare_Q:
		cpi		RECCHAR, $51
		breq	convert_Q
		cpi		RECCHAR, $71
		breq	convert_Q
		jmp		compare_R
	convert_Q:
		ldi		RECCHAR, 15
		ret
	compare_R:
		cpi		RECCHAR, $52
		breq	convert_R
		cpi		RECCHAR, $72
		breq	convert_R
		jmp		compare_S
	convert_R:
		ldi		RECCHAR, 16
		ret
	compare_S:
		cpi		RECCHAR, $53
		breq	convert_S
		cpi		RECCHAR, $73
		breq	convert_S
		jmp		compare_T
	convert_S:
		ldi		RECCHAR, 17
		ret
	compare_T:
		cpi		RECCHAR, $54
		breq	convert_T
		cpi		RECCHAR, $74
		breq	convert_T
		jmp		compare_U
	convert_T:
		ldi		RECCHAR, 18
		ret
	compare_U:
		cpi		RECCHAR, $55
		breq	convert_U
		cpi		RECCHAR, $75
		breq	convert_U
		jmp		compare_Y
	convert_U:
		ldi		RECCHAR, 19
		ret
	compare_Y:
		cpi		RECCHAR, $59
		breq	convert_Y
		cpi		RECCHAR, $79
		breq	convert_Y
		jmp		missmatch
	convert_Y:
		ldi		RECCHAR, 0
		ret
	missmatch: 
		ldi		RECCHAR, 32
		ret

;whether entered letter is in password
;if it is buzz its morse code
CONTROL_INPUT:
	cp		RECCHAR,pw1
	brne	p2
	sbrc	forOnce,7				;whether this letter founded before
	rjmp	p2			
	ldi		tmp, $80
	add		forOnce,tmp
	inc		numOfFounded			;increment number of founded letters
	rjmp	setFounded
	p2: cp	RECCHAR,pw2
	brne	p3
	sbrc	forOnce,6				;whether this letter founded before
	rjmp	p3
	ldi		tmp, $40
	add		forOnce,tmp
	inc		numOfFounded			;increment number of founded letters
	rjmp	setFounded
	p3: cp	RECCHAR,pw3
	brne	p4
	sbrc	forOnce,5				;whether this letter founded before
	rjmp	p4
	ldi		tmp, $20
	add		forOnce,tmp
	inc		numOfFounded			;increment number of founded letters
	rjmp	setFounded
	p4: cp	RECCHAR,pw4
	brne	wrong_letter
	sbrc	forOnce,4				;whether this letter founded before
	rjmp	wrong_letter
	ldi		tmp, $10
	add		forOnce, tmp
	inc		numOfFounded			;increment number of founded letters
	rjmp	setFounded
	wrong_letter:
		sbi		DDRD,PD4		; Set pin PD4 as output
		rcall	delay
		rcall	delay
		rcall	delay
		rcall	delay
		rcall	delay
		rcall	delay
		rcall	delay
		rcall	delay
		rcall	delay
		rcall	delay
		cbi		DDRD,PD4		; to silence the buzzer
		ret

setFounded:							;place letters in the order in which they were found
	mov		tmp, numOfFounded
	cpi		tmp, 1
	brne	f2
	mov		found1, RECCHAR
	rcall	MORSE
	ret
	f2:		cpi tmp, 2
	brne	f3
	mov		found2, RECCHAR
	rcall	MORSE
	ret
	f3:		cpi tmp, 3
	brne	f4
	mov		found3, RECCHAR
	rcall	MORSE
	ret
	f4:		mov found4, RECCHAR
	rcall	MORSE
	ret

MORSE:								;buzz the morse code of the letter
	cpi		RECCHAR, 1
	breq	MORSE_A
	jmp		comp_B


	MORSE_A:
		rcall	dot
		rcall	dash
		ret
	comp_B:
		cpi		RECCHAR, 2
		breq	MORSE_B
		jmp		comp_C
	MORSE_B:
		rcall	dash
		rcall	dot
		rcall	dot
		rcall	dot
		ret
	comp_C:
		cpi		RECCHAR, 3
		breq	MORSE_C
		jmp		comp_D
	MORSE_C:
		rcall	dash
		rcall	dot
		rcall	dash
		rcall	dot
		ret
	comp_D:
		cpi		RECCHAR, 4
		breq	MORSE_D
		jmp		comp_E
	MORSE_D:
		rcall	dash
		rcall	dot
		rcall	dot
		ret
	comp_E:
		cpi		RECCHAR, 5
		breq	MORSE_E
		jmp		comp_F
	MORSE_E:	
	    rcall	dot
	    ret
	comp_F:
		cpi		RECCHAR, 6
		breq	MORSE_F
		jmp		comp_G
	MORSE_F:	
		rcall	dot
		rcall	dot
		rcall	dash
		rcall	dot
		ret
	comp_G:
		cpi		RECCHAR, 7
		breq	MORSE_G
		jmp		comp_H
	MORSE_G:
		rcall	dash
		rcall	dash
		rcall	dot
		ret
	comp_H:
		cpi		RECCHAR, 8
		breq	MORSE_H
		jmp		comp_I
	MORSE_H:
		rcall	dot
		rcall	dot
		rcall	dot
		rcall	dot
		ret
	comp_I:
		cpi		RECCHAR, 9
		breq	MORSE_I
		jmp		comp_J
	MORSE_I:
		rcall	dot
		rcall	dot
		ret
	comp_J:
		cpi		RECCHAR, 10
		breq	MORSE_J
		jmp		comp_L
	MORSE_J:
		rcall	dot
		rcall	dash
		rcall	dash
		rcall	dash
		ret
	comp_L:
		cpi		RECCHAR, 11
		breq	MORSE_L
		jmp		comp_N
	MORSE_L:
		rcall	dot
		rcall	dash
		rcall	dot
		rcall	dot
		ret
	comp_N:
		cpi		RECCHAR, 12
		breq	MORSE_N
		jmp		comp_O
	MORSE_N:
		rcall	dash
		rcall	dot
		ret
	comp_O:
		cpi		RECCHAR, 13
		breq	MORSE_O
		jmp		comp_P
	MORSE_O:
		rcall	dash
		rcall	dash
		rcall	dash
		ret
	comp_P:
		cpi		RECCHAR, 14
		breq	MORSE_P
		jmp		comp_Q
	MORSE_P:
		rcall	dot
		rcall	dash
		rcall	dash
		rcall	dot
		ret
	comp_Q:
		cpi		RECCHAR, 15
		breq	MORSE_Q
		jmp		comp_R
	MORSE_Q:
		rcall	dash
		rcall	dash
		rcall	dot
		rcall	dash
		ret
	comp_R:
		cpi		RECCHAR, 16
		breq	MORSE_R
		jmp		comp_S
	MORSE_R:
		rcall	dot
		rcall	dash
		rcall	dot
		ret
	comp_S:
		cpi		RECCHAR, 17
		breq	MORSE_S
		jmp		comp_T
	MORSE_S:
		rcall	dot
		rcall	dot
		rcall	dot
		ret
	comp_T:
		cpi		RECCHAR, 18
		breq	MORSE_T
		jmp		comp_U
	MORSE_T:
		rcall	dash
		ret
	comp_U:
		cpi		RECCHAR, 19
		breq	MORSE_U
		jmp		comp_Y
	MORSE_U:
		rcall	dot
		rcall	dot
		rcall	dash
		ret
	comp_Y:
		cpi		RECCHAR, 0
		breq	MORSE_Y
		jmp		retur
	MORSE_Y:
		rcall	dash
		rcall	dot
		rcall	dash
		rcall	dash
		ret
		retur: ret

generate_pw:					;generate random password
	mov		tmp2, tmp
	add		tmp, tmp2
	add		tmp, tmp2
	ldi		tmp2, 7
	add		tmp, tmp2
generate_pw_loop:
	cpi		tmp, 20	
	brlo generate_pw_exit
	subi	tmp, 20
	rjmp generate_pw_loop
generate_pw_exit:
	ret

dot:
	rcall	delay
	sbi		DDRD,PD4			; Set pin PD4 as output
	rcall	delay				;making a delay for beep song
	cbi		DDRD,PD4			; to silence the buzzer
	ret
dash:
	rcall	delay
	sbi		DDRD,PD4			; Set pin PD4 as output
	rcall	delay				;making a delay for beep song
	rcall	delay
	rcall	delay
	cbi		DDRD,PD4			; to silence the buzzer
	ret

sev_seg_display:
	cpi		tmp, 0
	breq	dis_0
	cpi		tmp, 1
	breq	dis_1
	cpi		tmp, 2
	breq	dis_2
	cpi		tmp, 3
	breq	dis_3
	cpi		tmp, 4
	breq	dis_4
	cpi		tmp, 5
	breq	dis_5
	cpi		tmp, 6
	breq	dis_6
	cpi		tmp, 7
	breq	dis_7
	cpi		tmp, 8
	breq	dis_8
	cpi		tmp, 9
	breq	dis_9
	ret
dis_0:
	ldi		tmp,0b00111111
	ret
dis_1:
	ldi		tmp,0b00000110
	ret
dis_2:
	ldi		tmp,0b01011011
	ret
dis_3:
	ldi		tmp,0b01001111
	ret
dis_4:
	ldi		tmp,0b01100110
	ret
dis_5:
	ldi		tmp,0b01101101
	ret
dis_6:
	ldi		tmp,0b01111101
	ret
dis_7:
	ldi		tmp,0b00000111
	ret
dis_8:
	ldi		tmp,0b01111111
	ret
dis_9:
	ldi		tmp,0b01101111
	ret

sev_seg_letters:
	cpi		tmp, 0
	breq	dis_Y
	cpi		tmp, 1
	breq	dis_A
	cpi		tmp, 2
	breq	dis_B
	cpi		tmp, 3
	breq	dis_C
	cpi		tmp, 4
	breq	dis_D
	cpi		tmp, 5
	breq	dis_E
	cpi		tmp, 6
	breq	dis_F
	cpi		tmp, 7
	breq	dis_G
	cpi		tmp, 8
	breq	dis_H
	cpi		tmp, 9
	breq	dis_I
	cpi		tmp, 10
	breq	dis_J
	cpi		tmp, 11
	breq	dis_L
	cpi		tmp, 12
	breq	dis_N
	cpi		tmp, 13
	breq	dis_O
	cpi		tmp, 14
	breq	dis_P
	cpi		tmp, 15
	breq	dis_Q
	cpi		tmp, 16
	breq	dis_R
	cpi		tmp, 17
	breq	dis_S
	cpi		tmp, 18
	breq	dis_T
	cpi		tmp, 19
	breq	dis_U
	ret
dis_Y:
	ldi		tmp,0b01100110
	ret
dis_A:
	ldi		tmp,0b01110111
	ret
dis_B:
	ldi		tmp,0b01111100
	ret
dis_C:
	ldi		tmp,0b00111001
	ret
dis_D:
	ldi		tmp,0b01011110
	ret
dis_E:
	ldi		tmp,0b01111001
	ret
dis_F:
	ldi		tmp,0b01110001
	ret
dis_G:
	ldi		tmp,0b01101111
	ret
dis_H:
	ldi		tmp,0b01110110
	ret
dis_I:
	ldi		tmp,0b00000110
	ret
dis_J:
	ldi		tmp,0b00001110
	ret
dis_L:
	ldi		tmp,0b00111000
	ret
dis_N:
	ldi		tmp,0b01010100
	ret
dis_O:
	ldi		tmp,0b01011100
	ret
dis_P:
	ldi		tmp,0b01110011
	ret
dis_Q:
	ldi		tmp,0b01100111
	ret
dis_R:
	ldi		tmp,0b01010000
	ret
dis_S:
	ldi		tmp,0b01101101
	ret
dis_T:
	ldi		tmp,0b01111000
	ret
dis_U:
	ldi		tmp,0b00111110
	ret

;; Delay subroutine for seven segment
DelayS:	
	ldi		Delay0, $00
	ldi		Delay1, $05
Wait:	
	subi	Delay0, 1
	sbci	Delay1, 0
	brcc	Wait
	ret

; function delay for morse
delayss:		
		ldi		tmp,$FF			; load immediate (1)
d1:		dec		tmp				; decrement (1)
		brne	d1				; branch on not equal (2/1)
		ret						; return (4)

;*** Subroutine long delay (0.199 sec @ 1MHz) ******************************
delay:	
		ser		tmp2			; load $FF (1)
ld1:	rcall	delayss			; call (3)
		dec		tmp2			; decrement temp2 (1)
		brne	ld1				; branch on not equal (2/1)
		ret						; return (4)


;*************************** SIN TABLE *************************************
; Samples table : one period sampled on 128 samples and
; quantized on 7 bit
;******************************************************************************
sine_tbl:
.db	64,67
.db 70,73
.db 76,79
.db 82,85
.db 88,91
.db 94,96
.db 99,102
.db 104,106
.db 109,111
.db 113,115
.db 117,118
.db 120,121
.db 123,124
.db 125,126
.db 126,127
.db 127,127
.db 127,127
.db 127,127
.db 126,126
.db 125,124
.db 123,121
.db 120,118
.db 117,115
.db 113,111
.db 109,106
.db 104,102
.db 99,96
.db 94,91
.db 88,85
.db 82,79
.db 76,73
.db 70,67
.db 64,60
.db 57,54
.db 51,48
.db 45,42
.db 39,36
.db 33,31
.db 28,25
.db 23,21
.db 18,16
.db 14,12
.db 10,9
.db 7,6
.db 4,3
.db 2,1
.db 1,0
.db 0,0
.db 0,0
.db 0,0
.db 1,1
.db 2,3
.db 4,6
.db 7,9
.db 10,12
.db 14,16
.db 18,21
.db 23,25
.db 28,31
.db 33,36
.db 39,42
.db 45,48
.db 51,54
.db 57,60