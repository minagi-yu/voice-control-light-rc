;-------------------------------------------------------------------------
; Voice control light - IR remote controller
;
; Copylight (C) 2025 Minagi Yu
;-------------------------------------------------------------------------

; ATtiny10
; PB0: Out, IR Out (OC0A)
; PB1: In,  Status in
; PB2: Out, NC (Debug LED)
; PB3: In,  NC (Reset)

.include "tn10def.inc"

.def temp       = r16
.def zero       = r17

.equ S_PORT     = PORTB
.equ S_PIN      = PINB
.equ S_PUE      = PUEB
.equ S_BIT      = 1
.equ S_DDR      = DDRB
.equ IR_PORT    = PORTB
.equ IR_BIT     = 0
.equ IR_DDR     = DDRB

;-------------------------------------------------------------------------
; Data Section
.dseg
.org SRAM_START

;-------------------------------------------------------------------------
; Code Section
.cseg
.org 0
;
; Interrupt Vectors
;
vectors:
vector_reset:               ; Reset Handler
    rjmp    reset
vector_int0:                ; IRQ0 Handler
;   rjmp    isr_int0
    rjmp    reset
vector_pcint0:              ; PCINT0 Handler
    rjmp    isr_pcint0
;   rjmp    reset
vector_tim0_capt:           ; Timer0 Capture Handler
;   rjmp    isr_tim0_capt
    rjmp    reset
vector_tim0_ovf:            ; Timer0 Overflow Handler
;   rjmp    isr_tim0_ovf
    rjmp    reset
vector_tim0_compa:          ; Timer0 Compare A Handler
;   rjmp    isr_tim0_compa
    rjmp    reset
vector_tim0_compb:          ; Timer0 Compare B Handler
;   rjmp    isr_tim0_compb
    rjmp    reset
vector_ana_comp:            ; Analog Comparator Handler
;   rjmp    isr_ana_comp
    rjmp    reset
vector_wdt:                 ; Watchdog Interrupt Handler
;   rjmp    isr_wdt
    rjmp    reset
vector_vlm:                 ; Voltage Level Monitor Handler
;   rjmp    isr_vlm
    rjmp    reset
vector_adc:                 ; ADC Conversion Handler
;   rjmp    isr_adc
    rjmp    reset

;
; Interrupt Service Routine
;
isr_pcint0:
    ldi     temp, 0b00000100        ; Sleep disable
    out     SMCR, temp              ; v
    cbi     PCICR, 0                ; Pin change interrupt disable

    reti

;
; Startup routine
;
reset:
    clr     zero                    ; Clear zero register
    ldi     YH, high(SRAM_START)    ; Clear RAM
    ldi     YL, low(SRAM_START)     ; |
    ldi     temp, SRAM_SIZE         ; |
    st      Y+, zero                ; |
    dec     temp                    ; |
    brne    PC-2                    ; v
;   rjmp    main                    ; Go to main

;
; Main routine
;
main:
;    sbi     S_PUE, S_BIT            ; Enable pull up

    ldi     temp, (1 << S_BIT)      ; Unmask pin change interrupt PB1
    out     PCMSK, temp             ; v

    ldi     temp, 12                ; f_oc0a=1MHz/(2*1*(1+12))=38kHz
    out     OCR0AL, temp            ; v

    ldi     temp, 0b01000000        ; OCR0A(PB0) toggle enable
    out     TCCR0A, temp            ; v
    ldi     temp, 0b00001001        ; CTC Mode, no prescaling clock
    out     TCCR0B, temp            ; v

    sbi     DDRB, 2

loop:                               ; Main Loop
    sbi     PCICR, 0                ; Pin change interrupt enable
    sbi     PCIFR, 0                ; Clear interrupt flag
    sei                             ; Interrupt enable
    ldi     temp, 0b00000101        ; Enter Power-down mode
    out     SMCR, temp              ; |
    sleep                           ; v

    ; Wake up
    cli                             ; Interrupt disable

    ldi     r24, 10                 ; Wait 10ms
    rcall   delay_ms                ; v

    sbi     PORTB, 2
    in      r25, S_PIN              ; Get current status

    ldi     r24, 210                ; Wait 210ms
    rcall   delay_ms                ; v

    cbi     PORTB, 2
    in      temp, S_PIN             ; Get current status
    eor     r25, temp               ; Compare previous status
    sbrc    r25, S_BIT              ; if (status changed)
    rjmp    loop                    ; then goto sleep

    sbrc    temp, S_BIT                     ; if (light on)
    rjmp    PC+4                            ;
    ldi     ZH, high(0x4000+(light_on<<1))  ; then
    ldi     ZL, low(0x4000+(light_on<<1))   ;
    rjmp    PC+3                            ;
    ldi     ZH, high(0x4000+(light_off<<1)) ; else
    ldi     ZL, low(0x4000+(light_off<<1))  ;

send:
    ldi     temp, 3                 ; temp =3; do {
    ldi     r24, 5                  ;
    rcall   ir_send                 ; ir_send(r24=4, &Z);
    ldi     r24, 130                ;
    rcall   delay_ms                ; delay_ms(r24=130);
    dec     temp                    ;
    brne    PC-5                    ; } while (--temp);

    rjmp    loop

;
; delay_ms()
; Parameters:
; 	r24: time(ms)
; Returns:
; 	none
; Clobbered:
;   r24
;
delay_ms:
    push    r18
    ldi     r18, 100
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    dec     r18
    brne    PC-8
    dec     r24
    brne    PC-11
    pop     r18
    ret

;
; ir_send()
;
; Send AEHA protocol IR signal
;
; Parameters:
; 	r24: size(bytes)
;   Z  : address
; Returns:
; 	none
; Clobbered:
;   r24, Z
;
ir_send:
    push    r18
    push    r19
    push    r20
    ; Leader
    mov     r18, r24                ;
    sbi     IR_DDR, IR_BIT          ; IR ON
    ldi     r24, 8                  ;
    rcall   delay_425us             ;
    cbi     IR_DDR, IR_BIT          ; IR OFF
    ldi     r24, 4                  ;
    rcall   delay_425us             ;
    ; Data
                                    ; do {
    ld      r19, Z+                 ;   r19 = *Z
    ldi     r20, 8                  ;   r20 = 8
                                    ;   do {
    sbi     IR_DDR, IR_BIT          ;     IR ON
    ldi     r24, 1                  ;     delay_425us(1)
    rcall   delay_425us             ;     v
    cbi     IR_DDR, IR_BIT          ;     IR OFF
    ldi     r24, 3                  ;     r24 = 3
    sbrs    r19, 7                  ;     if (!(r19 & 0x80))
    ldi     r24, 1                  ;       r24 = 1;
    lsl     r19                     ;     r19 <<= 1
    rcall   delay_425us             ;     delay_425us(r24)
    dec     r20                     ;   } while (--r20);
    brne    PC-10                   ;   v
    dec     r18                     ; } while (--r18);
    brne    PC-14                   ; v
    ; Trailer
    sbi     IR_DDR, IR_BIT          ; IR ON
    ldi     r24, 1                  ; delay_425us(1)
    rcall   delay_425us             ; v
    cbi     IR_DDR, IR_BIT          ; IR OFF
    ldi     r24, 8
    rcall   delay_ms
    pop     r20
    pop     r19
    pop     r18
    ret


;
; delay_425us()
; Parameters:
; 	r24: time(425us)
; Returns:
; 	none
; Clobbered:
;   r24
;
delay_425us:
    push    r18
    ldi     r18, 85
    nop
    nop
    dec     r18
    brne    PC-3
    dec     r24
    brne    PC-6
    pop     r18
    ret

;
; Constants
;
; Panasonic HK9493
light_off:
    .db 0b00110100, 0b01001010, 0b10010000, 0b11110100, 0b01100100, 0
light_on:
    .db 0b00110100, 0b01001010, 0b10010000, 0b00110100, 0b10100100, 0
