    .cseg
    .org        0x00

    .def    r_acc = r0
    .def    r_acc_b = r1
    .def    r_zero = r2             ; always zero
    .def    r_unprotect = r3        ; set to unprotect signature
    .def    r_status = r4           ; holds status register during interupt

    .def    r_tmp = r16             ; temporary register
    .def    r_tmp_h = r17

    .def    rx = r26                ; temporary register
    .def    rx_h = r27
    .def    ry = r28
    .def    ry_h = r29
    .def    rz = r30
    .def    rz_h = r31


    .macro  unlock
    out     CPU_CCP,r_unprotect
    .endmacro

    ; reset and interupt vector table
    rjmp    power_on_reset          ; RESET 
    rjmp    not_implemented         ; CRCSCAN_NMI     
    rjmp    low_voltage             ; BOD_VLM
    rjmp    not_implemented         ; PORTA_PORT
    rjmp    not_implemented         ; PORTB_PORT
    rjmp    not_implemented         ; not used???
    rjmp    not_implemented         ; RTC_CNT
    rjmp    not_implemented         ; RTC_PIT
    rjmp    not_implemented         ; TCA0_LUNF / TCA0_OVF
    rjmp    not_implemented         ; TCA_HUNF
    rjmp    not_implemented         ; TCA0_LCMP0 / TCA_CMP0
    rjmp    not_implemented         ; TCA0_LCMP1 / TCA_CMP1
    rjmp    not_implemented         ; TCA0_LCMP2 / TCA_CMP2
    rjmp    not_implemented         ; TCB0_INT
    rjmp    not_implemented         ; TCD0_OVF
    rjmp    not_implemented         ; TCD0_TRIG
    rjmp    not_implemented         ; AC0_AC
    rjmp    adc_result_ready        ; ADC0_RESRDY
    rjmp    not_implemented         ; ADC0_WCOMP
    rjmp    not_implemented         ; TWI0_TWIS
    rjmp    not_implemented         ; TWI0_TWIM
    rjmp    not_implemented         ; SPI0_INT
    rjmp    not_implemented         ; USART0_RXC
    rjmp    not_implemented         ; USART0_DRE
    rjmp    not_implemented         ; USART0_TXC
    rjmp    not_implemented         ; NVMCTRL_EE

low_voltage:
    ldi     r_tmp,BOD_VLMIF_bm
    sts     BOD_INTFLAGS,r_tmp          ; clear bod interupt flag
    reti

not_implemented:
power_on_reset:
    eor     r_zero,r_zero 
    out     CPU_SREG,r_zero             ; clear status register
    ldi     r_tmp,0xD8                  ; for ccp unprotect registers
    mov     r_unprotect,r_tmp
    
    ldi     r_tmp,LOW(INTERNAL_SRAM_END)  ; set stack pointer to top of memory
    ldi     r_tmp_h,HIGH(INTERNAL_SRAM_END)
    out     CPU_SPL,r_tmp
    out     CPU_SPH,r_tmp_h

;; configure clock
;; The clock is set to use the internal 20Mhz clock, with a divide by 2
;; prescaler, so the CPU will be running at 10Mhz

    unlock
    sts     CLKCTRL_MCLKCTRLA,r_zero
    unlock
    ldi     r_tmp,0b00000001            ; clock prescaler enabled, div by 2
    sts     CLKCTRL_MCLKCTRLB,r_tmp   
    
;; brown out detection
;; generally disabled for now, we do set some values

    unlock
    sts     BOD_CTRLA,r_zero            ; disable bod detection
    unlock
    ldi     r_tmp,0x04
    sts     BOD_CTRLB,r_tmp             ; brown out level = 3.3V
    sts     BOD_VLMCTRLA,r_zero         ; bod level 5% above level
    sts     BOD_INTCTRL,r_zero          ; level goes below, interupts disabled

;; sleep control - disabled
    sts     SLPCTRL_CTRLA,r_zero
 
;; disable watch dog timer
    unlock
    sts     WDT_CTRLA,r_zero

;; setup realtime counter 
;; 1Khz, no interupts
    ldi     r_tmp,0x01
	sts     RTC_CLKSEL,r_tmp
	sts     RTC_DBGCTRL,r_zero
	sts     RTC_INTCTRL,r_zero
wait_rtc:
	lds     r_tmp,RTC_STATUS
	and     r_tmp,r_tmp
	brne    wait_rtc
	ldi     r_tmp,0x01
	sts     RTC_CTRLA,r_tmp

;; Port A configuration
;; PA0 - not used (used as UPDI)
;; PA1 - Alt SDA
;; PA2 - Alt SCL
;; PA3 - Left/Right ID
;; PA4 - TCD WOA output
;; PA5 - TCD WOB output
;; PA6 - DAC output - speed sense
;; PA7 - Tilt/Kill - open collector, pull up

    sts     PORTA_DIR,r_zero            ; set all pins to input
    sts     PORTA_OUT,r_zero            ; outputs are set low, we set the direction to make it go low
    
    ldi     rx,LOW(PORTA_PIN0CTRL)
    ldi     rx_h,HIGH(PORTA_PIN0CTRL)
    st      x+,r_zero                   ; Pin A0 NO ISR
    st      x+,r_zero                   ; Pin A1 NO ISR
    st      x+,r_zero                   ; Pin A2 NO ISR
    st      x+,r_zero                   ; Pin A3 NO ISR
    st      x+,r_zero                   ; Pin A4 NO ISR
    st      x+,r_zero                   ; Pin A5 NO ISR
    st      x+,r_zero                   ; Pin A6 NO ISR
    ldi     r_tmp,0x08          
    st      x+,r_tmp                    ; Pin A7 NO ISR, Pull up enabled

;; Port B configuration
;; PB0 - AIN10 - analog in - speed input
;; PB1 - AIN11 - analog in - current sense
;; PB2 - Wheel Encoder in
;; PB3 - Wheel Encoder in

    sts     PORTB_DIR,r_zero            ; B0-B3 all inputs
    sts     PORTB_OUT,r_zero            ; set B0-B3 to low
    
    ldi     rx,LOW(PORTB_PIN0CTRL)
    ldi     rx_h,HIGH(PORTB_PIN0CTRL)
    st      x+,r_zero                   ; Pin B0 NO ISR
    st      x+,r_zero                   ; Pin B1 NO ISR
    st      x+,r_zero                   ; Pin B2 NO ISR
    st      x+,r_zero                   ; Pin B3 NO ISR

;; Port Multiplexer configuration
;; No alternate assignments, set everything to 0
    ldi     rx,LOW(PORTMUX_CTRLA)    
    ldi     rx_h,HIGH(PORTMUX_CTRLA)
    st      x+,r_zero                   ; PORTMUX CTRLA = 0
    ldi     r_tmp,0x11                  
    st      x+,r_tmp                    ; PORTMUX CTRLB = Alternate UART and TWI
    st      x+,r_zero                   ; PORTMUX CTRLC = 0
    st      x+,r_zero                   ; PORTMUX CTRLD = 0
 
;; Event Gen/User configuration
;; no events configured
    ldi     rx,LOW(EVSYS_ASYNCCH0)     
    ldi     rx_h,HIGH(EVSYS_ASYNCCH0)
    ldi     r_tmp,0x10                  ; async gen0 - Port A6
    st      x+,r_tmp
    st      x+,r_zero                   ; async gen1 - off
    st      x+,r_zero                   ; async gen2 - off
    st      x+,r_zero                   ; async gen3 - off
    st      x+,r_zero                   ; sync gen0 - off
    st      x+,r_zero                   ; sync gen1 - off
    ldi     rx,LOW(EVSYS_ASYNCUSER0)
    ldi     rx_h,HIGH(EVSYS_ASYNCUSER0)
    ldi     r_tmp,0x03                  ; TCB0 - use async gen0
    st      x+,r_tmp                    ; (this connects PA6 to TCB0)
    st      x+,r_zero                   ; ADC0 - no connection
    st      x+,r_zero                   ; CCL_LUT0EV0 - no connection
    st      x+,r_zero                   ; CCL_LUT1EV0 - no connection
    st      x+,r_zero                   ; CCL_LUT0EV1 - no connection
    st      x+,r_zero                   ; CCL_LUT1EV1 - no connection
    st      x+,r_zero                   ; TCD0_EV0 - no connection
    st      x+,r_zero                   ; TCD0_EV1 - no connection
    st      x+,r_zero                   ; EVOUT0 - no connection
    st      x+,r_zero                   ; EVOUT1 - no connection
    st      x+,r_zero                   ; EVOUT2 - no connection
    ldi     rx,LOW(EVSYS_SYNCUSER0)
    ldi     rx_h,HIGH(EVSYS_SYNCUSER0)
    st      x+,r_zero                   ; TCA0 - no connection
    st      x+,r_zero                   ; USART0 - no connection

;; set up internal voltage reference
    ldi     r_tmp,0x33                  ; ADC 4.3V, DAC 4.3V
    sts     VREF_CTRLA,r_tmp

;; set up ADC
    ldi     r_tmp,0x00                  ; accumulate 2 results
    sts     ADC0_CTRLB,r_tmp
    ldi     r_tmp,0b01010101            ; reduced capcitance, vdd reference, clock/64
    sts     ADC0_CTRLC,r_tmp
    sts     ADC0_CTRLD,r_zero           ; no delays
    sts     ADC0_CTRLE,r_zero           ; no comparator
    sts     ADC0_SAMPCTRL,r_zero        ; no extension is sampling
    ldi     r_tmp,0x0B                  ; mux = AIN11 (PB0 pin 9)
    sts     ADC0_MUXPOS,r_tmp
    sts     ADC0_EVCTRL,r_zero          ; no event control
    ldi     r_tmp,0x01                  ; interrupt on result ready
    sts     ADC0_INTCTRL,r_tmp
    sts     ADC0_DBGCTRL,r_zero         ; do not run in debug mode
    ldi     r_tmp,0b00000011            ; free run mode, enable ADC
    sts     ADC0_CTRLA,r_tmp
    ldi     r_tmp,0x01                  ; start conversion
    sts     ADC0_COMMAND,r_tmp

;; set up DAC
    ldi     r_tmp,0xC1
    sts     DAC0_CTRLA,r_tmp
    ldi     r_tmp,0x80
    sts     DAC0_DATA,r_tmp

;; Timer D configuration
;; Timer D is used to create the 10khz PWM signals for the H bridge
;; motor controller.  These PWM signals are output as WOA (PA4) and WOB (PA5)
;; Only one output is pulsed the other is held low, the speed is determined
;; by the pulse width and the direction is determined by which output is low

    sts     TCD0_CTRLB,r_zero           ; one ramp mode
    ldi     r_tmp,0x82
    sts     TCD0_CTRLC,r_tmp            ; CMPDSEL PWMB; CMPCSEL PWMA
    sts     TCD0_CTRLD,r_zero           ; no overrides
    sts     TCD0_EVCTRLA,r_zero         ; disable event control
    sts     TCD0_EVCTRLB,r_zero
    sts     TCD0_INTCTRL,r_zero         ; disable interupts
    sts     TCD0_INPUTCTRLA,r_zero      ; disable input control
    sts     TCD0_INPUTCTRLB,r_zero
	unlock
    ldi     r_tmp,0x33
    sts     TCD0_FAULTCTRL,r_tmp        ; cmp A and cmp B enabled
    sts     TCD0_DLYCTRL,r_zero         ; disable delay control
    sts     TCD0_DLYVAL,r_zero          ; no delay value
    sts     TCD0_DITCTRL,r_zero         ; no dithering
    sts     TCD0_DITVAL,r_zero
    sts     TCD0_DBGCTRL,r_zero         ; disable in debug mode
    
    ldi     r_tmp,LOW(1000)             ; set clear compare to 2000
    ldi     r_tmp_h,HIGH(1000)
    sts     TCD0_CMPACLRL,r_tmp         ; this will give us a 10khz pwm
    sts     TCD0_CMPACLRH,r_tmp_h    
    sts     TCD0_CMPBCLRL,r_tmp
    sts     TCD0_CMPBCLRH,r_tmp_h
    
    ;; set the cmpA set and cmpB set to zero - full stop
    sts     TCD0_CMPASETL,r_zero
    sts     TCD0_CMPASETH,r_zero
    sts     TCD0_CMPBSETL,r_zero
    sts     TCD0_CMPBSETH,r_zero

    ldi     r_tmp,0x01                  ; probably not needed yet
    sts     TCD0_CTRLE,r_tmp
    
wait_for_timer_d_ready:
    lds     r_tmp,TCD0_STATUS
    sbrs    r_tmp,0
    rjmp    wait_for_timer_d_ready

    sts     TCD0_STATUS,r_zero 
    ldi     r_tmp,0x01                  ; 20Mhz clock, enable timer
    sts     TCD0_CTRLA,r_tmp


;; Interupt configuration
    out     CPU_CCP,r18
    sts     CPUINT_CTRLA,r_zero
    sts     CPUINT_LVL0PRI,r_zero
    sts     CPUINT_LVL1VEC,r_zero
    sei                                 ; enable interupts


loop:
    rjmp    loop


;; Interupt handler - ADC result is ready
adc_result_ready:
    in      r_status,CPU_SREG           ; save status register
    lds     r_tmp,ADC0_RESL             ; get result of ADC conversion
    lds     r_tmp_h,ADC0_RESH

    subi    r_tmp_h,0x2                 ; normalize
    brmi    reverse_dir

    sts     TCD0_CMPASETL,r_tmp
    sts     TCD0_CMPASETH,r_tmp_h
    sts     TCD0_CMPBSETL,r_zero
    sts     TCD0_CMPBSETH,r_zero
    ldi     r_tmp,0x01                  ; update values on next cycle
    sts     TCD0_CTRLE,r_tmp
    rjmp    adc_return

reverse_dir:
    com     r_tmp_h                     ; negate value and pulse other output
    neg     r_tmp
    sbci    r_tmp_h,255

    sts     TCD0_CMPBSETL,r_tmp
    sts     TCD0_CMPBSETH,r_tmp_h
    sts     TCD0_CMPASETL,r_zero
    sts     TCD0_CMPASETH,r_zero
    ldi     r_tmp,0x01                  ; update values on next cycle
    sts     TCD0_CTRLE,r_tmp

adc_return:
    ldi     r_tmp,0x01                 ; clear result ready flag
    sts     ADC0_INTFLAGS,r_tmp
    out     CPU_SREG,r_status          ; restore status register
    reti
