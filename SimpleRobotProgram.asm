; SimpleRobotProgram.asm
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.

ORG 0  ; Begin program at x000
;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)

WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	IN     SWITCHES
	OUT    BEEP
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	LOADI  0
	OUT    BEEP
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here.
	OUT     RESETPOS    ; reset odometer in case wheels moved after programming
	; Coordinate units are feet!
	LOAD	Zero
	STORE	XIdx
	STORE	YIdx

; Coordinate Reading Begins
CoordBegin:
	LOADI -5
	STORE temp 
	
Funk:
	LOAD   temp
	SUB    Five
	JZERO  Displaytemp
	JNEG   Displaytemp
	LOADI  -5
	STORE  temp
Displaytemp:
	LOAD   temp 
	OUT    LCD
	IN      XIO
	AND     Mask1
	JZERO   AllowToStore
	IN     XIO 
	AND    Mask0
	JPOS   Funk
	JUMP   Pressed
	OUT    XLEDS

	IN     XIO
	AND    Mask1
	JPOS   Funk
	LOADI  0
	STORE  temp
	JPOS   Funk

	IN     XIO
	AND    Mask1
	JZERO  Funk ; might have to check this after the zero check on PB2 (next round of testing)
	JPOS   Funk
	OUT    XLEDS
	LOADI  0
	STORE  temp
	JUMP   Funk
  
Pressed:
  	IN     XIO 
  	AND    Mask0
  	JZERO   Pressed
  
Increment:   
	LOAD   temp
	ADD    One
	STORE  temp
	OUT    LCD
	IN      XIO
	AND     Mask1
	JZERO   AllowToStore
  	JUMP   Funk
  
  
AllowToStore:
	IN      XIO
	AND     Mask1
	JPOS    StartStore 
	JUMP    AllowToStore
	
StartStore:
	LOAD	temp
	STORE	m16sA
	LOAD	OneFoot
	STORE	m16sB
	CALL	Mult16s
	LOAD	mres16sL
	STORE	temp

	LOAD	XIdx
	SUB		Eleven
	JPOS	StoreY

StoreX:
	LOAD    temp
	CALL	StoreXCoord
	LOAD	XIdx
	ADD		One
	STORE	XIdx
	STORE	ActiveLoadIdx
	JUMP	EndStoreCheck

StoreY:
	LOAD    temp
	CALL	StoreYCoord
	LOAD	YIdx
	ADD		One
	STORE	YIdx
	STORE	ActiveLoadIdx

EndStoreCheck:
	LOAD	YIdx
	SUB		Eleven
	JPOS	DisplayPoints
	
	LOAD	ActiveLoadIdx
	SUB		Eleven
	JPOS	CleanupY
	JUMP	DisplayVal
	
CleanupY:
	LOAD	Zero
	STORE	ActiveLoadIdx
	
DisplayVal:
	LOAD	ActiveLoadIdx
	OUT		SSEG1
	
	LOAD	temp
	OUT     SSEG2

	JUMP	CoordBegin
; Coordinate Reading Over

DisplayPoints:
	CALL	Wait1
	LOAD	Zero
	STORE	XIdx
	STORE	YIdx

; Coords Confirmed
	
LoadPosition:
 	IN	    XPOS
 	STORE	CurXPos
 	
 	IN	    YPOS
 	STORE	CurYPos
	
SetupMinDistanceLoop:
	LOAD	Zero
	STORE	XIdx
	STORE	YIdx
	
	LOAD	GigaMax
	STORE	MinDistance
	
MinDistanceLoop:
	LOAD	XIdx
	CALL	LoadXCoord
	
	SUB		CurXPos
	STORE	L2X
	
	LOAD	YIdx
	CALL	LoadYCoord
	
	SUB		CurYPos
	STORE	L2Y
	
	CALL	L2Estimate
	STORE	distance
	SUB		MinDistance
	
	JPOS	ContinueDistanceLoop

UpdateMinDistance:
	LOAD	distance
	STORE	MinDistance
	
	LOAD	XIdx
	STORE	MinDex
	
	LOAD	L2X
	STORE	AtanX
	
	LOAD	L2Y
	STORE	AtanY
	
	CALL	Atan2
	STORE	MinAngle
	
ContinueDistanceLoop:
	LOAD	XIdx
	ADD		One
	STORE	XIdx
	
	LOAD	YIdx
	ADD		One
	STORE	YIdx
	
	SUB		Twelve
	JNEG	MinDistanceLoop

MoveRobot:
	LOAD	MinAngle
	CALL    TurnACDegrees ; Turn to correct angle
	
	LOAD	MinDistance
	CALL	MoveACUnits ; Move the correct amount
	
	LOAD	One
	OUT 	BEEP
	
	CALL	Wait1 ; Pause to prevent skew
	
	LOAD	Zero
	OUT 	BEEP
	
	LOAD	MinDex
	CALL	ResetXY ; Reset the current position so it is not reached again
	
	LOAD	NumMovements
	ADD		One
	STORE	NumMovements
	SUB		Eleven
	
	JPOS	Die ; Leave if all of the positions have been reached
	
	JUMP	LoadPosition
	
Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero         ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD         ; An indication that we are dead
Forever:
	JUMP   Forever      ; Do this forever.
	DEAD:  DW &HDEAD    ; Example of a "local" variable

;***************************************************************
;* User Defined Subroutines
;***************************************************************

; Subroutine to stall until button is pressed
Stall:
	STORE	temp
StallLoop:
	IN		XIO
	AND		Mask2
	JPOS	StallLoop
	LOAD	temp
	RETURN

; Subroutine to move the amount of units stored in the AC
MoveACUnits:
	STORE	Distance
	
	IN		LPOS
	STORE	StartPosL
	IN		RPOS
	STORE	StartPosR
	
	LOAD	FFast
	STORE	MoveACUnits_RSpeed
	STORE	MoveACUnits_LSpeed
	
CheckCurPos: ; for now just check left, maybe do both later
	IN		XPOS
	OUT		SSEG2
	IN		YPOS
	OUT		LCD	
	
CheckEnd:
	IN		LPOS
	SUB		StartPosL
	SUB		Distance
	JPOS	StopMotion

ContinueMotion: ; add the motion correction here based off of position
	LOAD	MoveACUnits_LSpeed
	OUT		LVELCMD
	LOAD	MoveACUnits_RSpeed
	OUT		RVELCMD
	JUMP	CheckCurPos

StopMotion:
	LOAD	Zero
	OUT		LVELCMD
	OUT		RVELCMD
	
	RETURN
	
; Subroutine to turn to the angle stored in AC
TurnACDegrees:
	STORE  DesTheta   ; store the turning angle
	IN	   THETA
	STORE  NowTheta
	
	SUB    DesTheta    ; subtract desired angle
	CALL   Mod360      ; remove negative numbers
	ADD    DegN180     ; test which semicircle error is in
	JPOS   NeedLeft    ; >180 means need left turn
	JUMP   NeedRight   ; otherwise, need right turn
NeedLeft:
	IN	   THETA
	STORE  NowTheta    ; update current theta
	LOAD   DesTheta
	SUB    NowTheta    ; get the turn error
	CALL   Mod360      ; fix errors around 0
	SUB    DeadZone
	JNEG   NoTurn      ; stop moving if close
TurnLeft:
	LOAD   FSlow         ; replace the 100 from before
	OUT    RVELCMD     ; set right wheel forward
	LOAD   RSlow 
	OUT    LVELCMD     ; set left wheel backwards
	JUMP   NeedLeft
	RETURN               ; exit ISR
NeedRight:
	IN	   THETA
	STORE  NowTheta
	LOAD   NowTheta
	SUB    DesTheta    ; get the turn error
	CALL   Mod360      ; fix errors around 0
	SUB    DeadZone
	JNEG   NoTurn      ; stop moving if close
TurnRight:
	LOAD   RSlow         ; replace the 100 from before
	OUT    RVELCMD     ; set right wheel backwards
	LOAD   FSlow 
	OUT    LVELCMD     ; set left wheel forward
	JUMP   NeedRight
	RETURN             ; exit ISR
	
NoTurn:
	LOAD   Zero
	OUT    LVELCMD
	OUT    RVELCMD
	RETURN

; Reset the input coordinates 
ResetXY:
	STORE  temp
	LOAD   XCoords
	ADD	   temp
	STORE  temp2
	LOAD   GigaMax
	ISTORE temp2
	
	STORE  temp
	LOAD   YCoords
	ADD	   temp
	STORE  temp2
	LOAD   GigaMax
	ISTORE temp2
	
	RETURN

; Subroutine to load X coordinate at index in AC into AC
LoadXCoord:
	STORE  temp
	LOAD   XCoords
	ADD	   temp
	STORE  temp
	ILOAD  temp
	RETURN

; Subroutine to load Y coordinate at index in AC into AC
LoadYCoord:
	STORE  temp
	LOAD   YCoords
	ADD	   temp
	STORE  temp
	ILOAD  temp
	RETURN

; Subroutine to store X coordinate in AC at index XIdx
StoreXCoord:
	STORE  temp
	LOAD   XCoords
	ADD	   XIdx
	STORE  temp2
	LOAD   temp
	ISTORE temp2
	RETURN

; Subroutine to store Y coordinate in AC at index yIdx
StoreYCoord:
	STORE  temp
	LOAD   YCoords
	ADD	   YIdx
	STORE  temp2
	LOAD   temp
	ISTORE temp2
	RETURN
		
;***************************************************************
;* Predefined Subroutines
;***************************************************************

; Subroutine to calculate arctan
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADD    Deg360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADD    Deg180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	XOR    NegOne
	ADD    One            ; negate the angle
	ADD    Deg270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	XOR    NegOne
	ADD    One            ; negate the angle
	ADD    Deg90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADD    Val256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADD    One            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOAD  Nine            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADD    NegOne
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL

; Subroutine to multiply by 16
Mult16s:
	LOAD  Zero
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOAD  Sixteen       ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ; add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ; subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADD    NegOne           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done

; Subroutine to mod the value in AC by 360
Mod360:
	JNEG   M360N       ; loop exit condition
	ADD    Neg360        ; start removing 360 at a time
	JUMP   Mod360      ; keep going until negative
M360N:
	ADD    Deg360         ; get back to positive
	JNEG   M360N       ; (keep adding 360 until non-negative)
	RETURN
	
; Subroutine to take abs(AC) and store in AC
Abs:
	JPOS   Abs_r
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

; Subroutine to wait (block) for 1 second
Wait1:
	STORE  temp
	OUT    TIMER
Wloop:
	IN     LIN
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	LOAD   temp
	RETURN

; Subroutine to calculate distance
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOAD   twofoursix       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOAD   onezerofour       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
	
; Division Subroutine
Div16s:
	LOAD  Zero
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOAD   Seventeen
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOAD   One
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOAD   Zero
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

;***************************************************************
;* Variables
;***************************************************************
; General
temp2:  DW 0
moveangle: DW 0
movedistance: DW 0
NumMovements: DW 0
ActiveLoadIdx:	DW 0

; Movement Function
MoveACUnits_RSpeed:	DW 0
MoveACUnits_LSpeed:	DW 0
StartPosL:	DW 0
StartPosR:	DW 0
Distance:	DW 0
DiffPos:	DW 0

; Turning Function
TurnAngle:	DW 0
temp:		DW 0
DesTheta:	DW 0
NowTheta: DW 0
DeadZone: DW 3
TCount: DW 0

; Distance Calculation
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0

; Multiplication
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

; arctan calculation
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

; Division
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

; GPTP Algorithm
MinDistance: DW 2000
MinDex:	 DW 0
MinAngle: DW 0
CurXPos: DW 0
CurYPos: DW 0
XIdx:   DW 0
YIdx:   DW 0

XCoords:    DW 2000
YCoords:	DW 2012

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

Neg45:		DW -45
NegEleven:	DW -11
negfive:	DW -5
NegOneHalf:	DW -.5
OneHalf:	DW .5
Eleven:	  DW 11
Twelve:		DW 12
Sixteen:	  DW 16
Seventeen: DW 17
FortyFive:	DW 45
Fifty:	  DW 50
OneHundred:	DW 100
onezerofour: DW 104
twofoursix: DW 246
yintercept:	DW 1480
Neg360:	DW -360
Val256: DW 256

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
OneFoot:  DW 293       ; ~2ft in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
DegN90:   DW -90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
DegN180:  DW -180      ; -180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 150       ; 100 is about the lowest velocity value that will move
RSlow:    DW -150
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500
GigaMax:  DW 20000

MinBatt:  DW 60       ; 10.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9

ORG 2000
DW 0 ; X Coords
DW 0
DW 0
DW 0
DW 0 ; X Coords
DW 0
DW 0
DW 0
DW 0 ; X Coords
DW 0
DW 0
DW 0
DW 0 ; Y Coords
DW 0
DW 0
DW 0
DW 0 ; Y Coords
DW 0
DW 0
DW 0
DW 0 ; Y Coords
DW 0
DW 0
DW 0