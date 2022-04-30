; SimpleRobotProgram.asm 
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.

ORG 0  ; Begin program at x000
;***************************************************************
;* Initialization
; ;***************************************************************
; Init:
; ; Always a good idea to make sure the robot
; ; stops in the event of a reset.
; LOAD   Zero
; OUT    LVELCMD     ; Stop motors
; OUT    RVELCMD
; OUT    SONAREN     ; Disable sonar (optional)
; OUT    BEEP        ; Stop any beeping (optional)
; 
; CALL   SetupI2C    ; Configure the I2C to read the battery voltage
; CALL   BattCheck   ; Get battery voltage (and end if too low).
; OUT    LCD         ; Display battery voltage (hex, tenths of volts)
; 
; WaitForSafety:
; ; This loop will wait for the user to toggle SW17.  Note that
; ; SCOMP does not have direct access to SW17; it only has access
; ; to the SAFETY signal contained in XIO.
; IN     SWITCHES
; OUT    BEEP
; ; Wait for safety switch to be toggled
; IN     XIO         ; XIO contains SAFETY signal
; AND    Mask4       ; SAFETY signal is bit 4
; JPOS   WaitForUser ; If ready, jump to wait for PB3
; IN     TIMER       ; We'll use the timer value to
; AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
; SHIFT  8           ; Shift over to LED17
; OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
; JUMP   WaitForSafety
; 
;
; 
; WaitForUser:
; ; This loop will wait for the user to press PB3, to ensure that
; ; they have a chance to prepare for any movement in the main code.
; LOADI  0
; OUT    BEEP
; ; Wait for user to press PB3
; IN     TIMER       ; We'll blink the LEDs above PB3
; AND    Mask1
; SHIFT  5           ; Both LEDG6 and LEDG7
; STORE  Temp        ; (overkill, but looks nice)
; SHIFT  1
; OR     Temp
; OUT    XLEDS
; IN     XIO         ; XIO contains KEYs
; AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
; JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
; LOAD   Zero
; OUT    XLEDS       ; clear LEDs once ready to continue
;CALL   StartLoading ;;;;;fdkjaslfu493eifjoidjaskl;ajfkldjasfldajsfkljadsfkjdkasfjkldjasfkljdadfklj

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here. 

;CALL   StartLoading
LOADI -5
STORE temp 

Funk:
;LOADI   0
;STORE   temp 
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
  ;JZERO  Funk
;   JUMP   Pressed
;   IN     XIO
;   AND    Mask0
  JPOS   Funk
  JUMP   Pressed
;   LOAD   temp
;   ADD    One
;   STORE  temp
  
  OUT    XLEDS
  
  IN     XIO
  AND    Mask1
  JPOS   Funk
  ;OUT    XLEDS
  LOADI  0
  STORE  temp
  JPOS   Funk
;   IN     XIO
;   AND    Mask1
;   JZERO  Funk
;   JUMP   Funk
  
  
  
  IN     XIO
  AND    Mask1
  JZERO  Funk ; might have to check this after the zero check on PB2 (next round of testing)
  JPOS   Funk
  OUT    XLEDS
  LOADI  0
  STORE  temp
  JUMP   Funk
  ;CALL   Pressed
  ;CALL   Increment
  
Pressed:
  IN     XIO 
  AND    Mask0
  JZERO   Pressed
  
Increment:   
  LOAD   temp
  ADD    One
    STORE  temp
  OUT    LCD
  ;OR     Load&StoreA
  ;JZERO  Funk
  ;JPOS   Funk
  ;JUMP   Funk

  ;CALL   Load&StoreA
  ;CALL   Load&StoreB
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
LOAD    temp
OUT     SSEG2  
; Load&StoreA:
; IN XIO
; AND     Mask1
; 
; JZERO   Load&StoreA
; OUT XLEDS
;   
; Load&StoreB:
; IN XIO
; AND     Mask1
; JPOS    Load&StoreB
; ;IN      TIMER ; maybe time it would work?
; ;LOAD    Zero
; ;AND     Mask2
; OUT     XLEDS ;check if its been stored
; LOADI   0
; STORE   temp
; JUMP    Funk
; 
; IntoX:
; LOADI   XCoords
; LOAD    count12X
;JPOS    
;***********
;Coordinate entry code
;***********
StartLoading: ; call on startup
LOADI 0
;OUT BEEP
CALL LoadingValsX ;need to finish this off dummty (JFKLJKLSNAKLDNKFHEIF ATTENTION) 
CALL    LoadingValsY
LoadingValsX:
LOAD    count12X
JPOS    count12X
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsX ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsX
AND Mask1 ; PB3
JPOS    LoadingValsX
;imagine a tab in here
IN      XCoords ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    XIdx
ADD     One
LOAD    count12X
SUB     One
JUMP    LoadingValsX
LoadingValsY:
LOAD    count12Y
JPOS    count12Y
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsY ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsY
AND Mask1 ; PB3
JPOS    LoadingValsY
;imagine a tab in here
IN      YCoords  ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    YIdx
ADD     One
LOAD    count12Y
SUB     One
JUMP    LoadingValsY
WeMadeIt:
LOAD    Zero
JPOS    WeMadeIt
LOAD    XIdx
SUB     One
LOAD    count
ADD     One
JUMP    WeMadeIt
JPOS    WeMadeIt
LOAD    count
SUB     One
AND     Mask1
OUT XLEDS
OUT BEEP


; OUT    RESETPOS    ; reset odometer in case wheels moved after programming 
; CALL   StartLoading

; The following code ("GetAngle" through "DeadZone") is purely for example.
; It attempts to gently keep the robot facing 0 degrees, showing how the
; odometer and motor controllers work.
; GetAngle:
; ; The 0/359 jump in THETA can be difficult to deal with.
; ; This code shows one way to handle it: by converting it
; ; to a +/-180 offset from heading 0.
; IN     THETA       ; get the current angular position
; ADDI   -180        ; test whether facing 0-179 or 180-359
; JPOS   NegAngle    ; robot facing 180-360; handle that separately
; PosAngle:
; ADDI   180         ; undo previous subtraction
; JUMP   CheckAngle  ; THETA positive, so carry on
; NegAngle:
; ADDI   -180        ; finish conversion to negative angle:
;                   ;  angles 180 to 359 become -180 to -1
; 
; CheckAngle:
; ; AC now contains the +/- angular difference from 0
; OUT    LCD         ; Good data to display for debugging
; JPOS   TurnRight   ; handle +/- separately
; TurnLeft:
; ; If the angle is small, we don't want to do anything.
; ADD    DeadZone
; JPOS   NoTurn
; ; otherwise, turn CCW
; LOAD   RSlow
; JUMP   SendToMotors
; TurnRight:
; SUB    DeadZone    ; if near 0, don't turn
; JNEG   NoTurn
; LOAD   FSlow
; JUMP   SendToMotors
; NoTurn:
; LOADI  0           ; new LOADI instruction
; JUMP   SendToMotors
; 
; DeadZone:  DW 5        ; Note that you can place data anywhere.
;                        ; Just be careful that it doesn't get executed.
; 
; SendToMotors:
; ; Since we want to spin in place, we need to send inverted
; ;  velocities to the wheels.  The speed in AC is used for
; ;  the left wheel, and its negative is used for the right wheel
; STORE  Temp        ; store calculated desired velocity
; ; send the direct value to the left wheel
; OUT    LVELCMD
; OUT    SSEG1       ; for debugging purposes
; ; send the negated number to the right wheel
; LOADI  0
; SUB    Temp        ; AC = 0 - velocity
; OUT    RVELCMD 
; OUT    SSEG2       ; debugging
; 
; JUMP   GetAngle    ; repeat forever

; Die:
; ; Sometimes it's useful to permanently stop execution.
; ; This will also catch the execution if it accidentally
; ; falls through from above.
; LOAD   Zero         ; Stop everything.
; OUT    LVELCMD
; OUT    RVELCMD
; OUT    SONAREN
; LOAD   DEAD         ; An indication that we are dead
; OUT    SSEG2
; Forever:
; JUMP   Forever      ; Do this forever.
; DEAD:  DW &HDEAD    ; Example of a "local" variable

;***************************************************************
;* User Defined Subroutines
;***************************************************************

; Subroutine to move the amount of units stored in the AC
; MoveACUnits:
; CheckDir:
; LOAD FFast
; STORE MoveACUnits_RSpeed
; STORE MoveACUnits_LSpeed
; 
; CheckCurPos:
; IN LPOS
; 
; ContinueMotion:
; LOAD MoveACUnits_Speed
; OUT LVELCMD
; OUT RVELCMD
; JUMP CheckCurPos
; 
; StopMotion:
; LOAD Zero
; OUT LVELCMD
; OUT RVELCMD
; 
; RETURN
; 
; ; Subroutine to turn to the angle stored in AC
; TurnACDegreees:
; JUMP TurnACDegreees
; RETURN
;***************************************************************
;* Predefined Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
OUT    TIMER
Wloop:
IN     LIN
OUT    SSEG2
IN     TIMER
OUT    XLEDS       ; User-feedback that a pause is occurring.
ADDI   -10         ; 1 second in 10Hz.
JNEG   Wloop
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
MoveACUnits_RSpeed: DW 0
MoveACUnits_LSpeed: DW 0
StartPosL: DW 0
StartPosR: DW 0


;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
temp: DW 0 ;hjfdkh;asf9ioqp35rh'danjsxc'aofiu=8934jdfklal
count:  DW 0
Twelve: DW 12
XIdx:   DW 0
YIdx:   DW 0
XCoords: DW 2000
YCoords: DW 2004
count12X: DW 12
count12Y: DW 12



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

Leeway: DW 10 ; radius allowed around exact X,Y coordinate
; to display message (can make bigger LOL)

OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 100       ; 10.0V - minimum safe battery voltage
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
 
From: Dimitrova, Denitsa G <ddimitrova3@gatech.edu>
Sent: Friday, April 29, 2022 6:42 AM
To: Dimitrova, Denitsa G <ddimitrova3@gatech.edu>
Subject: Re: ASM CODE FOR THE ROBOT - AS OF 4/28 
 
SHE WORKS JUST NEED TO INPUT ARRAY STORAGE NOW
---------------------------------------------------------------------------
; SimpleRobotProgram.asm 
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.

ORG 0  ; Begin program at x000
;***************************************************************
;* Initialization
; ;***************************************************************
; Init:
; ; Always a good idea to make sure the robot
; ; stops in the event of a reset.
; LOAD   Zero
; OUT    LVELCMD     ; Stop motors
; OUT    RVELCMD
; OUT    SONAREN     ; Disable sonar (optional)
; OUT    BEEP        ; Stop any beeping (optional)
; 
; CALL   SetupI2C    ; Configure the I2C to read the battery voltage
; CALL   BattCheck   ; Get battery voltage (and end if too low).
; OUT    LCD         ; Display battery voltage (hex, tenths of volts)
; 
; WaitForSafety:
; ; This loop will wait for the user to toggle SW17.  Note that
; ; SCOMP does not have direct access to SW17; it only has access
; ; to the SAFETY signal contained in XIO.
; IN     SWITCHES
; OUT    BEEP
; ; Wait for safety switch to be toggled
; IN     XIO         ; XIO contains SAFETY signal
; AND    Mask4       ; SAFETY signal is bit 4
; JPOS   WaitForUser ; If ready, jump to wait for PB3
; IN     TIMER       ; We'll use the timer value to
; AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
; SHIFT  8           ; Shift over to LED17
; OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
; JUMP   WaitForSafety
; 
;
; 
; WaitForUser:
; ; This loop will wait for the user to press PB3, to ensure that
; ; they have a chance to prepare for any movement in the main code.
; LOADI  0
; OUT    BEEP
; ; Wait for user to press PB3
; IN     TIMER       ; We'll blink the LEDs above PB3
; AND    Mask1
; SHIFT  5           ; Both LEDG6 and LEDG7
; STORE  Temp        ; (overkill, but looks nice)
; SHIFT  1
; OR     Temp
; OUT    XLEDS
; IN     XIO         ; XIO contains KEYs
; AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
; JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
; LOAD   Zero
; OUT    XLEDS       ; clear LEDs once ready to continue
;CALL   StartLoading ;;;;;fdkjaslfu493eifjoidjaskl;ajfkldjasfldajsfkljadsfkjdkasfjkldjasfkljdadfklj

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here. 

;CALL   StartLoading
LOADI -5
STORE temp 

Funk:
;LOADI   0
;STORE   temp 
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
  ;JZERO  Funk
;   JUMP   Pressed
;   IN     XIO
;   AND    Mask0
  JPOS   Funk
  JUMP   Pressed
;   LOAD   temp
;   ADD    One
;   STORE  temp
  
  OUT    XLEDS
  
  IN     XIO
  AND    Mask1
  JPOS   Funk
  ;OUT    XLEDS
  LOADI  0
  STORE  temp
  JPOS   Funk
;   IN     XIO
;   AND    Mask1
;   JZERO  Funk
;   JUMP   Funk
  
  
  
  IN     XIO
  AND    Mask1
  JZERO  Funk ; might have to check this after the zero check on PB2 (next round of testing)
  JPOS   Funk
  OUT    XLEDS
  LOADI  0
  STORE  temp
  JUMP   Funk
  ;CALL   Pressed
  ;CALL   Increment
  
Pressed:
  IN     XIO 
  AND    Mask0
  JZERO   Pressed
  
Increment:   
  LOAD   temp
  ADD    One
    STORE  temp
  OUT    LCD
  ;OR     Load&StoreA
  ;JZERO  Funk
  ;JPOS   Funk
  ;JUMP   Funk

  ;CALL   Load&StoreA
  ;CALL   Load&StoreB
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
LOAD    temp
OUT     SSEG2  
; Load&StoreA:
; IN XIO
; AND     Mask1
; 
; JZERO   Load&StoreA
; OUT XLEDS
;   
; Load&StoreB:
; IN XIO
; AND     Mask1
; JPOS    Load&StoreB
; ;IN      TIMER ; maybe time it would work?
; ;LOAD    Zero
; ;AND     Mask2
; OUT     XLEDS ;check if its been stored
; LOADI   0
; STORE   temp
; JUMP    Funk
; 
; IntoX:
; LOADI   XCoords
; LOAD    count12X
;JPOS    
;***********
;Coordinate entry code
;***********
StartLoading: ; call on startup
LOADI 0
;OUT BEEP
CALL LoadingValsX ;need to finish this off dummty (JFKLJKLSNAKLDNKFHEIF ATTENTION) 
CALL    LoadingValsY
LoadingValsX:
LOAD    count12X
JPOS    count12X
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsX ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsX
AND Mask1 ; PB3
JPOS    LoadingValsX
;imagine a tab in here
IN      XCoords ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    XIdx
ADD     One
LOAD    count12X
SUB     One
JUMP    LoadingValsX
LoadingValsY:
LOAD    count12Y
JPOS    count12Y
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsY ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsY
AND Mask1 ; PB3
JPOS    LoadingValsY
;imagine a tab in here
IN      YCoords  ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    YIdx
ADD     One
LOAD    count12Y
SUB     One
JUMP    LoadingValsY
WeMadeIt:
LOAD    Zero
JPOS    WeMadeIt
LOAD    XIdx
SUB     One
LOAD    count
ADD     One
JUMP    WeMadeIt
JPOS    WeMadeIt
LOAD    count
SUB     One
AND     Mask1
OUT XLEDS
OUT BEEP


; OUT    RESETPOS    ; reset odometer in case wheels moved after programming 
; CALL   StartLoading

; The following code ("GetAngle" through "DeadZone") is purely for example.
; It attempts to gently keep the robot facing 0 degrees, showing how the
; odometer and motor controllers work.
; GetAngle:
; ; The 0/359 jump in THETA can be difficult to deal with.
; ; This code shows one way to handle it: by converting it
; ; to a +/-180 offset from heading 0.
; IN     THETA       ; get the current angular position
; ADDI   -180        ; test whether facing 0-179 or 180-359
; JPOS   NegAngle    ; robot facing 180-360; handle that separately
; PosAngle:
; ADDI   180         ; undo previous subtraction
; JUMP   CheckAngle  ; THETA positive, so carry on
; NegAngle:
; ADDI   -180        ; finish conversion to negative angle:
;                   ;  angles 180 to 359 become -180 to -1
; 
; CheckAngle:
; ; AC now contains the +/- angular difference from 0
; OUT    LCD         ; Good data to display for debugging
; JPOS   TurnRight   ; handle +/- separately
; TurnLeft:
; ; If the angle is small, we don't want to do anything.
; ADD    DeadZone
; JPOS   NoTurn
; ; otherwise, turn CCW
; LOAD   RSlow
; JUMP   SendToMotors
; TurnRight:
; SUB    DeadZone    ; if near 0, don't turn
; JNEG   NoTurn
; LOAD   FSlow
; JUMP   SendToMotors
; NoTurn:
; LOADI  0           ; new LOADI instruction
; JUMP   SendToMotors
; 
; DeadZone:  DW 5        ; Note that you can place data anywhere.
;                        ; Just be careful that it doesn't get executed.
; 
; SendToMotors:
; ; Since we want to spin in place, we need to send inverted
; ;  velocities to the wheels.  The speed in AC is used for
; ;  the left wheel, and its negative is used for the right wheel
; STORE  Temp        ; store calculated desired velocity
; ; send the direct value to the left wheel
; OUT    LVELCMD
; OUT    SSEG1       ; for debugging purposes
; ; send the negated number to the right wheel
; LOADI  0
; SUB    Temp        ; AC = 0 - velocity
; OUT    RVELCMD 
; OUT    SSEG2       ; debugging
; 
; JUMP   GetAngle    ; repeat forever

; Die:
; ; Sometimes it's useful to permanently stop execution.
; ; This will also catch the execution if it accidentally
; ; falls through from above.
; LOAD   Zero         ; Stop everything.
; OUT    LVELCMD
; OUT    RVELCMD
; OUT    SONAREN
; LOAD   DEAD         ; An indication that we are dead
; OUT    SSEG2
; Forever:
; JUMP   Forever      ; Do this forever.
; DEAD:  DW &HDEAD    ; Example of a "local" variable

;***************************************************************
;* User Defined Subroutines
;***************************************************************

; Subroutine to move the amount of units stored in the AC
; MoveACUnits:
; CheckDir:
; LOAD FFast
; STORE MoveACUnits_RSpeed
; STORE MoveACUnits_LSpeed
; 
; CheckCurPos:
; IN LPOS
; 
; ContinueMotion:
; LOAD MoveACUnits_Speed
; OUT LVELCMD
; OUT RVELCMD
; JUMP CheckCurPos
; 
; StopMotion:
; LOAD Zero
; OUT LVELCMD
; OUT RVELCMD
; 
; RETURN
; 
; ; Subroutine to turn to the angle stored in AC
; TurnACDegreees:
; JUMP TurnACDegreees
; RETURN
;***************************************************************
;* Predefined Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
OUT    TIMER
Wloop:
IN     LIN
OUT    SSEG2
IN     TIMER
OUT    XLEDS       ; User-feedback that a pause is occurring.
ADDI   -10         ; 1 second in 10Hz.
JNEG   Wloop
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
MoveACUnits_RSpeed: DW 0
MoveACUnits_LSpeed: DW 0
StartPosL: DW 0
StartPosR: DW 0


;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
temp: DW 0 ;hjfdkh;asf9ioqp35rh'danjsxc'aofiu=8934jdfklal
count:  DW 0
Twelve: DW 12
XIdx:   DW 0
YIdx:   DW 0
XCoords: DW 2000
YCoords: DW 2004
count12X: DW 12
count12Y: DW 12



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

Leeway: DW 10 ; radius allowed around exact X,Y coordinate
; to display message (can make bigger LOL)

OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 100       ; 10.0V - minimum safe battery voltage
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
 
From: Dimitrova, Denitsa G <ddimitrova3@gatech.edu>
Sent: Friday, April 29, 2022 6:20 AM
To: Dimitrova, Denitsa G <ddimitrova3@gatech.edu>
Subject: Re: ASM CODE FOR THE ROBOT - AS OF 4/28 
 
THIS IS THE CODE WHERE IT JUMPS BACK FINE TO -5 AFTER REACHING 5 ON PB1
*****************************************************************
; SimpleRobotProgram.asm 
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.

ORG 0  ; Begin program at x000
;***************************************************************
;* Initialization
; ;***************************************************************
; Init:
; ; Always a good idea to make sure the robot
; ; stops in the event of a reset.
; LOAD   Zero
; OUT    LVELCMD     ; Stop motors
; OUT    RVELCMD
; OUT    SONAREN     ; Disable sonar (optional)
; OUT    BEEP        ; Stop any beeping (optional)
; 
; CALL   SetupI2C    ; Configure the I2C to read the battery voltage
; CALL   BattCheck   ; Get battery voltage (and end if too low).
; OUT    LCD         ; Display battery voltage (hex, tenths of volts)
; 
; WaitForSafety:
; ; This loop will wait for the user to toggle SW17.  Note that
; ; SCOMP does not have direct access to SW17; it only has access
; ; to the SAFETY signal contained in XIO.
; IN     SWITCHES
; OUT    BEEP
; ; Wait for safety switch to be toggled
; IN     XIO         ; XIO contains SAFETY signal
; AND    Mask4       ; SAFETY signal is bit 4
; JPOS   WaitForUser ; If ready, jump to wait for PB3
; IN     TIMER       ; We'll use the timer value to
; AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
; SHIFT  8           ; Shift over to LED17
; OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
; JUMP   WaitForSafety
; 
;
; 
; WaitForUser:
; ; This loop will wait for the user to press PB3, to ensure that
; ; they have a chance to prepare for any movement in the main code.
; LOADI  0
; OUT    BEEP
; ; Wait for user to press PB3
; IN     TIMER       ; We'll blink the LEDs above PB3
; AND    Mask1
; SHIFT  5           ; Both LEDG6 and LEDG7
; STORE  Temp        ; (overkill, but looks nice)
; SHIFT  1
; OR     Temp
; OUT    XLEDS
; IN     XIO         ; XIO contains KEYs
; AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
; JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
; LOAD   Zero
; OUT    XLEDS       ; clear LEDs once ready to continue
;CALL   StartLoading ;;;;;fdkjaslfu493eifjoidjaskl;ajfkldjasfldajsfkljadsfkjdkasfjkldjasfkljdadfklj

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here. 

;CALL   StartLoading
LOADI -5
STORE temp 

Funk:
;LOADI   0
;STORE   temp 
LOAD   temp
SUB    Five
JZERO  Displaytemp
JNEG   Displaytemp
LOADI  -5
STORE  temp
Displaytemp:
LOAD   temp 
OUT    LCD
  IN     XIO 
  AND    Mask0
  ;JZERO  Funk
;   JUMP   Pressed
;   IN     XIO
;   AND    Mask0
  JPOS   Funk
  JUMP   Pressed
;   LOAD   temp
;   ADD    One
;   STORE  temp
  
  OUT    XLEDS
  
  IN     XIO
  AND    Mask1
  JPOS   Funk
  ;OUT    XLEDS
  LOADI  0
  STORE  temp
  JPOS   Funk
;   IN     XIO
;   AND    Mask1
;   JZERO  Funk
;   JUMP   Funk
  
  
  
  IN     XIO
  AND    Mask1
  JZERO  Funk ; might have to check this after the zero check on PB2 (next round of testing)
  JPOS   Funk
  OUT    XLEDS
  LOADI  0
  STORE  temp
  JUMP   Funk
  ;CALL   Pressed
  ;CALL   Increment
  
Pressed:
  IN     XIO 
  AND    Mask0
  JZERO   Pressed
  
Increment:   
  LOAD   temp
  ADD    One
    STORE  temp
  OUT    LCD
  ;OR     Load&StoreA
  ;JZERO  Funk
  ;JPOS   Funk
  ;JUMP   Funk

  ;CALL   Load&StoreA
  ;CALL   Load&StoreB
  JUMP   Funk
  
Load&StoreA:
IN XIO
AND     Mask1
JZERO   Load&StoreA
OUT XLEDS
  
Load&StoreB:
IN XIO
AND     Mask1
JPOS    Load&StoreB
;IN      TIMER ; maybe time it would work?
;LOAD    Zero
;AND     Mask2
OUT     XLEDS ;check if its been stored
LOADI   0
STORE   temp
JUMP    Funk
IntoX:
LOADI   XCoords
LOAD    count12X
;JPOS    
;***********
;Coordinate entry code
;***********
StartLoading: ; call on startup
LOADI 0
;OUT BEEP
CALL LoadingValsX ;need to finish this off dummty (JFKLJKLSNAKLDNKFHEIF ATTENTION) 
CALL    LoadingValsY
LoadingValsX:
LOAD    count12X
JPOS    count12X
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsX ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsX
AND Mask1 ; PB3
JPOS    LoadingValsX
;imagine a tab in here
IN      XCoords ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    XIdx
ADD     One
LOAD    count12X
SUB     One
JUMP    LoadingValsX
LoadingValsY:
LOAD    count12Y
JPOS    count12Y
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsY ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsY
AND Mask1 ; PB3
JPOS    LoadingValsY
;imagine a tab in here
IN      YCoords  ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    YIdx
ADD     One
LOAD    count12Y
SUB     One
JUMP    LoadingValsY
WeMadeIt:
LOAD    Zero
JPOS    WeMadeIt
LOAD    XIdx
SUB     One
LOAD    count
ADD     One
JUMP    WeMadeIt
JPOS    WeMadeIt
LOAD    count
SUB     One
AND     Mask1
OUT XLEDS
OUT BEEP


; OUT    RESETPOS    ; reset odometer in case wheels moved after programming 
; CALL   StartLoading

; The following code ("GetAngle" through "DeadZone") is purely for example.
; It attempts to gently keep the robot facing 0 degrees, showing how the
; odometer and motor controllers work.
; GetAngle:
; ; The 0/359 jump in THETA can be difficult to deal with.
; ; This code shows one way to handle it: by converting it
; ; to a +/-180 offset from heading 0.
; IN     THETA       ; get the current angular position
; ADDI   -180        ; test whether facing 0-179 or 180-359
; JPOS   NegAngle    ; robot facing 180-360; handle that separately
; PosAngle:
; ADDI   180         ; undo previous subtraction
; JUMP   CheckAngle  ; THETA positive, so carry on
; NegAngle:
; ADDI   -180        ; finish conversion to negative angle:
;                   ;  angles 180 to 359 become -180 to -1
; 
; CheckAngle:
; ; AC now contains the +/- angular difference from 0
; OUT    LCD         ; Good data to display for debugging
; JPOS   TurnRight   ; handle +/- separately
; TurnLeft:
; ; If the angle is small, we don't want to do anything.
; ADD    DeadZone
; JPOS   NoTurn
; ; otherwise, turn CCW
; LOAD   RSlow
; JUMP   SendToMotors
; TurnRight:
; SUB    DeadZone    ; if near 0, don't turn
; JNEG   NoTurn
; LOAD   FSlow
; JUMP   SendToMotors
; NoTurn:
; LOADI  0           ; new LOADI instruction
; JUMP   SendToMotors
; 
; DeadZone:  DW 5        ; Note that you can place data anywhere.
;                        ; Just be careful that it doesn't get executed.
; 
; SendToMotors:
; ; Since we want to spin in place, we need to send inverted
; ;  velocities to the wheels.  The speed in AC is used for
; ;  the left wheel, and its negative is used for the right wheel
; STORE  Temp        ; store calculated desired velocity
; ; send the direct value to the left wheel
; OUT    LVELCMD
; OUT    SSEG1       ; for debugging purposes
; ; send the negated number to the right wheel
; LOADI  0
; SUB    Temp        ; AC = 0 - velocity
; OUT    RVELCMD 
; OUT    SSEG2       ; debugging
; 
; JUMP   GetAngle    ; repeat forever

; Die:
; ; Sometimes it's useful to permanently stop execution.
; ; This will also catch the execution if it accidentally
; ; falls through from above.
; LOAD   Zero         ; Stop everything.
; OUT    LVELCMD
; OUT    RVELCMD
; OUT    SONAREN
; LOAD   DEAD         ; An indication that we are dead
; OUT    SSEG2
; Forever:
; JUMP   Forever      ; Do this forever.
; DEAD:  DW &HDEAD    ; Example of a "local" variable

;***************************************************************
;* User Defined Subroutines
;***************************************************************

; Subroutine to move the amount of units stored in the AC
; MoveACUnits:
; CheckDir:
; LOAD FFast
; STORE MoveACUnits_RSpeed
; STORE MoveACUnits_LSpeed
; 
; CheckCurPos:
; IN LPOS
; 
; ContinueMotion:
; LOAD MoveACUnits_Speed
; OUT LVELCMD
; OUT RVELCMD
; JUMP CheckCurPos
; 
; StopMotion:
; LOAD Zero
; OUT LVELCMD
; OUT RVELCMD
; 
; RETURN
; 
; ; Subroutine to turn to the angle stored in AC
; TurnACDegreees:
; JUMP TurnACDegreees
; RETURN
;***************************************************************
;* Predefined Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
OUT    TIMER
Wloop:
IN     LIN
OUT    SSEG2
IN     TIMER
OUT    XLEDS       ; User-feedback that a pause is occurring.
ADDI   -10         ; 1 second in 10Hz.
JNEG   Wloop
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
MoveACUnits_RSpeed: DW 0
MoveACUnits_LSpeed: DW 0
StartPosL: DW 0
StartPosR: DW 0


;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
temp: DW 0 ;hjfdkh;asf9ioqp35rh'danjsxc'aofiu=8934jdfklal
count:  DW 0
Twelve: DW 12
XIdx:   DW 0
YIdx:   DW 0
XCoords: DW 2000
YCoords: DW 2004
count12X: DW 12
count12Y: DW 12



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

Leeway: DW 10 ; radius allowed around exact X,Y coordinate
; to display message (can make bigger LOL)

OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 100       ; 10.0V - minimum safe battery voltage
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
 
From: Dimitrova, Denitsa G <ddimitrova3@gatech.edu>
Sent: Thursday, April 28, 2022 9:03 AM
To: Dimitrova, Denitsa G <ddimitrova3@gatech.edu>
Subject: Re: ASM CODE FOR THE ROBOT - AS OF 4/28 
 
CHANGE JUMP PRESSED TO JPOS PRESSED
 
From: Dimitrova, Denitsa G
Sent: Thursday, April 28, 2022 8:40 AM
To: Dimitrova, Denitsa G <ddimitrova3@gatech.edu>
Subject: ASM CODE FOR THE ROBOT - AS OF 4/28 
 
; SimpleRobotProgram.asm 
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some robot control.

; Section labels are for clarity only.

ORG 0  ; Begin program at x000
;***************************************************************
;* Initialization
; ;***************************************************************
; Init:
; ; Always a good idea to make sure the robot
; ; stops in the event of a reset.
; LOAD   Zero
; OUT    LVELCMD     ; Stop motors
; OUT    RVELCMD
; OUT    SONAREN     ; Disable sonar (optional)
; OUT    BEEP        ; Stop any beeping (optional)
; 
; CALL   SetupI2C    ; Configure the I2C to read the battery voltage
; CALL   BattCheck   ; Get battery voltage (and end if too low).
; OUT    LCD         ; Display battery voltage (hex, tenths of volts)
; 
; WaitForSafety:
; ; This loop will wait for the user to toggle SW17.  Note that
; ; SCOMP does not have direct access to SW17; it only has access
; ; to the SAFETY signal contained in XIO.
; IN     SWITCHES
; OUT    BEEP
; ; Wait for safety switch to be toggled
; IN     XIO         ; XIO contains SAFETY signal
; AND    Mask4       ; SAFETY signal is bit 4
; JPOS   WaitForUser ; If ready, jump to wait for PB3
; IN     TIMER       ; We'll use the timer value to
; AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
; SHIFT  8           ; Shift over to LED17
; OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
; JUMP   WaitForSafety
; 
;
; 
; WaitForUser:
; ; This loop will wait for the user to press PB3, to ensure that
; ; they have a chance to prepare for any movement in the main code.
; LOADI  0
; OUT    BEEP
; ; Wait for user to press PB3
; IN     TIMER       ; We'll blink the LEDs above PB3
; AND    Mask1
; SHIFT  5           ; Both LEDG6 and LEDG7
; STORE  Temp        ; (overkill, but looks nice)
; SHIFT  1
; OR     Temp
; OUT    XLEDS
; IN     XIO         ; XIO contains KEYs
; AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
; JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
; LOAD   Zero
; OUT    XLEDS       ; clear LEDs once ready to continue
;CALL   StartLoading ;;;;;fdkjaslfu493eifjoidjaskl;ajfkldjasfldajsfkljadsfkjdkasfjkldjasfkljdadfklj

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here. 

;CALL   StartLoading
LOADI -5
STORE temp 

Funk:
;LOADI   0
;STORE   temp 
LOAD   temp
OUT    LCD
  IN     XIO 
  AND    Mask0
  ;JZERO  Funk
;   JUMP   Pressed
;   IN     XIO
;   AND    Mask0
  JZERO   Funk
  JUMP   Pressed
;   LOAD   temp
;   ADD    One
;   STORE  temp
  
  OUT    XLEDS
  
  IN     XIO
  AND    Mask1
  JPOS   Funk
  ;OUT    XLEDS
  LOADI  0
  STORE  temp
  JUMP   Funk
;   IN     XIO
;   AND    Mask1
;   JZERO  Funk
;   JUMP   Funk
  
  
  
  IN     XIO
  AND    Mask1
  JZERO  Funk ; might have to check this after the zero check on PB2 (next round of testing)
  JPOS   Funk
  OUT    XLEDS
  LOADI  0
  STORE  temp
  JUMP   Funk
  ;CALL   Pressed
  ;CALL   Increment
  
Pressed:
  IN     XIO 
  AND    Mask0
  JPOS   Pressed
  
Increment:   
  LOAD   temp
  ADD    One
    STORE  temp
  OUT    LCD
  ;OR     Load&StoreA
  ;JZERO  Funk
  ;JPOS   Funk
  ;JUMP   Funk

  ;CALL   Load&StoreA
  ;CALL   Load&StoreB
  ;JUMP   Funk
  
Load&StoreA:
IN XIO
AND     Mask1
JZERO   Load&StoreA
OUT XLEDS
  
Load&StoreB:
IN XIO
AND     Mask1
JPOS    Load&StoreB
;IN      TIMER ; maybe time it would work?
;LOAD    Zero
;AND     Mask2
OUT     XLEDS ;check if its been stored
LOADI   0
STORE   temp
JUMP    Funk
IntoX:
LOADI   XCoords
LOAD    count12X
;JPOS    
;***********
;Coordinate entry code
;***********
StartLoading: ; call on startup
LOADI 0
;OUT BEEP
CALL LoadingValsX ;need to finish this off dummty (JFKLJKLSNAKLDNKFHEIF ATTENTION) 
CALL    LoadingValsY
LoadingValsX:
LOAD    count12X
JPOS    count12X
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsX ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsX
AND Mask1 ; PB3
JPOS    LoadingValsX
;imagine a tab in here
IN      XCoords ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    XIdx
ADD     One
LOAD    count12X
SUB     One
JUMP    LoadingValsX
LoadingValsY:
LOAD    count12Y
JPOS    count12Y
IN XIO
AND     Mask1 ; PB2
IN   temp
OUT LCD
;LOAD 0
JPOS    LoadingValsY ; was it pressed?
ADD One
OUT LCD
JUMP LoadingValsY
AND Mask1 ; PB3
JPOS    LoadingValsY
;imagine a tab in here
IN      YCoords  ; or whatever its called
ISTORE  temp
OUT LCD
LOAD    YIdx
ADD     One
LOAD    count12Y
SUB     One
JUMP    LoadingValsY
WeMadeIt:
LOAD    Zero
JPOS    WeMadeIt
LOAD    XIdx
SUB     One
LOAD    count
ADD     One
JUMP    WeMadeIt
JPOS    WeMadeIt
LOAD    count
SUB     One
AND     Mask1
OUT XLEDS
OUT BEEP


; OUT    RESETPOS    ; reset odometer in case wheels moved after programming 
; CALL   StartLoading

; The following code ("GetAngle" through "DeadZone") is purely for example.
; It attempts to gently keep the robot facing 0 degrees, showing how the
; odometer and motor controllers work.
; GetAngle:
; ; The 0/359 jump in THETA can be difficult to deal with.
; ; This code shows one way to handle it: by converting it
; ; to a +/-180 offset from heading 0.
; IN     THETA       ; get the current angular position
; ADDI   -180        ; test whether facing 0-179 or 180-359
; JPOS   NegAngle    ; robot facing 180-360; handle that separately
; PosAngle:
; ADDI   180         ; undo previous subtraction
; JUMP   CheckAngle  ; THETA positive, so carry on
; NegAngle:
; ADDI   -180        ; finish conversion to negative angle:
;                   ;  angles 180 to 359 become -180 to -1
; 
; CheckAngle:
; ; AC now contains the +/- angular difference from 0
; OUT    LCD         ; Good data to display for debugging
; JPOS   TurnRight   ; handle +/- separately
; TurnLeft:
; ; If the angle is small, we don't want to do anything.
; ADD    DeadZone
; JPOS   NoTurn
; ; otherwise, turn CCW
; LOAD   RSlow
; JUMP   SendToMotors
; TurnRight:
; SUB    DeadZone    ; if near 0, don't turn
; JNEG   NoTurn
; LOAD   FSlow
; JUMP   SendToMotors
; NoTurn:
; LOADI  0           ; new LOADI instruction
; JUMP   SendToMotors
; 
; DeadZone:  DW 5        ; Note that you can place data anywhere.
;                        ; Just be careful that it doesn't get executed.
; 
; SendToMotors:
; ; Since we want to spin in place, we need to send inverted
; ;  velocities to the wheels.  The speed in AC is used for
; ;  the left wheel, and its negative is used for the right wheel
; STORE  Temp        ; store calculated desired velocity
; ; send the direct value to the left wheel
; OUT    LVELCMD
; OUT    SSEG1       ; for debugging purposes
; ; send the negated number to the right wheel
; LOADI  0
; SUB    Temp        ; AC = 0 - velocity
; OUT    RVELCMD 
; OUT    SSEG2       ; debugging
; 
; JUMP   GetAngle    ; repeat forever

; Die:
; ; Sometimes it's useful to permanently stop execution.
; ; This will also catch the execution if it accidentally
; ; falls through from above.
; LOAD   Zero         ; Stop everything.
; OUT    LVELCMD
; OUT    RVELCMD
; OUT    SONAREN
; LOAD   DEAD         ; An indication that we are dead
; OUT    SSEG2
; Forever:
; JUMP   Forever      ; Do this forever.
; DEAD:  DW &HDEAD    ; Example of a "local" variable

;***************************************************************
;* User Defined Subroutines
;***************************************************************

; Subroutine to move the amount of units stored in the AC
; MoveACUnits:
; CheckDir:
; LOAD FFast
; STORE MoveACUnits_RSpeed
; STORE MoveACUnits_LSpeed
; 
; CheckCurPos:
; IN LPOS
; 
; ContinueMotion:
; LOAD MoveACUnits_Speed
; OUT LVELCMD
; OUT RVELCMD
; JUMP CheckCurPos
; 
; StopMotion:
; LOAD Zero
; OUT LVELCMD
; OUT RVELCMD
; 
; RETURN
; 
; ; Subroutine to turn to the angle stored in AC
; TurnACDegreees:
; JUMP TurnACDegreees
; RETURN
;***************************************************************
;* Predefined Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
OUT    TIMER
Wloop:
IN     LIN
OUT    SSEG2
IN     TIMER
OUT    XLEDS       ; User-feedback that a pause is occurring.
ADDI   -10         ; 1 second in 10Hz.
JNEG   Wloop
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
MoveACUnits_RSpeed: DW 0
MoveACUnits_LSpeed: DW 0
StartPosL: DW 0
StartPosR: DW 0


;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
temp: DW 0 ;hjfdkh;asf9ioqp35rh'danjsxc'aofiu=8934jdfklal
count:  DW 0
Twelve: DW 12
XIdx:   DW 0
YIdx:   DW 0
XCoords: DW 2000
YCoords: DW 2004
count12X: DW 12
count12Y: DW 12



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

Leeway: DW 10 ; radius allowed around exact X,Y coordinate
; to display message (can make bigger LOL)

OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 100       ; 10.0V - minimum safe battery voltage
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
