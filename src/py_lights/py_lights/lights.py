def main():
    import RPi.GPIO as GPIO
    
    # LED PINS
    RED_PIN = 2  # Closest to the top of the breadboard
    GREEN_PIN = 3
    BLUE_PIN = 4
    ORANGE_PIN = 17

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RED_PIN, GPIO.OUT)
    GPIO.setup(GREEN_PIN, GPIO.OUT)
    GPIO.setup(BLUE_PIN, GPIO.OUT)
    GPIO.setup(ORANGE_PIN, GPIO.OUT)

    red_on = False
    green_on = False
    blue_on = False
    orange_on = False
    loop = True

    while loop:
        print("\nHello! Please choose a number to toggle an LED.\n")
        val = input("\nFor Red: 1, for Blue: 2, for Green: 3, and for Orange: 4. Click 0 to exit.\n")

        match int(val):
            case 0:
                GPIO.output(RED_PIN, False)
                GPIO.output(GREEN_PIN, False)
                GPIO.output(BLUE_PIN, False)
                GPIO.output(ORANGE_PIN, False)
                GPIO.cleanup()
                loop = False

            case 1:
                red_on = not red_on
                GPIO.output(RED_PIN, red_on)

            case 2:
                green_on = not green_on
                GPIO.output(GREEN_PIN, green_on)

            case 3:
                blue_on = not blue_on
                GPIO.output(BLUE_PIN, blue_on)

            case 4:
                orange_on = not orange_on
                GPIO.output(ORANGE_PIN, orange_on)

            case _:
                print("\nWeird input, but ok. Try again.\n")

if __name__ == '__main__':
    try:
        main()
    finally:
        GPIO.cleanup()


# GPIO pin 0  [Pin 27] (ID_SD)
# GPI0 Pin 1  [Pin 28] (ID_SC)
# GPI0 Pin 2  [Pin 3 ] (SDA)
# GPI0 Pin 3  [Pin 5 ] (SCL)
# GPI0 Pin 4  [Pin 7 ] (GPCLK0)
# GPI0 Pin 5  [Pin 29] 
# GPI0 Pin 6  [Pin 31] 
# GPI0 Pin 7  [Pin 26] (CE1)
# GPI0 Pin 8  [Pin 24] 
# GPI0 Pin 9  [Pin 21] (MISO)
# GPI0 Pin 10 [Pin 19] (MOSI)
# GPI0 Pin 11 [Pin 23] (SCLK)
# GPI0 Pin 12 [Pin 32] (PWM0)
# GPI0 Pin 13 [Pin 33] (PWM1)
# GPI0 Pin 14 [Pin 8 ] (TXD) 
# GPI0 Pin 15 [Pin 10] (RXD)
# GPI0 Pin 16 [Pin 36]
# GPI0 Pin 17 [Pin 11]
# GPI0 Pin 18 [Pin 12] (PCM_CLK)
# GPI0 Pin 19 [Pin 35] (PCK_FS)
# GPI0 Pin 20 [Pin 38] (PCM_DIN) 
# GPI0 Pin 21 [Pin 40] (PCM_DOUT) 
# GPI0 Pin 22 [Pin 15] 
# GPI0 Pin 23 [Pin 16] 
# GPI0 Pin 24 [Pin 18]
# GPI0 Pin 25 [Pin 22]
# GPI0 Pin 26 [Pin 37]

#############################################
#There are 10 GPIO pins with no output type
# GPI0 Pin 5  [Pin 29] 
# GPI0 Pin 6  [Pin 31] 
# GPI0 Pin 8  [Pin 24] 
# GPI0 Pin 16 [Pin 36]
# GPI0 Pin 17 [Pin 11]
# GPI0 Pin 22 [Pin 15] 
# GPI0 Pin 23 [Pin 16] 
# GPI0 Pin 24 [Pin 18]
# GPI0 Pin 25 [Pin 22]
# GPI0 Pin 26 [Pin 37]
#############################################

#LED PINS
# RED_PIN = 2 #Closest to top of bread board
# GREEN_PIN = 3
# BLUE_PIN = 4
# ORANGE_PIN = 17

# GPIO.setmode(GPIO.BCM)

# GPIO.setup(RED_PIN, GPIO.OUT)
# GPIO.setup(GREEN_PIN, GPIO.OUT)
# GPIO.setup(BLUE_PIN, GPIO.OUT)
# GPIO.setup(ORANGE_PIN, GPIO.OUT)

# red_on = False
# green_on = False
# blue_on = False
# orange_on = False
# loop = True

# while loop:
#     print("\nHello! Please choose a number to toggle a LED.\n")
    
    
#     val = input("\nFor Red: 1, for Blue: 2, for Green: 3, and for Orange: 4. Click 0 to exit.\n")
    
#     match int(val):

#         case 0: 
#             GPIO.output(RED_PIN, False)
#             GPIO.output(GREEN_PIN, False)
#             GPIO.output(BLUE_PIN, False)
#             GPIO.output(ORANGE_PIN, False)
#             GPIO.cleanup()
#             loop = False

#         case 1:
            
#             if(red_on):
#                 red_on = False

#             else:
#                 red_on = True
        
#             GPIO.output(RED_PIN, red_on)

#         case 2:

#             if(green_on):
#                 green_on = False
        
#             else:
#                 green_on = True

#             GPIO.output(GREEN_PIN, green_on)
    
#         case 3:
        
#             if(blue_on):
#                 blue_on = False

#             else:
#                 blue_on = True
        
#             GPIO.output(BLUE_PIN, blue_on)
    
#         case 4:

#             if(orange_on):
#                 orange_on = False
        
#             else:
#                 orange_on = True
        
#             GPIO.output(ORANGE_PIN, orange_on)

#         case _:

#             print("\nWeird input, but ok. Try again.\n")