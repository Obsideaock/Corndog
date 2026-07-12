#! /usr/bin/env python

# Import necessary libraries for communication and display use
import drivers
from time import sleep

# Load the driver and set it to "display"
display = drivers.Lcd()

def lcd(text):
    """
    Display a string on a 16x2 I2C LCD.
    
    Clears the screen at the start of each call.
    The first 8 characters are displayed on the first line.
    The rest is displayed on the second line.
    If the second line exceeds 16 characters, it will scroll.
    """
    # Clear the screen first
    display.lcd_clear()

    # Split the text into two parts: First line (first 8 characters), and the rest for the second line.
    first_line = text[:16]
    second_line = text[16:]
    
    # Display the first line (up to 8 characters)
    display.lcd_display_string(first_line, 1)
    
    # If second line is longer than 16 characters, enable scrolling
    if len(second_line) > 16:
        # Scroll the text in the second line
        display.lcd_display_string(second_line[:16], 2)
        sleep(1)
        for i in range(len(second_line) - 16 + 1):
            text_to_print = second_line[i:i+16]
            display.lcd_display_string(text_to_print, 2)
            sleep(0.2)
        sleep(1)
    else:
        # If the second line fits in 16 characters, just display it
        display.lcd_display_string(second_line, 2)
        
def clear():
	 # Clear the screen first
	 display.lcd_clear()
	 
