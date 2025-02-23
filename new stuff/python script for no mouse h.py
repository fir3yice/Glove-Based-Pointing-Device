import serial
import pyautogui
import time
# Change 'COM3' to the correct port for your Arduino (check Arduino IDE's Tools > Port)
arduino = serial.Serial('COM4', 9600, timeout=1)

while True:
    data = arduino.readline().decode('utf-8').strip()
    if data == "LEFT":
        pyautogui.click()  # Simulate a mouse click
        # add a sleep
        time.sleep(0.2)
        print ("left click")
    elif data == "RIGHT":
        pyautogui.rightClick()
        print ("right click")
    elif data == "MIDDLE":
        pyautogui.middleClick()
        print ("middle click")
