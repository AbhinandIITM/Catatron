import rospy
import numpy as np
import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont
import time

class Display:
    def __init__(self):
        self.light = True
        self.blink = False

        # OLED display dimensions
        self.WIDTH = 128
        self.HEIGHT = 64

        # I2C setup for two displays
        i2c_1 = busio.I2C(board.SCL, board.SDA)  # Primary I2C bus
        i2c_2 = busio.I2C(board.SCL, board.SDA)  # Secondary I2C bus (if required)

        # Initialize the SSD1306 OLED displays
        self.oled_1 = SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c_1, addr=0x3C)  # First display at address 0x3C
        self.oled_2 = SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c_2, addr=0x3C)  # Second display at address 0x3D
        self.Open_Eye = True

        # Clear both displays
        self.oled_1.fill(0)
        self.oled_1.show()
        self.oled_2.fill(0)
        self.oled_2.show()

    def eye_control(self):
        if self.blink:
            counter = 0
            while counter <= 5:
                self.draw_eye(self.oled_1, self.Open_Eye)
                self.draw_eye(self.oled_2, self.Open_Eye)
                time.sleep(1)
                counter += 1
                self.Open_Eye = not self.Open_Eye

    # Function to draw an eye
    def draw_eye(self, oled, open_eye=True):
        # Create a blank image for drawing
        image = Image.new("1", (self.WIDTH, self.HEIGHT))
        draw = ImageDraw.Draw(image)

        # Define the eye's shape and position
        center_x, center_y = self.WIDTH // 2, self.HEIGHT // 2
        eye_width, eye_height = 60, 30

        if open_eye:
            # Draw an oval to represent an open eye
            draw.ellipse(
                [
                    (center_x - eye_width // 2, center_y - eye_height // 2),
                    (center_x + eye_width // 2, center_y + eye_height // 2),
                ],
                outline=255,
                fill=255,
            )
            # Draw a smaller circle inside for the pupil
            pupil_radius = 10
            draw.ellipse(
                [
                    (center_x - pupil_radius, center_y - pupil_radius),
                    (center_x + pupil_radius, center_y + pupil_radius),
                ],
                outline=0,
                fill=0,
            )
        else:
            # Draw a closed eye (a horizontal line)
            draw.line(
                [
                    (center_x - eye_width // 2, center_y),
                    (center_x + eye_width // 2, center_y),
                ],
                fill=255,
                width=2,
            )

        # Display the image on the specified OLED
        oled.image(image)
        oled.show()
