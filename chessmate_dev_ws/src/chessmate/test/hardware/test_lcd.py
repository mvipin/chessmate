import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Display dimensions
WIDTH = 128
HEIGHT = 64

# Initialize display
display = SSD1306_I2C(WIDTH, HEIGHT, i2c)
display.fill(0)
display.show()

# Create blank image for drawing
image = Image.new("1", (WIDTH, HEIGHT))
draw = ImageDraw.Draw(image)

# Load default font
font = ImageFont.load_default()
draw.text((0, 0), "Hello, Pi!", font=font, fill=255)

# Display image
display.image(image)
display.show()
