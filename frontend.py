import pygame
import sys

from tillerpilot import Tillerpilot

# TODO: add two text fields for desired heading and actual heading, enable messages from drone

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH = 400
SCREEN_HEIGHT = 600
BACKGROUND_COLOR = (240, 240, 240)
BUTTON_COLOR_BLACK = (40, 40, 40)
BUTTON_COLOR_RED = (200, 50, 50)
BUTTON_HOVER_BLACK = (70, 70, 70)
BUTTON_HOVER_RED = (230, 80, 80)
BUTTON_TEXT_COLOR = (255, 255, 255)
BUTTON_RADIUS = 60
FONT_SIZE = 24

# Create the screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Andihelm 2000")

# Create font
font = pygame.font.Font(None, FONT_SIZE)

# Define button positions (3 rows, 2 columns)
button_positions = [
    # Row 1
    (100, 150),
    (300, 150),
    # Row 2
    (100, 300),
    (300, 300),
    # Row 3
    (100, 450),
    (300, 450)
]

tillerpilot = Tillerpilot()
tillerpilot.init()

headingDeg = 0
manual = True
manualServoValue = 1500

button_labels = ["-1", "+1", "-10", "+10", "STANDBY", "AUTO"]

# Define button colors (first 4 black, last 2 red)
button_colors = [
    (BUTTON_COLOR_BLACK, BUTTON_HOVER_BLACK),  # Button 1
    (BUTTON_COLOR_BLACK, BUTTON_HOVER_BLACK),  # Button 2
    (BUTTON_COLOR_BLACK, BUTTON_HOVER_BLACK),  # Button 3
    (BUTTON_COLOR_BLACK, BUTTON_HOVER_BLACK),  # Button 4
    (BUTTON_COLOR_RED, BUTTON_HOVER_RED),      # Button 5
    (BUTTON_COLOR_RED, BUTTON_HOVER_RED)       # Button 6
]


# Timer for periodic updates
update_interval = 1000 / 5  # milliseconds
last_update_time = pygame.time.get_ticks()


# This doesn't work very well bc of sync, needs listener...
gps_update_interval = 500
last_gps_update_time = pygame.time.get_ticks()

class RoundButton:
    def __init__(self, x, y, radius, label, button_id, normal_color, hover_color):
        self.x = x
        self.y = y
        self.radius = radius
        self.label = label
        self.button_id = button_id
        self.normal_color = normal_color
        self.hover_color = hover_color
        self.is_hovered = False



    def draw(self, surface):
        color = self.hover_color if self.is_hovered else self.normal_color
        pygame.draw.circle(surface, color, (self.x, self.y), self.radius)

        # Draw text
        text = font.render(self.label, True, BUTTON_TEXT_COLOR)
        text_rect = text.get_rect(center=(self.x, self.y))
        surface.blit(text, text_rect)

    def is_clicked(self, pos):
        # Check if click position is within circle
        distance = ((pos[0] - self.x) ** 2 + (pos[1] - self.y) ** 2) ** 0.5
        return distance <= self.radius

    def update_hover(self, pos):
        distance = ((pos[0] - self.x) ** 2 + (pos[1] - self.y) ** 2) ** 0.5
        self.is_hovered = distance <= self.radius

    def on_click(self):
        global headingDeg  # Declare that we're using the global variable
        global manual
        global manualServoValue

        inc = 0

        if self.label == "-1":
            inc = -1
        # set something like delay = DELAY_SHORT
        elif self.label == "+1":
            inc = 1
        elif self.label == "-10":
            inc = -10
        elif self.label == "+10":
            inc = 10
        elif self.label == "STANDBY":
            print("STANDBY mode activated")
            manual = True
            tillerpilot.set_standby_mode()
        elif self.label == "AUTO":
            print("AUTO mode activated")
            manual = False
            tillerpilot.set_auto_mode()

        if manual:
            manualServoValue += inc
        else:
            headingDeg += inc
            if headingDeg > 359:
                headingDeg -= 360
            if headingDeg < 0:
                headingDeg += 360

        print(f"{self.label} was clicked, heading {headingDeg}!")

# Create buttons
buttons = []
for i, (x, y) in enumerate(button_positions):
    normal_color, hover_color = button_colors[i]
    button = RoundButton(x, y, BUTTON_RADIUS, button_labels[i], i + 1, normal_color, hover_color)
    buttons.append(button)

# Main game loop
clock = pygame.time.Clock()
running = True

while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                mouse_pos = pygame.mouse.get_pos()
                for button in buttons:
                    if button.is_clicked(mouse_pos):
                        button.on_click()

    # TODO: reset rudder to center after xy ticks if not in servo mode (aka using tiller without position feedback)

    # Call periodic update function at specified rate
    current_time = pygame.time.get_ticks()
    if current_time - last_update_time >= update_interval:
        if manual:
            # TODO: choose values manually to avoid deadband, release after half a second or so....
            tillerpilot.set_servo1_rc_override(manualServoValue)
        else:
            tillerpilot.send_heading(headingDeg)

        last_update_time = current_time

    # Get GPS position every second
    if current_time - last_gps_update_time >= gps_update_interval:
        gps_data = tillerpilot.get_gps_position()
        if gps_data:
            gps_lat = gps_data['lat']
            gps_lon = gps_data['lon']
            gps_alt = gps_data['alt']
            gps_hdg = gps_data['hdg']
            print(f"GPS: Lat={gps_lat:.7f}, Lon={gps_lon:.7f}, Alt={gps_alt:.2f}m, curent_hdg={gps_hdg}, target_hdg")
        last_gps_update_time = current_time

    # Update hover states
    mouse_pos = pygame.mouse.get_pos()
    for button in buttons:
        button.update_hover(mouse_pos)

    # Draw everything
    screen.fill(BACKGROUND_COLOR)

    for button in buttons:
        button.draw(screen)

    # Update display
    pygame.display.flip()
    clock.tick(60)

# Quit
pygame.quit()
sys.exit()