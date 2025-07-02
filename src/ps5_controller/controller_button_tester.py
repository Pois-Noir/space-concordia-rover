import pygame
import sys
import os

# Init pygame and ps5
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
# Make sure at least one ps5 is connected
if pygame.joystick.get_count() == 0:
    print("No ps5 found.")
    sys.exit()

ps5 = pygame.joystick.Joystick(0)
ps5.init()
print(f"Using ps5: {ps5.get_name()}")

# Read stick positions
left_x = ps5.get_axis(0)
left_y = -ps5.get_axis(1)
right_x = ps5.get_axis(3)
right_y = -ps5.get_axis(4)
left_button = ps5.get_button(11)
right_button = ps5.get_button(12)

left_trigger = ps5.get_axis(2)
right_trigger = ps5.get_axis(5)
left_trigger_button = ps5.get_button(6)
right_trigger_button = ps5.get_button(7)

button_cross = ps5.get_button(0)
button_circle = ps5.get_button(1)
button_triangle = ps5.get_button(2)
button_square = ps5.get_button(3)

left_bumper = ps5.get_button(4)
right_bumper = ps5.get_button(5)

d_pad = ps5.get_hat(0)

share_button = ps5.get_button(8)
options_button = ps5.get_button(9)
ps_button = ps5.get_button(10)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    button_layout = {
        'left_x' : ps5.get_axis(0),
        'left_y' : -ps5.get_axis(1),
        'right_x' : ps5.get_axis(3),
        'right_y' : -ps5.get_axis(4),
        'left_button' : ps5.get_button(11),
        'right_button' : ps5.get_button(12),

        'left_trigger' : ps5.get_axis(2),
        'right_trigger' : ps5.get_axis(5),
        'left_trigger_button' : ps5.get_button(6),
        'right_trigger_button' : ps5.get_button(7),

        'button_cross' : ps5.get_button(0),
        'button_circle' : ps5.get_button(1),
        'button_triangle' : ps5.get_button(2),
        'button_square' : ps5.get_button(3),

        'left_bumper' : ps5.get_button(4),
        'right_bumper' : ps5.get_button(5),

        'd_pad' : ps5.get_hat(0),

        'share_button' : ps5.get_button(8),
        'options_button' : ps5.get_button(9),
        'ps_button' : ps5.get_button(10)
    }
    for button_name, button_value in button_layout.items():
        print(button_name,':', button_value)
    clock.tick(60)
    os.system('clear')	#change depending on computer OS
