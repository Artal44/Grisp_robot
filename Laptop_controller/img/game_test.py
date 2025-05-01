import pygame
import sys
import numpy as np
import serial
import time


# Initialisation de Pygame
pygame.init()

ser = serial.Serial(port="COM4", baudrate=115200)

# Définition des dimensions de la fenêtre
width, height = 800, 600
screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
pygame.display.set_caption("Rotation de la flèche")

# Couleurs
white = (255, 255, 255)

# hello

# Chargement de l'image de la flèche
arrow_img = pygame.image.load('arrow.png')
# make it half the size
arrow_img = pygame.transform.scale(arrow_img, (arrow_img.get_width() // 2, arrow_img.get_height() // 2))
arrow_rect = arrow_img.get_rect(center=(width // 2, height // 2))
circle_img = pygame.image.load('point.png')
circle_img = pygame.transform.scale(circle_img, (circle_img.get_width() // 2, circle_img.get_height() // 2))
stop_img = pygame.image.load('Stop_sign.png')
stop_img = pygame.transform.scale(stop_img, (stop_img.get_width() // 5, stop_img.get_height() // 5))

font = pygame.font.Font(None, 36)


running = True
message = 0
run = True
stand = True
kalman = True
release_space = True
release_enter = True
release_t = True
release_tab = True


test_time = 0

# main loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    #get window size
    width, height = screen.get_size()
    arrow_rect = arrow_img.get_rect(center=(width // 2, height // 2))

    # Récupération des touches pressées
    keys = pygame.key.get_pressed()
    x = 0
    string = ""

    if keys[pygame.K_z] or keys[pygame.K_UP]:
        x += 1
    if keys[pygame.K_s] or keys[pygame.K_DOWN]:
        x += -1
    if keys[pygame.K_q] or keys[pygame.K_LEFT]:
        x += 1j
    if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
        x += -1j
    if keys[pygame.K_ESCAPE]:
        running = False
    if keys[pygame.K_SPACE]:
        if release_space:
            release_space = False
            if message < 10000000:
                run = True
            else:
                run = False
    else:
        release_space = True

    # tab
    if keys[pygame.K_TAB]:
        if release_tab:
            release_tab = False
            kalman = not kalman
    else:
        release_tab = True

    if keys[pygame.K_k]:
        kalman = True

    if keys[pygame.K_c]:
        kalman = False

    if kalman :
        string += "Kalman filter\n"
    else:
        string += "Complementary filter\n"

    test = False
    if keys[pygame.K_t]:
        if release_t:
            release_t = False
            test = True
    else:
        release_t = True

    if keys[pygame.K_RETURN]:
        if release_enter:
            release_enter = False
            stand = not stand
    else:
        release_enter = True

    if not stand:
        string += "DOWN \n"
    else:
        string += "UP \n"



    if test:
        test_time = time.time()

    #if (1<= time.time()-test_time < 10) : stand=True
    #if (10<= time.time()-test_time < 25) : stand=False





    # Remplissage de l'écran en blanc
    screen.fill(white)

    # Rotation de l'image de la flèche
    if message < 10000000:
        screen.blit(stop_img, stop_img.get_rect(center=arrow_rect.center))
    elif abs(x) == 0:
        screen.blit(circle_img, circle_img.get_rect(center=arrow_rect.center))
    else:
        angle = np.angle(x, deg=True)
        rotated_arrow = pygame.transform.rotate(arrow_img, angle)
        rotated_rect = rotated_arrow.get_rect(center=arrow_rect.center)
        # Affichage de l'image de la flèche
        screen.blit(rotated_arrow, rotated_rect.topleft)

    # Mise à jour de l'affichage
    #write the message on the screen


    if run:
        string += "Running\n"
    else:
        string += "Stopped\n"

    string += "Message: " + str(message) + "\n"

    for i, line in enumerate(string.split("\n")):
        text = font.render(line, True, (0, 128, 0))
        screen.blit(text, (10, 10 + i * 30))




    pygame.display.flip()

    # Limitation du FPS
    pygame.time.Clock().tick(200)

    data = run << 7 | kalman << 6 | test << 5 | stand << 4 | (x.real == 1) << 3 | (x.real == -1) << 2 | (
                x.imag == 1) << 1 | (x.imag == -1)
    ser.write(bytes([data]))
    # print data as binary
    # print(bin(data))

    # read data until \n is received
    Content = ser.readline()
    # remove the \r and \n from the string
    Content = Content.decode().replace("\r\n", "")
    # print(Content)
    message = int(Content)
    print(message)

# Fermeture de Pygame
pygame.quit()
sys.exit()
