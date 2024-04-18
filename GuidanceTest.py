import pygame
import numpy as np
from Entity import Entity
import random
import HelperFunctions as hp

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1920, 1080))
clock = pygame.time.Clock()
running = True
dt = 0
update = True

accelerationConstant = 24
maxAccel = 9
targetMaxVelocity = 2
wallProximity = 50
seekerVelocity = 4

targetProperties = Entity(1200,400,10,4,0,0, 0)
targetPos = pygame.Vector2(targetProperties.getPosition()[0], targetProperties.getPosition()[1])

PNProperties = Entity(0, 350, 12, 0, 0, 0, 0)  
PNPos = pygame.Vector2(PNProperties.getPosition()[0], PNProperties.getPosition()[1])
PNMissile = [PNProperties,PNPos,"PN","green"]

ZEMAPNProperties = Entity(0, 350, 12, 0, 0, 0, 0)  
APNPos = pygame.Vector2(ZEMAPNProperties.getPosition()[0], ZEMAPNProperties.getPosition()[1])
APNMissile = [ZEMAPNProperties,APNPos,"ZEMAassssasPN","red"]


allSeekers = [PNMissile, APNMissile]

frames = 0

def drawSeekers(seekers):
    global targetPos
    for seeker in seekers:

        seekerProperties = seeker[0]
        seekerPos = seeker[1]
        seekerName = seeker[2]
        color = seeker[3]

        if frames % 1 == 0:
            seekerProperties.positionHistory.append([seekerPos.copy(), seekerProperties.getHeading()]) 
        if frames % 2 == 0:
            targetProperties.positionHistory.append([targetPos.copy()])

        if len(seekerProperties.positionHistory) > 250:
            seekerProperties.positionHistory.pop(0)
        
        if len(targetProperties.positionHistory) > 200:
            targetProperties.positionHistory.pop(0)

        for seekerHistory in seekerProperties.positionHistory:
            pygame.draw.circle(screen, color, seekerHistory[0], 2)

            # seekerHistoryPos = seekerHistory[0]
            # rotation = seekerHistory[1]
            # triangle_points = hp.calculate_triangle_points(seekerHistoryPos, rotation, 20) 
            # pygame.draw.polygon(screen, color, triangle_points)
            # font = pygame.font.Font(None, 24)  
            # text = font.render(seekerName, True, "white")
            # text_rect = text.get_rect(center=(seekerHistoryPos.x, seekerHistoryPos.y - 25)) 
            # screen.blit(text, text_rect) 
        
        for targetHistory in targetProperties.positionHistory:
            targetPos = targetHistory[0]
            pygame.draw.circle(screen, "white", targetPos, 3)


        rotation = seekerProperties.getHeading()
        triangle_points = hp.calculate_triangle_points(seekerPos, rotation, 20) 
        pygame.draw.polygon(screen, color, triangle_points)
        font = pygame.font.Font(None, 24)  
        text = font.render(seekerName, True, "white")
        text_rect = text.get_rect(center=(seekerPos.x, seekerPos.y - 25)) 
        screen.blit(text, text_rect) 


while running:

    if update:
        screen.fill("black")

        targetVelocity = targetProperties.getVelocity()
        targetAcceleration = targetProperties.getAcceleration()

        velocityX = hp.TruncateABSNumber((targetVelocity[0] + targetAcceleration[0]), targetMaxVelocity)
        velocityY = hp.TruncateABSNumber((targetVelocity[1] + targetAcceleration[1]), targetMaxVelocity)
        accelerationX, accelerationY = hp.calculateAcceleration(targetPos.x, targetPos.y, screen.get_width(), screen.get_height(), maxAccel, wallProximity, accelerationConstant)
        
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            accelerationY -= .15
        if keys[pygame.K_s]:
            accelerationY += .15
        if keys[pygame.K_a]:
            accelerationX -= .15
        if keys[pygame.K_d]:
            accelerationX += .15

        targetPos.x += targetVelocity[0]
        targetPos.y += targetVelocity[1]
        targetProperties.setVelocity(velocityX, velocityY)
        targetProperties.setAcceleration(accelerationX, accelerationY)
        targetProperties.setHeading(hp.calculateHeadingFromVelocity(velocityX, velocityY))
        targetPos.x = min(max(0,targetPos.x),screen.get_width()-1)
        targetPos.y = min(max(0,targetPos.y),screen.get_height()-12)

        for i, seeker in enumerate(allSeekers):
            seekerProperties = allSeekers[i][0]
            seekerPos = allSeekers[i][1]

            seekerPos.x += seekerProperties.getVelocity()[0]
            seekerPos.y +=seekerProperties.getVelocity()[1]

            seekerPos.x = min(max(0,seekerPos.x),screen.get_width())
            seekerPos.y = min(max(0,seekerPos.y),screen.get_height())
            if seekerPos.x == 0: seekerProperties.setAcceleration(0,seekerProperties.getAcceleration()[1])
            if seekerPos.y == 0: seekerProperties.setAcceleration(seekerProperties.getAcceleration()[0],0)

            seekerVelocityX = seekerProperties.getVelocity()[0]
            seekerVelocityY = seekerProperties.getVelocity()[1]

            accelX, accelY = hp.getAcceleration(seeker, targetPos,targetProperties,seekerProperties)
   
            seekerProperties.setAcceleration(accelX, accelY)
            
            velocityX = hp.TruncateABSNumber(seekerVelocityX + accelX, seekerVelocity)
            velocityY = hp.TruncateABSNumber(seekerVelocityY + accelY, seekerVelocity)

            seekerProperties.setVelocity(velocityX, velocityY)
            seekerProperties.setHeading(hp.calculateHeadingFromVelocity(velocityX,velocityY))

            if abs(seekerPos.x - targetPos.x) < 15 and abs(seekerPos.y - targetPos.y) < 15:
                update = False
                # running = False

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False



    pygame.draw.circle(screen, "red", targetPos, 10)
    drawSeekers(allSeekers)


    pygame.display.flip()
    dt = clock.tick(60) / 1000

    frames += 1

pygame.quit()