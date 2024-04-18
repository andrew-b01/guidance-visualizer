import numpy as np

def calculateAcceleration(posX, posY, screenX, screenY, maxAccel, wallProximity, accelerationConstant):

    if posX == 0: posX = 1
    if posY == 0: posY = 1

    if posY < wallProximity:
        accelerationY = ((1/(posY))*accelerationConstant)
        accelerationY = TruncateABSNumber(accelerationY, maxAccel)
    elif posY > (screenY-wallProximity):
        accelerationY = ((1/((screenY-posY)))*-accelerationConstant)
        accelerationY = TruncateABSNumber(accelerationY, maxAccel)
    else:
        accelerationY = 0

    if posX < wallProximity:
        accelerationX = ((1/(posX))*accelerationConstant)
        accelerationX = TruncateABSNumber(accelerationX, maxAccel)
    elif posX > (screenX-wallProximity):
        accelerationX = ((1/((screenX-posX)))*-accelerationConstant)
        accelerationX = TruncateABSNumber(accelerationX, maxAccel)
    else:
        accelerationX = 0
    
    return(accelerationX, accelerationY)

def TruncateABSNumber(number, maxVal):
    return max(-maxVal,min(number,maxVal))

def calculate_triangle_points(center, rotation, size):
    p1 = np.array([0, -size * 2/3])  
    p2 = np.array([-size / 3, size / 4]) 
    p3 = np.array([size / 3, size / 4])
    rotation = rotation-90
    rotation_matrix = np.array([[np.cos(np.radians(rotation)), -np.sin(np.radians(rotation))],
                                [np.sin(np.radians(rotation)), np.cos(np.radians(rotation))]])

    rotated_p1 = center + p1.dot(rotation_matrix)
    rotated_p2 = center + p2.dot(rotation_matrix)
    rotated_p3 = center + p3.dot(rotation_matrix)

    return [(rotated_p1[0], rotated_p1[1]), 
            (rotated_p2[0], rotated_p2[1]),
            (rotated_p3[0], rotated_p3[1])] 

def calculateVelocityFromHeading(velocity,heading):
    return ((np.cos(np.deg2rad(heading))*velocity),(-np.sin(np.deg2rad(heading))*velocity))

def calculateHeadingFromVelocity(velocityX,velocityY):
    return (-np.degrees(np.arctan2(velocityY,velocityX)))

def Hypotenuse(x1,y1):
    return np.sqrt(np.power((x1),2)+np.power((y1),2))

def getAcceleration(seeker, targetPos,targetProperties,seekerProperties):
    seekerPos = seeker[1]
    seekerName = seeker[2]

    if seekerName == "PN":
        return CalculatePN(seekerPos,targetPos,targetProperties, seekerProperties)
    if seekerName == "ZEMAPN":
        return CalculateZEMAPN(seekerPos,targetPos,targetProperties, seekerProperties)
    if seekerName == "APN":
        return CalculateAPN(seekerPos,targetPos,targetProperties, seekerProperties)
    return [0,0]

def vectorReject(A,B):
    return A-(B*(np.dot(A,B)))

lastYAPN = [0,0]
def CalculateAPN(targetPos,seekerPos, targetProperties, seekerProperties):
    global lastYAPN
    N = .3

    Rtm = np.sqrt(np.power((targetPos[0] - seekerPos[0]),2) + np.power((targetPos[1] - seekerPos[1]),2))
    Vc = np.linalg.norm(seekerProperties.getVelocity()-targetProperties.getVelocity())

    y = targetPos-seekerPos

    sinLOS = y/Rtm
    Nt = vectorReject(seekerProperties.getAcceleration(),targetProperties.getAcceleration())
    acceleration = -((N*Vc*sinLOS) + ((N*Nt)/2))

    lastYAPN = y
    return acceleration

def CalculatePN(targetPos,seekerPos, targetProperties, seekerProperties):
    N = 2
    y = targetPos-seekerPos
    Rtm = np.sqrt(np.power((targetPos[0] - seekerPos[0]),2) + np.power((targetPos[1] - seekerPos[1]),2))
    sinLOS = y/Rtm
    acceleration = -N*sinLOS

    return acceleration

lastYZEMAPN = [0,0]
def CalculateZEMAPN(targetPos,seekerPos, targetProperties, seekerProperties):
    global lastYZEMAPN
    N = 1

    Rtm = np.sqrt(np.power((targetPos[0] - seekerPos[0]),2) + np.power((targetPos[1] - seekerPos[1]),2))
    Vc = np.linalg.norm(seekerProperties.getVelocity()-targetProperties.getVelocity())
    tGO = Rtm / Vc

    y = targetPos-seekerPos
    yprime = y-lastYZEMAPN
    zem = y + yprime*tGO

    Nt = vectorReject(seekerProperties.getAcceleration(),targetProperties.getAcceleration())
    acceleration = -((N*zem)/np.power(tGO,2)) + ((N*Nt)/2)

    lastYZEMAPN = y
    return acceleration
    