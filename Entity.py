import numpy as np

class Entity:
    def __init__(self,posX, posY, velocityX, velocityY, accelerationX, accelerationY, heading):
      self.posX = posX
      self.posY = posY
      self.velocityX = velocityX
      self.velocityY = velocityY
      self.accelerationX = accelerationX
      self.accelerationY = accelerationY
      self.heading = heading
      self.positionHistory = []

    def getHeading(self):
        heading = self.heading
        return heading

    def getPosition(self):
        position = [self.posX, self.posY]
        return  np.array(position)

    def getVelocity(self):
        velocity = [self.velocityX, self.velocityY]
        return  np.array(velocity)
    
    def getAcceleration(self):
        acceleration = [self.accelerationX, self.accelerationY]
        return np.array(acceleration)

    def setVelocity(self, velocityX, velocityY):
        self.velocityX = velocityX
        self.velocityY = velocityY

    def setAcceleration(self, accelerationX, accelerationY):
        self.accelerationX = accelerationX
        self.accelerationY = accelerationY
    
    def setHeading(self, heading):
        self.heading = heading