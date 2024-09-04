import pygame
import pymunk
import pymunk.pygame_util

pygame.init()

# temp here:
iTotal = 0
errorP = 0

WIDTH, HEIGHT = 1200, 900
COLL_GROUP_ROCKET = 4
FPS = 60
window = pygame.display.set_mode((WIDTH, HEIGHT))

def draw(space, window):
    window.fill("white")
    space.debug_draw(pymunk.pygame_util.DrawOptions(window))
    pygame.display.update()

def createFloor(space):
    pos = (WIDTH / 2, HEIGHT - 10)
    size = (WIDTH, 20)
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position = pos
    shape = pymunk.Poly.create_box(body, size)
    shape.color = (0, 100, 0, 100)
    shape.elasticity = 0.001
    shape.friction = 0.3
    space.add(body, shape)
    return shape

class Rocket:

    thrustSetpoint = 0
    thrustActual   = 0
    thrustIncS     = 5000
    thrustMax      = 2000

    pivotSetpoint  = 0
    pivotActual    = 0
    pivotRate      = 2
    pivotMax       = 1   #1 radian

    posOld = (0, 0)
    speedY = 0
    speedX = 0
    speedYOld = 0
    speedXOld = 0

    def __init__(self, space, pos):
        #Rocket init
        self.bodyRocket = pymunk.Body()
        self.bodyRocket.position = pos
        self.shapeRocket = pymunk.Poly.create_box(self.bodyRocket, (20, 100))
        self.shapeRocket.mass = 0.8
        self.shapeRocket.elasticity = 0.5
        self.shapeRocket.friction = 1
        self.shapeRocket.filter = pymunk.ShapeFilter(group=COLL_GROUP_ROCKET)
        self.shapeRocket.color  = (0, 0, 255, 100)

        #Thruster
        self.bodyThruster = pymunk.Body(body_type=pymunk.Body.DYNAMIC) 
        self.bodyThruster.position = (pos[0], pos[1] + 100)
        self.shapeThruster = pymunk.Poly.create_box(self.bodyThruster, (10, 25))
        self.shapeThruster.mass = 0.2
        self.shapeThruster.elasticity = 1
        self.shapeThruster.friction = 0.1
        self.shapeThruster.filter = pymunk.ShapeFilter(group=COLL_GROUP_ROCKET)
        self.shapeThruster.color = (0, 0, 0, 100)

        #Stitch them together with pivot joint and motor
        aRocket = (0, 50)
        aThruster = (0, 0)
        self.joint = pymunk.PivotJoint(self.bodyRocket, self.bodyThruster, aRocket, aThruster)
        self.pivotMotor = pymunk.SimpleMotor(self.bodyRocket, self.bodyThruster, 0)
    
        space.add(self.bodyRocket,   self.shapeRocket)
        space.add(self.bodyThruster, self.shapeThruster)
        space.add(self.pivotMotor)
        space.add(self.joint)

    def setPivot(self, angle):
        if (angle >= -(self.pivotMax)) and (angle <= self.pivotMax):
            self.pivotSetpoint = angle

    def setThrust(self, thrust):
        if thrust <= self.thrustMax and thrust >= 0:
            self.thrustSetpoint = thrust
        elif thrust > self.thrustMax:
            self.thrustSetpoint = self.thrustMax
        elif thrust < 0:
            self.thrustSetpoint = 0

    def getSpeedX(self):
        return self.speedX
    
    def getSpeedXOld(self):
        return self.speedXOld

    def getSpeedY(self):
        return self.speedY
    
    def getSpeedYOld(self):
        return self.speedYOld

    def handle(self):
        #Thruster handling
        if (self.thrustSetpoint - self.thrustActual) > (self.thrustIncS / FPS):
            self.thrustActual += self.thrustIncS / FPS
        elif (self.thrustSetpoint - self.thrustActual) < -(self.thrustIncS / FPS):
            self.thrustActual -= self.thrustIncS / FPS
        else:
            self.thrustActual = self.thrustSetpoint
        red = (self.thrustActual / self.thrustMax) * 255 
        self.shapeThruster.color = (red, 0, 0, 100)

        #Pivotmotor handling
        if self.pivotActual != self.pivotSetpoint:
            if self.pivotSetpoint < self.pivotActual:
                self.pivotMotor.rate = -(self.pivotRate)
                self.pivotActual -= self.pivotRate * (1 / FPS)
            elif self.pivotSetpoint > self.pivotActual:
                self.pivotMotor.rate = self.pivotRate
                self.pivotActual += self.pivotRate * (1 / FPS)
        else:
            self.pivotMotor.rate = 0         

        self.bodyThruster.apply_force_at_local_point((0,-(self.thrustActual)), (0,0))

        #Calculate speed:
        self.speedXOld = self.speedX
        self.speedYOld = self.speedY

        self.speedX = -(self.posOld[0] - self.bodyRocket.position[0]) * FPS
        self.speedY =  (self.posOld[1] - self.bodyRocket.position[1]) * FPS

        self.posOld = self.bodyRocket.position
        #print(self.speedX, self.speedY)

def PIDthrust(rocket, speedSetpoint):
    error = speedSetpoint - rocket.getSpeedY()
    print(rocket.getSpeedY())
    # *** Proportional factor *****
    kP = 30    #Proportional gain
    p = kP * error

    # *** Integral factor *********
    kI = 2
    i = kI * error * (1 / FPS)
    global iTotal
    iTotal += i

    # *** Derivative factor *******
    global errorP
    kD = 0.6
    d = kD * (error - errorP) / (1 / FPS)

    errorP = error
    rocket.setThrust(p + iTotal + d)
    return

def run(window, width, height):
    running = True
    clock = pygame.time.Clock()

    # Our pymunk space
    space = pymunk.Space()
    space.gravity = (0, 500)

    floor  = createFloor(space)
    rocket = Rocket(space,(WIDTH / 2, HEIGHT / 2 - 200))
    rocket.setThrust(0)

    currentPivot  = 0

    while running:
        # Does all rocket related tasks per frame             
        rocket.handle()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                # Break out of while loop
                break 
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_LEFT:
                    # Applies rate for only 1 frame
                    currentPivot -= 0.1
                if event.key == pygame.K_RIGHT:
                    # Applies rate for only 1 frame
                    currentPivot += 0.1
        #For pid, the input is the speed of the rocket, the process value is speed in pixels/s 
        # and the output is thrust(force) or pivot(angle)

        rocket.setPivot(currentPivot)
        #rocket.setThrust(currentThrust)
        PIDthrust(rocket, 0) # Speed setpoint is 0

        draw(space, window)
        space.step(1 / FPS)
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    run(window, WIDTH, HEIGHT)
