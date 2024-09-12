import pygame
import pymunk
import pymunk.pygame_util

pygame.init()

WIDTH, HEIGHT = 1200, 900
COLL_GROUP_ROCKET = 4
FPS = 240
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
    shape.elasticity = 0.0001
    shape.friction = 0.3
    space.add(body, shape)
    return shape

class Rocket:

    thrustSetpoint = 0
    thrustActual   = 0
    thrustIncS     = 5000
    thrustMax      = 5000

    pivotSetpoint  = 0
    pivotActual    = 0
    pivotRate      = 10
    pivotMax       = 2

    posOld = (0, 0)
    speedY = 0
    speedX = 0

    def __init__(self, space, pos):
        #Rocket init
        self.bodyRocket = pymunk.Body()
        self.bodyRocket.position = pos
        self.shapeRocket = pymunk.Poly.create_box(self.bodyRocket, (20, 100))
        self.shapeRocket.mass = 1
        self.shapeRocket.elasticity = 0.5
        self.shapeRocket.friction = 1
        self.shapeRocket.filter = pymunk.ShapeFilter(group=COLL_GROUP_ROCKET)
        self.shapeRocket.color  = (0, 0, 255, 100)

        #Thruster
        self.bodyThruster = pymunk.Body(body_type=pymunk.Body.DYNAMIC) 
        self.bodyThruster.position = (pos[0], pos[1] + 50)
        self.shapeThruster = pymunk.Poly.create_box(self.bodyThruster, (10, 25))
        self.shapeThruster.mass = 1
        self.shapeThruster.elasticity = 0.01
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

    # Set the desired pivot angle of the thruster.
    def setPivot(self, angle):
        angle = angle / 100
        if (angle >= -(self.pivotMax)) and (angle <= self.pivotMax):
            self.pivotSetpoint = angle
        elif angle < -(self.pivotMax):
            self.pivotSetpoint = -(self.pivotMax)
        elif angle > self.pivotMax:
            self.pivotSetpoint = self.pivotMax

    # Set the desired upwards thrust.
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

    # Should be called every frame
    def handle(self):
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

        #Thruster handling
        if (self.thrustSetpoint - self.thrustActual) > (self.thrustIncS / FPS):
            self.thrustActual += self.thrustIncS / FPS
        elif (self.thrustSetpoint - self.thrustActual) < -(self.thrustIncS / FPS):
            self.thrustActual -= self.thrustIncS / FPS
        else:
            self.thrustActual = self.thrustSetpoint
        red = (self.thrustActual / self.thrustMax) * 255 
        self.shapeThruster.color = (red, 0, 0, 100)

        self.bodyThruster.apply_force_at_local_point((0,-(self.thrustActual)), (0,0))

        self.speedX = -(self.posOld[0] - self.bodyRocket.position[0]) * FPS
        self.speedY =  (self.posOld[1] - self.bodyRocket.position[1]) * FPS

        self.posOld = self.bodyRocket.position

class PID():
    kP = 0
    kI = 0
    kD = 0
    iTotal = 0
    errorP = 0
    setpoint = 0
    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint
    
    def run(self, PIDInput, dTime):
        error = self.setpoint - PIDInput
        p = self.kP * error
        self.iTotal += error * dTime
        #print(self.iTotal)
        i = self.iTotal * self.kI
        d = self.kD * (error - self.errorP) / dTime
        self.errorP = error
        output = p + i + d
        return output

class Controller():

    def __init__(self, rocket):
        self.rocket = rocket

        self.pidSpeedY = PID(600, 4.1, 60)
        self.pidAngle  = PID(100, 0,   2)

    def angleSetpoint(self, setpoint):
        self.pidAngle.setSetpoint(setpoint)

    def ySpeedSetpoint(self, setpoint):
        self.pidSpeedY.setSetpoint(setpoint)

    def handle(self):
        outputThrust = self.pidSpeedY.run(self.rocket.getSpeedY(), (1 / FPS))
        outputPivot  = self.pidAngle.run(self.rocket.bodyRocket.angle, (1 / FPS))
        
        self.rocket.setPivot(outputPivot)
        self.rocket.setThrust(outputThrust)
        return

def run(window, width, height):
    running = True
    clock = pygame.time.Clock()

    # Our pymunk space
    space = pymunk.Space()
    space.gravity = (0, 981)

    setpointY = 0
    setpointAngle = 0

    createFloor(space)
    rocket = Rocket(space,(WIDTH / 2, HEIGHT / 2))
    controller = Controller(rocket)
    
    while running:
        # Does all rocket related tasks per frame             
        controller.angleSetpoint(setpointAngle)
        controller.ySpeedSetpoint(setpointY)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                # Break out of while loop
                break 
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_UP:
                    setpointY += 5
                if event.key == pygame.K_DOWN:
                    setpointY -= 5
                if event.key == pygame.K_LEFT:
                    setpointAngle -= 0.02
                if event.key == pygame.K_RIGHT:
                    setpointAngle += 0.02

        controller.handle()
        rocket.handle()
        #print(rocket.getSpeedX(), rocket.getSpeedY())
        
        draw(space, window)
        space.step(1 / FPS)
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    run(window, WIDTH, HEIGHT)
