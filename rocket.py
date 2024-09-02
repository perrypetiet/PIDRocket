import pygame
import pymunk
import pymunk.pygame_util

pygame.init()

pivotMotor = 0

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
    thrustIncS     = 150
    thrustMax      = 200

    pivotSetpoint  = 0
    pivotActual    = 0
    pivotRate      = 2
    pivotMax       = 1   #1 radian

    def __init__(self, space, pos):
        #Rocket init
        self.bodyRocket = pymunk.Body()
        self.bodyRocket.position = (WIDTH / 2, HEIGHT / 2)
        self.shapeRocket = pymunk.Poly.create_box(self.bodyRocket, (20, 100))
        self.shapeRocket.mass = 0.8
        self.shapeRocket.elasticity = 0.5
        self.shapeRocket.friction = 1
        self.shapeRocket.filter = pymunk.ShapeFilter(group=COLL_GROUP_ROCKET)
        self.shapeRocket.color  = (0, 0, 255, 100)

        #Thruster
        self.bodyThruster = pymunk.Body(body_type=pymunk.Body.DYNAMIC) 
        self.bodyThruster.position = (WIDTH / 2, HEIGHT / 2 + 100)
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

        print("Thrust: ", self.thrustActual)
        print("Pivot: ", self.pivotActual)

def run(window, width, height):
    running = True
    clock = pygame.time.Clock()
  
    font = pygame.font.SysFont("Arial", 20)

    # Our pymunk space
    space = pymunk.Space()
    space.gravity = (0, 100)

    floor  = createFloor(space)
    rocket = Rocket(space,(WIDTH / 2, HEIGHT / 2))

    currentPivot  = 0
    currentThrust = 0

    while running:
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
                if event.key == pygame.K_UP:
                    currentThrust += 10
                if event.key == pygame.K_DOWN:
                    currentThrust -= 10

        rocket.setPivot(currentPivot)
        rocket.setThrust(currentThrust)
        
        # Does all rocket related tasks per frame             
        rocket.handle()

        draw(space, window)
        space.step(1 / FPS)
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    run(window, WIDTH, HEIGHT)
