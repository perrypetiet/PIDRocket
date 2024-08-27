import pygame
import pymunk
import pymunk.pygame_util

pygame.init()

pivotMotor = 0

WIDTH, HEIGHT = 900, 900
COLL_GROUP_ROCKET = 4
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

def createRocket(space):
    #rocket
    bodyRocket = pymunk.Body()
    bodyRocket.position = (WIDTH / 2, HEIGHT / 2)
    shapeRocket = pymunk.Poly.create_box(bodyRocket, (20, 100))
    shapeRocket.mass = 1
    shapeRocket.elasticity = 0.5
    shapeRocket.friction = 1
    shapeRocket.filter = pymunk.ShapeFilter(group=COLL_GROUP_ROCKET)
    shapeRocket.color  = (0, 0, 255, 100)

    #Thruster
    bodyThruster = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
    bodyThruster.position = (WIDTH / 2 + 30, HEIGHT / 2 + 30)
    shapeThruster = pymunk.Poly.create_box(bodyThruster, (10, 25))
    shapeThruster.mass = 1
    shapeThruster.elasticity = 1
    shapeThruster.friction = 0.1
    shapeThruster.filter = pymunk.ShapeFilter(group=COLL_GROUP_ROCKET)
    shapeThruster.color = (0, 0, 0, 100)

    #Stitch them together with pivot joint and motor
    aRocket = (0, 50)
    aThruster = (0, 0)
    joint = pymunk.PivotJoint(bodyRocket, bodyThruster, aRocket, aThruster)
    pivotMotor = pymunk.SimpleMotor(bodyRocket, bodyThruster, 0)
    

    space.add(bodyRocket,   shapeRocket)
    space.add(bodyThruster, shapeThruster)
    space.add(pivotMotor)
    space.add(joint)
    return shapeRocket, bodyThruster, pivotMotor


def run(window, width, height):
    running = True
    clock = pygame.time.Clock()
    fps = 60
  
    # Our pymunk space
    space = pymunk.Space()
    space.gravity = (0, 981)

    floor = createFloor(space)
    rocket, thrusterBody, pivotMotor = createRocket(space)

    #Thruster variables
    thrusterForce = 1962 
    pivotRate     = 0

    while running:
        thrusterBody.apply_force_at_local_point((0,-thrusterForce), (0,0))
        pivotMotor.rate = 0
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
                    pivotMotor.rate = -5
                    pivotRate = pivotRate - 5
                if event.key == pygame.K_RIGHT:
                    # Applies rate for only 1 frame
                    pivotMotor.rate = 5
                    pivotRate = pivotRate + 5

        draw(space, window)
        space.step(1 / fps)
        clock.tick(fps)

    pygame.quit()


if __name__ == "__main__":
    run(window, WIDTH, HEIGHT)
