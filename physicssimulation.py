import pygame
import pymunk
import pymunk.pygame_util
import math

pygame.init()

WIDTH, HEIGHT = 1000, 800
window = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Interactive Physics Simulation")

def calculate_distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def calculate_angle(p1, p2):
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

def draw(space, window, draw_options, line):
    window.fill("white")

    if line:
        pygame.draw.line(window, "black", line[0], line[1], 3)

    space.debug_draw(draw_options)

    pygame.display.update()

def create_boundaries(space, width, height):
    rects = [
        [(width / 2, height - 5), (width, 10)],
        [(width / 2, 5), (width, 10)],
        [(5, height / 2), (10, height)],
        [(width - 5, height / 2), (10, height)]
    ]

    for pos, size in rects:
        body = pymunk.Body(body_type=pymunk.Body.STATIC)
        body.position = pos
        shape = pymunk.Poly.create_box(body, size)
        shape.elasticity = 0.4
        shape.friction = 0.5
        shape.color = (0, 0, 0, 255)  # Black color for boundaries
        space.add(body, shape)

def create_structure(space):
    OBSTACLE_COLOR = (255, 100, 0, 100)  # Orange color for obstacles
    rects = [
        [(600, HEIGHT - 120), (40, 200), OBSTACLE_COLOR, 100],
        [(900, HEIGHT - 120), (40, 200), OBSTACLE_COLOR, 100],
        [(750, HEIGHT - 240), (340, 40), OBSTACLE_COLOR, 150]
    ]

    for pos, size, color, mass in rects:
        body = pymunk.Body()
        body.position = pos
        shape = pymunk.Poly.create_box(body, size, radius=2)
        shape.color = color
        shape.mass = mass
        shape.elasticity = 0.4
        shape.friction = 0.4
        space.add(body, shape)

def create_swinging_ball(space):
    rotation_center_body = pymunk.Body(body_type=pymunk.Body.STATIC)
    rotation_center_body.position = (300, 300)

    body = pymunk.Body()
    body.position = (300, 300)
    line = pymunk.Segment(body, (0, 0), (0, 200), 5)
    circle = pymunk.Circle(body, 40, (0, 200))
    line.friction = 1
    circle.friction = 1
    line.mass = 8
    circle.mass = 30
    circle.elasticity = 0.95
    circle.color = (255, 100, 0, 100)  # Same color as obstacles
    rotation_center_joint = pymunk.PinJoint(body, rotation_center_body, (0, 0), (0, 0))
    space.add(circle, line, body, rotation_center_joint)

def create_ball(space, radius, mass, pos):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position = pos
    shape = pymunk.Circle(body, radius)
    shape.mass = mass
    shape.elasticity = 0.9
    shape.friction = 0.4
    shape.color = (0, 150, 255, 100)  # Blue color for balls
    space.add(body, shape)
    return shape

def get_shape_at_point(space, point):
    shape_list = space.point_query_nearest(point, 1, pymunk.ShapeFilter())
    if shape_list and shape_list.shape:
        return shape_list.shape
    return None

def run(window, width, height):
    run = True
    clock = pygame.time.Clock()
    fps = 60
    dt = 1 / fps

    space = pymunk.Space()
    space.gravity = (0, 981)

    # Align coordinate systems
    pymunk.pygame_util.positive_y_is_up = False

    create_boundaries(space, width, height)
    create_structure(space)
    create_swinging_ball(space)

    draw_options = pymunk.pygame_util.DrawOptions(window)

    pressed_pos = None
    ball = None
    selected_shape = None
    mouse_joint = None
    mouse_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)

    while run:
        line = None
        if ball and pressed_pos:
            line = [pressed_pos, pygame.mouse.get_pos()]

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break

            elif event.type == pygame.MOUSEBUTTONDOWN:
                position = pygame.mouse.get_pos()
                shape = get_shape_at_point(space, position)
                if shape:
                    if shape.body.body_type == pymunk.Body.DYNAMIC:
                        selected_shape = shape
                        mouse_body.position = position
                        offset = selected_shape.body.world_to_local(position)
                        mouse_joint = pymunk.PivotJoint(mouse_body, selected_shape.body, (0, 0), offset)
                        mouse_joint.max_force = 5000
                        space.add(mouse_joint)
                else:
                    if not ball:
                        pressed_pos = position
                        ball = create_ball(space, 30, 10, pressed_pos)
                    elif pressed_pos:
                        pass
                    else:
                        space.remove(ball, ball.body)
                        ball = None

            elif event.type == pygame.MOUSEBUTTONUP:
                if mouse_joint:
                    space.remove(mouse_joint)
                    mouse_joint = None
                    selected_shape = None
                elif ball and pressed_pos:
                    # Launch the ball
                    ball.body.body_type = pymunk.Body.DYNAMIC
                    angle = calculate_angle(pressed_pos, pygame.mouse.get_pos())
                    force = calculate_distance(pressed_pos, pygame.mouse.get_pos()) * 50
                    fx = math.cos(angle) * force
                    fy = math.sin(angle) * force
                    ball.body.apply_impulse_at_local_point((fx, fy), (0, 0))
                    pressed_pos = None
                else:
                    space.remove(ball, ball.body)
                    ball = None

            elif event.type == pygame.MOUSEMOTION:
                if mouse_joint and selected_shape:
                    mouse_body.position = event.pos
                elif ball and pressed_pos:
                    line = [pressed_pos, pygame.mouse.get_pos()]

        draw(space, window, draw_options, line)
        space.step(dt)
        clock.tick(fps)

    pygame.quit()

if __name__ == "__main__":
    run(window, WIDTH, HEIGHT)