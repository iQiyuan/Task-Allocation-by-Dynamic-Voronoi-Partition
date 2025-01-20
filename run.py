import pygame
import random
import math
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Point, Polygon
from robots import *
from utils import *

pygame.init()

ENV_SIZE = 800
screen = pygame.display.set_mode((ENV_SIZE, ENV_SIZE))
pygame.display.set_caption("Weighted VSP-based Task Allocation Simulation - Voronoi")
clock = pygame.time.Clock()

drone = Drone(0, 0, ENV_SIZE)

cars = []
for _ in range(12):
    x = random.uniform(0, ENV_SIZE)
    y = random.uniform(0, ENV_SIZE)
    car = Car(x, y, ENV_SIZE)
    cars.append(car)

items = [Item(ENV_SIZE) for _ in range(8)]
items_detected = [False] * len(items)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    drone.update()
    active_cars = [car for car in cars if not car.assigned_task]

    if len(active_cars) > 0:
        points = np.array([[car.x, car.y] for car in active_cars])
        vor = Voronoi(points)
        regions, vertices = voronoi_finite_polygons_2d(vor, ENV_SIZE)
    else:
        vor = None
        regions, vertices = [], []

    min_x = 0
    max_x = ENV_SIZE
    min_y = 0
    max_y = ENV_SIZE
    boundary = Polygon([(min_x, min_y), (min_x, max_y), (max_x, max_y), (max_x, min_y)])
    car_polygons = {car: None for car in cars}

    if vor is not None:
        for idx, region in enumerate(regions):
            car = active_cars[idx]
            polygon = vertices[region]
            poly = Polygon(polygon)
            poly = poly.intersection(boundary)
            if not poly.is_empty and poly.is_valid and poly.geom_type == 'Polygon':
                car_polygons[car] = poly
                centroid = poly.centroid.coords[0]
                if not car.target_item and not car.delivering:
                    car.centroid = centroid

    for idx, item in enumerate(items):
        if not items_detected[idx] and not item.picked:
            dx = item.x - drone.x
            dy = item.y - drone.y
            distance = math.hypot(dx, dy)
            if distance <= drone.sensor_range:
                items_detected[idx] = True
                print(f"Drone has detected item {idx + 1}.")
                item_point = Point(item.x, item.y)
                for car, poly in car_polygons.items():
                    if poly and poly.contains(item_point):
                        car.set_target(item)
                        print(f"Car at ({car.x:.2f}, {car.y:.2f}) has been assigned to pick up item {idx + 1}.")
                        break

    for car in cars:
        car.dx_total = 0
        car.dy_total = 0

    for car in cars:
        polygon = car_polygons[car]
        car.update(cars, ENV_SIZE, polygon)

    for car in cars:
        total_speed = math.hypot(car.dx_total, car.dy_total)
        if total_speed > car.speed:
            scale = car.speed / total_speed
            car.dx_total *= scale
            car.dy_total *= scale

        car.x += car.dx_total
        car.y += car.dy_total

        if car.delivering:
            dx = car.centroid[0] - car.x
            dy = car.centroid[1] - car.y
            distance_to_target = math.hypot(dx, dy)
            if distance_to_target < car.speed:
                car.has_item = False
                car.delivering = False
                car.item_color = None
                car.just_delivered = True
                car.assigned_task = False
                print(f"Car at ({car.x:.2f}, {car.y:.2f}) has delivered the item.")

        if car.target_item and not car.has_item:
            dx_item = car.target_item.x - car.x
            dy_item = car.target_item.y - car.y
            distance = math.hypot(dx_item, dy_item)
            if distance < car.size + car.target_item.size:
                car.has_item = True
                car.item_color = car.target_item.color
                car.target_item.picked = True
                car.target_item = None
                print(f"Car at ({car.x:.2f}, {car.y:.2f}) has picked up the item.")

    deliveries = False
    for car in cars:
        if car.just_delivered:
            deliveries = True
            car.just_delivered = False

    if deliveries:
        new_items = [Item(ENV_SIZE) for _ in range(1)]
        items.extend(new_items)
        items_detected.extend([False] * len(new_items))
        print("Three new items have been generated.")

    screen.fill((255, 255, 255))

    draw_quarter_circle(screen, (ENV_SIZE, ENV_SIZE), 40, (255, 182, 193), ENV_SIZE)

    for poly in car_polygons.values():
        if poly and poly.exterior:
            x, y = poly.exterior.coords.xy
            coords = [(int(xx), int(yy)) for xx, yy in zip(x, y)]
            pygame.draw.polygon(screen, (0, 255, 0), coords, 1)

    drone.draw(screen)
    for car in cars:
        car.draw(screen)
    for item in items:
        item.draw(screen)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
