import pygame
import math
import random
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Point, Polygon

pygame.init()

ENV_SIZE = 800
screen = pygame.display.set_mode((ENV_SIZE, ENV_SIZE))
pygame.display.set_caption("Multi-Agent Cooperation Simulation - Voronoi")
clock = pygame.time.Clock()

class Drone:
    def __init__(self, x, y, env_size):
        self.x = x
        self.y = y
        self.size = 10
        self.color = (0, 0, 255)
        self.env_size = env_size
        self.path = self.generate_zigzag_path()
        self.path_index = 0
        self.speed = 6
        self.sensor_range = 100

    def generate_zigzag_path(self):
        """Generate a zigzag path."""
        path = []
        step = 200
        num_steps = int(self.env_size / step) + 1
        direction = 1

        for i in range(num_steps):
            x = i * step
            if direction == 1:
                y_range = np.arange(100, self.env_size, step)
            else:
                y_range = np.arange(self.env_size - 100, 0, -step)
            for y in y_range:
                path.append((x, y))
            direction *= -1

        return path

    def update(self):
        """Update drone position."""
        if self.path_index < len(self.path):
            target_x, target_y = self.path[self.path_index]
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)
            if distance > self.speed:
                self.x += (dx / distance) * self.speed
                self.y += (dy / distance) * self.speed
            else:
                self.x = target_x
                self.y = target_y
                self.path_index += 1
        else:
            self.path_index = 0

    def draw(self, screen):
        """Draw the drone."""
        point1 = (int(self.x), int(self.y - self.size))
        point2 = (int(self.x - self.size), int(self.y + self.size))
        point3 = (int(self.x + self.size), int(self.y + self.size))
        pygame.draw.polygon(screen, self.color, [point1, point2, point3])
        pygame.draw.circle(screen, (173, 216, 230), (int(self.x), int(self.y)), int(self.sensor_range), 1)


class Car:
    def __init__(self, x, y, env_size):
        self.x = x
        self.y = y
        self.size = 10
        self.color = (255, 0, 0)
        self.env_size = env_size
        self.speed = 4
        self.K_att = 1.0
        self.centroid = (x, y)
        self.has_item = False
        self.item_color = None
        self.target_item = None
        self.delivering = False
        self.just_delivered = False
        self.assigned_task = False

    def set_target(self, item):
        """Assign item and move to pick it up."""
        self.centroid = (item.x, item.y)
        self.target_item = item
        self.assigned_task = True
        print(f"Car at ({self.x:.2f}, {self.y:.2f}) is heading to the item.")

    def update(self, cars, polygon):
        """Update car position."""
        dx_total = 0
        dy_total = 0

        if self.has_item and not self.delivering:
            self.delivering = True
            radius = 20
            angle = random.uniform(180, 270)
            rad = math.radians(angle)
            self.centroid = (ENV_SIZE + radius * math.cos(rad), ENV_SIZE + radius * math.sin(rad))
            print(f"Car at ({self.x:.2f}, {self.y:.2f}) is delivering the item.")

        if self.delivering:
            dx = self.centroid[0] - self.x
            dy = self.centroid[1] - self.y
            distance_to_target = math.hypot(dx, dy)
            if distance_to_target > 0:
                dx_norm = (dx / distance_to_target) * min(distance_to_target, self.speed)
                dy_norm = (dy / distance_to_target) * min(distance_to_target, self.speed)
            else:
                dx_norm, dy_norm = 0, 0

            dx_total += dx_norm
            dy_total += dy_norm
        else:
            dx = self.centroid[0] - self.x
            dy = self.centroid[1] - self.y
            distance_to_target = math.hypot(dx, dy)
            if distance_to_target > 0:
                dx_norm = (dx / distance_to_target) * min(distance_to_target, self.speed)
                dy_norm = (dy / distance_to_target) * min(distance_to_target, self.speed)
            else:
                dx_norm, dy_norm = 0, 0

            dx_total += dx_norm
            dy_total += dy_norm

        # Repulsive force from other cars
        for other_car in cars:
            if other_car != self:
                dx_oc = self.x - other_car.x
                dy_oc = self.y - other_car.y
                distance_oc = math.hypot(dx_oc, dy_oc)
                if distance_oc <= 50:
                    k = 1500
                    force_magnitude = k / (distance_oc ** 2 + 1e-8)
                    unit_vector = np.array([dx_oc, dy_oc]) / (distance_oc + 1e-8)
                    repulsive_force = force_magnitude * unit_vector

                    dx_total += repulsive_force[0]
                    dy_total += repulsive_force[1]

                    other_car.dx_total -= repulsive_force[0]
                    other_car.dy_total -= repulsive_force[1]

        self.dx_total += dx_total
        self.dy_total += dy_total

    def draw(self, screen):
        """Draw the car."""
        color = self.item_color if self.item_color else self.color
        rect = pygame.Rect(int(self.x - self.size), int(self.y - self.size), int(self.size * 2), int(self.size * 2))
        pygame.draw.rect(screen, color, rect)

        if self.assigned_task:
            pygame.draw.circle(screen, color, (int(self.x), int(self.y)), 50, 1)


class Item:
    def __init__(self, env_size):
        self.x = random.uniform(0, env_size)
        self.y = random.uniform(0, env_size)
        self.size = 8
        self.color = (255, 165, 0)
        self.picked = False

    def draw(self, screen):
        """Draw the item."""
        if not self.picked:
            pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.size)


def draw_quarter_circle(screen, center, radius, color):
    """Draw a quarter circle in the bottom-right corner."""
    points = []
    for angle in range(180, 271):
        rad = math.radians(angle)
        x = center[0] + radius * math.cos(rad)
        y = center[1] + radius * math.sin(rad)
        x = min(max(x, 0), ENV_SIZE)
        y = min(max(y, 0), ENV_SIZE)
        points.append((x, y))
    points.append(center)
    pygame.draw.polygon(screen, color, points)


def voronoi_finite_polygons_2d(vor, radius=None):
    """Compute finite Voronoi polygons."""
    if radius is None:
        radius = ENV_SIZE * 2

    new_regions = []
    new_vertices = vor.vertices.tolist()
    center = vor.points.mean(axis=0)
    all_ridges = {}

    for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
        all_ridges.setdefault(p1, []).append((p2, v1, v2))
        all_ridges.setdefault(p2, []).append((p1, v1, v2))

    for p1, region in enumerate(vor.point_region):
        vertices = vor.regions[region]

        if all(v >= 0 for v in vertices):
            new_regions.append(vertices)
            continue

        ridges = all_ridges[p1]
        new_region = [v for v in vertices if v >= 0]

        for p2, v1, v2 in ridges:
            if v2 < 0:
                v1, v2 = v2, v1
            if v1 >= 0:
                continue

            t = vor.points[p2 % len(vor.points)] - vor.points[p1 % len(vor.points)]
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])

            midpoint = vor.points[[p1 % len(vor.points), p2 % len(vor.points)]].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, n)) * n
            far_point = vor.vertices[v2] + direction * radius

            new_vertices.append(far_point.tolist())
            new_region.append(len(new_vertices) - 1)

        vs = np.asarray([new_vertices[v] for v in new_region])
        c = vs.mean(axis=0)
        angles = np.arctan2(vs[:, 1] - c[1], vs[:, 0] - c[0])
        new_region = [v for _, v in sorted(zip(angles, new_region))]
        new_regions.append(new_region)

    return new_regions, np.array(new_vertices)


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
        regions, vertices = voronoi_finite_polygons_2d(vor)
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
        car.update(cars, polygon)

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

    draw_quarter_circle(screen, (ENV_SIZE, ENV_SIZE), 40, (255, 182, 193))

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
