
import math
import pygame
import random
import numpy as np

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
        # Experiments parameters
        self.last_x = x
        self.last_y = y
        self.current_item = None

    def set_target(self, item):
        """Assign item and move to pick it up."""
        self.centroid = (item.x, item.y)
        self.target_item = item
        self.assigned_task = True
        print(f"Car at ({self.x:.2f}, {self.y:.2f}) is heading to the item.")

    def update(self, cars, ENV_SIZE, polygon):
        """Update car position."""
        dx_total = 0
        dy_total = 0

        if self.has_item and not self.delivering:
            self.delivering = True
            radius = 40
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

    def update_nml(self, cars, ENV_SIZE, polygon):
            dx_total = 0
            dy_total = 0

            if self.has_item and not self.delivering:
                self.delivering = True
                radius = 40
                angle = random.uniform(180, 270)
                rad = math.radians(angle)
                self.centroid = (ENV_SIZE + radius * math.cos(rad),
                                 ENV_SIZE + radius * math.sin(rad))
                print(f"Car at ({self.x:.2f}, {self.y:.2f}) is delivering the item.")

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

            # Repulsive force to avoid collision
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
        if not self.picked:
            pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.size)
