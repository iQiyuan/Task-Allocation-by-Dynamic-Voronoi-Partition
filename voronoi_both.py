import pygame
import math
import random
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Point, Polygon
import matplotlib.pyplot as plt

def voronoi_finite_polygons_2d(vor, radius=None):
    if radius is None:
        radius = 800 * 2
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
        angles = np.arctan2(vs[:,1]-c[1], vs[:,0]-c[0])
        new_region = [v for _,v in sorted(zip(angles, new_region))]
        new_regions.append(new_region)
    return new_regions, np.array(new_vertices)


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
        self.last_x = x
        self.last_y = y
        self.current_item = None  # 用于追踪已经拾取的物品直到送达结束再计算生命周期

    def set_target(self, item):
        self.centroid = (item.x, item.y)
        self.target_item = item
        self.assigned_task = True
        print(f"小车在 ({self.x:.2f}, {self.y:.2f}) 正在前往物品。")

    def update(self, cars, polygon):
        pass

    def draw(self, screen):
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


def draw_quarter_circle(screen, center, radius, color):
    points = []
    for angle in range(180, 271):
        rad = math.radians(angle)
        x = center[0] + radius * math.cos(rad)
        y = center[1] + radius * math.sin(rad)
        x = min(max(x, 0), 800)
        y = min(max(y, 0), 800)
        points.append((x, y))
    points.append(center)
    pygame.draw.polygon(screen, color, points)


def run_experiment_1():
    ENV_SIZE = 800
    pygame.init()
    screen = pygame.display.set_mode((ENV_SIZE, ENV_SIZE))
    pygame.display.set_caption("Experiment 1")
    clock = pygame.time.Clock()

    FPS = 60
    FRAMES_PER_MINUTE = 60 * FPS
    frame_count = 0

    deliveries_count = 0
    car_task_counts = []
    car_idle_frames = []
    total_distance_cars = []
    sum_task_overlap = 0
    appear_time_dict = {}
    delivery_time_diffs = []  # 在送达后计算完整周期
    sum_busy_cars = 0

    boxes_delivered_list = []
    std_task_list = []
    idle_ratio_list = []
    box_delivery_eff_list = []
    busy_cars_list = []

    class Car_Exp1(Car):
        def update(self, cars, polygon):
            dx_total = 0
            dy_total = 0

            if self.has_item and not self.delivering:
                self.delivering = True
                radius = 40
                angle = random.uniform(180, 270)
                rad = math.radians(angle)
                self.centroid = (ENV_SIZE + radius * math.cos(rad), ENV_SIZE + radius * math.sin(rad))
                print(f"小车在 ({self.x:.2f}, {self.y:.2f}) 正在送货。")

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

            # 避让其他车辆
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

    drone = Drone(0,0,ENV_SIZE)
    cars = []
    for _ in range(12):
        x = random.uniform(0, ENV_SIZE)
        y = random.uniform(0, ENV_SIZE)
        car = Car_Exp1(x, y, ENV_SIZE)
        cars.append(car)
    car_idle_frames = [0]*len(cars)
    total_distance_cars = [0]*len(cars)

    items = [Item(ENV_SIZE) for _ in range(8)]
    items_detected = [False]*len(items)

    running = True
    while running:
        frame_count += 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        drone.update()

        points = np.array([[car.x, car.y] for car in cars])
        vor = Voronoi(points)
        regions, vertices = voronoi_finite_polygons_2d(vor)

        boundary = Polygon([
            (0, 0),
            (0, ENV_SIZE),
            (ENV_SIZE, ENV_SIZE),
            (ENV_SIZE, 0)
        ])

        car_polygons = {car: None for car in cars}
        for idx, region in enumerate(regions):
            polygon = vertices[region]
            poly = Polygon(polygon)
            poly = poly.intersection(boundary)
            if not poly.is_empty and poly.is_valid and poly.geom_type == 'Polygon':
                car_polygons[cars[idx]] = poly
                if (not cars[idx].assigned_task) and (not cars[idx].delivering) and (not cars[idx].has_item) and (not cars[idx].target_item):
                    cars[idx].centroid = poly.centroid.coords[0]

        for item in items:
            if item not in appear_time_dict:
                appear_time_dict[item] = frame_count

        # 物品分配尝试
        for idx, item in enumerate(items):
            if not item.picked:
                dx = item.x - drone.x
                dy = item.y - drone.y
                distance = math.hypot(dx, dy)
                if distance <= drone.sensor_range:
                    if not items_detected[idx]:
                        print(f"无人机检测到物品 {idx + 1}。")
                    item_point = Point(item.x, item.y)
                    assigned = False
                    for car, poly in car_polygons.items():
                        if poly and poly.contains(item_point) and (not car.has_item) and (not car.delivering) and (not car.assigned_task):
                            car.set_target(item)
                            print(f"小车在 ({car.x:.2f}, {car.y:.2f}) 被指派去拾取物品 {idx + 1}。")
                            assigned = True
                            break
                    if assigned:
                        items_detected[idx] = True

        for car in cars:
            car.dx_total = 0
            car.dy_total = 0

        for car in cars:
            polygon = car_polygons[car]
            old_x, old_y = car.x, car.y
            car.update(cars, polygon)

        # 计算任务重叠数
        target_map = {}
        for car in cars:
            if car.target_item and not car.has_item:
                target_map[car.target_item] = target_map.get(car.target_item, 0) + 1
        frame_overlap = 0
        for count in target_map.values():
            if count > 1:
                frame_overlap += (count - 1)
        sum_task_overlap += frame_overlap

        deliveries = False
        busy_cars_count = 0

        for i, car in enumerate(cars):
            total_speed = math.hypot(car.dx_total, car.dy_total)
            if total_speed > car.speed:
                scale = car.speed / total_speed
                car.dx_total *= scale
                car.dy_total *= scale

            old_x, old_y = car.x, car.y
            car.x += car.dx_total
            car.y += car.dy_total

            distance_moved = math.hypot(car.x - car.last_x, car.y - car.last_y)
            total_distance_cars[i] += distance_moved
            car.last_x, car.last_y = car.x, car.y

            if car.delivering:
                dx = car.centroid[0] - car.x
                dy = car.centroid[1] - car.y
                distance_to_target = math.hypot(dx, dy)
                if distance_to_target < car.speed:
                    # 完成送达，计算从出现到送达的lifecycle
                    if car.current_item is not None:
                        delivery_time_diffs.append(frame_count - appear_time_dict[car.current_item])
                        car.current_item = None

                    car.has_item = False
                    car.delivering = False
                    car.item_color = None
                    car.just_delivered = True
                    car.assigned_task = False
                    print(f"小车在 ({car.x:.2f}, {car.y:.2f}) 已送达物品。")

            if car.target_item and not car.has_item:
                dx_item = car.target_item.x - car.x
                dy_item = car.target_item.y - car.y
                distance = math.hypot(dx_item, dy_item)
                if distance < car.size + car.target_item.size:
                    car.has_item = True
                    car.item_color = car.target_item.color
                    car.target_item.picked = True
                    # 拾取时记录current_item
                    car.current_item = car.target_item
                    car.target_item = None
                    print(f"小车在 ({car.x:.2f}, {car.y:.2f}) 已拾取物品。")

            if not car.assigned_task and not car.delivering and not car.target_item and not car.has_item:
                car_idle_frames[i] += 1

            if car.just_delivered:
                deliveries = True
                car.just_delivered = False
                car_task_counts.append(i)
                deliveries_count += 1

            if car.assigned_task or car.delivering:
                busy_cars_count += 1

        sum_busy_cars += busy_cars_count

        if deliveries:
            new_item = Item(ENV_SIZE)
            items.append(new_item)
            items_detected.append(False)
            appear_time_dict[new_item] = frame_count
            print("生成了一个新的物品。")

        # 每帧记录
        boxes_delivered_list.append(deliveries_count)
        task_count_array = [0]*len(cars)
        for car_index in car_task_counts:
            task_count_array[car_index] += 1
        if len(task_count_array) > 1:
            current_std_task = np.std(task_count_array)
        else:
            current_std_task = 0.0
        std_task_list.append(current_std_task)

        total_idle_frames = sum(car_idle_frames)
        total_car_frames = frame_count * len(cars)
        if total_car_frames > 0:
            current_idle_ratio = total_idle_frames / total_car_frames
        else:
            current_idle_ratio = 0
        idle_ratio_list.append(current_idle_ratio)

        if len(items) > 0:
            current_box_delivery_eff = (deliveries_count / len(items)) * 100
        else:
            current_box_delivery_eff = 0
        box_delivery_eff_list.append(current_box_delivery_eff)

        busy_cars_list.append(busy_cars_count)

        screen.fill((255, 255, 255))
        draw_quarter_circle(screen, (ENV_SIZE, ENV_SIZE), 40, (255, 182, 193))

        for poly in car_polygons.values():
            if poly and poly.exterior:
                x, y = poly.exterior.xy
                coords = [(int(xx), int(yy)) for xx, yy in zip(x, y)]
                pygame.draw.polygon(screen, (0, 255, 0), coords, 1)

        drone.draw(screen)
        for car in cars:
            car.draw(screen)

        for item in items:
            item.draw(screen)

        pygame.display.flip()
        clock.tick(FPS)

        if frame_count >= FRAMES_PER_MINUTE:
            running = False

    pygame.quit()

    boxes_per_minute = deliveries_count
    task_count_array = [0]*len(cars)
    for car_index in car_task_counts:
        task_count_array[car_index] += 1
    if len(task_count_array) > 1:
        std_task = np.std(task_count_array)
    else:
        std_task = 0.0

    total_idle_frames = sum(car_idle_frames)
    total_car_frames = FRAMES_PER_MINUTE * len(cars)
    idle_ratio = total_idle_frames / total_car_frames if total_car_frames > 0 else 0

    box_delivery_efficiency = (deliveries_count / len(items))*100 if len(items)>0 else 0

    total_dist_all_cars = sum(total_distance_cars)
    avg_distance_per_car = total_dist_all_cars / len(cars) if len(cars)>0 else 0

    if len(delivery_time_diffs) > 0:
        avg_box_lifecycle = sum(delivery_time_diffs)/len(delivery_time_diffs)/60
    else:
        avg_box_lifecycle = 0.0

    avg_busy_cars = sum_busy_cars / frame_count if frame_count>0 else 0

    final_results = {
        'Boxes Delivered per Minute': boxes_per_minute,
        'Task Distribution Efficiency': std_task,
        'Idle Time Ratio': idle_ratio,
        'Box Delivery Efficiency': box_delivery_efficiency,
        'Average Distance per Car': avg_distance_per_car,
        'Average Box Lifecycle': avg_box_lifecycle,
        'Average Busy Cars': avg_busy_cars
    }

    time_axis = np.arange(len(boxes_delivered_list)) / FPS

    return (time_axis, boxes_delivered_list, std_task_list, idle_ratio_list, box_delivery_eff_list, busy_cars_list, final_results)


def run_experiment_2():
    # 实验2（以前版本）的逻辑，但修改Box Lifecycle计算与实验1一致
    # 仅保留原先代码基础上修改的生命周期计算方法，不使用多线程
    ENV_SIZE = 800
    pygame.init()
    screen = pygame.display.set_mode((ENV_SIZE, ENV_SIZE))
    pygame.display.set_caption("Experiment 2")
    clock = pygame.time.Clock()

    FPS = 60
    FRAMES_PER_MINUTE = 60 * FPS
    frame_count = 0

    deliveries_count = 0
    car_task_counts = []
    car_idle_frames = []
    total_distance_cars = []
    sum_task_overlap = 0
    appear_time_dict = {}
    delivery_time_diffs = []
    sum_busy_cars = 0

    boxes_delivered_list = []
    std_task_list = []
    idle_ratio_list = []
    box_delivery_eff_list = []
    busy_cars_list = []

    class Car_Exp2(Car):
        def update(self, cars, polygon):
            dx_total = 0
            dy_total = 0

            if self.has_item and not self.delivering:
                self.delivering = True
                radius = 40
                angle = random.uniform(180, 270)
                rad = math.radians(angle)
                self.centroid = (ENV_SIZE + radius * math.cos(rad), ENV_SIZE + radius * math.sin(rad))
                print(f"小车在 ({self.x:.2f}, {self.y:.2f}) 正在送货。")

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

    drone = Drone(0,0,ENV_SIZE)
    cars = []
    for _ in range(12):
        x = random.uniform(0, ENV_SIZE)
        y = random.uniform(0, ENV_SIZE)
        car = Car_Exp2(x, y, ENV_SIZE)
        cars.append(car)
    car_idle_frames = [0]*len(cars)
    total_distance_cars = [0]*len(cars)

    items = [Item(ENV_SIZE) for _ in range(8)]
    items_detected = [False]*len(items)

    running = True
    while running:
        frame_count += 1
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

        boundary = Polygon([
            (0, 0),
            (0, ENV_SIZE),
            (ENV_SIZE, ENV_SIZE),
            (ENV_SIZE, 0)
        ])

        car_polygons = {car: None for car in cars}

        if vor is not None:
            for idx, region in enumerate(regions):
                car = active_cars[idx]
                polygon = vertices[region]
                poly = Polygon(polygon)
                poly = poly.intersection(boundary)
                if not poly.is_empty and poly.is_valid and poly.geom_type == 'Polygon':
                    car_polygons[car] = poly
                    if not car.target_item and not car.delivering:
                        car.centroid = poly.centroid.coords[0]

        for item in items:
            if item not in appear_time_dict:
                appear_time_dict[item] = frame_count

        for idx, item in enumerate(items):
            if not items_detected[idx] and not item.picked:
                dx = item.x - drone.x
                dy = item.y - drone.y
                distance = math.hypot(dx, dy)
                if distance <= drone.sensor_range:
                    items_detected[idx] = True
                    print(f"无人机检测到物品 {idx + 1}。")
                    item_point = Point(item.x, item.y)
                    for car, poly in car_polygons.items():
                        if poly and poly.contains(item_point):
                            car.set_target(item)
                            print(f"小车在 ({car.x:.2f}, {car.y:.2f}) 被指派去拾取物品 {idx + 1}。")
                            break

        for car in cars:
            car.dx_total = 0
            car.dy_total = 0

        for car in cars:
            polygon = car_polygons[car]
            old_x, old_y = car.x, car.y
            car.update(cars, polygon)

        target_map = {}
        for car in cars:
            if car.target_item and not car.has_item:
                target_map[car.target_item] = target_map.get(car.target_item, 0) + 1
        frame_overlap = 0
        for count in target_map.values():
            if count > 1:
                frame_overlap += (count - 1)
        sum_task_overlap += frame_overlap

        deliveries = False
        busy_cars_count = 0

        for i, car in enumerate(cars):
            total_speed = math.hypot(car.dx_total, car.dy_total)
            if total_speed > car.speed:
                scale = car.speed / total_speed
                car.dx_total *= scale
                car.dy_total *= scale

            old_x, old_y = car.x, car.y
            car.x += car.dx_total
            car.y += car.dy_total

            distance_moved = math.hypot(car.x - car.last_x, car.y - car.last_y)
            total_distance_cars[i] += distance_moved
            car.last_x, car.last_y = car.x, car.y

            if car.delivering:
                dx = car.centroid[0] - car.x
                dy = car.centroid[1] - car.y
                distance_to_target = math.hypot(dx, dy)
                if distance_to_target < car.speed:
                    # 完成送达计算生命周期
                    if car.current_item is not None:
                        delivery_time_diffs.append(frame_count - appear_time_dict[car.current_item])
                        car.current_item = None

                    car.has_item = False
                    car.delivering = False
                    car.item_color = None
                    car.just_delivered = True
                    car.assigned_task = False
                    print(f"小车在 ({car.x:.2f}, {car.y:.2f}) 已送达物品。")

            if car.target_item and not car.has_item:
                dx_item = car.target_item.x - car.x
                dy_item = car.target_item.y - car.y
                distance = math.hypot(dx_item, dy_item)
                if distance < car.size + car.target_item.size:
                    car.has_item = True
                    car.item_color = car.target_item.color
                    car.target_item.picked = True
                    car.current_item = car.target_item
                    car.target_item = None
                    print(f"小车在 ({car.x:.2f}, {car.y:.2f}) 已拾取物品。")

            if not car.assigned_task and not car.delivering and not car.target_item:
                if not car.has_item:
                    car_idle_frames[i] += 1

            if car.just_delivered:
                deliveries = True
                car.just_delivered = False
                car_task_counts.append(i)
                deliveries_count += 1

            if car.assigned_task or car.delivering:
                busy_cars_count += 1

        sum_busy_cars += busy_cars_count

        if deliveries:
            new_item = Item(ENV_SIZE)
            items.append(new_item)
            items_detected.append(False)
            appear_time_dict[new_item] = frame_count
            print("生成了一个新的物品。")

        boxes_delivered_list.append(deliveries_count)
        task_count_array = [0]*len(cars)
        for car_index in car_task_counts:
            task_count_array[car_index] += 1
        if len(task_count_array) > 1:
            current_std_task = np.std(task_count_array)
        else:
            current_std_task = 0.0
        std_task_list.append(current_std_task)

        total_idle_frames = sum(car_idle_frames)
        total_car_frames = frame_count * len(cars)
        if total_car_frames > 0:
            current_idle_ratio = total_idle_frames / total_car_frames
        else:
            current_idle_ratio = 0
        idle_ratio_list.append(current_idle_ratio)

        if len(items) > 0:
            current_box_delivery_eff = (deliveries_count / len(items)) * 100
        else:
            current_box_delivery_eff = 0
        box_delivery_eff_list.append(current_box_delivery_eff)

        busy_cars_list.append(busy_cars_count)

        screen.fill((255, 255, 255))
        draw_quarter_circle(screen, (ENV_SIZE, ENV_SIZE), 40, (255, 182, 193))

        if vor is not None:
            for poly in car_polygons.values():
                if poly and poly.exterior:
                    x, y = poly.exterior.xy
                    coords = [(int(xx), int(yy)) for xx, yy in zip(x, y)]
                    pygame.draw.polygon(screen, (0, 255, 0), coords, 1)

        drone.draw(screen)
        for car in cars:
            car.draw(screen)

        for item in items:
            item.draw(screen)

        pygame.display.flip()
        clock.tick(FPS)

        if frame_count >= FRAMES_PER_MINUTE:
            running = False

    pygame.quit()

    boxes_per_minute = deliveries_count
    task_count_array = [0]*len(cars)
    for car_index in car_task_counts:
        task_count_array[car_index] += 1
    if len(task_count_array) > 1:
        std_task = np.std(task_count_array)
    else:
        std_task = 0.0

    total_idle_frames = sum(car_idle_frames)
    total_car_frames = FRAMES_PER_MINUTE * len(cars)
    idle_ratio = total_idle_frames / total_car_frames if total_car_frames > 0 else 0

    box_delivery_efficiency = (deliveries_count / len(items))*100 if len(items)>0 else 0

    total_dist_all_cars = sum(total_distance_cars)
    avg_distance_per_car = total_dist_all_cars / len(cars) if len(cars)>0 else 0

    if len(delivery_time_diffs) > 0:
        avg_box_lifecycle = sum(delivery_time_diffs)/len(delivery_time_diffs)/60
    else:
        avg_box_lifecycle = 0.0

    avg_busy_cars = sum_busy_cars / frame_count if frame_count>0 else 0

    final_results = {
        'Boxes Delivered per Minute': boxes_per_minute,
        'Task Distribution Efficiency': std_task,
        'Idle Time Ratio': idle_ratio,
        'Box Delivery Efficiency': box_delivery_efficiency,
        'Average Distance per Car': avg_distance_per_car,
        'Average Box Lifecycle': avg_box_lifecycle,
        'Average Busy Cars': avg_busy_cars
    }

    time_axis = np.arange(len(boxes_delivered_list)) / FPS

    return (time_axis, boxes_delivered_list, std_task_list, idle_ratio_list, box_delivery_eff_list, busy_cars_list, final_results)


# 先运行实验1
(time_axis_1, boxes_delivered_1, std_task_1, idle_ratio_1, 
 box_delivery_eff_1, busy_cars_1, results_1) = run_experiment_1()

# 再运行实验2
(time_axis_2, boxes_delivered_2, std_task_2, idle_ratio_2, 
 box_delivery_eff_2, busy_cars_2, results_2) = run_experiment_2()

# 打印结果
print("========== Experiment 1 Final Metrics ==========")
for k,v in results_1.items():
    print(f"{k}: {v}")
print("===========================================")

print("========== Experiment 2 Final Metrics ==========")
for k,v in results_2.items():
    print(f"{k}: {v}")
print("===========================================")

# 绘制对比图(将两个实验的数据放在同一张图里对比)
plt.figure(figsize=(12, 10))

# Boxes Delivered Over Time
plt.subplot(3, 2, 1)
plt.plot(time_axis_1, boxes_delivered_1, label='Exp1', color='blue')
plt.plot(time_axis_2, boxes_delivered_2, label='Exp2', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Cumulative Boxes Delivered')
plt.title('Boxes Delivered Over Time')
plt.grid(True)
plt.legend()

# Task Distribution Efficiency Over Time
plt.subplot(3, 2, 2)
plt.plot(time_axis_1, std_task_1, label='Exp1', color='blue')
plt.plot(time_axis_2, std_task_2, label='Exp2', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Std of Task Counts')
plt.title('Task Distribution Efficiency Over Time')
plt.grid(True)
plt.legend()

# Idle Time Ratio Over Time
plt.subplot(3, 2, 3)
plt.plot(time_axis_1, idle_ratio_1, label='Exp1', color='blue')
plt.plot(time_axis_2, idle_ratio_2, label='Exp2', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Idle Time Ratio')
plt.title('Idle Time Ratio Over Time')
plt.grid(True)
plt.legend()

# Box Delivery Efficiency Over Time
plt.subplot(3, 2, 4)
plt.plot(time_axis_1, box_delivery_eff_1, label='Exp1', color='blue')
plt.plot(time_axis_2, box_delivery_eff_2, label='Exp2', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Box Delivery Efficiency (%)')
plt.title('Box Delivery Efficiency Over Time')
plt.grid(True)
plt.legend()

# Busy Cars Over Time
plt.subplot(3, 2, 5)
plt.plot(time_axis_1, busy_cars_1, label='Exp1', color='blue')
plt.plot(time_axis_2, busy_cars_2, label='Exp2', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Number of Busy Cars')
plt.title('Busy Cars Over Time')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
