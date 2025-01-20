import pygame
import random
import math
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Point, Polygon
import matplotlib.pyplot as plt
from robots import *
from utils import *

def run_experiment(voronoi=True):
    ENV_SIZE = 800
    pygame.init()
    screen = pygame.display.set_mode((ENV_SIZE, ENV_SIZE))
    if voronoi:
        pygame.display.set_caption("Experiment Voronoi")
    else:
        pygame.display.set_caption("Experiment Nml")
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

    drone = Drone(0, 0, ENV_SIZE)
    cars = []
    for _ in range(12):
        x = random.uniform(0, ENV_SIZE)
        y = random.uniform(0, ENV_SIZE)
        car = Car(x, y, ENV_SIZE)
        cars.append(car)
    car_idle_frames = [0] * len(cars)
    total_distance_cars = [0] * len(cars)

    items = [Item(ENV_SIZE) for _ in range(8)]
    items_detected = [False] * len(items)

    running = True
    while running:
        frame_count += 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        if voronoi:
            drone.update()
            active_cars = [car for car in cars if not car.assigned_task]
            if len(active_cars) > 0:
                points = np.array([[car.x, car.y] for car in active_cars])
                vor = Voronoi(points)
                regions, vertices = voronoi_finite_polygons_2d(vor)
            else:
                vor = None
                regions, vertices = [], []
        else:
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
        
        if voronoi:
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

        else:
            for idx, region in enumerate(regions):
                polygon = vertices[region]
                poly = Polygon(polygon)
                poly = poly.intersection(boundary)
                if not poly.is_empty and poly.is_valid and poly.geom_type == 'Polygon':
                    car_polygons[cars[idx]] = poly
                    if (not cars[idx].assigned_task and not cars[idx].delivering
                            and not cars[idx].has_item and not cars[idx].target_item):
                        cars[idx].centroid = poly.centroid.coords[0]

        for item in items:
            if item not in appear_time_dict:
                appear_time_dict[item] = frame_count

        # Attempt item assignment
        for idx, item in enumerate(items):
            
            if voronoi:
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
            else:
                if not item.picked:
                    dx = item.x - drone.x
                    dy = item.y - drone.y
                    distance = math.hypot(dx, dy)
                    if distance <= drone.sensor_range:
                        if not items_detected[idx]:
                            print(f"Drone has detected item {idx + 1}.")
                        item_point = Point(item.x, item.y)
                        assigned = False
                        for car, poly in car_polygons.items():
                            if (poly and poly.contains(item_point)
                                    and not car.has_item and not car.delivering
                                    and not car.assigned_task):
                                car.set_target(item)
                                print(f"Car at ({car.x:.2f}, {car.y:.2f}) has been assigned to pick up item {idx + 1}.")
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

            if voronoi:
                car.update(cars, ENV_SIZE, polygon)
            else:
                car.update_nml(cars, ENV_SIZE, polygon)

        # Calculate overlap
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

            # Delivery check
            if car.delivering:
                dx = car.centroid[0] - car.x
                dy = car.centroid[1] - car.y
                distance_to_target = math.hypot(dx, dy)
                if distance_to_target < car.speed:
                    # Compute lifecycle
                    if car.current_item is not None:
                        delivery_time_diffs.append(frame_count - appear_time_dict[car.current_item])
                        car.current_item = None

                    car.has_item = False
                    car.delivering = False
                    car.item_color = None
                    car.just_delivered = True
                    car.assigned_task = False
                    print(f"Car at({car.x:.2f}, {car.y:.2f}) has delivered the item.")

            # Pickup check
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
                    print(f"Car at ({car.x:.2f}, {car.y:.2f}) has picked up the item.")

            # Idle check
            if (not car.assigned_task and not car.delivering
                    and not car.target_item and not car.has_item):
                car_idle_frames[i] += 1

            if car.just_delivered:
                deliveries = True
                car.just_delivered = False
                car_task_counts.append(i)
                deliveries_count += 1

            if car.assigned_task or car.delivering:
                busy_cars_count += 1

        sum_busy_cars += busy_cars_count

        # New item after delivery
        if deliveries:
            new_item = Item(ENV_SIZE)
            items.append(new_item)
            items_detected.append(False)
            appear_time_dict[new_item] = frame_count
            print("A new item has appeared.")

        # Record data each frame
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
        current_idle_ratio = (total_idle_frames / total_car_frames) if total_car_frames > 0 else 0
        idle_ratio_list.append(current_idle_ratio)

        if len(items) > 0:
            current_box_delivery_eff = (deliveries_count / len(items)) * 100
        else:
            current_box_delivery_eff = 0
        box_delivery_eff_list.append(current_box_delivery_eff)

        busy_cars_list.append(busy_cars_count)

        screen.fill((255, 255, 255))
        draw_quarter_circle(screen, (ENV_SIZE, ENV_SIZE), 40, (255, 182, 193), ENV_SIZE)

        if voronoi:
            if vor is not None:
                for poly in car_polygons.values():
                    if poly and poly.exterior:
                        x, y = poly.exterior.xy
                        coords = [(int(xx), int(yy)) for xx, yy in zip(x, y)]
                        pygame.draw.polygon(screen, (0, 255, 0), coords, 1)
        else:
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

    # Final metrics
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

    # Create final dictionary
    results = {
        'Boxes Delivered per Minute': boxes_per_minute,
        'Task Distribution Efficiency': std_task,
        'Idle Time Ratio': idle_ratio,
        'Box Delivery Efficiency (%)': box_delivery_efficiency,
        'Average Distance per Car': avg_distance_per_car,
        'Average Box Lifecycle (minutes)': avg_box_lifecycle,
        'Average Busy Cars': avg_busy_cars
    }

    time_axis = np.arange(len(boxes_delivered_list)) / FPS

    return (time_axis, boxes_delivered_list, std_task_list, idle_ratio_list,
            box_delivery_eff_list, busy_cars_list, results)

# Run Experiment nml
(time_axis_nml, boxes_delivered_nml, std_task_nml, idle_ratio_nml,
 box_delivery_eff_nml, busy_cars_nml, results_nml) = run_experiment(voronoi=False)

# Run Experiment voronoi
(time_axis_vor, boxes_delivered_vor, std_task_vor, idle_ratio_vor,
 box_delivery_eff_vor, busy_cars_vor, results_vor) = run_experiment(voronoi=True)

print("========== Experiment nml Final Results ==========")
for k, v in results_nml.items():
    print(f"{k}: {v}")
print("====================================")

print("========== Experiment vor Final Results ==========")
for k, v in results_vor.items():
    print(f"{k}: {v}")
print("====================================")

# Plot comparisons (titles/labels remain in English)
plt.figure(figsize=(12, 10))

plt.subplot(3, 2, 1)
plt.plot(time_axis_nml, boxes_delivered_nml, label='Exp_nml', color='blue')
plt.plot(time_axis_vor, boxes_delivered_vor, label='Exp_vor', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Cumulative Boxes Delivered')
plt.title('Boxes Delivered Over Time')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 2)
plt.plot(time_axis_nml, std_task_nml, label='Exp_nml', color='blue')
plt.plot(time_axis_vor, std_task_vor, label='Exp_vor', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Std of Task Counts')
plt.title('Task Distribution Efficiency Over Time')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 3)
plt.plot(time_axis_nml, idle_ratio_nml, label='Exp_nml', color='blue')
plt.plot(time_axis_vor, idle_ratio_vor, label='Exp_vor', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Idle Time Ratio')
plt.title('Idle Time Ratio Over Time')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 4)
plt.plot(time_axis_nml, box_delivery_eff_nml, label='Exp_nml', color='blue')
plt.plot(time_axis_vor, box_delivery_eff_vor, label='Exp_vor', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Box Delivery Efficiency (%)')
plt.title('Box Delivery Efficiency Over Time')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 5)
plt.plot(time_axis_nml, busy_cars_nml, label='Exp_nml', color='blue')
plt.plot(time_axis_vor, busy_cars_vor, label='Exp_vor', color='red', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Number of Busy Cars')
plt.title('Busy Cars Over Time')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()