import random
import pygame
import math
import numpy as np

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

def draw_quarter_circle(screen, center, radius, color, ENV_SIZE):
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