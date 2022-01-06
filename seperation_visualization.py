import sys
import pygame as pg
import numpy as np
from pygame.locals import *

def get_avoid_vector(position, targets, radius):
    vectors = position - targets
    distances = np.reshape(np.linalg.norm(vectors, axis=1), (len(vectors), 1))
    norm_vectors = vectors / distances
    weighted_vectors = norm_vectors * (radius - distances)
    return np.sum(weighted_vectors, 0) / len(vectors)

def get_cohesion_force(current_pos, target_pos):
        center_position = np.sum(target_pos, 0) / len(target_pos)
        force = center_position - current_pos
        return force / np.linalg.norm(force)

def main():
    pg.init()
    display = pg.display.set_mode((600, 600))
    clock = pg.time.Clock()

    avoid_targets = np.array([[]])
    current_targets_mask = np.array([], dtype=bool)
    target_radius = 100

    while True:
        display.fill((42, 42, 42))
        cursor_position = np.array(pg.mouse.get_pos())

        pg.draw.circle(display, (50, 50, 50), cursor_position, target_radius)

        for event in pg.event.get():
            if event.type == QUIT:
                pg.quit()
                sys.exit()

            elif event.type == MOUSEBUTTONDOWN and event.button == 1:
                avoid_targets = np.append(avoid_targets, np.array([cursor_position]), int(avoid_targets.size == 0))
                current_targets_mask = np.append(current_targets_mask, True)

        if avoid_targets.size > 0:
            for i in range(len(avoid_targets)):
                current_targets_mask[i] = np.linalg.norm(avoid_targets[i] - cursor_position) < target_radius
                pg.draw.circle(display, (255, 255, 255), avoid_targets[i], 3)

        current_targets = avoid_targets[current_targets_mask]

        if current_targets.size > 0:

            for i in range(len(current_targets)):
                pg.draw.line(display, (255, 0, 0), current_targets[i], cursor_position, 2)

            avoid_vector = get_cohesion_force(cursor_position, current_targets) * 100
            pg.draw.line(display, (162, 255, 43), cursor_position, cursor_position + avoid_vector, 2)

        pg.display.update()
        clock.tick(60)

if __name__ == '__main__':
    main()
