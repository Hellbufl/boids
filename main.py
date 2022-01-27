from doctest import master
import numpy as np
import pygame as pg
import sys, ctypes

DISPLAY_SIZE_DEFAULT = (1600, 900)

def get_display_size():

    if sys.platform == "win32":
        user32 = ctypes.windll.user32
        return (user32.GetSystemMetrics(0), user32.GetSystemMetrics(1))
    else:
        return DISPLAY_SIZE_DEFAULT

def rotate_vector(vector, theta):
    return np.dot(vector, np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]))

def vector_angle(v1, v2):
    alpha = np.arccos(v1[0] / np.linalg.norm(v1)) * (v1[1] / abs(v1[1]))
    beta = np.arccos(v2[0] / np.linalg.norm(v2)) * (v2[1] / abs(v2[1]))

    return beta - alpha

class Boids:

    def __init__(self, n, r):
        pg.init()
        self.resolution = get_display_size() #(800, 600)
        self.display = pg.display.set_mode(self.resolution, pg.HWSURFACE | pg.FULLSCREEN)
        self.clock = pg.time.Clock()

        self.n = n
        self.radius = r
        self.boid_positions, self.boid_velocities = self.generate_agents(0)
        self.pred_positions, self.pred_velocities = self.generate_agents(1)
        self.obstacle_positions = np.random.rand(self.n[2], 2) * self.resolution

    def generate_agents(self, typ):
        # Generiert zufällige Positionen und Geschwindigkeiten
        # Gibt Tupel (pos, vel) zurück
        # typ 0: boids / typ 1: preds

        positions = np.random.rand(self.n[typ], 2) * self.resolution
        velocities = np.ones([self.n[typ], 2]) * [1, 0]
        angles = np.random.rand(self.n[typ]) * np.pi * 2

        for i in range(self.n[typ]):
            velocities[i] = rotate_vector(velocities[i], angles[i])

        return (positions, velocities * 2)

    def get_target_mask(self, current_pos, current_vel):
        # Bestimmt welche Boids vom aktuellen sichtbar sind
        # Gibt Array aus Bools zurück

        boid_mask = np.zeros([self.n[0]], dtype=bool)
        pred_mask = np.zeros([self.n[1]], dtype=bool)
        obstacle_mask = np.zeros([self.n[2]], dtype=bool)

        for i in range(self.n[0]):
            if (self.boid_positions[i] != current_pos).all() and np.dot(current_vel, self.boid_positions[i] - current_pos) > 0:
                if current_pos in self.pred_positions:
                    boid_mask[i] = np.linalg.norm(self.boid_positions[i] - current_pos) < self.radius * 3
                else:
                    boid_mask[i] = np.linalg.norm(self.boid_positions[i] - current_pos) < self.radius

        for i in range(self.n[1]):
            if (self.pred_positions[i] != current_pos).all():# and np.dot(current_vel, self.pred_positions[i] - current_pos) > 0:
                pred_mask[i] = np.linalg.norm(self.pred_positions[i] - current_pos) < self.radius

        for i in range(self.n[2]):
            if (self.obstacle_positions[i] != current_pos).all() and np.dot(current_vel, self.obstacle_positions[i] - current_pos) > 0:
                obstacle_mask[i] = np.linalg.norm(self.obstacle_positions[i] - current_pos) < self.radius

        return (boid_mask, pred_mask, obstacle_mask)

    def get_separation_force(self, current_pos, target_pos):
        vectors = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors, axis=1), (len(vectors), 1))
        norm_vectors = vectors / distances
        weighted_vectors = norm_vectors * (self.radius - distances)
        force = np.sum(weighted_vectors, 0) / len(vectors)
        return force

    def get_alignment_force(self, current_pos, target_pos, target_vel):
        vectors_pos = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors_pos, axis=1), (len(vectors_pos), 1))

        length_vectors_vel = np.reshape(np.linalg.norm(target_vel, axis=1), (len(target_vel), 1))
        norm_vectors_vel = target_vel / length_vectors_vel
        weighted_vectors = norm_vectors_vel * (self.radius - distances)
        force = np.sum(weighted_vectors, 0) / len(weighted_vectors)
        return force

    def get_cohesion_force(self, current_pos, target_pos):
        center_position = np.sum(target_pos, 0) / len(target_pos)
        force = center_position - current_pos
        return force

    def update_velocity(self, current_pos, current_vel):
        # Ändert Geschwindigkeit anhand der wirkenden Kraft
        # Gibt neue Geschwindigkeit zurück

        # current_pos, current_vel = self.boid_positions[i], self.boid_velocities[i]

        boid_target_mask, pred_target_mask, obstacle_target_mask = self.get_target_mask(current_pos, current_vel)

        boid_target_pos, boid_target_vel = self.boid_positions[boid_target_mask], self.boid_velocities[boid_target_mask]
        pred_target_pos, pred_target_vel = self.pred_positions[pred_target_mask], self.pred_velocities[pred_target_mask]
        obstacle_target_pos = self.obstacle_positions[obstacle_target_mask]

        force = np.zeros([3, 2])

        if boid_target_pos.size > 0:
            force[0] = self.get_cohesion_force(current_pos, boid_target_pos)

            if current_pos not in self.pred_positions:
                force[1] = self.get_alignment_force(current_pos, boid_target_pos, boid_target_vel)

        if (boid_target_pos.size > 0 and current_pos not in self.pred_positions) or pred_target_pos.size > 0 or obstacle_target_pos.size > 0:
            if current_pos in self.pred_positions:
                target_pos = np.append(pred_target_pos, obstacle_target_pos, 0)

            else:
                target_pos = np.append(boid_target_pos, obstacle_target_pos, 0)
                target_pos = np.append(target_pos, pred_target_pos, 0)
                if pred_target_pos.size > 0:
                    force[2] = self.get_separation_force(current_pos, pred_target_pos) * 10

            force[2] += self.get_separation_force(current_pos, target_pos)

        sum_force = np.sum(force, 0)

        if (sum_force == 0).all():
            return current_vel

        theta = vector_angle(current_vel, sum_force)

        return rotate_vector(current_vel, theta * 0.05)

    def update(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()

        new_boid_velocities = np.zeros([self.n[0], 2])
        new_pred_velocities = np.zeros([self.n[1], 2])

        for i in range(self.n[0]):
            new_boid_velocities[i] = self.update_velocity(self.boid_positions[i], self.boid_velocities[i])

        for i in range(self.n[1]):
            new_pred_velocities[i] = self.update_velocity(self.pred_positions[i], self.pred_velocities[i])

        self.boid_velocities = new_boid_velocities
        self.pred_velocities = new_pred_velocities

        self.boid_positions += self.boid_velocities
        self.pred_positions += self.pred_velocities

        self.boid_positions %= self.resolution
        self.pred_positions %= self.resolution

    def draw(self):
        size = 20
        self.display.fill((42, 42, 42))

        agent_positions = np.append(self.boid_positions, self.pred_positions, 0)
        agent_velocities = np.append(self.boid_velocities, self.pred_velocities, 0)

        for i in range(self.n[0] + self.n[1]):
            position = agent_positions[i]
            velocity = agent_velocities[i]
            direction = velocity / np.linalg.norm(velocity)
 
            point_a = position + direction * size
            point_b = position + np.array([-direction[1], direction[0]]) * size / 4
            point_c = position - np.array([-direction[1], direction[0]]) * size / 4

            color = (255, 255, 255)
            if i >= self.n[0]:
                color = (255, 0, 0)

            pg.draw.polygon(self.display, color, (point_a, point_b, point_c))
        
        for i in range(self.n[2]):
            pg.draw.circle(self.display, (130, 130, 255), self.obstacle_positions[i], 5)

    def mainloop(self):
        while True:
            self.update()
            self.draw()
            pg.display.update()
            self.clock.tick(60)

if __name__ == '__main__':
    B = Boids([50, 5, 3], 100)
    B.mainloop()