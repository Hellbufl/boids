import numpy as np
import pygame as pg
import sys

def rotate_vector(vector, theta):
    return np.dot(vector, np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]))

def vector_angle(v1, v2):
    alpha = np.arccos(v1[0] / np.linalg.norm(v1))
    beta = np.arccos(v2[0] / np.linalg.norm(v2))
    
    if v1[1] < 0:
        alpha = -alpha
    if v2[1] < 0:
        beta = -beta
    
    return beta - alpha

class Boids:

    def __init__(self, n, r):
        pg.init()
        self.resolution = (800, 600)
        self.display = pg.display.set_mode(self.resolution)
        self.clock = pg.time.Clock()

        self.n = n
        self.radius = r
        self.agent_positions, self.agent_velocities = self.generate_agents()

    def generate_agents(self):
        # Generiert zufällige Positionen und Geschwindigkeiten
        # Gibt Tupel (pos, vel) zurück

        positions = np.random.rand(self.n, 2) * self.resolution
        velocities = np.ones([self.n, 2]) * [1, 0]
        angles = np.random.rand(self.n) * np.pi * 2

        for i in range(self.n):
            velocities[i] = rotate_vector(velocities[i], angles[i])

        return (positions, velocities * 2)

    def get_targets(self, current_pos, current_vel):
        # Bestimmt welche Agenten vom aktuellen sichtbar sind
        # Gibt Array aus Bools zurück
        mask = np.zeros([self.n], dtype=bool)

        for i in range(len(self.agent_positions)):
            if (self.agent_positions[i] != current_pos).all() and np.dot(current_vel, self.agent_positions[i] - current_pos) > 0:
                mask[i] = np.linalg.norm(self.agent_positions[i] - current_pos) < self.radius

        target_pos = self.agent_positions[mask]
        target_vel = self.agent_velocities[mask]
        return (target_pos, target_vel)

    def get_separation_force(self, current_pos, target_pos):
        vectors = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors, axis=1), (len(vectors), 1))
        norm_vectors = vectors / distances
        weighted_vectors = norm_vectors * (self.radius - distances)
        force = np.sum(weighted_vectors, 0) / len(vectors)
        return force / np.linalg.norm(force)

    def get_alignment_force(self, current_pos, target_pos, target_vel):
        vectors_pos = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors_pos, axis=1), (len(vectors_pos), 1))

        length_vectors_vel = np.reshape(np.linalg.norm(target_vel, axis=1), (len(target_vel), 1))
        norm_vectors_vel = target_vel / length_vectors_vel
        weighted_vectors = norm_vectors_vel * (self.radius - distances)
        force = np.sum(weighted_vectors, 0) / len(weighted_vectors)
        return force / np.linalg.norm(force)

    def get_cohesion_force(self, current_pos, target_pos):
        center_position = np.sum(target_pos, 0) / len(target_pos)
        force = center_position - current_pos
        return force / np.linalg.norm(force)

    def update_velocity(self, current_vel, current_pos):
        # Ändert Geschwindigkeit anhand der wirkenden Kraft
        # Gibt neue Geschwindigkeit zurück
        target_pos, target_vel = self.get_targets(current_pos, current_vel)

        if target_pos.size != 0:
            cohesion_force = self.get_cohesion_force(current_pos, target_pos)
            alignment_force = self.get_alignment_force(current_pos, target_pos, target_vel)
            separation_force = self.get_separation_force(current_pos, target_pos)
            force = cohesion_force + alignment_force + separation_force

            theta = vector_angle(current_vel, force)

            return rotate_vector(current_vel, theta * 0.05)
        
        return current_vel

    def update(self):

        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()

        new_velocities = np.zeros([self.n, 2])

        for i in range(self.n):
            new_velocities[i] = self.update_velocity(self.agent_velocities[i], self.agent_positions[i])

        self.agent_velocities = new_velocities

        self.agent_positions += self.agent_velocities
        self.agent_positions %= self.resolution

    def draw(self):
        size = 20
        self.display.fill((42, 42, 42))

        for i in range(self.n):
            position = self.agent_positions[i]
            velocity = self.agent_velocities[i]
            direction = velocity / np.linalg.norm(velocity)
 
            point_a = position + direction * size
            point_b = position + np.array([-direction[1], direction[0]]) * size / 4
            point_c = position - np.array([-direction[1], direction[0]]) * size / 4

            pg.draw.polygon(self.display, (255, 255, 255), (point_a, point_b, point_c))

    def mainloop(self):
        while True:
            self.update()
            self.draw()
            pg.display.update()
            self.clock.tick(60)

if __name__ == '__main__':
    B = Boids(30, 50)
    B.mainloop()