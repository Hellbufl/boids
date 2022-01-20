from doctest import master
import numpy as np
import pygame as pg
import sys

def rotate_vector(vector, theta):
    return np.dot(vector, np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]))

def vector_angle(v1, v2):
    alpha = np.arccos(v1[0] / np.linalg.norm(v1)) * (v1[1] / abs(v1[1]))
    beta = np.arccos(v2[0] / np.linalg.norm(v2)) * (v2[1] / abs(v2[1]))

    return beta - alpha

class Obstacle:

    def __init__(self, n,):
        pg.init()
        self.resolution = (800, 600)
        self.display = pg.display.set_mode(self.resolution)
        self.clock = pg.time.Clock()

        self.n = n
        self.obstacle_positions, self.obstacle_velocities = self.generate_obstacle()

    def generate_obstacle(self):

        opositions = np.random.rand(self.n, 2) * self.resolution
        ovelocities = np.ones([self.n, 2]) * [1, 0]
        oangles = np.random.rand(self.n) * np.pi * 2

        for i in range(self.n):
            ovelocities[i] = rotate_vector(ovelocities[i], oangles[i])

        return (opositions, ovelocities * 2)

class Boids:

    def __init__(self, n, r):
        pg.init()
        self.resolution = (800, 600)
        self.display = pg.display.set_mode(self.resolution)
        self.clock = pg.time.Clock()

        self.n = n
        self.n_obstacle = 1
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

    def get_target_mask(self, i):
        # Bestimmt welche Agenten vom aktuellen sichtbar sind
        # Gibt Array aus Bools zurück
        current_pos, current_vel = self.agent_positions[i], self.agent_velocities[i]
        mask = np.zeros([self.n], dtype=bool)

        for i in range(len(self.agent_positions)):
            if (self.agent_positions[i] != current_pos).all() and np.dot(current_vel, self.agent_positions[i] - current_pos) > 0:
                mask[i] = np.linalg.norm(self.agent_positions[i] - current_pos) < self.radius

        return mask

    def get_oseparation_force(self, current_pos):
        force = [0,0]
        for k in range(len(O.obstacle_positions)):
            #bei einer Methode kann man die Maske genauso gut in der Methode erstellen
            if np.linalg.norm(current_pos - O.obstacle_positions[k]) < self.radius:
                vectors = current_pos - O.obstacle_positions[k]
                distances = ((vectors[1]**2)+vectors[0]**2)**(1/2)
                norm_vectors = vectors / distances
                weighted_vectors = norm_vectors * (self.radius - distances)
                force = weighted_vectors / len(vectors)
        if np.linalg.norm(force) != 0:
            return (force / np.linalg.norm(force))
        else:
            return force

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

    def update_velocity(self, i):
        # Ändert Geschwindigkeit anhand der wirkenden Kraft
        # Gibt neue Geschwindigkeit zurück
        current_pos, current_vel = self.agent_positions[i], self.agent_velocities[i]
        target_mask = self.get_target_mask(i)
        target_pos, target_vel = self.agent_positions[target_mask], self.agent_velocities[target_mask]
        oseparation_force = self.get_oseparation_force(current_pos)
        if target_pos.size != 0 :
            cohesion_force = self.get_cohesion_force(current_pos, target_pos)
            alignment_force = self.get_alignment_force(current_pos, target_pos, target_vel)

            separation_force = self.get_separation_force(current_pos, target_pos)

            force =separation_force + cohesion_force+ alignment_force + oseparation_force
            theta = vector_angle(current_vel, force)

            return rotate_vector(current_vel, theta * 0.05)
        elif np.linalg.norm(oseparation_force) != 0:
            theta = vector_angle(current_vel, oseparation_force)

            return rotate_vector(current_vel, theta * 0.05)
        return current_vel

    def update(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()

        new_velocities = np.zeros([self.n, 2])

        for i in range(self.n):
            new_velocities[i] = self.update_velocity(i)

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

        for i in range(O.n):
            pg.draw.circle(O.display, (130, 130, 255), O.obstacle_positions[i], 5)

    def mainloop(self):
        while True:
            self.update()
            self.draw()
            pg.display.update()
            self.clock.tick(60)

if __name__ == '__main__':
    B = Boids(20, 50)
    O = Obstacle(20,)
    B.mainloop()
