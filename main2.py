import numpy as np
import pygame as pg

class Boids:

    def __init__(self, n, r):
        self.n = n
        self.radius = r
        self.agent_positions, self.agent_velocities = self.generate_agents()

    def generate_agents(self):
        # Generiert zufällige Positionen und Geschwindigkeiten
        # Gibt Tupel (pos, vel) zurück
        pass

    def get_target_mask(self, current_pos):
        l = []
        for i in range(len(self.agent_positions)-1):
            if np.abs(self.agent_positions[i]-current_pos)
        # Bestimmt welche Agenten vom aktuellen sichtbar sind
        # Gibt Array aus Bools zurück
        pass

    def get_seperation_force(self, current_pos, target_pos):
        vectors = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors, axis=1), (len(vectors), 1))
        norm_vectors = vectors / distances
        weighted_vectors = norm_vectors * (self.radius - distances)
        return np.sum(weighted_vectors, 0) / len(vectors)

    def get_alignment_force(self, current_pos, target_pos, target_vel):

        vectors_pos = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors_pos, axis=1), (len(vectors_pos), 1))

        length_vectors_vel = np.reshape(np.linalg.norm(target_vel, axis=1), (len(target_vel), 1))
        norm_vectors_vel = target_vel / length_vectors_vel
        weighted_vectors = norm_vector_vel * (self.radius - distances)
        return np.sum(target_vel, 0)/len(target_vel)


    def get_cohesion_force(self, current_pos, target_pos):
        center_position = np.sum(target_pos, 0)/(len(target_pos)+1)
        return current_pos - center_position

    def update_velocity(self, current_vel, force):
        # Ändert Geschwindigkeit anhand der wirkenden Kraft
        # Gibt neue Geschwindigkeit zurück
        pass

    def update(self):
        # Loop über alle Agenten
        # Bestimmt neuen Status der Simulation
        # Gibt neue Positionen und Geschwindigkeiten zurück
        # WICHTIG: neuen Status in neues Array schreiben. Nicht die Arrays verändern, die noch zur Berechung gebraucht werden!
        pass

    def draw(self):
        # Pygame shit
        pass

    def mainloop(self):
        pass
