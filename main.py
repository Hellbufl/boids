import numpy as np
import pygame as pg

class Boids:

    def __init__(self, n, r):
        self.n = n
        self.radius = r
        self.agent_positions, self.agent_velocities = self.generate_agents(n)

    def generate_agents(self, n):
        # Generiert zufällige Positionen und Geschwindigkeiten
        # Gibt Tupel (pos, vel) zurück
        pass
    
    def get_target_mask(self, current):
        # Bestimmt welche Agenten vom aktuellen sichtbar sind
        # Gibt Array aus Bools zurück
        pass
    
    def get_seperation_force(self, current_pos, targets):
        vectors = current - targets
        distances = np.reshape(np.linalg.norm(vectors, axis=1), (len(vectors), 1))
        norm_vectors = vectors / distances
        weighted_vectors = norm_vectors * (self.radius - distances)
        return np.sum(weighted_vectors, 0) / len(vectors)

    def get_alignment_force(self, current_pos, targets):
        pass
    
    def get_cohesion_force(self, current_pos, targets):
        pass

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