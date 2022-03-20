import numpy as np
import pygame as pg
from pygame.locals import *
import sys, ctypes

DISPLAY_SIZE_DEFAULT = (1920, 1080)

# scales canvas to monitor size (on windows)
def get_display_size():

    if sys.platform == "win32":
        user32 = ctypes.windll.user32
        return (user32.GetSystemMetrics(0), user32.GetSystemMetrics(1))
    else:
        return DISPLAY_SIZE_DEFAULT

# rotates a 2D vector counterclockwise with a simple rotation matrix
def rotate_vector(vector, angle):
    return np.dot(vector, np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]]))

# returns the smallest angle between 2 vectors
# with a positive angle being counterclockwise and negative clockwise
def vector_angle(v1, v2):
    alpha = np.arccos(v1[0] / np.linalg.norm(v1)) * (v1[1] / abs(v1[1]))
    beta = np.arccos(v2[0] / np.linalg.norm(v2)) * (v2[1] / abs(v2[1]))

    return beta - alpha

class Boids:

    def __init__(self, n, s, r):
        pg.init()
        self.resolution = np.array([800, 600]) # get_display_size()
        self.display = pg.display.set_mode(self.resolution, pg.HWSURFACE | pg.SCALED)# | pg.FULLSCREEN)
        self.clock = pg.time.Clock()

        self.target_framerate = 60
        self.frame_time = 1000 / self.target_framerate

        self.scale = 100
        self.speed = 60 / self.target_framerate
        self.rotation_speed = 0.05 * 60 / self.target_framerate

        self.cam_pos = np.zeros(2)
        self.mouse_pos = np.zeros([2, 2])
        self.click_pos = np.zeros([2, 2])
        self.is_dragging = False

        self.n = n
        self.boundry_size = s
        self.radius = r
        self.boid_positions, self.boid_velocities = self.generate_agents(0)
        self.pred_positions, self.pred_velocities = self.generate_agents(1)
        self.obstacle_positions = np.random.rand(self.n[2], 2) * self.boundry_size

        self.debug_mode = 0
        self.debug_force = np.zeros(2)
        self.debug_position = np.zeros(2)
        self.debug_font = pg.font.SysFont('comic sans ms', 20)

        self.framerate = self.target_framerate
        self.framerate_buffer_size = int(self.target_framerate / 2)
        self.framerate_buffer = np.zeros(self.framerate_buffer_size)
        self.framerate_buffer_index = 0

    # generates random positions and velocities
    # returns (pos, vel)
    # typ 0: boids / typ 1: predators
    def generate_agents(self, typ):
        positions = np.random.rand(self.n[typ], 2) * self.boundry_size
        velocities = np.ones([self.n[typ], 2]) * [1, 0]
        angles = np.random.rand(self.n[typ]) * np.pi * 2

        for i in range(self.n[typ]):
            velocities[i] = rotate_vector(velocities[i], angles[i])

        return (positions, velocities * self.speed * (2 + typ))

    # determines the boids, predators and obstacles visible from the current boids position
    # returns bool numpy arrays which can be used instead of an index to get all elements where the mask is true
    def get_target_mask(self, current_pos, current_vel = np.zeros(2)):
        boid_mask = np.zeros([self.n[0]], dtype=bool)
        pred_mask = np.zeros([self.n[1]], dtype=bool)
        obstacle_mask = np.zeros([self.n[2]], dtype=bool)

        # looping over all agents and determining for each if its visible
        # this is what kills the performance
        for i in range(self.n[0]):
            if (self.boid_positions[i] != current_pos).all() and np.dot(current_vel, self.boid_positions[i] - current_pos) >= 0:
                if current_pos in self.pred_positions:
                    boid_mask[i] = np.linalg.norm(self.boid_positions[i] - current_pos) < self.radius * 3
                else:
                    boid_mask[i] = np.linalg.norm(self.boid_positions[i] - current_pos) < self.radius

        for i in range(self.n[1]):
            if (self.pred_positions[i] != current_pos).all():# and np.dot(current_vel, self.pred_positions[i] - current_pos) > 0:
                pred_mask[i] = np.linalg.norm(self.pred_positions[i] - current_pos) < self.radius

        for i in range(self.n[2]):
            if (self.obstacle_positions[i] != current_pos).all() and np.dot(current_vel, self.obstacle_positions[i] - current_pos) >= 0:
                obstacle_mask[i] = np.linalg.norm(self.obstacle_positions[i] - current_pos) < self.radius

        return (boid_mask, pred_mask, obstacle_mask)


    ## forces acting on each boid ##

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
    
    # currently not in use
    def get_boundry_force(self, current_pos):
        vector = current_pos - np.array(self.boundry_size) / 2
        distance = np.linalg.norm(vector)
        force = -vector * 5 * (distance > self.boundry_size[1] * 0.1)
        return force
    
    # calculates forces that would act on a boid at the mouse cursers position
    # this is almost the same as in update_velocity()
    def set_debug_forces(self, position):
        boid_target_mask, pred_target_mask, obstacle_target_mask = self.get_target_mask(position)

        boid_target_pos, boid_target_vel = self.boid_positions[boid_target_mask], self.boid_velocities[boid_target_mask]
        pred_target_pos, pred_target_vel = self.pred_positions[pred_target_mask], self.pred_velocities[pred_target_mask]
        obstacle_target_pos = self.obstacle_positions[obstacle_target_mask]

        force = np.zeros([3, 2])

        if boid_target_pos.size > 0:
            force[0] = self.get_cohesion_force(position, boid_target_pos)

            if position not in self.pred_positions:
                force[1] = self.get_alignment_force(position, boid_target_pos, boid_target_vel)

        if (boid_target_pos.size > 0 and position not in self.pred_positions) or pred_target_pos.size > 0 or obstacle_target_pos.size > 0:
            if position in self.pred_positions:
                target_pos = np.append(pred_target_pos, obstacle_target_pos, 0)

            else:
                target_pos = np.append(boid_target_pos, obstacle_target_pos, 0)
                target_pos = np.append(target_pos, pred_target_pos, 0)
                if pred_target_pos.size > 0:
                    force[2] = self.get_separation_force(position, pred_target_pos) * 10

            force[2] += self.get_separation_force(position, target_pos)

        self.debug_force = force * self.scale / 100
        self.debug_position = (position - self.cam_pos) * self.scale / 100

    # applies forces and returns new velocity for a boid
    def update_velocity(self, current_pos, current_vel):

        boid_target_mask, pred_target_mask, obstacle_target_mask = self.get_target_mask(current_pos, current_vel)

        # applying target masks
        # to get the positions and velocities of visible boids and obstacles
        boid_target_pos, boid_target_vel = self.boid_positions[boid_target_mask], self.boid_velocities[boid_target_mask]
        pred_target_pos, pred_target_vel = self.pred_positions[pred_target_mask], self.pred_velocities[pred_target_mask]
        obstacle_target_pos = self.obstacle_positions[obstacle_target_mask]

        force = np.zeros([3, 2])

        # confusing logic to differentiate between boids and predators
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

        angle = vector_angle(current_vel, sum_force)

        return rotate_vector(current_vel, angle * self.rotation_speed * self.frame_time * self.target_framerate / 1000)

    # main simulation update
    def update(self):
        self.mouse_pos = np.array(pg.mouse.get_pos())

        self.framerate_buffer[self.framerate_buffer_index] = 1000 / self.frame_time
        self.framerate_buffer_index += 1
        self.framerate_buffer_index %= self.framerate_buffer_size

        if self.framerate_buffer_index == self.framerate_buffer_size - 1:
            self.framerate = int(np.sum(self.framerate_buffer) / self.framerate_buffer_size)

            if self.framerate > 1:
                self.framerate_buffer_size = int(self.framerate / 2)
                self.framerate_buffer = np.zeros(self.framerate_buffer_size)
                self.framerate_buffer_index = 0

        # pygame input
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()
            
            if event.type == pg.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.is_dragging = True
                    self.click_pos = np.array([self.cam_pos, self.mouse_pos])
                
                elif event.button == 4 and self.scale < 1000:
                    self.scale *= 1.1
                    self.cam_pos -= (self.resolution * 100 / self.scale - self.resolution * 110 / self.scale) / 2

                elif event.button == 5 and self.scale > 1.1:
                    self.scale /= 1.1
                    self.cam_pos += (self.resolution * 100 / self.scale - self.resolution * 110 / self.scale) / 2
            
            if event.type == pg.MOUSEBUTTONUP:
                if event.button == 1:
                    self.is_dragging = False
            
            if event.type == pg.KEYDOWN:

                if event.key == K_F11:
                    pg.display.toggle_fullscreen()

                if event.key == K_HOME:

                    if self.debug_mode:
                        self.debug_mode = 0
                    else:
                        self.debug_mode = 1

                if self.debug_mode:
                    if event.key == K_1:
                        self.debug_mode = 1
                    elif event.key == K_2:
                        self.debug_mode = 2
                    elif event.key == K_3:
                        self.debug_mode = 3
                    elif event.key == K_4:
                        self.debug_mode = 4
                    elif event.key == K_5 and self.n[0] > 0:
                        self.debug_mode = 5
        
        if self.is_dragging:
            self.cam_pos = self.click_pos[0] + (self.click_pos[1] - pg.mouse.get_pos()) * 100 / self.scale

        screen_mid = self.to_world_space(self.resolution / 2)
        
        new_screen_mid = (screen_mid <= self.boundry_size) * screen_mid + (self.boundry_size < screen_mid) * self.boundry_size
        new_screen_mid = (0 <= new_screen_mid) * new_screen_mid

        self.cam_pos += new_screen_mid - screen_mid

        if not self.debug_mode or self.debug_mode == 5:
            # updating velocities and positions

            new_boid_velocities = np.zeros([self.n[0], 2])
            new_pred_velocities = np.zeros([self.n[1], 2])

            for i in range(self.n[0]):
                new_boid_velocities[i] = self.update_velocity(self.boid_positions[i], self.boid_velocities[i])

            for i in range(self.n[1]):
                new_pred_velocities[i] = self.update_velocity(self.pred_positions[i], self.pred_velocities[i])

            self.boid_velocities = new_boid_velocities
            self.pred_velocities = new_pred_velocities

            self.boid_positions += self.boid_velocities * self.frame_time * self.target_framerate / 1000
            self.pred_positions += self.pred_velocities * self.frame_time * self.target_framerate / 1000

            self.boid_positions %= self.boundry_size
            self.pred_positions %= self.boundry_size

            if self.debug_mode == 5:
                self.set_debug_forces(self.boid_positions[0])

        else:
            # debug mode
            self.set_debug_forces(self.to_world_space(pg.mouse.get_pos()))

    # pygame graphics
    def draw(self):
        self.display.fill((42, 42, 42))

        if self.debug_mode:
            pg.draw.circle(self.display, (69, 69, 69), self.debug_position, self.radius * self.scale / 100)

            if self.debug_mode > 3:
                for i in range(3):
                    pg.draw.line(self.display, (255, 255, 255), self.debug_position, self.debug_position + self.debug_force[i], int(2 * self.scale / 100))
                
                sum_force = np.sum(self.debug_force, 0)

                if (sum_force != 0).all():
                    sum_force = sum_force * self.radius * self.scale / (100 * np.linalg.norm(sum_force))

                pg.draw.line(self.display, (162, 255, 43), self.debug_position, self.debug_position + sum_force, int(3 * self.scale / 100))
            
            else:
                pg.draw.line(self.display, (0, 255, 255), self.debug_position, self.debug_position + self.debug_force[self.debug_mode - 1], int(2 * self.scale / 100))

        agent_positions = np.append(self.boid_positions, self.pred_positions, 0)
        agent_velocities = np.append(self.boid_velocities, self.pred_velocities, 0)

        # drawing the boid triangles
        for i in range(self.n[0] + self.n[1]):
            position = self.to_screen_space(agent_positions[i])
            velocity = agent_velocities[i]
            direction = velocity / np.linalg.norm(velocity)
 
            point_a = position + direction * self.scale / 5
            point_b = position + np.array([-direction[1], direction[0]]) * self.scale / 20
            point_c = position - np.array([-direction[1], direction[0]]) * self.scale / 20

            color = (255, 255, 255)
            if i >= self.n[0]:
                color = (255, 0, 0)

            pg.draw.polygon(self.display, color, (point_a, point_b, point_c))
        
        # drawing the obstacles
        for i in range(self.n[2]):
            pg.draw.circle(self.display, (130, 130, 255), self.obstacle_positions[i]  - self.cam_pos, 5)
        
        framerate_text = self.debug_font.render(str(self.framerate), True, (255, 255, 255))
        self.display.blit(framerate_text, (5, 5))

        scale_text = self.debug_font.render(str(int(self.scale)), True, (255, 255, 255))
        self.display.blit(scale_text, (5, 30))
    
    def to_screen_space(self, vector):
        return (np.array(vector) - self.cam_pos) * self.scale / 100
    
    def to_world_space(self, vector):
        return np.array(vector) * 100 / self.scale + self.cam_pos

    def mainloop(self):
        while True:
            self.update()
            self.draw()
            pg.display.update()
            self.frame_time = self.clock.tick(self.target_framerate)

if __name__ == '__main__':
    B = Boids([30, 1, 0], [1000, 1000], 100)
    B.mainloop()