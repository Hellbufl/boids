import numpy as np
import vpython as vp

# rotation in 3D needs an axis to rotate around
# and is just way more complicated than 2D
# using quaternions because theyre not affected by gimbal lock
def rotate_vector(vector, axis, angle):
    # dont even ask
    angle /= 2

    # axis must be unit vector so q is a unit quaternion
    axis = axis / np.linalg.norm(axis)

    # unit quaternion q = cos(angle) + sin(angle)(v[0]i + v[1]j + v[2]k)
    q = np.append(np.cos(angle), axis * np.sin(angle))

    # dont worry about it :) https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Using_quaternions_as_rotations
    rotation_matrix = [[1 - 2*(np.power(q[2], 2) + np.power(q[3], 2)),      2*(q[1]*q[2] - q[3]*q[0]),                          2*(q[1]*q[3] + q[2]*q[0])],
                        [2*(q[1]*q[2] + q[3]*q[0]),                         1 - 2*(np.power(q[1], 2) + np.power(q[3], 2)),      2*(q[2]*q[3] - q[1]*q[0])],
                        [2*(q[1]*q[3] - q[2]*q[0]),                         2*(q[2]*q[3] + q[1]*q[0]),                          1 - 2*(np.power(q[1], 2) + np.power(q[2], 2))]]
    
    return np.dot(vector, rotation_matrix)

# same as in 2D
def vector_angle(v1, v2):
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

# vpython is trash
def arr_to_vec(array):
    return vp.vector(array[0], array[1], array[2])

class Boids:

    def __init__(self, n, r, size=100):
        self.boundry_size = size
        self.n = n
        self.radius = r
        self.boid_positions, self.boid_velocities = self.generate_agents(0)
        self.pred_positions, self.pred_velocities = self.generate_agents(1)
        self.obstacle_positions = np.random.rand(self.n[2], 3) * self.boundry_size

        self.boids = []
        self.preds = []
        self.obstacles = []

        self.scale = size / 100


        ### VPYTHON ###

        vp.scene.width, vp.scene.height = 600, 400 # 1920, 1080

        # initializing vpython objects for each boid, predator and obstacle
        for i in range(n[0]):
            self.boids.append(vp.cone(pos = arr_to_vec(self.boid_positions[i]), axis = arr_to_vec(self.boid_velocities[i] * 8), radius = 5))

        for i in range(n[1]):
            self.preds.append(vp.cone(pos = arr_to_vec(self.pred_positions[i]), axis = arr_to_vec(self.pred_velocities[i] * 8), radius = 5, color = vp.color.red))
        
        for i in range(n[2]):
            self.obstacles.append(vp.sphere(pos = arr_to_vec(self.obstacle_positions[i]), radius = 10, color = vp.color.blue))
        
        gray = vp.color.gray(0.7)
        r = 1
        d = size

        # bounding box of the space the boids are in
        boxbottom = vp.curve(color=gray, radius=r)
        boxbottom.append([vp.vector(0, 0, 0), vp.vector(0, 0, d), vp.vector(d, 0, d), vp.vector(d, 0, 0), vp.vector(0, 0, 0)])
        boxtop = vp.curve(color=gray, radius=r)
        boxtop.append([vp.vector(0,d,0), vp.vector(0,d,d), vp.vector(d,d,d), vp.vector(d,d,0), vp.vector(0,d,0)])
        vert1 = vp.curve(color=gray, radius=r)
        vert2 = vp.curve(color=gray, radius=r)
        vert3 = vp.curve(color=gray, radius=r)
        vert4 = vp.curve(color=gray, radius=r)
        vert1.append([vp.vector(0,0,0), vp.vector(0,d,0)])
        vert2.append([vp.vector(0,0,d), vp.vector(0,d,d)])
        vert3.append([vp.vector(d,0,d), vp.vector(d,d,d)])
        vert4.append([vp.vector(d,0,0), vp.vector(d,d,0)])

        # x, y and z axes
        R = size/100
        d = size-2
        xaxis = vp.cylinder(pos=vp.vec(0,0,0), axis=vp.vec(0,0,d), radius=R, color=vp.color.yellow)
        yaxis = vp.cylinder(pos=vp.vec(0,0,0), axis=vp.vec(d,0,0), radius=R, color=vp.color.yellow)
        zaxis = vp.cylinder(pos=vp.vec(0,0,0), axis=vp.vec(0,d,0), radius=R, color=vp.color.yellow)
        k = 1.02
        h = 0.05*size
        vp.text(pos=xaxis.pos+k*xaxis.axis, text='x', height=h, align='center', billboard=True, emissive=True)
        vp.text(pos=yaxis.pos+k*yaxis.axis, text='y', height=h, align='center', billboard=True, emissive=True)
        vp.text(pos=zaxis.pos+k*zaxis.axis, text='z', height=h, align='center', billboard=True, emissive=True)
    
    # same as in 2D
    def generate_agents(self, typ):
        positions = np.random.rand(self.n[typ], 3) * self.boundry_size
        velocities = np.ones([self.n[typ], 3]) * [1, 0, 0]

        axes = np.random.rand(self.n[typ], 3)
        angles = np.random.rand(self.n[typ]) * np.pi * 2

        for i in range(self.n[typ]):
            velocities[i] = rotate_vector(velocities[i], axes[i], angles[i])

        return (positions, velocities * 1.5 * (2 + (typ)))

    # same as in 2D
    def get_target_mask(self, current_pos, current_vel):
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

    # same as in 2D
    def get_separation_force(self, current_pos, target_pos):
        vectors = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors, axis=1), (len(vectors), 1))
        norm_vectors = vectors / distances
        weighted_vectors = norm_vectors * (self.radius - distances)
        force = np.sum(weighted_vectors, 0) / len(vectors)
        return force

    # same as in 2D
    def get_alignment_force(self, current_pos, target_pos, target_vel):
        vectors_pos = current_pos - target_pos
        distances = np.reshape(np.linalg.norm(vectors_pos, axis=1), (len(vectors_pos), 1))

        length_vectors_vel = np.reshape(np.linalg.norm(target_vel, axis=1), (len(target_vel), 1))
        norm_vectors_vel = target_vel / length_vectors_vel
        weighted_vectors = norm_vectors_vel * (self.radius - distances)
        force = np.sum(weighted_vectors, 0) / len(weighted_vectors)
        return force

    # same as in 2D
    def get_cohesion_force(self, current_pos, target_pos):
        center_position = np.sum(target_pos, 0) / len(target_pos)
        force = center_position - current_pos
        return force
    
    # extra force to keep the boids more in the center of the space
    # to avoid them going over the edges (because its more confusing in 3D)
    def get_boundry_force(self, current_pos):
        vector = current_pos - self.boundry_size / 2
        distance = np.linalg.norm(vector)
        force = -vector * 0.004 * (distance > self.boundry_size * 0.4)
        return force

    # (almost) same as in 2D
    def update_velocity(self, current_pos, current_vel):
        boid_target_mask, pred_target_mask, obstacle_target_mask = self.get_target_mask(current_pos, current_vel)

        boid_target_pos, boid_target_vel = self.boid_positions[boid_target_mask], self.boid_velocities[boid_target_mask]
        pred_target_pos, pred_target_vel = self.pred_positions[pred_target_mask], self.pred_velocities[pred_target_mask]
        obstacle_target_pos = self.obstacle_positions[obstacle_target_mask]

        force = np.zeros([3, 3])

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

        sum_force = np.sum(force, 0) + self.get_boundry_force(current_pos)

        if (sum_force == 0).all():
            return current_vel

        axis = np.cross(sum_force, current_vel)
        angle = vector_angle(current_vel, sum_force)

        return rotate_vector(current_vel, axis, angle * 0.05)

    # (almost) same as in 2D
    def update(self):

        new_boid_velocities = np.zeros([self.n[0], 3])
        new_pred_velocities = np.zeros([self.n[1], 3])

        for i in range(self.n[0]):
            new_boid_velocities[i] = self.update_velocity(self.boid_positions[i], self.boid_velocities[i])

        for i in range(self.n[1]):
            new_pred_velocities[i] = self.update_velocity(self.pred_positions[i], self.pred_velocities[i])

        self.boid_velocities = new_boid_velocities
        self.pred_velocities = new_pred_velocities

        self.boid_positions += self.boid_velocities
        self.pred_positions += self.pred_velocities

        self.boid_positions %= self.boundry_size
        self.pred_positions %= self.boundry_size

        # updating the vpython objects positions and velocities (directions)
        for i in range(self.n[0]):
            self.boids[i].pos = arr_to_vec(self.boid_positions[i])
            self.boids[i].axis = arr_to_vec(self.boid_velocities[i] * 16 / np.linalg.norm(self.boid_velocities[i]))
        
        for i in range(self.n[1]):
            self.preds[i].pos = arr_to_vec(self.pred_positions[i])
            self.preds[i].axis = arr_to_vec(self.pred_velocities[i] * 16 / np.linalg.norm(self.pred_velocities[i]))

    def mainloop(self):
        while True:
            vp.rate(30)
            self.update()

if __name__ == '__main__':
    B = Boids([20, 1, 0], 100, 1000)
    B.mainloop()