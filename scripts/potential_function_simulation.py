import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, position, goal, max_speed, max_turn_rate):
        self.position = np.array(position, dtype=float)  # Robotun mevcut konumu
        self.goal = np.array(goal, dtype=float)          # Robotun hedef konumu
        self.max_speed = max_speed                      # Maksimum hız (birim/s)
        self.max_turn_rate = max_turn_rate              # Maksimum dönüş hızı (radyan/s)

    def move_towards_goal(self, dt, obstacles):
        attractive_force = self.calculate_attractive_force()
        repulsive_force = self.calculate_repulsive_force(obstacles)

        total_force = attractive_force + repulsive_force

        desired_angle = np.arctan2(total_force[1], total_force[0])
        angular_error = self.normalize_angle(desired_angle - self.position[2])

        turn_rate = np.clip(angular_error, -self.max_turn_rate * dt, self.max_turn_rate * dt)
        self.position[2] += turn_rate

        speed = np.linalg.norm(total_force[:2])
        speed = np.clip(speed, 0, self.max_speed)

        self.position[0] += speed * np.cos(self.position[2]) * dt
        self.position[1] += speed * np.sin(self.position[2]) * dt

    def calculate_attractive_force(self):
        direction = self.goal - self.position[:2]
        distance = np.linalg.norm(direction)
        if distance == 0:
            return np.zeros(2)
        return direction / distance

    def calculate_repulsive_force(self, obstacles):
        repulsive_force = np.zeros(2)
        for obstacle in obstacles:
            direction = self.position[:2] - obstacle.position
            distance = np.linalg.norm(direction)
            if distance < obstacle.radius:
                distance = obstacle.radius
            repulsive_force += direction / distance**2
        return repulsive_force

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def plot_robot(self):
        plt.scatter(self.position[0], self.position[1], color='blue', marker='o', label='Robot')

class Obstacle:
    def __init__(self, position, radius):
        self.position = np.array(position, dtype=float)  # Engel konumu
        self.radius = radius  # Engel yarıçapı

    def plot_obstacle(self):
        circle = plt.Circle((self.position[0], self.position[1]), self.radius, color='black', fill=True)
        plt.gca().add_patch(circle)

# Kullanım örneği
robot = Robot([0, 0, 0], [5, 5], max_speed=1.0, max_turn_rate=np.pi/4)  # Robotu başlangıçta (0, 0) konumuna ve yönüne yerleştir
dt = 0.1  # Zaman adımı
num_steps = 10000  # Toplam adım sayısı

# Engelleri oluştur
obstacle1 = Obstacle([2, 2], radius=1)
obstacle2 = Obstacle([3, 3], radius=0.5)
obstacles = [obstacle1, obstacle2]

# Hareketi görselleştirme
plt.figure(figsize=(8, 6))
for _ in range(num_steps):
    plt.clf()
    for obstacle in obstacles:
        obstacle.plot_obstacle()
    robot.move_towards_goal(dt, obstacles)
    robot.plot_robot()
    plt.scatter(robot.goal[0], robot.goal[1], color='red', marker='o', label='Hedef')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Hareketi')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.pause(0.05)  # Görselleştirme arası bekletme süresi
plt.show()
