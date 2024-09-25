import numpy as np
import matplotlib.pyplot as plt


class GenSPo:
    def __init__(self) -> None:
        pass

    def Gen_Rectangle_SPo(self, length, width, height, num_points):
        t = np.linspace(0, 4 * np.pi, num_points)
        x = length * np.sign(np.sin(t))
        y = width * np.cos(t)
        z = height * np.ones_like(t)
        return x, y, z

        # # 生成矩形轨迹
        # length_rect = 2
        # width_rect = 1
        # height_rect = 10
        # num_points = 1000
        # x_rect, y_rect, z_rect = SPo.Gen_Rectangle_SPo(length_rect, width_rect, height_rect, num_points)
        # SPo.Plot_Trajectory(x_rect, y_rect, z_rect, 'Rectangle Trajectory')
        
    def Gen_Circle_SPo(self, radius, height, num_points):
        t = np.linspace(0, 2 * np.pi, num_points)
        x = radius * np.cos(t)
        y = radius * np.sin(t)
        z = height * np.ones_like(t) 
        return x, y, z

        # 生成圆形轨迹
        # radius_circle = 8
        # height_circle = -15
        # num_points = 1000
        # x_circle, y_circle, z_circle = SPo.Gen_Circle_SPo(radius_circle, height_circle, num_points)
        # # SPo.Plot_Trajectory(x_circle, y_circle, z_circle, 'Circle Trajectory')
        
    def Gen_Sinewave_SPo(self, amplitude, frequency, length, height, num_points):
        x = np.linspace(0, length, num_points)
        y = amplitude * np.sin(frequency * x)
        z = height * np.ones_like(x)
        return x, y, z

        # # 生成正弦波轨迹
        # amplitude = 10
        # frequency = 0.5
        # length = 4 * np.pi
        # height_sine = 10
        # num_points = 1000
        # x_sine, y_sine, z_sine = SPo.Gen_Sinewave_SPo(amplitude, frequency, length, height_sine, num_points)
        # SPo.Plot_Trajectory(x_sine, y_sine, z_sine, 'Sine Wave Trajectory')
        
    def Plot_Trajectory(self, x, y, z, title):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x, y, z, label='Trajectory')
        ax.scatter(x[0], y[0], z[0], color='red', label='Start Point')
        ax.scatter(x[-1], y[-1], z[-1], color='green', label='End Point')
        ax.set_title(title)
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.legend()
        plt.show()