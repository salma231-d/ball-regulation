import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        
    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        if abs(error) < 0.01:  # Limiter l'accumulation de l'intégrale
            self.integral = 0
        else:
            self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        return (self.Kp * error + 
                self.Ki * self.integral + 
                self.Kd * derivative)
    
    def set_target(self, new_setpoint):
        self.setpoint = new_setpoint
        self.integral = 0  # Réinitialisation de l'intégrale pour une transition fluide

class BallPlateCircularSimulation:
    def __init__(self):
        # Constantes du système
        self.BALL_MASS = 0.0027  # kg
        self.GRAVITY = 9.81      # m/s²
        self.PLATE_RADIUS = 0.15  # m (plate is circular with this radius)
        self.DT = 0.02           # Augmenter le pas de temps
        self.BALL_RADIUS = 0.02  # m
        self.Ib = (2/5) * self.BALL_MASS * self.BALL_RADIUS**2
        self.SERVO_ARM = 0.05    # m

        # Calcul du gain Kb
        self.Kb = -self.BALL_MASS * self.GRAVITY * self.PLATE_RADIUS * 2 / (
            (self.BALL_MASS + self.Ib / self.BALL_RADIUS**2) * self.SERVO_ARM)

        # Initialisation des PID
        self.pid_x = PID(Kp=-0.92, Ki=-1.52, Kd=-0.23, setpoint=0)  # Gains
        self.pid_y = PID(Kp=-0.92, Ki=-1.52, Kd=-0.23, setpoint=0)  # Gains

        # État initial de la balle
        self.ball_state = {
            "x": 0.05,     # Position X initiale
            "y": -0.05,     # Position Y initiale
            "vx": 0.0,     # Vitesse X initiale
            "vy": 0.0      # Vitesse Y initiale
        }

        # Points cibles
        self.target_points = [
            (0, 0),              # Centre
            (0, 0)              # Retour au centre
        ]
        self.current_target = 0
        self.time_at_target = 0
        self.target_dwell_time = 3.0  # Temps à rester sur chaque cible réduit

        # Historique des positions
        self.history_length = 100  # Reduire la longueur de l'historique
        self.x_history = []
        self.y_history = []
        self.time_history = []
        self.current_time = 0
        
        # Initialisation de la première cible
        self.update_target(self.target_points[0])

    def update_target(self, point):
        self.pid_x.set_target(point[0])
        self.pid_y.set_target(point[1])

    def check_target_reached(self):
        dx = abs(self.ball_state["x"] - self.pid_x.setpoint)
        dy = abs(self.ball_state["y"] - self.pid_y.setpoint)
        return dx < 0.005 and dy < 0.005  # Tolérance plus stricte

    def check_circular_boundary(self):
        # Assurez-vous que la balle reste dans le plateau circulaire
        r = np.sqrt(self.ball_state["x"]**2 + self.ball_state["y"]**2)
        if r > self.PLATE_RADIUS:
            angle = np.arctan2(self.ball_state["y"], self.ball_state["x"])
            self.ball_state["x"] = self.PLATE_RADIUS * np.cos(angle)
            self.ball_state["y"] = self.PLATE_RADIUS * np.sin(angle)

    def simulate(self):
        fig = plt.figure(figsize=(15, 8))
        gs = gridspec.GridSpec(2, 2)

        def update(frame):
            # Vérifier si la cible est atteinte
            if self.check_target_reached():
                self.time_at_target += self.DT
                if self.time_at_target >= self.target_dwell_time:
                    self.current_target = (self.current_target + 1) % len(self.target_points)
                    self.update_target(self.target_points[self.current_target])
                    self.time_at_target = 0

            
            # Appliquer une perturbation après 2 secondes
            if self.current_time > 2.0 and self.current_time <= 2.01:
                self.ball_state["vx"] += 0.5  # Ajouter une vitesse sur l'axe X
                self.ball_state["vy"] -= 0.3  # Ajouter une vitesse sur l'axe Y




            # Correction PID
            x_correction = self.pid_x.compute(self.ball_state["x"], self.DT)
            y_correction = self.pid_y.compute(self.ball_state["y"], self.DT)

            # Calcul des angles d'inclinaison du plateau
            angle_x = np.clip(x_correction, -0.5, 0.5)  # Angle X (en radians)
            angle_y = np.clip(y_correction, -0.5, 0.5)  # Angle Y (en radians)
            
            # Conversion en degrés pour les moteurs
            angle_x_deg = np.degrees(angle_x)
            angle_y_deg = np.degrees(angle_y)

            # Mise à jour de la dynamique de la balle
            self.ball_state["vx"] += self.Kb * angle_x * self.DT
            self.ball_state["vy"] += self.Kb * angle_y * self.DT
            self.ball_state["x"] += self.ball_state["vx"] * self.DT
            self.ball_state["y"] += self.ball_state["vy"] * self.DT

            # Réduction artificielle des oscillations
            self.ball_state["vx"] *= 0.99
            self.ball_state["vy"] *= 0.99

            # Vérification des limites circulaires
            self.check_circular_boundary()

            # Mise à jour de l'historique
            self.x_history.append(self.ball_state["x"])
            self.y_history.append(self.ball_state["y"])
            self.current_time += self.DT
            self.time_history.append(self.current_time)

            if len(self.x_history) > self.history_length:
                self.x_history.pop(0)
                self.y_history.pop(0)
                self.time_history.pop(0)

            # Effacement des graphiques
            plt.clf()

            # Affichage de la position et des angles d'inclinaison
            plt.suptitle(f"Inclinaison du plateau: θx = {angle_x_deg:.2f}°, θy = {angle_y_deg:.2f}°")

            # Trajectoire de la balle sur le plateau circulaire
            ax1 = plt.subplot(gs[:, 0])
            circle = Circle((0, 0), self.PLATE_RADIUS, fill=False, color='gray', linestyle='--')
            ax1.add_artist(circle)
            for i, (tx, ty) in enumerate(self.target_points):
                color = 'r+' if i == self.current_target else 'y+'
                ax1.plot(tx, ty, color, markersize=10)
            ax1.plot(self.ball_state["x"], self.ball_state["y"], 'bo', markersize=15, label='Ball')
            ax1.set_aspect('equal')
            ax1.set_xlim(-self.PLATE_RADIUS, self.PLATE_RADIUS)
            ax1.set_ylim(-self.PLATE_RADIUS, self.PLATE_RADIUS)
            ax1.grid(True)
            ax1.set_title('Ball Position on Circular Plate')
            ax1.set_xlabel('X Position (m)')
            ax1.set_ylabel('Y Position (m)')
            ax1.legend()

            # Historique de la position X
            ax2 = plt.subplot(gs[0, 1])
            ax2.plot(self.time_history, self.x_history, 'b-', label='X Position')
            ax2.axhline(y=self.pid_x.setpoint, color='r', linestyle='--', label='X Target')
            ax2.grid(True)
            ax2.set_title('X Position vs Time')
            ax2.legend()

            # Historique de la position Y
            ax3 = plt.subplot(gs[1, 1])
            ax3.plot(self.time_history, self.y_history, 'g-', label='Y Position')
            ax3.axhline(y=self.pid_y.setpoint, color='r', linestyle='--', label='Y Target')
            ax3.grid(True)
            ax3.set_title('Y Position vs Time')
            ax3.legend()

            plt.tight_layout()

        ani = animation.FuncAnimation(fig, update, interval=int(self.DT*1000), 
                                    cache_frame_data=False)
        plt.show()

if __name__ == "__main__":
    simulation = BallPlateCircularSimulation()
    simulation.simulate()
