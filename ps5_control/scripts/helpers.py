import numpy as np

class Vector2D:
    def __init__(self, x, y):
        self.data = np.array([x, y], dtype=float)

    @property
    def x(self):
        return self.data[0]

    @property
    def y(self):
        return self.data[1]

    @x.setter
    def x(self, value):
        self.data[0] = value

    @y.setter
    def y(self, value):
        self.data[1] = value

    def __add__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x + other.x, self.y + other.y)
        raise TypeError("Unsupported operand type for +")

    def __sub__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x - other.x, self.y - other.y)
        raise TypeError("Unsupported operand type for -")

    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Vector2D(self.x * scalar, self.y * scalar)
        raise TypeError("Unsupported operand type for *")

    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)):
            return Vector2D(self.x / scalar, self.y / scalar)
        raise TypeError("Unsupported operand type for /")

    def __str__(self):
        return f"({self.x}, {self.y})"

    def magnitude(self):
        return np.linalg.norm(self.data)

    def normalize(self):
        mag = self.magnitude()
        if mag != 0:
            return Vector2D(self.x / mag, self.y / mag)
        else:
            return Vector2D(0, 0)

def InputFilter(raw_input, previous_filtered_input, alpha=0.2):
    # Apply simple exponential moving average filter EMA
    filtered_input = alpha * raw_input + (1 - alpha) * previous_filtered_input
    return filtered_input

def DeadZone(dead_zone, value):
    if abs(value) < dead_zone:
        return 0.0, True
    else:
        return value, False
    
def apply_deadzone(x, y, deadzone):
    if -deadzone <= x <= deadzone:
        x = 0
    if -deadzone <= y <= deadzone:
        y = 0
    return x, y


def distance_to_force(distance, max_force=255, max_distance = 5):
    distance_percentage = 1 - (distance / max_distance)
    force = max_force * (1 - np.power(distance_percentage, 4))
    force = clamp(force, 0, 255)
    return int(force)


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))