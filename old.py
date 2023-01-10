from shapely.geometry import Polygon,LineString, Point
import random
import math
import matplotlib.pyplot as plt


def create_random_circle():
  # génération aléatoire du rayon du cercle (entre 0,5 et 1,5)
  radius = random.uniform(0.5, 1.5)
  # création d'un cercle autour de l'origine (0,0)
  circle = Point(0, 0).buffer(radius)
  return circle

def create_random_oval():
  # génération aléatoire de la longueur et de la largeur de l'ovale (entre 0,5 et 1,5)
  length = random.uniform(0.5, 1.5)
  width = random.uniform(0.5, 1.5)
  # création d'une ligne en forme de "8"
  line = LineString([(0, 0), (0, width), (length, width), (length, 0), (0, 0)])
  # création d'un rectangle qui enveloppe la ligne
  envelope = line.envelope
  # création d'un cercle autour de l'origine (0,0)
  circle = Point(0, 0).buffer(1)
  # intersection de l'ovale et du cercle pour ne garder que la partie de l'ovale à l'intérieur du cercle
  oval = envelope.intersection(circle)
  return oval

def create_random_circuit():
  # génération aléatoire d'un cercle ou d'un ovale
  #if random.choice([True, False]):
  if True:
    shape = create_random_circle()
  else:
    shape = create_random_oval()
  # conversion de la forme en un polygone
  if isinstance(shape, Polygon):
    points = shape.exterior.coords[:]
  else:
    points = shape.coords[:]
  return points


def add_cones(points, distance):
  # tableaux vides qui contiendront les points des cônes
  cones_inside = []
  cones_outside = []
  # angle de rotation des cônes par rapport au circuit
  angle = math.pi / 4
  # parcours des points du circuit
  for i in range(len(points)):
    # récupération des coordonnées du point courant
    x, y = points[i]
    # récupération des coordonnées du point suivant (ou du premier point si on est au dernier point)
    next_x, next_y = points[(i+1) % len(points)]
    # calcul de l'angle entre le point courant et le point suivant
    angle_circuit = math.atan2(next_y - y, next_x - x)
    # calcul des coordonnées des points des cônes à l'intérieur et à l'extérieur du point courant
    cone_inside_x, cone_inside_y = x + distance * math.cos(angle_circuit + angle), y + distance * math.sin(angle_circuit + angle)
    cone_outside_x, cone_outside_y = x + distance * math.cos(angle_circuit - angle), y + distance * math.sin(angle_circuit - angle)
    # ajout des points des cônes aux tableaux
    cones_inside.append((cone_inside_x, cone_inside_y))
    cones_outside.append((cone_outside_x, cone_outside_y))
  # ajout du dernier point des cônes à l'intérieur et à l'extérieur (pour fermer les cercles)
  cones_inside[-1]=cones_inside[0]
  cones_outside[-1] = cones_outside[0]
  return points, cones_inside, cones_outside



class Sensor:
  def __init__(self, field_of_view, reliability, orientation, distance_vision):
    self.field_of_view = field_of_view
    self.reliability = reliability
    self.orientation = orientation
    self.distance_vision = distance_vision

  def detect_cones(self, cones, position, car_orientation):
    detect_cones = []
    for i, cone in enumerate(cones):
      dx = cone[0] - position[0]
      dy = cone[1] - position[1]
      distance = math.sqrt(dx ** 2 + dy ** 2)
      angle = math.atan2(dy, dx) - self.orientation - car_orientation

      if angle < -math.pi:
        angle += 2 * math.pi
      if angle > math.pi:
        angle -= 2 * math.pi

      if abs(angle) < self.field_of_view / 2 and distance < self.distance_vision:
        cone_x = cone[0] + (random.random() * 2 - 1) * self.reliability
        cone_y = cone[1] + (random.random() * 2 - 1) * self.reliability
        detect_cones.append((cone_x, cone_y, i))
    return detect_cones


class Object:
  def __init__(self, path, speed, reliability, sensors):
    self.speed = speed
    self.reliability = reliability
    self.sensors = sensors
    self.path = path
    self.path_index = 0
    self.position = path[0]
    dx = self.path[self.path_index][0] - self.path[self.path_index - 1][0]
    dy = self.path[self.path_index][1] - self.path[self.path_index - 1][1]
    self.orientation = math.atan2(dy, dx)

  def follow_path(self):
    self.path_index += 1
    if self.path_index == len(self.path):
      self.path_index = 0
    self.position = self.path[self.path_index]
    dx = self.path[self.path_index][0] - self.path[self.path_index - 1][0]
    dy = self.path[self.path_index][1] - self.path[self.path_index - 1][1]
    self.orientation = math.atan2(dy, dx)

  def move(self, inner_cones, outer_cones):
    self.follow_path()
    for sensor in self.sensors:
      sensor.detect_cones(inner_cones + outer_cones, self.position, self.orientation)


def create_object_and_move(inner_cones, outer_cones, speed, reliability, sensors, path):
  object = Object(path, speed, reliability, sensors)
  while True:
    object.move(inner_cones, outer_cones)
    object_x = object.position[0] + (random.random() * 2 - 1) * object.reliability
    object_y = object.position[1] + (random.random() * 2 - 1) * object.reliability
    detected_cones = [[] for _ in range(len(sensors))]
    for i, sensor in enumerate(sensors):
      detected_cones[i] = sensor.detect_cones(inner_cones + outer_cones, object.position, object.orientation)
    yield (object_x, object_y), detected_cones



#to create ce circuit, points is the coordonate(tupple x,y), and the same for the cones inside and outside

points = create_random_circuit()
points, cones_inside, cones_outside = add_cones(points, 0.5)


#print(f"{points}\n{cones_inside}\n{cones_outside}\n{len(points)}\n{len(cones_inside)}\n{len(cones_outside)}")
#plt.plot(*zip(*points))
#plt.plot(*zip(*cones_inside), 'g')
#plt.plot(*zip(*cones_outside), 'r')
#plt.show()


import pygame
import math


from matplotlib.animation import FuncAnimation

def animate(inner_cones, outer_cones, speed, reliability, sensors, path):
    object_and_sensors = create_object_and_move(inner_cones, outer_cones, speed, reliability, sensors, path)
    fig, ax = plt.subplots()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)

    # Draw the path
    path_line, = ax.plot([p[0] for p in path], [p[1] for p in path], 'k')

    # Draw the cones
    inner_cones_scatter = ax.scatter([c[0] for c in inner_cones], [c[1] for c in inner_cones], c='r')
    outer_cones_scatter = ax.scatter([c[0] for c in outer_cones], [c[1] for c in outer_cones], c='g')

    object_scatter = ax.scatter([], [], c='b')
    sensor_detection_scatters = []
    for i in range(len(sensors)):
        scatter, = ax.plot([], [], 'o', c='m', alpha=0.5)
        sensor_detection_scatters.append(scatter)

    def init():
      object_xy, sensor_detections = next(object_and_sensors)
      object_x, object_y = object_xy[0],object_xy[1]
      object_scatter.set_offsets((object_x, object_y))
      for i, sensor_detection in enumerate(sensor_detections):
        sensor_detection_scatters[i].set_data([d[0] for d in sensor_detection], [d[1] for d in sensor_detection])

    def update(frame):
      object_xy, sensor_detections = next(object_and_sensors)
      object_x, object_y = object_xy[0], object_xy[1]

      object_scatter.set_offsets((object_x, object_y))
      for i, sensor_detection in enumerate(sensor_detections):
        sensor_detection_scatters[i].set_data([d[0] for d in sensor_detection], [d[1] for d in sensor_detection])
    anim = FuncAnimation(fig, update, frames=200, repeat=True,init_func=init())
    plt.show()


#to create a sensor, the angle of the vision, the fiability (0 to 1, with 0 is 100% accurate), the direction of the sensor in radian, and the distance it 'see"

sensor1 = Sensor(math.radians(90),0.1,math.radians(270),0.3)
sensor2 = Sensor(math.radians(90),0.1,math.radians(90),0.5)


#to create an object, get the list of the cones, the speed, the accuracy of the position, list of sensors, and the path
#return yield of the position and the cones detected by all the sensor
test = create_object_and_move(cones_inside, cones_outside,0.1,0,[sensor1,sensor2],points)


sensors = [sensor1,sensor2]
speed = 20
reliability = 0.1
animate(cones_inside, cones_outside, speed, reliability, sensors, points)

#animate(cones_inside, cones_outside,0.1,0,[sensor1,sensor2],points)