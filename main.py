from shapely.geometry import Polygon,LineString, Point, LinearRing
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



METERINCERTITUDE = 10


def create_random_circle(minimum,maximum):
  # génération aléatoire du rayon du cercle
  radius = random.uniform(minimum,maximum)
  circle = Point(0, 0).buffer(radius)
  return circle

def create_random_oval(minimum,maximum):
  # génération aléatoire de la longueur et de la largeur de l'ovale
  length = random.uniform(minimum,maximum)
  width = random.uniform(minimum,maximum)
  line = LineString([(0, 0), (0, width), (length, width), (length, 0), (0, 0)])
  # création d'un rectangle qui enveloppe la ligne
  envelope = line.envelope
  circle = Point(0, 0).buffer(1)
  # intersection de l'ovale et du cercle pour ne garder que la partie de l'ovale à l'intérieur du cercle
  oval = envelope.intersection(circle)
  return oval

def create_random_circuit(minimum,maximum,numbPoints):
  # génération aléatoire d'un cercle ou d'un ovale
  #if random.choice([True, False]):
  if True:
    shape = create_random_circle(minimum,maximum)
  else:
    shape = create_random_oval(minimum,maximum)
  # conversion de la forme en un polygone
  if isinstance(shape, Polygon):
    points = shape.exterior.coords[:]
    factor = max(round(numbPoints/len(points)),1)
    new_points = []
    oldPoint = points[-1]
    for i in points:
      xMean,yMean = (oldPoint[0]+i[0])/factor,(oldPoint[1]+i[1])/factor
      for j in range(1,factor):
        new_points.append((j*xMean,j*yMean))
      oldPoint = i
      new_points.append((i[0], i[1]))
    #(len(points),len(new_points))
    #print(points)
    #print(new_points)
  else:
    new_points = shape.coords[:]
  return points


def add_cones(points, distance,numCones):
  cones_inside = []
  cones_outside = []

  # parcours des points du circuit
  allx, ally = [x[0] for x in points], [x[1] for x in points]
  centerX, centerY = sum(allx)/len(allx),sum(ally)/len(ally)
  allDistance = [math.sqrt((x[0]-centerX)**2 + (x[1]-centerY)**2) for x in points]
  maxDistance = max(allDistance)
  intervalCones = len(points)//numCones
  for i in range(len(points)):

    x, y = points[i]

    if (i%intervalCones==0):
      factorOutside = distance/maxDistance +1
      factorInside = 1-distance/maxDistance
      cone_inside_x, cone_inside_y = x*factorInside, y*factorInside
      cone_outside_x, cone_outside_y =x*factorOutside, y *factorOutside

      cones_inside.append((cone_inside_x, cone_inside_y))
      cones_outside.append((cone_outside_x, cone_outside_y))
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
        cone_x = (random.random()*2-1)*self.reliability*METERINCERTITUDE + cone[0]
        cone_y = cone[1] + (random.random()*2-1)*self.reliability*METERINCERTITUDE
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
    object_x = object.position[0] + (random.random()*2-1)*METERINCERTITUDE * object.reliability
    object_y = object.position[1] + (random.random()*2-1)*METERINCERTITUDE * object.reliability
    detected_cones = [[] for _ in range(len(sensors))]
    for i, sensor in enumerate(sensors):
      detected_cones[i] = sensor.detect_cones(inner_cones + outer_cones, object.position, object.orientation)
    yield (object_x, object_y), detected_cones




def animate(inner_cones, outer_cones, speed, reliability, sensors, path):
    object_and_sensors = create_object_and_move(inner_cones, outer_cones, speed, reliability, sensors, path)
    fig, ax = plt.subplots()
    ax.set_xlim(min([x[0] for x in outer_cones])*1.1, max([x[0] for x in outer_cones])*1.1)
    ax.set_ylim(min([x[1] for x in outer_cones])*1.1, max([x[1] for x in outer_cones])*1.1)

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
      sensorDistanceObject = []
      for count,sensor in enumerate(sensor_detections):
        if sensor:
          sensorDistanceObject.append([math.sqrt((object_x-sensor[0][0])**2 + (object_y-sensor[0][1])**2),math.atan2(object_y - sensor[0][1], object_x - sensor[0][0]),sensor[0][2]])
        else:
          sensorDistanceObject.append([]) #the sensor same we get (distance fromthe auto, the alpha and the id of the cone)

      object_scatter.set_offsets((object_x, object_y)) #plot the object, here to modify
      for i, sensor_detection in enumerate(sensor_detections):
        sensor_detection_scatters[i].set_data([d[0] for d in sensor_detection], [d[1] for d in sensor_detection])
    anim = FuncAnimation(fig, update, frames=200, repeat=True,init_func=init())
    plt.show()




#to create ce circuit, points is the coordonate(tupple x,y), and the same for the cones inside and outside

#points = lenths minimum, lenght maximum, number of "points" of the circuit (more we have, more HD it is)
#points, cones_inside,cones_outside = points,distance of the cones to right/left, numberofCones

points = create_random_circuit(100,300,130)
points, cones_inside, cones_outside = add_cones(points, 10,20)


'''#print(f"{points}\n{cones_inside}\n{cones_outside}\n{len(points)}\n{len(cones_inside)}\n{len(cones_outside)}")
plt.plot(*zip(*points),'b')
plt.plot(*zip(*cones_inside), 'g')
plt.plot(*zip(*cones_outside), 'r')
plt.show()'''


#to create a sensor, the angle of the vision, the fiability (0 to 1, with 0 is 100% accurate), the direction of the sensor in radian, and the distance it 'see"

sensor1 = Sensor(math.radians(90),0.3,math.radians(270),15)
sensor2 = Sensor(math.radians(90),0.3,math.radians(90),15)


#to create an object, get the list of the cones, the speed, the accuracy of the position, list of sensors, and the path
#return yield of the position and the cones detected by all the sensor


#test = create_object_and_move(cones_inside, cones_outside,0.01,0,[sensor1,sensor2],points)


sensors = [sensor1,sensor2]
speed = 10
reliability = 0.1
animate(cones_inside, cones_outside, speed, reliability, sensors, points)

#animate(cones_inside, cones_outside,0.1,0,[sensor1,sensor2],points)