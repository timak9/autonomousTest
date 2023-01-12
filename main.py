from shapely.geometry import Polygon,LineString, Point, LinearRing
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#function to generate a circle
def create_random_circle(minimum,maximum):
  radius = random.uniform(minimum,maximum)
  circle = Point(0, 0).buffer(radius)
  return circle


#function to generate an oval DONT WORK
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


#function to generate the circuit, WORK JUST WITH CIRCLE
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
  else:
    new_points = shape.coords[:]
  return new_points

#functuion to generate all the cone, get list of points, the distance of the cones (from
# the middle), and the number of cones
#return 3 lists, the list of points, list of all the cones outside, and the same for the cones inside
def add_cones(points, distance,numCones):
  cones_inside = []
  cones_outside = []
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


#class sensor, sensor it is the detector of the cones, not the position of the object!
#get in order: the angle of the view, the fiability, the orientation in radian, and the distance of vision
class Sensor:
  def __init__(self, field_of_view, reliability, orientation, distance_vision):
    self.field_of_view = field_of_view
    self.reliability = reliability
    self.orientation = orientation
    self.distance_vision = distance_vision

#return the cones detected by the sensor, with random aproximation
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
        cone_x = random.gauss(0,self.reliability**(1/2)) + cone[0]
        cone_y = cone[1] + random.normalvariate(0,self.reliability**(1/2))
        detect_cones.append((cone_x, cone_y, i))
    return detect_cones



#class detector, is a detector for the position of the object
#get the fiability of the position, and the fiability of the orientation
class Detector:
  def __init__(self,reliabilityPos,reliabilityOri):
    self.reliabilityPos = reliabilityPos
    self.reliabilityOri = reliabilityOri


#return the position and the orientation detected by the detector (with "error")
  def position(self,objectposition,orientation):
    detectorPositionX = random.gauss(0,math.sqrt(self.reliabilityPos)) + objectposition[0]
    detectorPositionY = random.gauss(0, math.sqrt(self.reliabilityPos)) + objectposition[1]
    detectorOrientation = random.gauss(0, math.sqrt(self.reliabilityOri)) + orientation
    return (detectorPositionX,detectorPositionY,detectorOrientation)


class Object:
  def __init__(self, path, speed, reliability, detectors, sensors):
    self.speed = speed
    self.reliability = reliability
    self.sensors = sensors
    self.detectors = detectors
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





def create_object_and_move(inner_cones, outer_cones, speed,detectors, sensors, path):
  object = Object(path, speed, reliability, detectors, sensors)
  while True:
    object.move(inner_cones, outer_cones)
    coordonateDetector = [[] for _ in range(len(detectors))]
    detected_cones = [[] for _ in range(len(sensors))]
    for count, detector in enumerate(detectors):
      coordonateDetector[count] = detector.position(object.position,object.orientation)
    for i, sensor in enumerate(sensors):
      detected_cones[i] = sensor.detect_cones(inner_cones + outer_cones, object.position, object.orientation)
    yield [[object.position[0],object.position[1],object.orientation],coordonateDetector, detected_cones]




def animate(inner_cones, outer_cones, speed, detectors, sensors, path):
    diffPosdetec = []

    object_and_sensors = create_object_and_move(inner_cones, outer_cones, speed, detectors, sensors, path)

    fig, ax = plt.subplots()
    ax.set_xlim(min([x[0] for x in outer_cones])*1.1, max([x[0] for x in outer_cones])*1.1)
    ax.set_ylim(min([x[1] for x in outer_cones])*1.1, max([x[1] for x in outer_cones])*1.1)

    # Draw the path
    path_line, = ax.plot([p[0] for p in path], [p[1] for p in path], 'k')

    # Draw the cones
    inner_cones_scatter = ax.scatter([c[0] for c in inner_cones], [c[1] for c in inner_cones], c='r')
    outer_cones_scatter = ax.scatter([c[0] for c in outer_cones], [c[1] for c in outer_cones], c='r')

    object_scatter = ax.scatter([], [], c='b')
    sensor_detection_scatters = []
    for i in range(len(sensors)):
        scatter, = ax.plot([], [], 'o', c='g', alpha=0.7)
        sensor_detection_scatters.append(scatter)

    detectors_scatters = []
    for i in range(len(detectors)):
      scatter2, = ax.plot([], [], 'o', c='cyan', alpha=0.8)
      detectors_scatters.append(scatter2)

    def init():


      object_xy, detectors_detections, sensor_detections = next(object_and_sensors)
      object_x, object_y = object_xy[0],object_xy[1]
      object_scatter.set_offsets((object_x, object_y))


      for i, detector_detection in enumerate(detectors_detections):
        diffPosdetec.append([detector_detection[0]-object_x])
        detectors_scatters[i].set_data(detector_detection[0], detector_detection[1])

      for i, sensor_detection in enumerate(sensor_detections):
        sensor_detection_scatters[i].set_data([d[0] for d in sensor_detection], [d[1] for d in sensor_detection])

    def update(frame):


      object_xy, detectors_detections, sensor_detections = next(object_and_sensors)
      object_x, object_y = object_xy[0], object_xy[1] # cicle blue







      sensorDistanceObject = []
      for count,sensor in enumerate(sensor_detections):
        if sensor:
          sensorDistanceObject.append([math.sqrt((-sensor[0][0])**2 + (object_y-sensor[0][1])**2),math.atan2(object_y - sensor[0][1], object_x - sensor[0][0]),sensor[0][2]])
        else:
          sensorDistanceObject.append([]) #the sensor same we get (distance fromthe auto, the alpha and the id of the cone)

      object_scatter.set_offsets((object_x, object_y)) #plot the object, here to modify

      for i, detector_detection in enumerate(detectors_detections):
        diffPosdetec[i].append(detector_detection[0] - object_x)
        #print(f"captor number {i}: {sum(diffPosdetec[i]) / len(diffPosdetec)}, detpos: {detector_detection[0]}, pos: {object_x}")
        detectors_scatters[i].set_data(detector_detection[0], detector_detection[1])

      for i, sensor_detection in enumerate(sensor_detections):
        sensor_detection_scatters[i].set_data([d[0] for d in sensor_detection], [d[1] for d in sensor_detection])

    anim = FuncAnimation(fig, update, frames=200, repeat=True,init_func=init())
    plt.show()




#to create ce circuit, points is the coordonate(tupple x,y), and the same for the cones inside and outside

#points = lenths minimum, lenght maximum, number of "points" of the circuit (more we have, more HD it is)
#points, cones_inside,cones_outside = points,distance of the cones to right/left, numberofCones

points = create_random_circuit(20,80,130)
points, cones_inside, cones_outside = add_cones(points, 2.5 ,20)


#to create a sensor, the angle of the vision, the fiability (number of meter or radian of potential error),
# the direction of the sensor in radian, and the distance it 'see"

sensor1 = Sensor(math.radians(90),1,math.radians(270),15)
sensor2 = Sensor(math.radians(90),1,math.radians(90),15)





#to create a detector, fiability of the position and fiability of the orientation
# (number of meter or radian of potential error),
detector1 = Detector(1,math.radians(1))
detector2 = Detector(5,math.radians(10))



sensors = [sensor1,sensor2]
detectors = [detector1,detector2]
speed = 1
reliability = 0.1

#to create an object, get the list of the cones, the speed, the accuracy of the position, list of sensors, and the path
#return yield of the position and the cones detected by all the sensor
animate(cones_inside, cones_outside, speed, detectors, sensors, points)
