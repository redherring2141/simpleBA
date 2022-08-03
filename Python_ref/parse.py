import numpy as np

f = open("problem-16-22106-pre.txt", 'r')
lines = f.readlines()

num_cameras       = 0
num_points        = 0
num_observations  = 0
observations = []
camera_parameters = []
point_information = []

for i, line in enumerate(lines):
    if i == 0:
        words = line.split(" ")
        num_cameras      = int(words[0])
        num_points       = int(words[1])
        num_observations = int(words[2])

    elif 1 <= i <= num_observations:
        words = line.split()
        camera, point, x, y = words[0], words[1], float(words[2]), float(words[3])
        observations.append((camera, point, x, y))

        # time.sleep(2)
    elif num_observations + 1 <= i < num_observations + 1 + num_cameras*9:
        words = line.split()
        camera_parameters.append(float(words[0]))

    else:
        words = line.split()
        point_information.append(float(words[0]))

camera_parameters = np.array(camera_parameters)
camera_parameters = camera_parameters.reshape((num_cameras,9))

point_information = np.array(point_information)
point_information = point_information.reshape((len(point_information)//3, 3))

print("==== top 2 camera parameters ====")
print(camera_parameters[:2])
print("==== top 3 point information ====")
print(point_information[:3])

# print(len(camera_parameters), len(camera_parameters)/9)
# print(len(point_information), len(point_information)/3)
        



