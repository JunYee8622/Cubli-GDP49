import pygame
import numpy as np
import matplotlib
import BodyDyn as BD
import time

pygame.init()

screen = pygame.display.set_mode((1280, 720))



reactionWheel_img = pygame.image.load("GUISprites/ReactionWheel.png").convert_alpha()

L = 0.15 #m
pixelscale = 1500 #px/m
cubePointsRelative = np.array([[0,0], [L, 0], [L, -L], [0, -L]]) * pixelscale
pivotPosition = np.array([1050, 590])
initialCubeAngle = -45 #deg



def rotationMatrix(angle):
    angle = np.deg2rad(angle)
    rotationMatrix = [[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]]
    return rotationMatrix

for i in range (0, len(cubePointsRelative)):
    cubePointsRelative[i] = np.dot(rotationMatrix(initialCubeAngle), cubePointsRelative[i])



running = True

clock = pygame.time.Clock()

delta_time = 0.1
theta = np.deg2rad(0)

simulate_time = 6
dt = 0.001
thetaInit_deg = 5
delayTime = 20e-3

'''here are what added my AI'''
Arr_timepoints, Arr_angles, Arr_wheelAngle, Arr_wheelVels = BD.simulate(False, simulate_time, dt, thetaInit_deg, delayTime)
#time.sleep(5)
bodyDyngraph_img = pygame.image.load("GUISprites/bodyDynamics.jpg").convert()
graphW, graphH = 900, 450
bodyDyngraph_img = pygame.transform.scale(bodyDyngraph_img, (graphW, graphH))

# Red dot on each graph: map (time, value) to pixels on the saved matplotlib figure.
# The figure is saved with default subplot layout, so axes positions come from rcParams;
# data limits are matplotlib's default 5 % margin around the plotted data.
def makeGraphMapper(axIndex, yData):
    p = matplotlib.rcParams
    left, right, wspace = p["figure.subplot.left"], p["figure.subplot.right"], p["figure.subplot.wspace"]
    bottom, top = p["figure.subplot.bottom"], p["figure.subplot.top"]
    axW = (right - left) / (2 + wspace)
    axLeft = left + axIndex * axW * (1 + wspace)
    px0, px1 = axLeft * graphW, (axLeft + axW) * graphW
    py0, py1 = (1 - bottom) * graphH, (1 - top) * graphH  # pixel y of axis bottom and top

    tMin, tMax = Arr_timepoints[0], Arr_timepoints[-1]
    yMin, yMax = min(yData), max(yData)
    xLo, xHi = tMin - 0.05 * (tMax - tMin), tMax + 0.05 * (tMax - tMin)
    yLo, yHi = yMin - 0.05 * (yMax - yMin), yMax + 0.05 * (yMax - yMin)

    def toPixel(i):
        x = px0 + (Arr_timepoints[i] - xLo) / (xHi - xLo) * (px1 - px0)
        y = py0 + (yData[i] - yLo) / (yHi - yLo) * (py1 - py0)
        return (int(x), int(y))
    return toPixel

angleDotPos = makeGraphMapper(0, Arr_angles)
wheelDotPos = makeGraphMapper(1, Arr_wheelVels)
'''until here '''


print(Arr_angles[25])
i = 0

scW, scH = screen.get_size()

angAmplVis = 4

while running:
    cubePoints = np.copy(cubePointsRelative)
    screen.fill((234, 226, 183))
    theta = Arr_angles[i]
    for j in range (0, len(cubePointsRelative)):
        cubePoints[j] = np.dot(rotationMatrix(theta * angAmplVis), cubePointsRelative[j]) + pivotPosition
    wheelCentre = ((cubePoints[0] + cubePoints[2])/2)


    wheelAngle_deg = np.rad2deg(Arr_wheelAngle[i])
    rotatedWheel = pygame.transform.rotate(reactionWheel_img, -wheelAngle_deg)
    wheelRect = rotatedWheel.get_rect(center=(float(wheelCentre[0]), float(wheelCentre[1])))

    
    screen.blit(bodyDyngraph_img, (0,0))
    pygame.draw.circle(screen, (255, 0, 0), angleDotPos(i), 5)
    pygame.draw.circle(screen, (255, 0, 0), wheelDotPos(i), 5)
    cubeface = pygame.draw.polygon(screen, (214, 40, 40), cubePoints)
    screen.blit(rotatedWheel, wheelRect)
    vertline = pygame.draw.line(screen, (0, 48, 73), (pivotPosition), (pivotPosition + (0, -2*L*pixelscale)))
    ProgressBar = pygame.draw.polygon(screen, (252, 191, 73), ((0, scH), (0, scH - 50), ((i/len(Arr_timepoints))*scW, scH-50), ((i/len(Arr_timepoints))*scW, scH)))

    


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()

    delta_time = clock.tick(60) / 1000
    delta_time = max(0.001, min(0.1, delta_time))
    i +=  int(0.5*delta_time / dt)
    if i >= len(Arr_timepoints):
        i = 0
        print("loop")



pygame.quit()