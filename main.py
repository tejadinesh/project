import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot


tableId = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3],
                     p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)


######################################################### Load Object Here!!!!#############################################################################
p.setGravity(0,0,-10)
# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
#objectId = p.loadURDF("random_urdfs/001/001.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

xpos = 0.95
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, 0, ang])

object_path ="random_urdfs/039/039.urdf"
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])


######################################################### Load tray Here!!!!#############################################################################

tray_x = 1.0410235933378673
tray_y = 0.3



trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])



###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]

#hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
# handReading = [0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 
# 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 
# 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])




##################################################################### Input Value Here############################################################

handInitial = [0.2326145468048594, 0.9747814806939182, 0.2518634423442439, 0.23193006603927302, 0.9769450268418096, 0.3163226667061904, 0.5777248596455962, 0.5809453327205668, 0.23885031397582157, 0.5766268927131794, 0.5858004118505267, 0.17001200278724815, 0.9562858528175234, 0.28003826604443705, 0.16198186462920428, 0.9352424044074108, 0.26652931384559364, 0.1700655802441482, 0.9057901547794448, 0.2544338523490341, 0.17000014756782897, 0.9052093314078942, 0.25177817412627973, 0.17006871416075456, 1.570065288840273, 0.34110784992426363, 0.34078970902645883]
grasp_orientation = [1.553832769393921, 3.141592653589793, 0.43010997772216797]
grasp_palmPosition = [0.8732525756978429, 0.055015806253440678, -0.07526337400078762]
handClose = [1.0708599937467354, 1.304764613611965, 0.619773549749019, 1.0703832828560678, 1.3067417603152605, 0.33555103631026345, 1.4381560959548867, 1.2616455363595631, 0.5648498271467679, 1.436582210744105, 1.2695812897736867, 0.170003303935885, 1.5025437312320844, 1.421078804483358, 0.17000452627328955, 1.5031612693575223, 1.4191548032903407, 0.17002357490049955, 1.4657752353211257, 1.4518899892369137, 0.17004402220181591, 1.4654038118011785, 1.4491704438669586, 0.16999994860155265, 1.5699197746066897,0.3432860544128454, 0.3431814265441947]
pu_palmPosition = [0.9214736791793257, 0.03971578646451235, 0.25]
pu_orientation = [1.6199712753295898, 3.141592653589793, 0.26476311683654785]
final_palmPosition = [1.480235933378673, 0.7, 0.25]
final_orientation = [1.4876940250396729, 3.125212983289037, 0.23169374465942383]
handOpen = [0.23821481261801647, 0.9737148527322002, 0.1716816188602339, 0.2377723513400452, 0.9743239607179522, 0.16993902983741901, 0.5888487053914103, 0.5839635669101421, 0.17501053399946276, 0.5810220902708451, 0.5857809335683982, 0.17000986854417502, 0.9369811798288749, 0.2684059365443086, 0.1700102160727572, 0.9353802355149652, 0.2714017601010345, 0.16999999999999996, 0.9061223690907538, 0.26325648327022716, 0.16990499319154878, 0.906499158862867, 0.2521793652544818, 0.17000000419949454, 0.1698511222564677, 0.34088991507796007, 0.3406454651389435]

##################################################################################################################################################################################################
initial_palmPosition = [0.85, -0.05, 0.1]
initial_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

#initial_palmPosition = [initial_palmPosition[0]-0.1,initial_palmPosition[1]-0.05,initial_palmPosition[2]]
##################################################################################################################################################################################################

# write the code for step 9-12


''' Step 1: move robot to the initial position '''

# Parameters: 
	# arm: initial_palmPosition, initial_orientation
	# hand: handInitial


''' Step 2: move robot to the grasping position '''

# Parameters: 
	# arm: grasp_orientation, grasp_palmPosition


''' Step 3: grasp object '''

# Parameters: 
	# hand: handClose

''' Step 4: pick up  the object '''

# Parameters: 
	# arm: pu_palmPosition, pu_orientation

''' Step 4: move object to the tray '''

# Parameters: 
	# arm: final_palmPosition, final_orientation


''' Step 5: put the object in the tray '''

# Parameters: 
	# hand: handOpen
startPos = [handInitial[24], handInitial[25], handInitial[18], handInitial[19], handInitial[12], handInitial[13], handInitial[6], handInitial[7], handInitial[0], handInitial[1]]
closePos = [handClose[24], handClose[25], handClose[18], handClose[19], handClose[12], handClose[13], handClose[6], handClose[7], handClose[0], handClose[1]]

k = 0
while 1:
    k = k + 1

    # move palm to initial postion
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([initial_palmPosition[0], initial_palmPosition[1], initial_palmPosition[2]],
                         p.getQuaternionFromEuler([initial_orientation[0], initial_orientation[1], initial_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('reached initial position')
            break  
    # move hand to initial postion
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()  
        thumb(startPos[0], startPos[1]) 
        indexF(startPos[2], startPos[3])
        midF(startPos[4], startPos[5])
        ringF(startPos[6], startPos[7])
        pinkyF(startPos[8], startPos[9])
        
        
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('move hand to initial position')
            break    

    # move palm to grasp postion
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([grasp_palmPosition[0], grasp_palmPosition[1], grasp_palmPosition[2]],
                         p.getQuaternionFromEuler([grasp_orientation[0], grasp_orientation[1], grasp_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('reached grasp position')
            break  

    # hand close
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        
        indexF(closePos[2], closePos[3])
        midF(closePos[4], closePos[5])
        ringF(closePos[6], closePos[7])
        pinkyF(closePos[8], closePos[9])
        thumb(closePos[0], closePos[1])
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('hand close')
            break  

    # move palm to tray
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([final_palmPosition[0], final_palmPosition[1], final_palmPosition[2]],
                         p.getQuaternionFromEuler([final_orientation[0], final_orientation[1], final_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('reached tray position')
            break 
    
     # move hand to initial postion
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
         
        indexF(startPos[2], startPos[3]) 
        midF(startPos[4], startPos[5])
        ringF(startPos[6], startPos[7])
        pinkyF(startPos[8], startPos[9])
        thumb(startPos[0], startPos[1]) 
        
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('move hand to initial position')
            break  
    
p.disconnect()
print("disconnected")







