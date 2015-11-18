import qi
import time
import random

arLastActuatorValue = [-1000.0, -1000.0]
detect_grasped_task = qi.PeriodicTask()
on_grasped_status_change = qi.Signal()
detect_arm_move_task = qi.PeriodicTask()
on_arm_moved = qi.Signal()
ask_for_object_task = qi.PeriodicTask()
tapped_to_quit = False

history = [False, False, False, False, False]


class JointMove:

    def __init__(self, strJointName, session):
        """Record the activity of a joint to know if it's moving or not and
        the side (L, R) of the move.
        :param strJointName: str of the joint name e.g. ShoulderPitch
        :param session: the qi session
        """
        self.strJointName = strJointName
        self.session = session
        self.mem = session.service("ALMemory")
        self.motion = session.service('ALMotion')
        self.strStmJointNameSensor = ("Device/SubDeviceList/" + strJointName +
                                      "/Position/Sensor/Value")
        self.strStmJointNameActuator = ("Device/SubDeviceList/" + strJointName +
                                        "/Position/Actuator/Value")
        self.strStmJointNameStiffness = ("Device/SubDeviceList/" + strJointName +
                                         "/Hardness/Actuator/Value")
        self.rDiffThreshold = 0.005  # threshold above which a joint is considered moved
        self.reset()

    def ensureJointIsSoft(self):
        rNewValueHardness = self.mem.getData(self.strStmJointNameStiffness, 0)
        rMin = 0.30  # below this value, arms couldn't move to the head level
        if "Elbow" in self.strJointName:
            rMin = 0.15
        if "Wrist" in self.strJointName:
            rMin = 0.15
        if rNewValueHardness > rMin:
            self.motion.setStiffnesses(self.strJointName, rMin)

    def update(self):
        """Determine if the joint was moved.

        Return 1 if sensed movement in the positive direction, -1 in the negative.
        Return 100 if there was a move command articulated by the robot.
        Return 0 if there was no movement detected.
        """
        rNewValueSensor = self.mem.getData(self.strStmJointNameSensor, 0)
        rNewValueActuator = self.mem.getData(self.strStmJointNameActuator, 0)
        rNewValueHardness = self.mem.getData(self.strStmJointNameStiffness, 0)

        print('{} stiffness: {}'.format(self.strJointName,
                                        self.motion.getStiffnesses(self.strJointName)))

        # if joint wasn't actuated by the robot but IS moving...
        if (abs(rNewValueActuator - self.rLastActuatorValue) < 0.005 or rNewValueHardness < 0.001):
            rDiff = rNewValueSensor - self.rLastValueSensor
            self.rLastValueSensor = rNewValueSensor

            if abs(rDiff) > self.rDiffThreshold:
                print('{} rDiff {} > {}'.format(self.strJointName,
                                                abs(rDiff),
                                                self.rDiffThreshold))
                if rDiff > 0:
                    return 1  # arm moved in positive direction
                return -1  # arm moved in negative direction
            else:
                print('{} rDiff {} <= {}'.format(self.strJointName,
                                                 abs(rDiff),
                                                 self.rDiffThreshold))
        # the joint was actuated by the robot: update the sensor and actuator values
        else:
            # slowly update value (but not too slowly)
            self.rLastActuatorValue = self.rLastActuatorValue * 0.2 + rNewValueActuator * 0.8
            # update sensor so it won't trigger next frame
            self.rLastValueSensor = rNewValueSensor
            return 100
        return 0

    def reset(self):
        self.ensureJointIsSoft()
        self.rLastValueSensor = self.mem.getData(self.strStmJointNameSensor, 0)
        self.rLastActuatorValue = self.mem.getData(self.strStmJointNameActuator, 0)


def detect_arm_move(session, detectors):
    """Detect if an arm joint has been moved since last check.
    - Ignore actuated movements.
    - Ignore sensed movements in the context of actuated movement of other joints. In the case that
      one joint is actuated and one is not, the latter could be detected as moving otherwise
    """
    print('call detect_arm_move')
    print('History: {}'.format(history))
    detectors = {d.strJointName: d.update() for d in detectors}
    if 100 not in detectors.values():  # no movements were actuated
        for name, val in detectors.items():
            # ignore non-movement (0)
            if val == 1:
                print('{}: sensed positive move'.format(name))
            if val == -1:
                print('{}: sensed negative move'.format(name))
            if val == 1 or val == -1:
                if True in history:  # last timestep was actuated movement
                    print('Not signaling because recent history includes actuated movment!!')
                else:
                    print('Signaling {}'.format(name))
                    on_arm_moved(val)
                    stop_monitoring_arm_move()
                    break
        history.append(False)
    else:
        print('Actuated movement detected, ignoring all joint movement')
        history.append(True)
    history.pop(0)  # remove most historic


def start_monitoring_arm_move(session, arm_side, r_period=0.1):
    """Monitor an arm for movement.
    :param arm_side: 'R' or 'L'
    :param r_period: frequeny of arm movement checking in seconds
    :param r_threshold: a dictionary or joint: threshold pairs. Above this
    threshold the joint counts as having moved.
    """
    print('START monitoring ARM MOVEMENT...')
    global detect_arm_move_task
    joints = {'RShoulderRoll': 0.002,
              'RShoulderPitch': 0.002,
              'RElbowRoll': 0.004,
              'RElbowYaw': 0.004,
              'RWristYaw': 0.002}
    detectors = list()
    for joint, threshold in joints.items():
        j = JointMove(joint, session)
        j.ensureJointIsSoft()
        j.rDiffThreshold = threshold
        detectors.append(j)

    # log the arm joint temperatures
    mem = session.service('ALMemory')
    for joint in joints.keys():
        temp = mem.getData('Device/SubDeviceList/{}/Temperature/Sensor/Value'.
                           format(joint))
        print('{} temp: {}'.format(joint, temp))

    def cb():
        detect_arm_move(session, detectors)
    detect_arm_move_task = qi.PeriodicTask()
    detect_arm_move_task.setCallback(cb)
    detect_arm_move_task.setUsPeriod(int(r_period * 1000000))
    detect_arm_move_task.start(True)
    return on_arm_moved


def stop_monitoring_arm_move():
    print('STOP monitoring ARM MOVEMENT')
    detect_arm_move_task.stop()
    on_arm_moved.disconnectAll()


def start_periodic_say(session, tag, topic, period):
    """Say proposal at tag in topic every period seconds."""
    print('call start_periodic_say')
    global ask_for_object_task
    dialog = session.service('ALDialog')

    def cb():
        dialog.gotoTag(tag, topic)
    ask_for_object_task = qi.PeriodicTask()
    ask_for_object_task.setCallback(cb)
    ask_for_object_task.setUsPeriod(period * 1000000)
    ask_for_object_task.start(False)


def stop_periodic_say():
    print('call stop_periodic_say')
    ask_for_object_task.stop()


def nearly_close_hand(grasping, hand):
    """Close the hand around the object.
    :param session: the qi session
    :param hand: str RHand or LHand
    """
    print('call nearly_close_hand')
    if not grasping.run:
        print('DONT close the hand')
        return
    baseVersion = grasping.memory.getData("RobotConfig/Body/BaseVersion")
    version = baseVersion[1:]

    bV32 = (version == '3.2') or (version == '3Plus')
    rValue = 0.15 if bV32 else 0.07
    grasping.motion.angleInterpolation(hand, rValue, 0.5, True)
    # on some robot, the moving of hand leave without the move being really finished
    time.sleep(0.2)


def is_object_in_hand(session):
    """Detect if an object is in the hand.

    Return a list where index 0 represents the left hand and 1 the right hand:
    Return 0 if nothing is in the hand
    Return 1 if the hand is moving
    Return 2 if something is in the hand
    """
    print('call is_object_in_hand')
    global arLastActuatorValue
    mem = session.service('ALMemory')
    arActuatorValue = [0, 0]
    arSensorValue = [0, 0]
    arConsumpValue = [0, 0]

    (arActuatorValue[0], arActuatorValue[1], arSensorValue[0],
     arSensorValue[1], arConsumpValue[0], arConsumpValue[1]) = mem.getListData(
        [
            "Device/SubDeviceList/LHand/Position/Actuator/Value",
            "Device/SubDeviceList/RHand/Position/Actuator/Value",
            "Device/SubDeviceList/LHand/Position/Sensor/Value",
            "Device/SubDeviceList/RHand/Position/Sensor/Value",
            "Device/SubDeviceList/LHand/ElectricCurrent/Sensor/Value",
            "Device/SubDeviceList/RHand/ElectricCurrent/Sensor/Value",
        ])

    anReturn = [0, 0]
    for i in range(2):
        if abs(arLastActuatorValue[i] - arActuatorValue[i]) > 0.01:  # did the hand move?
            anReturn[i] = 1  # moving
            arLastActuatorValue[i] = arActuatorValue[i]
        else:  # not moving
            # compute the difference between the actuator and sensor values
            rDiff = arSensorValue[i] - arActuatorValue[i] + arConsumpValue[i] / 70
            if rDiff > 0.009:
                anReturn[i] = 2  # object in hand
    return anReturn


def get_grasp_status_right(session, old_status):
    """Get the grasp status of the right hand.
    :param session: the qi session
    :param old_status: str 'absent', 'present', or 'moving' to compare to

    If old_status is 'moving' then we fire the signal for 'absent' or 'present', if not, we fire
    for the opposite status only (e.g. old_status: 'absent', fire for 'present' only)
    """
    print('call get_grasp_status_right')
    d = {2: 'present', 0: 'absent', 1: 'moving'}
    l_hand, r_hand = is_object_in_hand(session)
    status_l, status_r = d[l_hand], d[r_hand]
    print('left hand: {}, right hand: {}'.format(status_l, status_r))
    if status_r != old_status and status_r != 'moving':
        print('fire on_grasped_status_change with value: {}'.format(status_r))
        on_grasped_status_change(status_r)
        stop_monitoring_grasp_status()


def start_monitoring_grasp_status(session, old_status):
    """Monitor for a new grasp status."""
    print('START monitoring GRASP STATUS...')
    global detect_grasped_task

    def ggsr():
        get_grasp_status_right(session, old_status)
    detect_grasped_task = qi.PeriodicTask()
    detect_grasped_task.setCallback(ggsr)
    detect_grasped_task.setUsPeriod(int(0.2 * 1000000))
    detect_grasped_task.start(True)
    return on_grasped_status_change


def stop_monitoring_grasp_status():
    print('STOP monitoring GRASP STATUS')
    detect_grasped_task.stop()
    on_grasped_status_change.disconnectAll()


def play_rand_thinking_sound(grasping, delay):
    """Play a random thinking expression after delay seconds."""
    grasping.logger.debug('call play_rand_thinking_sound')
    if not grasping.run:
        return
    fp = random.choice([
        'enu_ono_exclamation_interested_01',
        'enu_ono_exclamation_interested_02',
        'enu_ono_exclamation_interested_03',
        'enu_ono_exclamation_interested_04',
        'enu_ono_exclamation_interested_05',
        'enu_ono_exclamation_surprised_13',
        'enu_ono_exclamation_surprised_12',
        'enu_ono_exclamation_wondering_01',
        'enu_ono_exclamation_wondering_02',
        'enu_ono_exclamation_wondering_03',
        'enu_ono_exclamation_wondering_04',
        'enu_ono_exclamation_wondering_05',
        'enu_ono_hesitation_long_01',
        'enu_ono_hesitation_long_02',
        'enu_ono_humph',
        'enu_ono_humphh',
        'enu_ono_humphh_02',
        'enu_ono_mmh_pleasure_01',
        'enu_ono_mmh_pleasure_02',
        'enu_ono_mmh_pleasure_03',
        'enu_ono_mmh_pleasure_04',
        'enu_ono_mmh_pleasure_05'
    ])
    fut = qi.async(grasping.ap.playSoundSetFile,
                   'Aldebaran',
                   fp,
                   delay=int(delay * 1000000))  # to microseconds
    return fut


def detect_falls(grasping):
    if grasping.life.isSafeguardEnabled("RobotFell"):
        grasping.life.setSafeguardEnabled("RobotFell", False)
        grasping.restore_fall_safeguard = True

    def handle_fall(value):
        grasping.motion.setStiffnesses('Body', 0.0)
        grasping.fell = True
        grasping.stop_all()

    sub = grasping.memory.subscriber('ALMotion/RobotIsFalling')
    sub_id = sub.signal.connect(handle_fall)
    return sub, sub_id


def too_hot(grasping):
    if grasping.memory.getData('Launchpad/TemperatureStatus') == 'Hot':
        joints = ['RShoulderRoll', 'RShoulderPitch', 'RElbowRoll', 'RElbowYaw',
                  'RWristYaw', 'RHand']
        hot_devices = grasping.memory.getData('HotDeviceDetected')
        if hot_devices:
            for joint in joints:
                if joint in hot_devices:
                    grasping.logger.warning('{} is too hot!'.format(joint))
                    grasping.dialog.gotoTag('heat', 'Grasping')
                    grasping.box.onStopped(1)
                    return True
    return False
