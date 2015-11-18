import time
import qi
import utils
import random
from functools import partial


class Grasping():

    def __init__(self, box):
        self.box = box
        self.sesh = box.session()
        self.uuid = box.packageUid()
        self.logger = box.logger

        # ======= services
        self.motion = self.sesh.service("ALMotion")
        self.ba = self.sesh.service("ALBasicAwareness")
        self.bm = self.sesh.service("ALBehaviorManager")
        self.posture = self.sesh.service('ALRobotPosture')
        self.dialog = self.sesh.service('ALDialog')
        self.memory = self.sesh.service('ALMemory')
        self.animspeech = self.sesh.service('ALAnimatedSpeech')
        self.ap = self.sesh.service('ALAudioPlayer')
        self.life = self.sesh.service('ALAutonomousLife')

        # ======== state
        self.fell = False
        self.restore_fall_safeguard = False
        self.run = False
        self.missed_count = 0

        # ms timeout before we give up waiting to grasp the object
        self.wait_duration = 65 * 1000

        # ms timeout before we give up trying to return the object to the human
        self.return_duration = 15 * 1000

        self.dialog.activateTopic('Grasping')
        self.animspeech.setBodyLanguageModeFromStr('disabled')

    def on_unload(self):
        """Unload the behavior."""
        if self.run:  # don't call stop_all a second time
            self.stop_all()

        # stop all in-progress grasping-related animations
        behs = self.bm.getRunningBehaviors()
        self.logger.debug('behaviors running: {}'.format(behs))
        for beh in behs:
            if self.uuid + '/animations' in beh:
                self.logger.warning('STOP behavior: ' + beh)
                qi.async(self.bm.stopBehavior, beh)

        if not self.fell:
            self.motion.setStiffnesses('RHand', 0.0)
            time.sleep(0.3)  # drop the object if it's still grasped
            self.motion.setStiffnesses('RHand', 0.8)
            qi.async(self.motion.closeHand, 'RHand')
            qi.async(self.motion.closeHand, 'LHand')

        if self.restore_fall_safeguard:
            self.life.setSafeguardEnabled('RobotFell', True)

    def stop_all(self):
        """Stop all active processes and handle fall."""
        self.run = False

        # Stop all processes, cancel promises, disconnect callbacks
        try:
            utils.stop_monitoring_arm_move()
            utils.stop_monitoring_grasp_status()
            utils.stop_periodic_say()
        except AttributeError:
            pass
        try:
            self.thinking_sound_fut_1.cancel()
        except (AttributeError, RuntimeError):
            pass
        try:
            self.thinking_sound_fut_2.cancel()
        except (AttributeError, RuntimeError):
            pass
        try:
            self.grasp_object.setCanceled()
            self.grasp_object = None
        except (AttributeError, RuntimeError):
            pass
        try:
            self.stop_sub.signal.disconnect(self.stop_sub_id)
        except:
            pass
        try:
            self.fall_sub.signal.disconnect(self.fall_sub_id)
        except:
            pass

        # Handle a fall: quit app
        if self.fell:
            self.logger.debug('handling fall...')
            time.sleep(3)
            self.stand_up(3)
            self.box.onStopped(1)

        # UGH: be robust to launching from Choregraphe without life
        elif self.memory.getData('AutonomousLife/State') == 'disabled':
            self.motion.setBreathEnabled('Legs', False)
            self.motion.setIdlePostureEnabled('Body', True)
            self.stand_up(3)

    def quit(self, value):
        self.logger.warning('quit')
        self.box.onStopped(1)

    def stand_up(self, n_tries):
        """Try to go to posture Stand n_tries times."""
        self.posture.setMaxTryNumber(n_tries)
        if not self.posture.goToPosture('Stand', 0.80):
            raise RuntimeError("Could not Stand after {} tries!".format(n_tries))

    def say_and_animate(self, tag, topic, anim_beh_name):
        """Simultaneously say tag in topic and run behavior anim_beh_name."""
        if self.run:
            fut = qi.async(self.dialog.gotoTag, tag, topic)
            self.bm.runBehavior(self.uuid + '/animations/' + anim_beh_name)
            fut.wait()

    def run_anim(self, name, forced=False):
        """Run grasping animation behavior."""
        if self.run or forced:
            self.logger.debug('run anim: {}'.format(name))
            self.bm.runBehavior(self.uuid + '/animations/{}'.format(name))

    def stop_anim(self, name):
        """Stop grasping animation behavior."""
        self.logger.debug('stop anim: {}'.format(name))
        self.bm.stopBehavior(self.uuid + '/animations/{}'.format(name))

    def ask_for_object(self):
        """Ask the user to put an object into Nao's hand."""
        self.logger.info('Ask for object')
        if not self.run:
            return
        self.say_and_animate('give', 'Grasping', 'ExtendHand')
        time.sleep(0.5)

    def wait_for_object(self):
        """Wait for the hand to grasp_demo an object.

        1. detect movement of the right arm
        2. nearly close the right hand
        3. check if an object is in the hand continuously
        4a. if it is, fulfill object_dropped
        4b. if it is not, say 'missed' and start over
        """
        self.logger.info('Wait for object')
        if not self.run:
            return

        def grasp_status_changed(stat):
            self.logger.debug('call grasp_status_changed: {}'.format(stat))
            if not self.run:
                return
            utils.stop_periodic_say()
            if stat == 'present':
                self.motion.stiffnessInterpolation("RArm", 1.0, 0.5)
                self.say_and_animate('thanks', 'Grasping', 'ThankYou')
                self.grasp_object.setValue(1)
            elif stat == 'absent':
                self.say_and_animate('missed', 'Grasping', 'Missed')
                self.run_anim('OpenHand')
                self.motion.stiffnessInterpolation("RArm", 1.0, 0.5)
                self.missed_count += 1
                if self.missed_count >= 3:
                    self.grasp_object.setCanceled()
                else:
                    self.run_anim('ExtendHand')
                    self.wait_for_object()

        def close_and_check_hand(value):
            self.logger.debug('call close_and_check_hand')
            if not self.run:
                return
            utils.nearly_close_hand(self, 'RHand')
            signal = utils.start_monitoring_grasp_status(self.sesh, 'moving')
            signal.connect(grasp_status_changed)

        utils.start_periodic_say(self.sesh, 'waiting', 'Grasping', 15)
        signal = utils.start_monitoring_arm_move(self.sesh, 'R')
        signal.connect(close_and_check_hand)

    def look_at_object(self):
        """Look at the object in Nao's hand.
        If we lose the object while we're inspecting it...
        - new grasp_status is 'absent'
        - we stop the animation and say something about losing it
        - restart the grasp demo
        Otherwise wait for the animation to finish (fut.wait)
        """
        self.logger.debug('call look_at_object')
        if not self.run:
            return
        object_dropped = qi.Promise()
        self.thinking_sound_fut_1 = None
        self.thinking_sound_fut_2 = None

        def handle_object_dropped(object_dropped, stat):
            """Handle the object being dropped."""
            self.logger.debug('call handle_object_dropped: {}'.format(stat))
            if not self.run:
                return
            if stat == 'absent':
                self.logger.warning('grasped object was dropped!')
                object_dropped.setValue(1)

        utils.nearly_close_hand(self, 'RHand')
        signal = utils.start_monitoring_grasp_status(self.sesh, 'present')
        signal.connect(partial(handle_object_dropped, object_dropped))
        self.anim = random.randint(1, 3)

        def inspect_object(object_dropped):
            """Observe the object with ooohs and ahhhs."""
            if not self.run:
                return
            self.thinking_sound_fut_1 = utils.play_rand_thinking_sound(self, 1.5)
            self.thinking_sound_fut_2 = utils.play_rand_thinking_sound(self, 5)
            self.run_anim('ObserveObject{}'.format(self.anim))
            try:
                object_dropped.setValue(0)
            except RuntimeError:
                self.logger.warning('"ObserveObject" animation was canceled ' +
                                    'because object was dropped.')

        qi.async(inspect_object, object_dropped)

        if object_dropped.future().value() == 1:  # dropped the object
            self.thinking_sound_fut_1.cancel()
            self.thinking_sound_fut_2.cancel()
            self.stop_anim('ObserveObject{}'.format(self.anim))
            self.say_and_animate('ohno', 'Grasping', 'LostIt')
            return False
        else:  # done looking at the object
            utils.stop_monitoring_grasp_status()
            return True

    def return_object(self):
        """Hand the object back to the user.

        Extend the arm out toward the user and (A) wait for the arm to move and
        (B) close the fingers and detect no object. Whichever happens first
        stops the other task. Finally open the hand and then joke.
        """
        self.logger.debug('call return_object')
        if not self.run:
            return
        self.say_and_animate('done', 'Grasping', 'ReturnObject')  # offer object
        time.sleep(1)  # try to ensure that arm monitoring doesn't false positive...
        object_dropped = qi.Promise()
        fut = object_dropped.future()

        # 1. Register a callback to arm movement and start detecting arm movement
        def handle_arm_moved(object_dropped, value):
            self.logger.debug('call handle_arm_moved')
            if not self.run:
                return
            utils.stop_monitoring_grasp_status()
            object_dropped.setValue(1)

        signal = utils.start_monitoring_arm_move(self.sesh, 'R')
        signal.connect(partial(handle_arm_moved, object_dropped))

        # 2. asynchronously close the hand and keep checking if its empty
        def grasp_status_changed(object_dropped, stat):
            self.logger.debug('call grasp_status_changed: {}'.format(stat))
            if not self.run:
                return
            utils.nearly_close_hand(self, 'RHand')
            if stat == 'absent':
                utils.stop_monitoring_arm_move()
                object_dropped.setValue(1)

        signal = utils.start_monitoring_grasp_status(self.sesh, 'present')
        signal.connect(partial(grasp_status_changed, object_dropped))

        # proceed when one of the above two has finished
        try:
            fut.value(self.return_duration)
            self.run_anim('OpenHand')
            time.sleep(1)  # for continuity
            self.say_and_animate('giveback', 'Grasping', 'DontForget')
            time.sleep(1)  # for continuity
        except RuntimeError:
            self.logger.warning("give up on returning the grasped object!")
            self.run_anim('OpenHand')
            self.box.onStopped(1)

    def grasp_demo(self):
        """Control flow."""
        self.logger.debug('call grasp_demo')
        if utils.too_hot(self):
            return
        self.grasp_object = qi.Promise()
        self.ask_for_object()
        self.wait_for_object()
        try:
            self.logger.debug('waiting on grasp_object promise...')
            self.grasp_object.future().value(self.wait_duration)
            if self.look_at_object():
                self.return_object()
                self.box.onStopped(1)
            else:
                self.grasp_demo()
        except (RuntimeError, AttributeError) as e:
            self.logger.warning('quit grasping: {}'.format(e))
            self.box.onStopped(1)

    def main(self):
        self.run = True
        self.fall_sub, self.fall_sub_id = utils.detect_falls(self)
        self.stop_sub = self.memory.subscriber('Grasping/Stop')
        self.stop_sub_id = self.stop_sub.signal.connect(self.quit)
        self.stand_up(3)
        self.ba.setEngagementMode('SemiEngaged')
        self.ba.setTrackingMode('Head')
        self.motion.setIdlePostureEnabled('Body', False)
        self.motion.setBreathEnabled('Legs', True)
        time.sleep(0.3)
        self.grasp_demo()
