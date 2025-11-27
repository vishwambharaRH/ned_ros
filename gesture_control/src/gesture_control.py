#!/usr/bin/env python3
import rospy
import cv2
import mediapipe as mp
import numpy as np
from niryo_robot_arm_commander.msg import CommandJog

# MediaPipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)
mpDraw = mp.solutions.drawing_utils

def find_working_camera():
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                return cap, i
            cap.release()
    return None, -1

class IKGestureTeleop:
    def __init__(self):
        rospy.init_node("gesture_ik_teleop", anonymous=True)

        self.pub = rospy.Publisher(
            "/niryo_robot_arm_commander/send_jog_command_ik",
            CommandJog,
            queue_size=10
        )

        # How big a pose shift per cycle (in meters / radians)
        # tune these:
        self.scale_x = 0.10   # full camera width -> 10 cm in X (left-right)
        self.scale_y = 0.10   # full camera height -> 10 cm in Y (forward-back) (optional)
        self.scale_z = 0.08   # full camera height -> 8 cm in Z (up-down)

        # Per-cycle max limits (safety):
        self.max_step_xyz = 0.06  # 6 cm per cycle max
        self.max_step_rot = 0.2   # radians per cycle for roll/pitch/yaw (if used)

        # Deadzone: ignore tiny movements
        self.deadzone = 0.03  # normalized camera units

        # Camera
        self.cap, self.cam_idx = find_working_camera()
        if self.cap is None:
            rospy.logerr("No camera found")
            exit(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        rospy.loginfo("IK Gesture teleop started (publishing POSE_SHIFT)")

    def publish_pose_shift(self, dx, dy, dz, droll=0.0, dpitch=0.0, dyaw=0.0):
        # clamp values to max_step
        dx = float(np.clip(dx, -self.max_step_xyz, self.max_step_xyz))
        dy = float(np.clip(dy, -self.max_step_xyz, self.max_step_xyz))
        dz = float(np.clip(dz, -self.max_step_xyz, self.max_step_xyz))
        droll = float(np.clip(droll, -self.max_step_rot, self.max_step_rot))
        dpitch = float(np.clip(dpitch, -self.max_step_rot, self.max_step_rot))
        dyaw = float(np.clip(dyaw, -self.max_step_rot, self.max_step_rot))

        msg = CommandJog()
        msg.cmd = CommandJog.POSE_SHIFT

        # Ensure float32 list
        msg.shift_values = np.array([dx, dy, dz, droll, dpitch, dyaw], dtype=np.float32).tolist()
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)

            info_text = "No hand"
            if result.multi_hand_landmarks:
                lm = result.multi_hand_landmarks[0].landmark
                mpDraw.draw_landmarks(frame, result.multi_hand_landmarks[0],
                                      mpHands.HAND_CONNECTIONS)
                cx = lm[9].x  # normalized 0..1
                cy = lm[9].y

                # Map camera coords to desired pose shifts:
                # - left/right (camera X) -> robot X (or robot Y depending on your frame)
                # - up/down (camera Y) -> robot Z (we'll map up/down to Z)
                # You may need to swap axes depending on camera orientation vs robot frame.

                # normalized deltas from center
                dx_norm = cx - 0.5   # left/right
                dz_norm = 0.5 - cy   # up/down

                # deadzone
                if abs(dx_norm) < self.deadzone and abs(dz_norm) < self.deadzone:
                    # small movement - don't publish
                    info_text = "In deadzone - no command"
                    self.publish_pose_shift(0.0, 0.0, 0.0)
                else:
                    # scale to meters
                    dx = dx_norm * self.scale_x
                    dz = dz_norm * self.scale_z

                    # optional: keep forward/back (y) zero or derive from hand depth if available
                    dy = 0.0

                    # show approximate values
                    info_text = f"dx:{dx:.3f} dz:{dz:.3f}"

                    # Publish pose shift (note: coordinate mapping may need swapping)
                    # Here we publish [x, y, z, roll, pitch, yaw]
                    self.publish_pose_shift(dx, dy, dz, 0.0, 0.0, 0.0)

                cv2.putText(frame, info_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # no hand - publish zero shift to keep controller happy (or skip if you prefer)
                self.publish_pose_shift(0.0, 0.0, 0.0)
                cv2.putText(frame, "No hand", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 165, 255), 2)

            cv2.imshow("IK Gesture Teleop", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()
        hands.close()

if __name__ == "__main__":
    IKGestureTeleop().run()

