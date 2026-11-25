"""Persistent OpenCV operator GUI for target selection and gripper decisions."""

import threading

import numpy as np
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

from .camera_capture import decode_color_image


class OperatorGUI:
    """Keep the live camera window open through picking and release approval."""

    def __init__(self, app):
        import cv2

        self.cv2 = cv2
        self.app = app
        self.window = 'UR3 Pick and Release'
        self.latest = [None]
        self.current_image = None
        self.lock = threading.Lock()
        self.corners = []
        self.cursor = [0, 0]
        self.buttons = []
        self.action = None
        self.mode = 'select'
        self.closed = False
        self.subscription = app.node.create_subscription(
            Image, app.settings['color_image_topic'], self._image_callback,
            qos_profile_sensor_data)
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window, self._mouse_callback)

    def _image_callback(self, msg):
        with self.lock:
            self.latest[0] = msg

    def _mouse_callback(self, event, x, y, flags, userdata):
        self.cursor[:] = [x, y]
        if event != self.cv2.EVENT_LBUTTONDOWN:
            if event == self.cv2.EVENT_RBUTTONDOWN and self.mode == 'select' and self.corners:
                self.corners.pop()
            return
        if self.mode == 'select':
            if self.current_image is not None and len(self.corners) < 2:
                self.corners.append((x, y))
            return
        for x0, y0, x1, y1, value in self.buttons:
            if x0 <= x <= x1 and y0 <= y <= y1:
                self.action = value
                return

    def _update_image(self):
        with self.lock:
            msg = self.latest[0]
        if msg is not None:
            self.current_image = decode_color_image(msg)

    def _render(self, prompt, buttons=()):
        cv2 = self.cv2
        self._update_image()
        if self.current_image is None:
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(image, 'Waiting for live color image...', (18, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            image = self.current_image.copy()
        if self.mode == 'select' and self.corners:
            end = self.corners[1] if len(self.corners) == 2 else tuple(self.cursor)
            cv2.rectangle(image, self.corners[0], end, (255, 220, 0), 2)
            cv2.circle(image, self.corners[0], 4, (255, 40, 40), -1)
            if len(self.corners) == 2:
                cv2.circle(image, self.corners[1], 4, (255, 40, 40), -1)
        cv2.rectangle(image, (0, 0), (image.shape[1], 68), (25, 25, 25), -1)
        cv2.putText(image, prompt, (12, 27), cv2.FONT_HERSHEY_SIMPLEX,
                    .58, (255, 255, 255), 2, cv2.LINE_AA)
        if not buttons:
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        self.buttons = []
        height, width = image.shape[:2]
        gap, margin, button_height = 12, 12, 48
        count = len(buttons)
        button_width = (width - 2 * margin - gap * (count - 1)) // count
        top = max(72, height - button_height - margin)
        for index, (label, value) in enumerate(buttons):
            left = margin + index * (button_width + gap)
            right = left + button_width
            self.buttons.append((left, top, right, top + button_height, value))
            cv2.rectangle(image, (left, top), (right, top + button_height),
                          (40, 140, 40) if index == 0 else (90, 75, 65), -1)
            cv2.rectangle(image, (left, top), (right, top + button_height),
                          (235, 235, 235), 1)
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .52, 1)[0]
            tx = left + max(5, (button_width - text_size[0]) // 2)
            ty = top + (button_height + text_size[1]) // 2
            cv2.putText(image, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        .52, (255, 255, 255), 1, cv2.LINE_AA)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    def _window_is_open(self):
        try:
            return self.cv2.getWindowProperty(self.window, self.cv2.WND_PROP_VISIBLE) >= 1
        except self.cv2.error:
            return False

    def _pump(self, prompt, buttons=()):
        self.cv2.imshow(self.window, self._render(prompt, buttons))
        key = self.cv2.waitKey(30) & 0xFF
        if key == 27 or not self._window_is_open():
            return 'cancel'
        return key

    def select_target_rectangle(self):
        """Return a selected mask while leaving the GUI window open."""
        self.mode = 'select'
        self.corners.clear()
        while True:
            key = self._pump('LIVE | Click two opposite corners; Enter accepts; U undo; R reset; Esc cancels')
            if key == 'cancel':
                return None
            if key in (10, 13) and len(self.corners) == 2 and self.current_image is not None:
                x0, x1 = sorted((self.corners[0][0], self.corners[1][0]))
                y0, y1 = sorted((self.corners[0][1], self.corners[1][1]))
                if x0 == x1 or y0 == y1:
                    continue
                mask = np.zeros(self.current_image.shape[:2], dtype=np.uint8)
                self.cv2.rectangle(mask, (x0, y0), (x1, y1), 1, thickness=-1)
                self.mode = 'idle'
                return mask.astype(bool)
            if key in (ord('u'), ord('U'), 8, 127) and self.corners:
                self.corners.pop()
            elif key in (ord('r'), ord('R')):
                self.corners.clear()

    def confirm_execution(self):
        """Ask in the camera GUI before sending arm or gripper commands."""
        self.mode = 'buttons'
        self.action = None
        choices = (('EXECUTE PICK', 'yes'), ('CANCEL', 'no'))
        while self.action is None:
            event = self._pump('Plan ready | Click EXECUTE PICK or CANCEL', choices)
            if event == 'cancel':
                return False
            if event in (ord('y'), ord('Y')):
                self.action = 'yes'
            elif event in (ord('n'), ord('N')):
                self.action = 'no'
        return self.action == 'yes'

    def confirm_release(self):
        """Wait for a GUI command before opening the gripper at the start pose."""
        self.mode = 'buttons'
        self.action = None
        choices = (('OPEN GRIPPER / RELEASE', 'release'), ('ABORT / KEEP CLOSED', 'abort'))
        while self.action is None:
            event = self._pump('Returned to start | Click RELEASE or ABORT', choices)
            if event == 'cancel':
                return False
            if event in (ord('o'), ord('O')):
                self.action = 'release'
            elif event in (ord('q'), ord('Q')):
                self.action = 'abort'
        return self.action == 'release'

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.app.node.destroy_subscription(self.subscription)
        try:
            self.cv2.destroyWindow(self.window)
            self.cv2.waitKey(1)
        except self.cv2.error:
            pass
