"""Narrator: prints incoming narration to the terminal and (optionally) speaks it.

The terminal print is intentionally bold/colored so it is obvious during the
demo. TTS is best-effort: if pyttsx3 is not installed (or audio is muted) the
node continues to function as a printer only.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import pyttsx3
    HAVE_TTS = True
except ImportError:
    HAVE_TTS = False


GREEN = '\033[1;32m'
RESET = '\033[0m'


class Narrator(Node):

    def __init__(self):
        super().__init__('narrator')
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('rate_wpm', 165)

        self.engine = None
        if HAVE_TTS and self.get_parameter('enable_tts').value:
            try:
                self.engine = pyttsx3.init()
                self.engine.setProperty('rate', int(self.get_parameter('rate_wpm').value))
                self.get_logger().info('TTS engine ready.')
            except Exception as exc:
                self.get_logger().warn(f'TTS unavailable: {exc}')
                self.engine = None
        else:
            self.get_logger().info('TTS disabled (pyttsx3 missing or disabled by param).')

        self.create_subscription(String, '/tour/narration', self.on_narration, 10)

    def on_narration(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        sys.stdout.write(f'\n{GREEN}>> {text}{RESET}\n')
        sys.stdout.flush()
        if self.engine is not None:
            try:
                self.engine.say(text)
                self.engine.runAndWait()
            except Exception as exc:
                self.get_logger().warn(f'TTS error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = Narrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
