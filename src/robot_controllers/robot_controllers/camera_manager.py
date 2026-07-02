#!/usr/bin/env python3
"""
Nodo ROS2 sul ROBOT: avvia/ferma il sender video (stream_sender.sh) su comando.

Comandato da PARAMETRI (li setta la plancia sul controller, via parameter service):
  enabled (bool)  -> true = avvia il sender, false = ferma
  host    (str)   -> IP/nome del CONTROLLER dove mandare il video (lo passa la plancia:
                     il suo IP attuale -> niente IP scritti a mano, robusto ai cambi)
  port, codec, bitrate, width, height -> passati al sender

Cosi' "Avvia Video" sulla plancia accende anche il sender qui, senza SSH manuale.
Il video resta un piano dati SEPARATO da ROS (lo stream va su UDP, non su DDS);
qui ROS serve solo a fare da TELECOMANDO del sender.
"""

import os
import signal
import subprocess

import rclpy
from rclpy.node import Node

SENDER = os.path.expanduser(
    os.environ.get("ROBOTHEX_CAMERA_SENDER", "~/robothex_ws/camera/stream_sender.sh"))


class CameraManager(Node):
    def __init__(self):
        super().__init__("camera_manager")
        self.declare_parameter("enabled", False)
        self.declare_parameter("host", "")          # IP/nome del controller (lo passa la plancia)
        self.declare_parameter("port", 5000)
        self.declare_parameter("codec", "h264")
        self.declare_parameter("bitrate", 1000000)
        self.declare_parameter("width", 960)
        self.declare_parameter("height", 540)

        self.proc = None
        self.create_timer(1.0, self._tick)          # riconcilia stato desiderato vs reale
        self.get_logger().info(f"camera_manager avviato (sender: {SENDER})")

    def _p(self, name):
        return self.get_parameter(name).value

    def _running(self):
        return self.proc is not None and self.proc.poll() is None

    def _tick(self):
        want = bool(self._p("enabled")) and bool(str(self._p("host")))
        if want and not self._running():
            self._start()
        elif not want and self._running():
            self._stop()

    def _start(self):
        host = str(self._p("host"))
        port = str(self._p("port"))
        if not os.path.exists(SENDER):
            self.get_logger().error(f"sender non trovato: {SENDER}")
            return
        env = {**os.environ,
               "CODEC": str(self._p("codec")),
               "BITRATE": str(self._p("bitrate")),
               "WIDTH": str(self._p("width")),
               "HEIGHT": str(self._p("height"))}
        try:
            self.proc = subprocess.Popen(["bash", SENDER, host, port],
                                         env=env, preexec_fn=os.setsid)
            self.get_logger().info(f"sender avviato -> {host}:{port}")
        except Exception as exc:                     # noqa: BLE001
            self.get_logger().error(f"avvio sender fallito: {exc}")
            self.proc = None

    def _stop(self):
        if self.proc is not None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError, OSError):
                pass
            self.proc = None
            self.get_logger().info("sender fermato")


def main(args=None):
    rclpy.init(args=args)
    node = CameraManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
