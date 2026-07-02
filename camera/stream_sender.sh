#!/usr/bin/env bash
#
# Streaming video FPV (lato ROBOT): Pi Camera v1 (OV5647) -> H.264 -> RTP/UDP al controller.
#
# Stack camera: LEGACY MMAL (bcm2835-v4l2), che espone /dev/video0 come device V4L2
# con frame gia' processati e — meglio ancora — sa dare H.264 encodato in HARDWARE
# dalla GPU (VideoCore). Piano dati SEPARATO da ROS2 (handbook sez. 6b).
#
# PREREQUISITI (una volta, sul robot):
#   - modulo legacy caricato al boot:
#       echo bcm2835-v4l2 | sudo tee /etc/modules-load.d/bcm2835-v4l2.conf
#   - utente nel gruppo video (accesso a /dev/video0):
#       sudo usermod -aG video $USER   (poi ri-login)
#   - GStreamer: gstreamer1.0-tools plugins-good plugins-bad libav
#
# USO:
#   ./stream_sender.sh <IP_CONTROLLER> [PORTA]
#   es:  ./stream_sender.sh 192.168.1.50 5000
#   (IP del controller = dove gira stream_receiver.sh; `hostname -I` sul controller)
#
# Variabili opzionali: WIDTH HEIGHT FPS DEV BITRATE
#   ENCODE=sw  -> forza encode SOFTWARE (openh264) se l'H.264 hardware non negozia.
set -euo pipefail

RECEIVER_HOST="${1:-${RECEIVER_HOST:-}}"
PORT="${2:-${PORT:-5000}}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
DEV="${DEV:-/dev/video0}"
BITRATE="${BITRATE:-3000000}"   # bit/s H.264 (cappa il flusso per stare nel WiFi)

if [ -z "${RECEIVER_HOST}" ]; then
  echo "Uso: $0 <IP_CONTROLLER> [PORTA]   (o export RECEIVER_HOST=...)" >&2
  exit 1
fi

if [ ! -e "${DEV}" ]; then
  echo "ERRORE: ${DEV} non esiste. Carica il modulo legacy: sudo modprobe bcm2835-v4l2" >&2
  exit 1
fi

echo "Streaming ${WIDTH}x${HEIGHT}@${FPS}  ->  ${RECEIVER_HOST}:${PORT}  (encode=${ENCODE:-hw})"

if [ "${ENCODE:-hw}" = "sw" ]; then
  # Fallback SOFTWARE: cattura YUY2 -> openh264enc.
  exec gst-launch-1.0 -v \
    v4l2src device="${DEV}" ! \
    "video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
    videoconvert ! \
    openh264enc bitrate="${BITRATE}" ! \
    h264parse ! \
    rtph264pay config-interval=1 pt=96 mtu=1400 ! \
    udpsink host="${RECEIVER_HOST}" port="${PORT}" sync=false
else
  # HARDWARE: la GPU encoda H.264 direttamente (il device espone video/x-h264).
  # extra-controls: cappa il bitrate e mette un keyframe ogni secondo (recupero rapido).
  # rtph264pay mtu=1400: pacchetti sotto la MTU -> niente frammentazione IP sul WiFi.
  # config-interval=1 reinvia SPS/PPS: un receiver che si collega dopo decodifica subito.
  exec gst-launch-1.0 -v \
    v4l2src device="${DEV}" extra-controls="controls,video_bitrate=${BITRATE},h264_i_frame_period=${FPS}" ! \
    "video/x-h264,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
    h264parse ! \
    rtph264pay config-interval=1 pt=96 mtu=1400 ! \
    udpsink host="${RECEIVER_HOST}" port="${PORT}" sync=false
fi
