#!/usr/bin/env bash
#
# Streaming video FPV (lato ROBOT): Pi Camera -> H.264 HARDWARE -> RTP/UDP al controller.
#
# Piano dati SEPARATO da ROS2 (vedi ROBOTHEX_HANDBOOK.md sez. 6b): NON usa topic
# immagine (saturano il WiFi). Encode H.264 in hardware sul Pi 4 (GPU VideoCore VI),
# la CPU resta libera per i nodi ROS.
#
# USO:
#   ./stream_sender.sh <IP_CONTROLLER> [PORTA]
#   es:  ./stream_sender.sh 192.168.1.50 5000
#   (IP del controller = dove gira stream_receiver.sh; trovalo con `hostname -I` sul controller)
#
# Variabili opzionali: WIDTH HEIGHT FPS BITRATE (bit/s).
#
# ⚠️  DA VERIFICARE SULL'HARDWARE (stack camera): questo usa `libcamerasrc` +
#     `v4l2h264enc` (encoder HW). Se la tua immagine e' vecchia potresti avere
#     `rpicamsrc`; se la camera e' su /dev/video* usa `v4l2src`. Vedi README.md.
set -euo pipefail

RECEIVER_HOST="${1:-${RECEIVER_HOST:-}}"
PORT="${2:-${PORT:-5000}}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"   # 4 Mbps: buon compromesso 720p30 su WiFi

if [ -z "${RECEIVER_HOST}" ]; then
  echo "Uso: $0 <IP_CONTROLLER> [PORTA]   (o export RECEIVER_HOST=...)" >&2
  exit 1
fi

echo "Streaming ${WIDTH}x${HEIGHT}@${FPS}  ${BITRATE} bps  ->  ${RECEIVER_HOST}:${PORT}"

# Pipeline: camera -> encoder H.264 HW -> pacchettizzazione RTP -> UDP.
# config-interval=1 reinvia SPS/PPS ogni secondo (un receiver che si collega dopo decodifica subito).
exec gst-launch-1.0 -v \
  libcamerasrc ! \
  "video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
  videoconvert ! \
  v4l2h264enc extra-controls="controls,video_bitrate=${BITRATE}" ! \
  "video/x-h264,level=(string)4" ! \
  h264parse ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host="${RECEIVER_HOST}" port="${PORT}" sync=false
