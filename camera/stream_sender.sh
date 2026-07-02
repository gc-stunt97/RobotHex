#!/usr/bin/env bash
#
# Streaming video FPV (lato ROBOT): Pi Camera v1 (OV5647) -> RTP/UDP al controller.
#
# Stack camera: LEGACY MMAL (bcm2835-v4l2), /dev/video0 come device V4L2.
# Piano dati SEPARATO da ROS2 (handbook sez. 6b).
#
# CODEC (variabile CODEC, default mjpeg):
#   mjpeg -> ogni frame indipendente: robusto alle PERDITE WiFi (una perdita = un
#            singolo frame sporco, niente righe che si trascinano). Usa piu' banda.
#   h264  -> meno banda ma fragile alle perdite (P-frame): righe finche' non arriva
#            un keyframe. HW dalla GPU (o software con ENCODE=sw).
#
# PREREQUISITI (una volta, sul robot):
#   echo bcm2835-v4l2 | sudo tee /etc/modules-load.d/bcm2835-v4l2.conf   # modulo al boot
#   sudo usermod -aG video $USER                                         # accesso /dev/video0
#   sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good \
#                       gstreamer1.0-plugins-bad gstreamer1.0-libav
#
# USO:
#   ./stream_sender.sh <IP_CONTROLLER> [PORTA]
#   es:  ./stream_sender.sh 192.168.1.157 5000
#        CODEC=h264 ./stream_sender.sh 192.168.1.157 5000
#
# Variabili opzionali: WIDTH HEIGHT FPS DEV BITRATE QUALITY  (BITRATE/ENCODE solo h264,
#   QUALITY solo mjpeg 1-100). Se il WiFi soffre, abbassa risoluzione (WIDTH/HEIGHT).
set -euo pipefail

RECEIVER_HOST="${1:-${RECEIVER_HOST:-}}"
PORT="${2:-${PORT:-5000}}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
DEV="${DEV:-/dev/video0}"
CODEC="${CODEC:-mjpeg}"
QUALITY="${QUALITY:-65}"        # qualita' JPEG (mjpeg)
BITRATE="${BITRATE:-3000000}"   # bit/s (h264)

if [ -z "${RECEIVER_HOST}" ]; then
  echo "Uso: $0 <IP_CONTROLLER> [PORTA]   (o export RECEIVER_HOST=...)" >&2
  exit 1
fi
if [ ! -e "${DEV}" ]; then
  echo "ERRORE: ${DEV} non esiste. Modulo legacy: sudo modprobe bcm2835-v4l2" >&2
  exit 1
fi

echo "Streaming ${WIDTH}x${HEIGHT}@${FPS}  CODEC=${CODEC}  ->  ${RECEIVER_HOST}:${PORT}"

case "${CODEC}" in
  mjpeg)
    # Cattura raw -> JPEG (jpegenc) -> RTP. Ogni frame autonomo = robusto alle perdite.
    exec gst-launch-1.0 -v \
      v4l2src device="${DEV}" ! \
      "video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
      videoconvert ! \
      jpegenc quality="${QUALITY}" ! \
      rtpjpegpay ! \
      udpsink host="${RECEIVER_HOST}" port="${PORT}" sync=false
    ;;
  h264)
    if [ "${ENCODE:-hw}" = "sw" ]; then
      exec gst-launch-1.0 -v \
        v4l2src device="${DEV}" ! \
        "video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
        videoconvert ! openh264enc bitrate="${BITRATE}" ! h264parse ! \
        rtph264pay config-interval=1 pt=96 mtu=1400 ! \
        udpsink host="${RECEIVER_HOST}" port="${PORT}" sync=false
    else
      exec gst-launch-1.0 -v \
        v4l2src device="${DEV}" extra-controls="controls,video_bitrate=${BITRATE},h264_i_frame_period=${FPS}" ! \
        "video/x-h264,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
        h264parse ! rtph264pay config-interval=1 pt=96 mtu=1400 ! \
        udpsink host="${RECEIVER_HOST}" port="${PORT}" sync=false
    fi
    ;;
  *)
    echo "CODEC sconosciuto: ${CODEC} (usa mjpeg | h264)" >&2
    exit 1
    ;;
esac
