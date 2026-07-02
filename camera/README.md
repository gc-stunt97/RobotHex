# camera/ — Streaming video FPV (lato ROBOT)

Sorgente video del robot: **Pi Camera → H.264 hardware → RTP/UDP** verso il controller.
È un **piano dati separato da ROS2** (handbook sez. 6b): il video NON passa da topic
immagine (satura il WiFi), viaggia su UDP; ROS2 resta per il controllo (piccolo).

Il **pan/tilt** della testa è comandato via ROS (teleop → servo_node): è **disaccoppiato**
dai byte video. Qui ci sono solo i pixel.

## File
- `stream_sender.sh` — pipeline GStreamer che manda il video al controller.

## Stack camera su QUESTO robot (Ubuntu 22.04, Pi Camera v1 / OV5647)
Su Ubuntu 22.04 non c'è libcamera moderno né `libcamerasrc`. Funziona invece lo
stack **LEGACY MMAL** (`bcm2835-v4l2`), che espone `/dev/video0` come device V4L2
con frame processati e H.264 encodato in HW. In `config.txt` c'è già `start_x=1`.

## Prerequisiti (una volta, sul robot)
```bash
# GStreamer
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good \
                    gstreamer1.0-plugins-bad gstreamer1.0-libav
# modulo camera legacy caricato al boot
echo bcm2835-v4l2 | sudo tee /etc/modules-load.d/bcm2835-v4l2.conf
# accesso ai device video
sudo usermod -aG video $USER      # poi ri-login
```

## Uso
```bash
# IP del controller = dove gira il receiver (sul controller: hostname -I)
./stream_sender.sh 192.168.1.50 5000
```
Avvia **prima il receiver** sul controller, poi il sender (UDP: se non c'è nessuno in
ascolto i pacchetti si perdono e basta).

## Encoder
- Default: **H.264 hardware** diretto dal device (`video/x-h264` da `/dev/video0`).
- Fallback software: `ENCODE=sw ./stream_sender.sh ...` (usa `openh264enc`).

## Verifiche utili
- Frame dalla camera: `gst-launch-1.0 v4l2src device=/dev/video0 num-buffers=5 ! fakesink -v`
- H.264 HW disponibile: `gst-launch-1.0 v4l2src device=/dev/video0 ! 'video/x-h264,width=1280,height=720,framerate=30/1' ! fakesink -v`

## Latenza / qualità
- Regola `BITRATE`, `WIDTH/HEIGHT`, `FPS` via variabili d'ambiente.
- Target realistico: ~150 ms, 720p30 fluidi su WiFi buono.

## Prossimo (quando funziona)
Wrappare l'avvio in un launch/systemd del robot così parte insieme ai nodi (bringup),
e cablare la porta/host in un unico posto di config.
