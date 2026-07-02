# camera/ — Streaming video FPV (lato ROBOT)

Sorgente video del robot: **Pi Camera → H.264 hardware → RTP/UDP** verso il controller.
È un **piano dati separato da ROS2** (handbook sez. 6b): il video NON passa da topic
immagine (satura il WiFi), viaggia su UDP; ROS2 resta per il controllo (piccolo).

Il **pan/tilt** della testa è comandato via ROS (teleop → servo_node): è **disaccoppiato**
dai byte video. Qui ci sono solo i pixel.

## File
- `stream_sender.sh` — pipeline GStreamer che manda il video al controller.

## Prerequisiti (sul robot)
```bash
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good \
                    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-libcamera
```

## Uso
```bash
# IP del controller = dove gira il receiver (sul controller: hostname -I)
./stream_sender.sh 192.168.1.50 5000
```
Avvia **prima il receiver** sul controller, poi il sender (UDP: se non c'è nessuno in
ascolto i pacchetti si perdono e basta).

## ⚠️ Da verificare sull'hardware (stack camera)
La pipeline usa `libcamerasrc` + `v4l2h264enc` (encoder H.264 hardware del Pi 4).
A seconda dell'OS può servire un aggiustamento:
- **Camera visibile con libcamera?** prova: `libcamera-hello -t 2000` (o `cam -l`).
- **Elemento sorgente**: recente = `libcamerasrc`; vecchio Raspberry Pi OS = `rpicamsrc`;
  camera come `/dev/video0` = `v4l2src device=/dev/video0`.
- **Encoder HW presente?** `gst-inspect-1.0 v4l2h264enc` deve rispondere. Se manca,
  fallback software (più CPU): sostituisci con `x264enc tune=zerolatency bitrate=4000`.
- **Test rapido locale** (senza rete, mostra la camera sul robot):
  `gst-launch-1.0 libcamerasrc ! videoconvert ! autovideosink`

## Latenza / qualità
- Regola `BITRATE`, `WIDTH/HEIGHT`, `FPS` via variabili d'ambiente.
- Target realistico: ~150 ms, 720p30 fluidi su WiFi buono.

## Prossimo (quando funziona)
Wrappare l'avvio in un launch/systemd del robot così parte insieme ai nodi (bringup),
e cablare la porta/host in un unico posto di config.
