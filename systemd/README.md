# systemd/ — avvio automatico dei nodi robot

`robothex.service` avvia il **bringup** del robot all'accensione: `teleop` +
`servo_node` + `camera_manager` (via `robot_bringup.launch.py`).

**Sicuro all'avvio:** `servo_node` e `camera_manager` partono **spenti**
(`enabled=false`) → all'accensione NON si muove nulla e NON parte video. Si
accendono dal controller (plancia): toggle SIM→REAL per i servi, "Avvia Video".

## Installazione (una volta, sul robot)
```bash
sudo cp ~/robothex_ws/systemd/robothex.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robothex.service    # avvio automatico ad ogni boot
sudo systemctl start robothex.service     # avvia adesso
sudo systemctl status robothex.service    # verifica (q per uscire)
```

## Lavorare a mano (sviluppo) — NON killare, si usa stop
```bash
sudo systemctl stop robothex.service      # ferma i nodi auto-avviati
# ... lavori a mano: git pull, colcon build, ros2 run/launch come sempre ...
sudo systemctl start robothex.service     # li riaccende
```
Per togliere del tutto l'auto-avvio: `sudo systemctl disable robothex.service`.

## Vedere i log
```bash
journalctl -u robothex.service -f         # log in tempo reale
```

## Note
- Gira come utente `giulio` (serve accesso I2C per i servi e gruppo `video` per la
  camera — gia' a posto).
- Se cambi il codice: `git pull` + `colcon build` + `sudo systemctl restart robothex`.
- Se rinomini/sposti il workspace, aggiorna il path in `ExecStart`.
