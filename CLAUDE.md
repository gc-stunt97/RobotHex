# Robothex — istruzioni di progetto per Claude

> Questo file viene letto in automatico da Claude Code all'avvio, in qualsiasi
> macchina (PC Windows o Raspberry via SSH). Serve a darti subito il contesto.

## ⚠️ Primo passo, sempre

**Leggi `ROBOTHEX_HANDBOOK.md`** (stessa cartella) prima di lavorare sul progetto.
Contiene: hardware, mappatura dei 12 servo, architettura ROS2, refresh comandi,
bug noti del codice e roadmap. È la fonte di verità del progetto.

## In breve (per orientarti, i dettagli sono nell'handbook)

- **Cos'è:** replica dell'esapode *Genghis* (Brooks/MIT 1991). 6 zampe × 2 servo = 12 servo,
  PCA9685 via I2C su Raspberry Pi 4. ROS2 (`rclpy`). Controller remoto separato via WiFi.
- **Obiettivo della ripresa:** camminata **fluida** con cinematica inversa (IK) + gait engine
  a fase 0→1 con offset. Gait preferito: **ripple**. L'attuale tripode a pose discrete è rigido.
- **Mappatura/inversione dei 12 servo:** già fatta e funzionante (handbook sez. 3) — è la base.

## Stato e attenzioni

- Il codice "vero" vive **solo sui due Raspberry**, senza backup. **TODO #1: git/backup.**
- Copia locale parziale: `images/leg_control_node.py` — ha bug noti (handbook sez. 4):
  IndentationError nel callback, `node.shutdown()` → usare `destroy_node()`, ServoKit duplicati.
- Decisione presa: **si tiene ROS2** (dà rqt/rviz/ros2 bag per debug; il lavoro vero è IK + gait).

## Come lavorare con Giulio

- Rispondere **in italiano**.
- Spiegare prima il **perché** (concettuale) e poi il **come**. Vuole imparare a farlo "in modo
  serio", non workaround macchinosi.
- Programmatore autodidatta, esperienza ROS2 di base → utile il refresh comandi (handbook sez. 5).
