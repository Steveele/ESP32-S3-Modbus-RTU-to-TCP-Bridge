# Wireless Modbus RTU ↔ TCP Bridge (ESP32)

This project replaces a wired RS-485 link between an inverter (Modbus RTU master) and a smart meter (Modbus RTU slave) with a transparent Wi-Fi bridge made of **two ESP32 boards**.

| Side | ESP32 Role | Network Mode | Sketch | Serial Pins | Default Baud |
|------|------------|--------------|--------|-------------|--------------|
| Inverter | RTU-to-TCP client | Station | `esp32_rtu2tcp.ino` (ESP A) | RX 17, TX 18, DE/RE 5 | 9600 |
| Smart-meter | TCP-to-RTU server | Soft-AP | `esp32_tcp2rtu.ino` (ESP B) | RX 17, TX 18, DE/RE 5 | 9600 |

ESP A connects to the soft-AP hosted by ESP B (`SSID: ModbusLink`, `PASS: Bridge123`). Requests from the inverter are forwarded over Wi-Fi as Modbus TCP frames, then translated back to RTU on the meter side, and vice-versa.

---

## 1. Repository layout

/root
├─ esp32_rtu2tcp/ # Sketch for the inverter side (ESP A)
├─ esp32_tcp2rtu/ # Sketch for the meter side (ESP B)
└─ docs/ # Wiring, screenshots, reference material


---

## 2. Hardware

* **2 × ESP32-WROOM (or compatible)**
* **2 × MAX485 / SN75176** RS-485 transceivers  
  * RO → GPIO 17  
  * DI → GPIO 18  
  * DE + RE → GPIO 5
* Termination resistors (120 Ω) at each RS-485 end
* Power supply 5 V / 3.3 V as required by the boards

> The sketches use `Serial2` on ESP A and `Serial1` on ESP B. Adjust pins or baud rate in `setup()` if needed.

---

## 3. Network topology

┌────────────┐ RS-485 ┌────────────┐
│ Inverter │◄──────────────►│ ESP32 A │
└────────────┘ │ RTU→TCP │
└─────▲─────┘ Wi-Fi (TCP)
│
192.168.10.46 ⇄ 192.168.10.1
│
┌────────────┐ ┌─────▼─────┐ RS-485 ┌────────────┐
│ Simulator │◄──────────────►│ ESP32 B │◄──────────────►│ Smart Meter │
│ (Python) │ │ TCP→RTU │ └────────────┘
└────────────┘ └────────────┘


---

## 4. Building and flashing

### Arduino IDE

1. Install the *esp32* core and select the correct board.  
2. Install library **Modbus-esp8266** by *Alexander Emelianov*.  
3. Open each sketch in its own IDE window, set your COM port, then **Upload**.

### PlatformIO

```bash
git clone https://github.com/<your-user>/<repo>.git
cd <repo>/esp32_rtu2tcp
pio run -t upload
cd ../esp32_tcp2rtu
pio run -t upload
