# roborock_valetudo

A local control panel and AI (MCP) bridge for a Roborock vacuum running [Valetudo](https://valetudo.cloud):
2D/3D map, live raw lidar, MCU sensor data (bumper, cliff, wheel and brush current), cleaning history,
Wi-Fi heatmap, "problem spots" where the robot keeps bumping into things, no-go zones and scripted routes.

> [!WARNING]
> **This whole project was written by AI** (Claude, with a human giving directions and testing).
> It has not been reviewed line by line by an experienced developer. Read the code before you run it,
> especially the part that modifies the robot. **Use at your own risk. The authors take no responsibility
> for a damaged or bricked robot, lost data, or anything else.** See the [LICENSE](LICENSE) (MIT, no warranty).

> [!NOTE]
> **Tested only on a Roborock S5 (S50/S55 family) with firmware 2034 and Valetudo 2026.10.0.**
> Other models (S5 Max, S6, S7, ...) have different firmware, processes and lidar formats. The Valetudo part
> may work, the lidar tap will most likely need changes. The panel UI is in Polish.

## What you need

- A Roborock S5 rooted with Valetudo, reachable on your LAN, with SSH access as `root` using a key.
- A PC with Python 3 (standard library only, no `pip install`), `ssh` on the PATH, and a browser.
- Optional: [Zig](https://ziglang.org) (`python -m pip install ziglang`) to build the lidar tap.

## Step 1: Root the robot and install Valetudo

Follow the official guide: <https://valetudo.cloud/pages/installation/roborock.html>.
Do not skip the full factory reset. Keep your SSH key: you need it in step 3.

Check: `http://<robot-ip>` opens the Valetudo UI.

## Step 2: Run the panel (no changes to the robot)

```sh
python server.py --robot http://<robot-ip>
# no robot at hand: python server.py --robot mock
```

Open <http://127.0.0.1:8780>. Map, status, cleaning, rooms, zones, timers, consumables and all other
Valetudo functions work at this point. Live lidar and sensors need step 3.

The panel listens only on `127.0.0.1`. It does not support Valetudo HTTP Basic Auth.

## Step 3 (optional): Lidar and sensor tap on the robot

Valetudo does not expose raw lidar or sensor data, so a small library copies it inside the robot:

- `tap/lidartap.so` is loaded into Roborock's `rr_loader` with `LD_PRELOAD`. It hooks `read_lds_mmap_buf()`
  (lidar) and `get_package()` (MCU) and **copies** the bytes over UDP to `127.0.0.1:9876` on the robot.
  It does not change or block anything.
- `tap/lidartap_relay` prints those packets to stdout and adds the SLAM pose (10 Hz).
  `server.py` starts it over SSH and decodes the data.

1. Build (or skip if you trust prebuilt binaries you made yourself):
   ```sh
   sh tap/build.sh          # Zig, target arm-linux-gnueabihf.2.19 (Allwinner R16, glibc 2.19)
   ```
2. Copy to the robot:
   ```sh
   scp -i <key> tap/lidartap.so tap/lidartap_relay root@<robot-ip>:/mnt/data/
   ssh -i <key> root@<robot-ip> chmod +x /mnt/data/lidartap_relay
   ```
3. Back up the watchdog config, then edit one line:
   ```sh
   ssh -i <key> root@<robot-ip> cp /opt/rockrobo/watchdog/ProcessList.conf /opt/rockrobo/watchdog/ProcessList.conf.orig
   ```
   In `/opt/rockrobo/watchdog/ProcessList.conf` change the `rr_loader` line to:
   ```
   rr_loader,LD_PRELOAD=/mnt/data/lidartap.so setsid rr_loader -d&,1,5,1
   ```
   Compare with the original line first: your firmware may differ.
4. Reboot the robot, then start the panel with the key:
   ```sh
   python server.py --robot http://<robot-ip> --ssh-key <key>
   ```
   Check: the lidar scan appears on the map and "lidar age" in the status is under a second.

**Undo:**
```sh
ssh -i <key> root@<robot-ip> 'cp /opt/rockrobo/watchdog/ProcessList.conf.orig /opt/rockrobo/watchdog/ProcessList.conf && sync && reboot'
```

## Step 4 (optional): Connect an AI agent (MCP)

`mcp_server.py` is a stdio MCP server (standard library only) with 26 tools: status, start/stop, rooms, zones,
go-to, routes, step-by-step programs, manual driving, problem spots, no-go zones, timers and raw Valetudo API.
It talks to the running panel (`ROBOROCK_PANEL`, default `http://127.0.0.1:8780`).

```sh
claude mcp add roborock -s user -- python /path/to/mcp_server.py
python mcp_server.py --test     # self-check
```

The agent can move a real robot. Risky Valetudo calls (map reset, Wi-Fi change, persistent map, quirks,
voice pack) need explicit confirmation. Updater and Valetudo settings are blocked.

## How data is used

- **Everything stays on your machine.** The panel talks only to the robot on your LAN. No telemetry,
  no accounts, no cloud. Valetudo itself is cloud-free.
- Stored locally in `data/` (excluded from git):
  - `sessions/<id>.jsonl`: one frame every 0.5 s per run (position, battery, Wi-Fi signal, sensors, lidar scan) and events.
  - `events.jsonl`: bumper, cliff, brush, door and error events. Used for problem spots and history.
  - `wifi_samples.jsonl`: signal strength by position, for the Wi-Fi heatmap.
  - `lidar_<date>.bin`: raw lidar bytes (about 20 to 40 MB per day of cleaning). Delete freely.
  - `known_hosts`: the robot's SSH host key.
- The 3D view loads three.js from the jsDelivr CDN, so your browser contacts that CDN.
- **If you use the MCP server with a cloud AI** (Claude, ChatGPT, ...), the tool results (map coordinates,
  room names, status, events, Wi-Fi names) are sent to that AI provider. Use a local model if that matters to you.
- Never commit your SSH private key. `.gitignore` excludes `roborock`, `*.key` and `data/`.

## Files

| File | Purpose |
| :--- | :--- |
| `server.py` | Panel web server, Valetudo proxy with write rules, recorder, tap reader, `/local/*` endpoints |
| `mcp_server.py` | MCP server for AI agents |
| `lds.py`, `sensors.py`, `analysis.py` | Lidar and MCU decoders, map analysis, problem spots (each has a self-check: `python <file>`) |
| `mock_robot.py` | Fake robot for `--robot mock` |
| `static/` | Panel UI (three.js scene, history charts) |
| `programs/` | Saved step programs (JSON) |
| `tap/` | C sources for the on-robot tap and build script |
| `README.pl.md` | Original Polish notes with calibration details |

## License

[MIT](LICENSE). Provided "as is", without warranty of any kind. Not affiliated with Roborock or Valetudo.
