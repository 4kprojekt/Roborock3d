# roborock_valetudo

Klient API Valetudo dla Roborock S5 (S552-00): panel sterowania z mapą 2D/3D/FPV, surowym lidarem, czujnikami MCU,
historią przejazdów i mapą zasięgu Wi-Fi.

## Uruchomienie

```powershell
cd C:\AI_WorkSpace\roborock_valetudo
python server.py                                 # domyślnie robot http://192.168.0.25
python server.py --robot http://192.168.8.1     # robot na własnym AP (przed konfiguracją Wi-Fi)
# bez robota:
python server.py --robot mock
```
Otwórz http://127.0.0.1:8780. Tylko Python 3 (stdlib). Widok 3D pobiera three.js z jsdelivr, więc potrzebuje internetu. 2D działa bez niego.

## Co robi

Panel sterowania (układ wg makiety `Panel sterowania — Roborock S5-html/`): jedna scena three.js z widokami
2D / 3D / ISO / Orbita / FPV, sterowanie robotem, telemetria, historia przejazdów z odtwarzaniem.

| Funkcja | Skąd dane |
| :--- | :--- |
| Mapa, pokoje, ściany, stan, bateria, Wi-Fi, materiały, harmonogram | REST API Valetudo (`/api/v2/...`) |
| Skan lidaru na żywo, chmura punktów | podsłuch `lidartap` na robocie (port 3), dekoder `lds.py` |
| Pozycja 10 Hz | `/run/shm/SharedMapInfo` na robocie (port 8) |
| Zderzak, krawędzie, prądy kół i szczotki, przechył, błędy MCU | pakiety MCU (port 7), dekoder `sensors.py` |
| Mapa Wi-Fi (heatmapa + średnie wg pokoi) | sygnał dBm + pozycja, `data/wifi_samples.jsonl` |

**Analiza lidaru** (`analysis.py`, liczona w `server.py: process_scan`):
- Ściany konstrukcyjne (wysokie) vs meble z mapy (35 cm) vs drobne przeszkody z lidaru (10 cm, do 30 cm = „drobna”).
- Chmura jest dynamiczna: echo dodaje punkt, promień przechodzący przez komórkę go odejmuje. Zabrana rzecz znika z mapy.
- Obiekty ruchome: echo musi przetrwać `STABLE_S` = 4 s, zanim stanie się przeszkodą. Co znikło przed `MOVING_MAX_S` = 25 s, jest liczone jako ruchome i pomijane.
- Lustra i szyby: echo zza ściany konstrukcyjnej nie trafia do chmury. Odcinek ściany, przez który promienie głównie przechodzą, jest klasyfikowany:
  `lustro` (punkty po odbiciu względem ściany trafiają w prawdziwe ściany), `drzwi?` (55–115 cm), `szyba/otwór`, `brak odbicia`.
  Echa zza wykrytego lustra nie są rysowane ani zapisywane.
- Drzwi: przejścia między pokojami z mapy (`doorways`), stan otwarte/zamknięte sprawdzany co ok. 2 s podczas skanowania
  (`door_state`: echo na linii drzwi = zamknięte, promień przez nią = otwarte). Zmiana stanu trafia do zdarzeń.

**Skanuj mieszkanie** (`explore_run`): jeden punkt na pokój (duże pokoje więcej), bez sprzątania. Do każdego celu robot jedzie,
aż sam się zatrzyma (do 5 min). Na końcu zagląda za otwory wielkości drzwi, przez które lidar coś widział (do 6 prób).

**Historia:** każdy przejazd to `data/sessions/<id>.jsonl` (klatka co 0,5 s: pozycja, bateria, Wi-Fi, czujniki, skan + zdarzenia).
Odtwarzanie interpoluje pozycję między klatkami, FPV działa na żywo i w historii.

**Zdjęcia pomieszczeń:** plik + dwa kliknięcia na mapie 2D (lewy i prawy koniec ściany), zdjęcie staje pionowo w 3D.

## Kalibracja (ponytail: pokrętła, fizyka wymaga strojenia)

- `lds.py`: `LDS_ANGLE_OFFSET_DEG = 10`, `POSE_LAG_MS = 200` (pozycja SLAM jest publikowana z opóźnieniem, skan czeka na pozycję ze swojej chwili).
  Dane do ponownego dopasowania: `/local/debug`.
- `sensors.py`: progi dywanu (prąd szczotki ×1,5), zablokowanej szczotki (×2,6), oporu jazdy (×2,2) i przechyłu (4,5°) to heurystyki.
  Kolejność bajtów zderzaka L/P jest założeniem.
- `scene.js`: `HEADING_OFFSET_DEG = -90` (kierunek jazdy = kąt Valetudo − 90°), `SMOOTH_S` (wygładzanie ruchu na żywo).

## Podsłuch na robocie: lidartap (zainstalowane 2026-11-09)

Valetudo nie udostępnia surowego lidaru ani czujników, więc są kopiowane na robocie bez zmiany firmware:

- `tap/lidartap.so` jest ładowane do `rr_loader` przez `LD_PRELOAD`. Przechwytuje `read_lds_mmap_buf()` (lidar) i `get_package()` (MCU)
  i **kopiuje** bajty przez UDP na `127.0.0.1:9876` robota. Niczego nie zmienia ani nie blokuje.
- `tap/lidartap_relay` wypisuje pakiety na stdout i co 100 ms dokłada pozycję SLAM. `server.py` uruchamia go przez SSH (klucz `~/.ssh/roborock`).
- Datagram: `"LT" | port u8 | dir u8 | len u16 | t_ms u32 | dane`. Porty: 3 lidar, 7 MCU, 8 pozycja.
- LDS: pakiety 22 B w stylu Neato XV-11, 300 obr/min, 5 skanów/s. Surowe bajty: `data/lidar_<data>.bin`.
- MCU TLV `[id][len][dane]`: 7 odometria + żyroskop, 8 bateria, 33 zderzak, 34 opadnięcie kół, 35 krawędzie, 37 zderzak wieżyczki,
  64 czujnik ściany, 80 wentylator, 81 prąd kół, 82 szczotka główna, 83 boczna, 4 błąd, 247–250 logi tekstowe.
- Budowanie: `tap/build.sh` (Zig, `arm-linux-gnueabihf.2.19`, `-z norelro -z lazy`, bo loader glibc 2.19 wywala się na RELRO z lld).

**Zmiana na robocie** (jedna linia, kopia zapasowa obok):
`/opt/rockrobo/watchdog/ProcessList.conf`: `rr_loader,LD_PRELOAD=/mnt/data/lidartap.so setsid rr_loader -d&,1,5,1`

**Cofnięcie:**
```sh
ssh -i ~/.ssh/roborock root@192.168.0.25 'cp /opt/rockrobo/watchdog/ProcessList.conf.orig /opt/rockrobo/watchdog/ProcessList.conf && sync && reboot'
```

## Pliki

- `server.py`: UI, proxy `/api/*` (GET wszystko; zapis wg `WRITE_RULES`: wszystkie capabilities, timery, zdarzenia, poziom logu; updater/ustawienia Valetudo zablokowane; `CONFIRM_CAPS` - reset mapy, Wi-Fi, trwała mapa, kopie mapy, quirks, pakiet głosowy - tylko z nagłówkiem `X-Confirm: tak`), wątki: poller, recorder, tap_reader; endpointy `/local/*`
  - `GET /local/hotspots`: miejsca problemowe (`analysis.hotspots`: zderzak/przechył/opór kół/szczotka z całej historii, grupowane w promieniu 30 cm, punkt styku z kierunku jazdy; „przy ścianie z mapy” vs „poza mapą”, proponowana strefa zakazana). Na żywo zdarzenie `hotspot` przy 3/10/25/50/100 problemach w tym samym miejscu.
  - `POST /local/program/start` (JSON: lista kroków), `GET /local/program` (wyniki + opis kroków), stop: `POST /local/explore/stop`. Kroki: `goto`, `forward`/`back` (cm, sterowanie ręczne, staje przed przeszkodą, `push` = do zderzaka), `turn`/`face`/`face_point`, `look` (odległości lidarem w 4 sektorach), `wait`, `wait_done`, `clean_zone`, `clean_rooms`, `fan`, `locate`, `home`, `stop`. Raport per krok: wynik, przesunięcie, pozycja, zdarzenia czujników.
- `lds.py`, `sensors.py`, `analysis.py`: dekodery i analiza, każdy z self-checkiem (`python <plik>`)
- `mock_robot.py`: udawany robot (`--robot mock`)
- `static/`: `index.html`, `app.js`, `scene.js` (three.js), `history.js` (wykresy), `mapdata.js` (+ `node test_mapdata.mjs`)
- `tap/`: źródła podsłuchu; `data/`: sesje, próbki Wi-Fi, zdarzenia, zdjęcia (poza gitem)
- `mcp_server.py`: serwer MCP (stdio, sam stdlib) dla agentów AI; nakładka na panel, więc wymaga działającego `server.py` (adres: env `ROBOROCK_PANEL`, domyślnie `http://127.0.0.1:8780`). Zapisy idą przez proxy, więc obowiązuje `PUT_ALLOWED`. Self-check: `python mcp_server.py --test`.
  Rejestracja: `claude mcp add roborock -s user -- python C:/AI_WorkSpace/roborock_valetudo/mcp_server.py`.
  Narzędzia (26): `robot_status` (też surowe mV/mA/temp z MCU), `robot_control` (start/stop/pause/home/locate), `list_rooms`, `rename_room`, `clean_rooms`, `clean_zone`, `go_to`, `explore` (skan lub punkt z próbą przejazdu przez drzwi), `run_route`, `run_program`, `drive` (ręcznie: forward/back/turn/face/look), `job_status`, `stop_job`, `hotspots`, `restrictions` (strefy zakazane / wirtualne ściany, także z hotspotu), `schedule`, `fan_speed`, `carpet_mode`, `consumables`, `statistics`, `events`, `map_analysis`, `sessions`, `wifi_report`, `valetudo_capabilities`, `valetudo_api` (dowolne wywołanie, ryzykowne z `confirm`).
- Panel: sekcje „Sterowanie ręczne i program” (pad, trasa klikana na mapie, edytor programu, raport kroków), „Miejsca problemowe” (lista, Jedź/Zablokuj, strefy i ściany rysowane na mapie), „Valetudo · wszystkie funkcje” (nie przeszkadzać, dźwięk, pakiet głosowy, nazwy/łączenie/podział pokoi, harmonogram, trwała mapa, kopie i reset mapy, Wi-Fi, quirks, zdarzenia, log, system) + konsola API; warstwa mapy „Miejsca problemowe”.

## Ograniczenia

- Brak obsługi HTTP Basic Auth Valetudo (do dodania w `make_fetch`, jeśli włączysz hasło).
- Mapa ścian pochodzi z Valetudo i zmienia się dopiero po przejeździe robota. Na bieżąco zmienia się chmura przeszkód i stan drzwi.
- Chmura punktów nie jest zapisywana między restartami serwera.
- S5 nie ma regulacji wody ani kamery: „FPV” to widok 3D z pozycji robota.

## Wgrywanie Valetudo (status 2026-11-08: wgrane, Valetudo 2026.10.0, robot pod 192.168.0.25, Swagger: /swagger)

Firmware: Dustfab, S5, ver 2034, OTA, plik `v11_002034.pkg`, narzędzie `valetudo-helper-miioota.exe install-firmware`.
Pierwsza próba: robot pobrał plik, ale się nie zainstalował. Przyczyna: reset nie był pełnym resetem fabrycznym
(trzymaj Dock, naciśnij Reset spinaczem, trzymaj Dock ~5 s do komunikatu głosowego). Podczas uploadu odłącz LAN
i wyłącz zaporę, robot ma stać na stacji.
