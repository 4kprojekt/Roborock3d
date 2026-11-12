// UI language: English by default, Polish when the browser is Polish; the header button overrides (localStorage).
// ponytail: the source strings in the code stay Polish. In English mode every text node, title/placeholder,
// dialog and canvas label is passed through the dictionary below ("{}" = any value, translated recursively).
// Untranslated strings are collected in window.i18nMissing - add them to D.
(() => {
  let saved = null;
  try { saved = localStorage.getItem("lang"); } catch { /* private mode */ }
  const LANG = saved || ((navigator.language || "").toLowerCase().startsWith("pl") ? "pl" : "en");
  window.LANG = LANG;
  window.LOCALE = LANG === "pl" ? "pl-PL" : "en-GB";
  window.i18n = s => s;
  document.documentElement.lang = LANG;
  document.addEventListener("DOMContentLoaded", () => {
    const b = document.getElementById("lang");
    if (!b) return;
    b.textContent = LANG === "pl" ? "EN" : "PL";
    b.onclick = () => { try { localStorage.setItem("lang", LANG === "pl" ? "en" : "pl"); } catch { /* ignore */ } location.reload(); };
  });
  if (LANG === "pl") return;

  const HELP = `Program steps (a list of objects, one key = one operation):
  {"goto": [x, y]}             drive to a map point (cm), robot plans the path itself
  {"forward": cm} / {"back": cm}  straight drive in manual control (max 300 cm; forward stops before an obstacle)
  {"forward": cm, "push": true}   forward until the bumper hits (a lidar obstacle only slows it down)
  {"turn": deg}                turn by an angle (+ = right / clockwise on the map)
  {"face": deg} / {"face_point": [x, y]}  turn to a map direction (0 = +x, 90 = +y down the map) / towards a point
  {"look": true}               lidar measurement: nearest echo front / left / right / back (cm)
  {"wait": s}                  wait s seconds
  {"wait_done": s}             wait until the robot finishes its task (status docked/idle), max s
  {"clean_zone": [x1, y1, x2, y2]} / {"clean_rooms": ["1", "2"]}  start cleaning
  {"fan": "max"} / {"locate": true} / {"home": true} / {"stop": true}
  {"explore": true}            full lidar scan of the flat (same as "Scan the flat"), ends on the dock
Every step returns: result, movement, pose and sensor events (bumper, tilt...) that happened during it.`;

  const D = {
    // --- index.html ---
    "Panel sterowania — Roborock S5": "Control panel — Roborock S5",
    "Stan": "State", "Bateria": "Battery", "Czas": "Time", "Posprzątano": "Cleaned", "Do stacji": "Dock",
    "Skanuj mieszkanie": "Scan the flat", "Mapa 3D · warstwy": "3D map · layers", "Śledź robota": "Follow robot",
    "Ściany": "Walls", "Warstwy": "Layers", "Zasięg Wi-Fi · dBm": "Wi-Fi coverage · dBm", "TRASA": "PATH", "MEBLE": "FURNITURE",
    "PRZESZKODA": "OBSTACLE", "LUSTRO": "MIRROR", "SZYBA": "GLASS", "ODBICIE": "REFLECTION", "Telemetria": "Telemetry",
    "Bieżące zadanie": "Current task", "Parametry sprzątania": "Cleaning settings", "Pomieszczenia": "Rooms",
    "Moc ssania": "Suction power", "Liczba przejść": "Passes", "Uruchom sprzątanie zaznaczonych": "Clean selected rooms",
    "Zużycie baterii": "Battery usage", "▶ Odtwórz": "▶ Play", "✕ Zamknij": "✕ Close", "Skan lidar 360°": "Lidar scan 360°",
    "Wi-Fi wg pomieszczeń": "Wi-Fi by room", "Wyczyść pomiary": "Clear samples", "Historia przejazdów": "Run history",
    "Typ": "Type", "Dystans": "Distance", "Zdarzenia czujników": "Sensor events", "Konserwacja": "Maintenance",
    "Czujniki na żywo": "Live sensors", "Analiza lidaru": "Lidar analysis", "Wyczyść chmurę": "Clear cloud",
    "Zdjęcia pomieszczeń": "Room photos", "+ Dodaj zdjęcie": "+ Add photo",
    "Wybierz zdjęcie ściany, a potem kliknij na mapie jej początek i koniec. Zdjęcie zostanie rozpięte na tym odcinku w widoku 3D i FPV.":
      "Choose a photo of a wall, then click its start and end on the map. The photo is stretched over that segment in the 3D and FPV views.",
    "Harmonogram": "Schedule", "Edytuj w Valetudo": "Edit in Valetudo", "Zdarzenia": "Events", "ostatnie": "latest",
    "Sterowanie ręczne i program": "Manual control and program", "Pomiar": "Measure",
    "Do przodu robot staje przed przeszkodą widzianą przez lidar. Do tyłu nie ma zderzaka, więc jedzie tylko na krótki dystans.":
      "Going forward the robot stops before an obstacle seen by the lidar. There is no rear bumper, so it only reverses a short distance.",
    "Trasa": "Route", "+ Wskaż punkty na mapie": "+ Pick points on the map", "Jedź trasą": "Drive route", "Wyczyść": "Clear",
    "Zapisane": "Saved", "— wybierz program —": "— choose a program —", "Program (kroki JSON)": "Program (JSON steps)",
    "Uruchom program": "Run program", "Opis kroków": "Step reference", "Miejsca problemowe": "Problem spots",
    "Miejsca, w których robot wielokrotnie w coś uderza albo na coś najeżdża (zderzak, przechył, opór kół, szczotka), zebrane z całej historii.":
      "Places where the robot repeatedly hits or runs onto something (bumper, tilt, wheel load, brush), collected from the whole history.",
    "Pomarańczowe": "Orange", "to coś, czego nie ma na mapie,": "is something that is not on the map,", "szare": "grey",
    "to zwykłe dojeżdżanie do ścian.": "is ordinary driving up to walls.",
    "Ograniczenia": "Restrictions", "+ Strefa zakazana": "+ No-go zone", "+ Wirtualna ściana": "+ Virtual wall",
    "Valetudo · wszystkie funkcje": "Valetudo · all functions", "Odśwież": "Refresh",
    "Konsola API (dowolne wywołanie)": "API console (any call)", "potwierdzam ryzykowną operację": "I confirm a risky operation",
    "Wyślij": "Send", "Wszystkie dane z API robota": "All data from the robot API", "Wczytaj": "Load",
    "Objazd wszystkich pokoi bez sprzątania: lidar skanuje całe mieszkanie": "Drive through all rooms without cleaning: the lidar scans the whole flat",
    "Znajdź robota": "Locate robot", "Znajdź robota (sygnał dźwiękowy)": "Locate robot (plays a sound)", "Widok": "View",
    "Zakres wykresu": "Chart range", "Prędkość odtwarzania": "Playback speed", "Prędkość": "Speed",
    "Dodatkowa seria na wykresie": "Extra chart series", "Seria": "Series", "treść JSON (dla PUT/POST)": "JSON body (for PUT/POST)",
    "Język": "Language",
    // --- app.js: status, presets, layers ---
    "SPRZĄTANIE": "CLEANING", "JAZDA": "MOVING", "POWRÓT": "RETURNING", "W STACJI": "DOCKED", "POSTÓJ": "IDLE", "PAUZA": "PAUSED",
    "RĘCZNE": "MANUAL", "BŁĄD": "ERROR", "Cicha": "Quiet", "Balans": "Balanced", "Wył.": "Off",
    "Szczotka główna": "Main brush", "Szczotka boczna": "Side brush", "Filtr": "Filter", "Czujniki": "Sensors", "Orbita": "Orbit",
    "Meble i przeszkody": "Furniture, obstacles", "Heatmapa Wi-Fi": "Wi-Fi heatmap", "Chmura lidar": "Lidar cloud",
    "Skan na żywo": "Live scan", "Etykiety zdarzeń": "Event labels", "Etykiety": "Labels", "Lustra / szyby": "Mirrors / glass",
    "Odbicia (błędne)": "Reflections (false)", "Zdjęcia": "Photos",
    "Nie udało się uruchomić widoku 3D: {}": "Could not start the 3D view: {}",
    "Pauza": "Pause", "Wznów": "Resume", "Stop skanowania": "Stop scan", "Program: {}": "Program: {}",
    "Skanowanie mieszkania:": "Flat scan:", "Skanowanie mieszkania: {}": "Flat scan: {}",
    "napięcie": "voltage", "prąd": "current", "temp. baterii": "battery temp.",
    "czujniki: brak połączenia z podsłuchem": "sensors: tap not connected", "Sygnał Wi-Fi": "Wi-Fi signal",
    "SSID · pasmo": "SSID · band", "Przepustowość": "Link speed", "KĄT": "ANGLE", "POZYCJA": "POSE", "PRZECHYŁ": "TILT",
    "Pomiary Wi-Fi": "Wi-Fi samples", "Punkty lidar": "Lidar points", "Przeszkody": "Obstacles", "Lustra/szyby": "Mirrors/glass",
    "Ostatnia synchronizacja {}": "Last sync {}", "{} pakietów MCU": "{} MCU packets", "brak danych": "no data",
    "Dane czujników idą przez podsłuch na robocie (SSH). Teraz nie jest połączony.": "Sensor data comes through the tap on the robot (SSH). It is not connected now.",
    "WCIŚNIĘTY": "PRESSED", "Zderzak L / P": "Bumper L / R", "Wieżyczka lidaru": "Lidar turret", "Czujniki krawędzi": "Cliff sensors",
    "KRAWĘDŹ": "CLIFF", "Koła opadły": "Wheels dropped", "TAK": "YES", "Prąd kół L / P": "Wheel current L / R",
    "Prąd szczotki": "Brush current", "Czujnik ściany": "Wall sensor", "Przechył": "Tilt", "Wentylator / szczotka": "Fan / brush",
    "Żyroskop Z": "Gyro Z", "Brak zdarzeń.": "No events.",
    "Mapa z lidaru · {} cm/piksel · {} × {} m · {} m²": "Lidar map · {} cm/pixel · {} × {} m · {} m²",
    "Punkty na ścianach": "Points on walls", "Punkty przeszkód": "Obstacle points", "Przeszkody drobne ≤ 30 cm": "Small obstacles ≤ 30 cm",
    "Przeszkody duże": "Large obstacles", "Odbicia odrzucone": "Reflections rejected", "Ruchome (pominięte, 30 s)": "Moving (skipped, 30 s)",
    "Meble z mapy (piksele)": "Furniture from the map (pixels)",
    "Nie wykryto jeszcze luster ani szyb. Potrzebny jest przejazd z włączonym lidarem (sprzątanie albo „Skanuj mieszkanie”).":
      "No mirrors or glass detected yet. A run with the lidar on is needed (cleaning or “Scan the flat”).",
    "min {} · maks {} · {} pomiarów": "min {} · max {} · {} samples",
    "Brak pomiarów. Zbierają się podczas jazdy robota.": "No samples. They are collected while the robot drives.",
    "Najsłabszy zasięg:": "Weakest coverage:", "(średnio {} dBm, minimum {} dBm).{}": "(average {} dBm, minimum {} dBm).{}",
    "z mapy (lidar stoi)": "from the map (lidar stopped)", "na żywo · {} RPM · bez echa: {}": "live · {} RPM · no echo: {}",
    "{} pkt": "{} pts", "Źródło": "Source", "Min. odl.": "Min. dist.", "Zasięg maks.": "Max range", "Rozdz. 1° · 5 Hz": "Res. 1° · 5 Hz",
    "Mapa nie ma podziału na pokoje.": "The map has no rooms.", "Powierzchnia": "Area", "Szac. czas": "Est. time",
    "Szac. bateria": "Est. battery", "Wzmocnienie na dywanie": "Carpet boost",
    "robot sam zwiększa ssanie na dywanie": "the robot raises suction on carpet by itself", "Nie przeszkadzać": "Do not disturb",
    "włączone": "on", "wyłączone": "off", "{} · zmiana niżej, w „Valetudo · wszystkie funkcje”": "{} · change below, in “Valetudo · all functions”",
    "Nie udało się: {}": "Failed: {}",
    "Robot objedzie wszystkie pokoje bez sprzątania (punkty na środku pomieszczeń), a potem wróci na stację. Start?":
      "The robot will drive through all rooms without cleaning (points in the middle of rooms) and then return to the dock. Start?",
    "Usunąć wszystkie zapisane pomiary zasięgu Wi-Fi?": "Delete all saved Wi-Fi coverage samples?",
    "Wyczyścić chmurę punktów lidaru i wykryte lustra/szyby?": "Clear the lidar point cloud and detected mirrors/glass?",
    "Brak danych o materiałach eksploatacyjnych.": "No consumables data.",
    "Zresetować licznik „{}”? Zrób to tylko po faktycznej wymianie lub czyszczeniu.": "Reset the “{}” counter? Do it only after actually replacing or cleaning the part.",
    "Pn": "Mo", "Wt": "Tu", "Śr": "We", "Cz": "Th", "Pt": "Fr", "Sb": "Sa", "Nd": "Su",
    "zaplanowane ({})": "scheduled ({})", "nie przeszkadzać{}": "do not disturb{}",
    // --- history ---
    "Zapisane tutaj": "Saved here", "{} przejazdów": "{} runs", "Robot łącznie": "Robot total", "{} sprzątań": "{} cleanups",
    "Łączny czas": "Total time",
    "Brak zapisanych przejazdów. Każde sprzątanie i skanowanie zapisuje się automatycznie od teraz.": "No saved runs. Every cleaning and scan is recorded automatically from now on.",
    "Odtwarzanie": "Replay", "Sesje": "Sessions", "14 dni": "14 days", "Przejazd {}": "Run {}", "Przejazd": "Run",
    "linia: bateria (kolor = poziom)": "line: battery (colour = level)", "kliknij wykres, żeby przewinąć": "click the chart to seek",
    "Szczotka / koła": "Brush / wheels", "Energia na dzień [Wh] · 14 dni · łącznie": "Energy per day [Wh] · 14 days · total",
    "· szacunek z % baterii (pakiet 5200 mAh / 14,4 V)": "· estimated from battery % (5200 mAh / 14.4 V pack)",
    "ostatni przejazd": "last run", "poprzednie": "previous", "kliknij wiersz historii, żeby odtworzyć przejazd": "click a history row to replay the run",
    "Śr. zużycie": "Avg. usage", "Zużyto łącznie": "Used in total", "Napięcie": "Voltage", "Temp. baterii": "Battery temp.",
    "bez dodatkowej serii": "no extra series", "zapis przejazdu": "run recording",
    "Ten przejazd jest za krótki, żeby go odtworzyć.": "This run is too short to replay.", "⏸ Pauza": "⏸ Pause",
    "sprzątanie": "cleaning", "skanowanie": "scan", "skanowanie mieszkania": "flat scan",
    // --- photos ---
    "{} cm ściany": "{} cm of wall", "Usuń": "Delete", "Nie udało się odczytać obrazu.": "Could not read the image.",
    "Podpis zdjęcia (np. Salon – ściana z oknem):": "Photo caption (e.g. Living room – wall with the window):",
    "Kliknij na mapie POCZĄTEK ściany ze zdjęcia (lewy brzeg zdjęcia)": "Click on the map the START of the wall in the photo (left edge of the photo)",
    "Teraz kliknij KONIEC ściany (prawy brzeg zdjęcia)": "Now click the END of the wall (right edge of the photo)",
    "· Esc = anuluj": "· Esc = cancel", "{} · Esc = anuluj": "{} · Esc = cancel", "Usunąć to zdjęcie?": "Delete this photo?",
    // --- program / route / hotspots ---
    "przód {} · prawo {} · tył {} · lewo {} cm": "front {} · right {} · back {} · left {} cm", "· {} cm od celu": "· {} cm from target",
    "krok {}": "step {}", "Tu pojawi się raport z każdego kroku.": "The report from each step will appear here.",
    "Błędny JSON: {}": "Invalid JSON: {}", "{} ({} kroków)": "{} ({} steps)",
    "klikaj kolejne punkty trasy, potem „Jedź trasą”": "click the route points one by one, then “Drive route”",
    "Najpierw wskaż punkty trasy na mapie.": "Pick route points on the map first.",
    "{} poza mapą · {} przy ścianach": "{} off the map · {} at walls", "Jedź": "Go", "Zablokuj": "Block",
    "Brak powtarzających się problemów.": "No recurring problems.", "Strefa {}: {}": "Zone {}: {}", "Ściana {}: {}": "Wall {}: {}",
    "Brak stref zakazanych i wirtualnych ścian.": "No no-go zones or virtual walls.",
    "Wysłać robota do ({}, {})?": "Send the robot to ({}, {})?",
    "Dodać strefę zakazaną wokół miejsca {} ({} × {} cm)? Robot przestanie tam wjeżdżać.": "Add a no-go zone around spot {} ({} × {} cm)? The robot will stop going there.",
    "Kliknij JEDEN róg strefy zakazanej": "Click ONE corner of the no-go zone", "Kliknij PRZECIWLEGŁY róg": "Click the OPPOSITE corner",
    "Kliknij POCZĄTEK wirtualnej ściany": "Click the START of the virtual wall", "Kliknij KONIEC ściany": "Click the END of the wall",
    "{} · ostatnio {}": "{} · last {}", "{} dni": "{} days", "{} dzień": "{} day",
    // --- Valetudo functions ---
    "od": "from", "do": "to", "Zapisz": "Save", "Dźwięk": "Sound", "Zapisz głośność": "Save volume", "Pokoje": "Rooms",
    "Dodaj": "Add", "moc: bez zmiany": "power: unchanged", "Bez zaznaczonych pokoi = całe mieszkanie.": "No rooms selected = the whole flat.",
    "Dodaj sprzątanie": "Add cleaning", "Mapa": "Map", "Skanuj sieci": "Scan networks",
    "Zmień sieć (robot się rozłączy)": "Change network (the robot will disconnect)",
    "SSID, hasło. Po zmianie robot dostanie nowy adres IP w nowej sieci.": "SSID, password. After the change the robot gets a new IP address in the new network.",
    "Ustawienia firmware (quirks)": "Firmware settings (quirks)", "Zdarzenia Valetudo": "Valetudo events",
    "Brak nowych zdarzeń.": "No new events.", "Log i system": "Log and system", "poziom logu": "log level", "Pokaż log": "Show log",
    "pamięć wolna": "free memory", "· obciążenie": "· load", "· działa": "· uptime",
    "Włączyć trwałą mapę?": "Enable the persistent map?",
    "Wyłączyć trwałą mapę? Robot będzie budował mapę od zera przy każdym sprzątaniu, a strefy i pokoje przestaną działać.":
      "Disable the persistent map? The robot will build the map from scratch on every cleanup, and zones and rooms will stop working.",
    "Zmienić „{}” na „{}”? To ustawienie firmware robota.": "Change “{}” to “{}”? This is a robot firmware setting.",
    "Pobrać i zainstalować pakiet głosowy z podanego adresu?": "Download and install the voice pack from the given address?",
    "Wybierz dwa różne pokoje.": "Choose two different rooms.", "Połączyć te pokoje w jeden?": "Join these rooms into one?",
    "Kliknij POCZĄTEK linii podziału pokoju": "Click the START of the room split line", "Kliknij KONIEC linii podziału": "Click the END of the split line",
    "Uruchomić to sprzątanie teraz?": "Run this cleanup now?", "Usunąć to zaplanowane sprzątanie?": "Delete this scheduled cleanup?",
    "Zaznacz co najmniej jeden dzień.": "Select at least one day.",
    "Przywrócić tę kopię mapy? Obecna mapa zostanie zastąpiona.": "Restore this map snapshot? The current map will be replaced.",
    "Usunąć mapę robota? Pokoje, strefy zakazane i nazwy pokoi znikną. Tego nie da się cofnąć.": "Delete the robot's map? Rooms, no-go zones and room names will be gone. This cannot be undone.",
    "Wpisz USUŃ, żeby potwierdzić:": "Type DELETE to confirm:", "USUŃ": "DELETE",
    "Przełączyć robota na sieć „{}”? Jeśli hasło jest złe, robot zniknie z sieci i trzeba będzie go zresetować.":
      "Switch the robot to network “{}”? If the password is wrong, the robot disappears from the network and has to be reset.",
    "ładowanie…": "loading…", "Brak połączenia z lokalnym serwerem panelu (server.py). Ponawiam…": "No connection to the local panel server (server.py). Retrying…",
    "Przywróć": "Restore", "Kopia {}": "Snapshot {}", "Test": "Test", "(ukryta)": "(hidden)",
    "mapa": "map", "Salon": "Living room", "Kuchnia": "Kitchen",   // "mapa" = pose source; rooms of the mock robot
    "pobór": "draw", "ładowanie": "charging", "stoi": "stopped", "Sprzątanie": "Cleaning", "Skanowanie": "Scan", "Teraz": "Now",
    "{} promieni": "{} rays", "zgodność odbicia {}": "reflection match {}", "{} ↔ {}": "{} ↔ {}",
    "zostało {} h z {} h": "{} h left of {} h", "zostało {} h": "{} h left",
    "do wymiany lub zresetowania licznika": "replace the part or reset the counter",
    "Brak zaplanowanych sprzątań. Dodasz je niżej, w „Valetudo · wszystkie funkcje”.": "No scheduled cleanups. Add them below, in “Valetudo · all functions”.",
    "Brak zaplanowanych sprzątań.": "No scheduled cleanups.", "(wyłączone)": "(off)",
    "Dźwięk testowy": "Test sound", "Pakiet głosowy:": "Voice pack:", "Wgraj pakiet głosowy": "Install voice pack",
    "adres .tar.gz, kod języka, suma MD5": "URL of the .tar.gz, language code, MD5 sum", "Zmień nazwę": "Rename", "Połącz": "Join",
    "Wskaż linię podziału": "Pick the split line", "Usuń mapę (reset)": "Delete map (reset)",
    "trwała mapa (wyłączenie = mapa od zera przy każdym sprzątaniu)": "persistent map (off = a map from scratch on every cleanup)",
    "Wyślij robota w to miejsce": "Send the robot to this spot", "Strefa zakazana wokół tego miejsca": "No-go zone around this spot",
    "LIDAR + CZUJNIKI": "LIDAR + SENSORS", "tylko Valetudo": "Valetudo only", "W toku": "Running", "Zapisano": "Recorded",
    "stan nieznany": "state unknown", "{} cm od celu": "{} cm from target", "Pobierz i zainstaluj": "Download and install",
    "Podziel": "Split", "lustro {}": "mirror {}", "drzwi {}": "door {}", "drzwi? {}": "door? {}", "przejście {}": "passage {}",
    "szyba/otwór {}": "glass/opening {}", "brak odbicia {}": "no echo {}",
    "Uderzenie zderzakiem (L)": "Bumper hit (L)", "Uderzenie zderzakiem (P)": "Bumper hit (R)", "Uderzenie zderzakiem (L+P)": "Bumper hit (L+R)",
    "Serwis: inspekcja miejsc problemowych + skan mieszkania": "Service: inspect the problem spots + scan the flat",
    // --- scene.js / analysis.py kinds ---
    "lustro": "mirror", "szyba/otwór": "glass/opening", "brak odbicia": "no echo", "drzwi?": "door?", "drzwi": "door",
    "przejście": "passage", "zamknięte": "closed", "otwarte": "open", "Pokój {}": "Room {}", "Pokój": "Room",
    "drobna": "small", "duża": "large", "drzwi zamknięte na mapie": "door closed on the map",
    // --- history.js / sensors.py events ---
    // the history table shows only the FIRST word of these labels as a tag - keep it distinctive
    "Uderzenie zderzakiem": "Bumper hit", "Uderzenie wieżyczką lidaru": "Lidar turret hit", "Czujnik krawędzi": "Cliff sensor",
    "Czujnik krawędzi (schody/próg)": "Cliff sensor (stairs/threshold)", "Koło opadło / podniesiony": "Wheel dropped / lifted",
    "Koło opadło / robot podniesiony": "Wheel dropped / robot lifted", "Większy opór szczotki (dywan?)": "Carpet? Higher brush load",
    "Szczotka hamowana (kabel?)": "Brush slowed (cable?)", "Szczotka mocno hamowana (kabel? skarpetka?)": "Brush heavily slowed (cable? sock?)",
    "Zwiększony opór jazdy": "Drive load higher", "Przechył (próg / dywan)": "Tilt (threshold / carpet)",
    "Przechył (próg / gruby dywan)": "Tilt (threshold / thick carpet)", "Ostrzeżenie MCU": "Warning (MCU)", "Błąd MCU": "Error (MCU)",
    "Ostrzeżenie mikrokontrolera": "Microcontroller warning", "Błąd mikrokontrolera": "Microcontroller error",
    "Słaby sygnał Wi-Fi": "Weak Wi-Fi signal", "Błąd robota": "Robot error", "Zmiana stanu": "State change",
    "Drzwi zmieniły stan": "Door changed state", "Skanowanie mieszkania": "Scan of the flat", "Miejsce problemowe": "Problem spot",
    "Prąd kół": "Wheel current", "Przechył °": "Tilt °", "Pobór W": "Power W",
    "Brak zapisanych przejazdów - pojawią się po pierwszym sprzątaniu.": "No saved runs - they appear after the first cleanup.",
    // --- server.py messages ---
    "Stan: {}": "State: {}", "Słaby sygnał Wi-Fi {} dBm": "Weak Wi-Fi signal {} dBm", "Start: {}": "Start: {}",
    "Koniec przejazdu ({})": "Run ended ({})", "Miejsce problemowe: {}× w tym samym miejscu ({})": "Problem spot: {}× in the same place ({})",
    "Drzwi {}: {}": "Door {}: {}", "Cel „{}” nieosiągalny: zatrzymanie {} cm przed nim (przeszkoda?)": "Target “{}” unreachable: stopped {} cm before it (obstacle?)",
    "ta operacja nie jest dozwolona z panelu": "this operation is not allowed from the panel",
    "{} wymaga potwierdzenia (nagłówek X-Confirm: tak)": "{} needs confirmation (header X-Confirm: tak)",
    "tryb mock - brak robota": "mock mode - no robot", "robot zgłosił błąd": "the robot reported an error",
    "sterowanie ręczne nie obraca robota": "manual control does not turn the robot", "utknął": "stuck",
    "brak miejsca na podjazd": "no room to approach", "nie dojechał przed przejście": "did not reach the opening",
    "przejechał {} cm za linię drzwi ({})": "passed {} cm beyond the door line ({})", "nie przejechał ({})": "did not pass ({})",
    "robot zajęty (status: {})": "robot busy (status: {})", "{} punktów": "{} points",
    "jadę do punktu {}/{} ({})": "driving to point {}/{} ({})",
    "punkt {} ({}) zablokowany na mapie robota: próbuję przez {}": "point {} ({}) blocked on the robot map: trying through {}",
    "przejechał": "passed", "próbuję przejechać: {} przy ({}, {})": "trying to pass: {} at ({}, {})",
    "Próba przejazdu ({}): {}": "Pass attempt ({}): {}", "Mapa się powiększyła: {} nowych punktów": "Map grew: {} new points",
    "powrót na stację": "returning to the dock", "zakończone": "finished", "przerwane, powrót na stację": "stopped, returning to the dock",
    "błąd: {}": "error: {}", "brak świeżego skanu lidaru (lidar się nie kręci?)": "no fresh lidar scan (lidar not spinning?)",
    "dojechał": "arrived", "nie dojechał": "did not arrive", "obrócony": "turned", "nie obrócił się do końca": "did not finish the turn",
    "start sprzątania strefy": "zone cleaning started", "start sprzątania pokoi": "room cleaning started",
    "steps: lista 1..{} kroków": "steps: a list of 1..{} steps", "krok {}: brak znanej operacji ({})": "step {}: no known operation ({})",
    "krok {}/{}: {}": "step {}/{}: {}", "sterowanie ręczne wymaga postoju robota (status: {})": "manual control needs the robot to stand still (status: {})",
    "program zakończony": "program finished", "program przerwany": "program stopped", "błąd programu: {}": "program error: {}",
    "za duże": "too large", "już trwa": "already running", "złe parametry: {}": "bad parameters: {}",
    "pusty plik lub punkty poza mapą": "empty file or points outside the map",
    // --- analysis.py hotspots ---
    "uderzenie": "bump", "uderzenie wieżyczką": "turret bump", "najazd / przechył": "run-over / tilt", "opór kół": "wheel load",
    "wciąganie przez szczotkę": "pulled in by the brush", "krawędź": "cliff", "uniesienie koła": "wheel lift",
    "robot na coś najeżdża (próg, dywan, kabel, przedmiot pod robotem)": "the robot runs onto something (threshold, carpet, cable, object under the robot)",
    "uderzenia w ścianę/mebel z mapy (zwykle normalne przy sprzątaniu krawędzi)": "bumps into a wall/furniture from the map (usually normal when cleaning edges)",
    "przeszkoda, której nie ma na mapie (np. nogi krzesła, przedmiot na podłodze)": "an obstacle that is not on the map (e.g. chair legs, an object on the floor)",
  };

  const rx = s => s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  const exact = new Map(), pats = [];
  for (const [k, v] of Object.entries(D))
    if (k.includes("{}")) pats.push([new RegExp("^" + k.split("{}").map(rx).join("(.*?)") + "$", "s"), v, k.length]); else exact.set(k, v);
  pats.sort((a, b) => b[2] - a[2]);   // most specific first
  const cache = new Map(), missing = window.i18nMissing = new Set();

  function tr(s) {
    if (typeof s !== "string" || !/[A-Za-zÀ-ž]{2}/.test(s)) return s;
    if (s.trimStart().startsWith("Kroki programu")) return HELP;
    const m = s.match(/^(\s*)(.*?)(\s*)$/s), core = m[2].replace(/\s+/g, " ");
    let out = cache.get(core);
    if (out === undefined) {
      out = exact.get(core) ?? null;
      if (out === null) for (const [re, v] of pats) {
        const g = core.match(re);
        if (g) { let i = 1; out = v.replace(/\{\}/g, () => tr(g[i++])); break; }
      }
      // composed strings: "a · b", "a, b", "label: value" -> translate the parts
      if (out === null) for (const sep of [" · ", ", ", ": "]) {
        if (!core.includes(sep)) continue;
        const joined = core.split(sep).map(tr).join(sep);
        if (joined !== core) { out = joined; break; }
      }
      // "→ text", "⚠ text", "▼ text (20)", "12× text 3", "· text ·"
      if (out === null) {
        const g = core.match(/^(→ |⚠ |— |▼ |\d+× |· )?(.*?)( \d+| \(\d+\)| ·)?$/);
        if (g && (g[1] || g[3]) && tr(g[2]) !== g[2]) out = (g[1] ?? "") + tr(g[2]) + (g[3] ?? "");
      }
      if (out === null && /[ąćęłńóśźż]/i.test(core) && missing.size < 500) missing.add(core);
      if (cache.size > 4000) cache.clear();
      cache.set(core, out);
    }
    return out === null ? s : m[1] + out + m[3];
  }
  window.i18n = tr;

  const SKIP = "script,style,textarea,code,pre:not(#pg-doc),[data-no-i18n]", ATTRS = ["title", "placeholder", "aria-label"];
  const text = n => { if (n.parentElement?.closest(SKIP)) return; const v = tr(n.nodeValue); if (v !== n.nodeValue) n.nodeValue = v; };
  const attrs = el => { for (const a of ATTRS) { const v = el.getAttribute(a); if (v) { const w = tr(v); if (w !== v) el.setAttribute(a, w); } } };
  function walk(root) {
    if (root.nodeType === 3) return text(root);
    if (root.nodeType !== 1) return;
    attrs(root);
    const w = document.createTreeWalker(root, NodeFilter.SHOW_TEXT | NodeFilter.SHOW_ELEMENT);
    for (let n; (n = w.nextNode());) n.nodeType === 3 ? text(n) : attrs(n);
  }
  walk(document.documentElement);   // <title> and anything parsed before this script
  new MutationObserver(ms => {
    for (const m of ms) m.type === "childList" ? m.addedNodes.forEach(walk) : m.type === "characterData" ? text(m.target) : attrs(m.target);
  }).observe(document, { subtree: true, childList: true, characterData: true, attributes: true, attributeFilter: ATTRS });

  for (const f of ["alert", "confirm", "prompt"]) { const o = window[f].bind(window); window[f] = (msg, ...a) => o(tr(msg), ...a); }
  const C = CanvasRenderingContext2D.prototype;
  for (const f of ["fillText", "measureText"]) { const o = C[f]; C[f] = function (s, ...a) { return o.call(this, tr(String(s)), ...a); }; }
})();
