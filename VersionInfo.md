## Diese Version entspricht der Sunray Release Version 1.0.219 mit zusätzlichen Funktionen:
- Damit diese Version kompiliert werden kann, wird zusätzlich die Bibliothek „RunningMedian“ von Rob Tillaart benötigt. Diese kann in der Arduino IDE durch den Bibliotheksverwalter (Strg+Umschalt+I) installiert werden. Als Version habe ich die 0.3.6 genommen.
- Die Parameter, die in dieser Auflistung erwähnt werden sind in der **config_example.h** enthalten. Eine kurze Beschreibung und die Einheit des Parameters ist meistens vorhanden.
---
### Geänderte LINEAR_RAMP
Der Mäher beschleunigt und verzögert mit den unter **ACC_RAMP** und **DEC_RAMP** einstellbaren Rampen. 
Das Erreichen der Zielposition bleibt dabei genau, da der benötigte Bremsweg in Abhängigkeit der aktuellen Fahrgeschwindigkeit und der Verzögerungsrampe berechnet wird.
Die einstellbaren Zeiten beziehen sich auf das erreichen der Endgeschwindigkeit, die unter **MOTOR_MAX_SPEED** eingestellt werden muss.
Alle anderen Geschwindigkeitswerte für die Fahrantriebe sollten höchstens gleich, oder kleiner gewählt werden. Siehe dazu *GESCHWINDIGKEITS- UND ZEITWERTE*.  
Die Geschwindigkeit **MOTOR_MIN_SPEED** sollte so eingestellt werden, dass der Mäher sich noch sicher damit bewegen kann, da diese mit zur Anfahrt des Zielpunktes verwendet wird.  
In folgenden Situationen wird der Mäher ohne Bremsrampe sofort gestoppt:
- Auslösen des Bumpers
- GPS-Signal wechselt zu INVALID (nur wenn **REQUIRE_VALID_GPS** auf true)
- Kidnapping Funktion ausgelöst wird (nur wenn **KIDNAP_DETECT** auf true)
- Motorstörungen

Beispiel für **ACC_RAMP** = 2000 und **MOTOR_MAX_SPEED** = 0.50  
Diese Einstellung bewirkt, dass der Mäher innerhalb von 2 Sekunden aus dem Stand bis zu der Endgeschwindigkeit von 0,5m/s beschleunigt wird.
- **ACC_RAMP**  
Beschleunigungsrampe in Millisenkunden die der Mäher aus dem Stillstand bis zur **MOTOR_MAX_SPEED** benötigt
- **DEC_RAMP**  
Verzögerungsrampe in Millisekunden die der Mäher benötigt, um von der Geschwindigkeit **MOTOR_MAX_SPEED** bis zum Stillstand zu kommen.

---
### GESCHWINDIGKEITS- UND ZEITWERTE von MrTreeBark
Folgende Werte für die Fahr-, Dreh-, Beschleunigungs- und Verzögerungsgeschwindigkeit sind in die **config.h** aufgenommen:
- **MOW_SPINUPTIME**  
ist die Zeit in Millisekunden, die der Mäher nach starten des Mähmotors wartet, bis er losfährt (damit die Mähscheibe schon auf Geschwindigkeit ist, bevor Sie auf das Gras trifft
- **OVERLOADSPEED**  
Geschwindigkeit in m/s auf die die Fahrantriebe gedrosselt werden, wenn einer der Fahrantriebe den unter **MOTOR_OVERLOAD_CURRENT** eingestellten Wert oder der Mähmotor den unter **MOW_OVERLOAD_CURRENT** eingestellten Wert überschreitet
- **ROTATETOTARGETSPEED**  
Die Drehgeschwindigkeit in rad mit der der Mäher sich auf der Stelle Richtung Ziel dreht. Kann wie folgt berechnet werden:  
Gewünschte Winkeländerung/Sekunde / 180 * Pi  
Beispiel: Der Mäher soll sich pro Sekunde um 45Grad drehen (45/180) x 3,14 = 0,785
- **TRACKSLOWSPEED**  
Geschwindigkeit in m/s mit der der Mäher den Weg von und zur Ladestation fährt. Bei Verwendung der Funktion "GPS-REBOOT AN BESTIMMTEN DOCKINGPUNKT" wird diese Geschwindigkeit nicht für die komplette Strecke verwendet.
- **APPROACHWAYPOINTSPEED**  
Geschwindigkeit in m/s mit der sich der Mäher die letzten 25cm bis zum Erreichen der Zielposition nähert.  
Wird nur verwendet, wenn **LINEAR_RAMP** nicht aktiviert ist.
- **FLOATSPEED**  
Geschwindigkeit in m/s mit der der Mäher maximal Fährt, wenn nur GPS-Float vorhanden ist.
- **SONARSPEED**  
Geschwindigkeit in m/s mit der der Mäher weiterfährt, wenn das Sonar z.B. durch ein Hindernis ausgelöst wird.
- **DOCKANGULARSPEED**  
Die Drehgeschwindigkeit in rad mit der der Mäher sich beim fahren von und zur Ladestation auf der Stelle Richtung Ziel dreht. Berechnung siehe **ROTATETOTARGETSPEED**
- **OBSTACLEAVOIDANCESPEED**  
Geschwindigkeit in m/s mit der der Mäher versucht ein Hindernis zu umfahren.
- **OBSTACLEAVOIDANCEWAY**  
Der Weg in Metern die der Mäher Rückwärts vom Hindernis zurückfährt.
 - **MOTOR_MAX_SPEED**  
 Geschwindigkeit in m/s die der Mäher maximal Fahren darf. Da in der Sunray-App per Schieberegler Werte zwischen 0,01 und 0,59m/s eingestellt werden können, ist diese Begrenzung zur Sicherheit so zu Wählen, dass der Mäher noch Ordnungsgemäß fährt und eine versehentliche Überlastung vermieden wird.
- **MOTOR_MIN_SPEED**  
Geschwindigkeit in m/s bei der sich der Mäher noch sicher Bewegt.

---
### SKALIERUNG VON SPEED ALS MAXIMALE GESCHWINDIGKEIT FÜR DEN APP-JOYSTICK
Mit setzen des Parameter **USE_SETSPEED_FOR_APPJOYSTICK** auf true wird die mit dem Schieberegler eingestellte Geschwindigkeit (aber höhchstens bis **MOTOR_MAX_SPEED**) für den maximalausschlag des Joysticks in der Sunray-App verwendet. Will man sehr genau Navigieren, wählt man über den Schieberegler eine kleine Geschwindigkeit, will man weite Strecken überbrücken wählt man eine große Geschwindigkeit.

---
### KONTROLLE OB DER BUMPER BLOCKIERT IST
Es kann vorkommen, dass ein Ast oder anderer Fremdkörker den Bumper so blockiert, dass dieser die ganze Zeit betätigt bleibt. Der Mäher versucht dann immer wieder zurück zu fahren und auszuweichen, bis er im schlimsten Fall außerhalb des Perimeters landet.
Mit dem Parameter **BUMPER_MAX_TRIGGER_TIME** kann eine maximale permanente Betätigungsdauer in Sekunden festgelegt werden. Bei Überschreitung wird ein Bumper-Fehler ausgelöst und der Mäher bleibt stehen. Ein Wert von 0 (Null) deaktiviert die Überwachung.

---
### AUSLÖSEVERZÖGERUNG DES BUMPER
Mit dem Parameter **BUMPER_TRIGGER_DELAY** kann die Verzögerung in Millisekunden für das Auslösen des Bumpers eingestellt werden. Wenn der Bumper z.B. nur kurz durch einen etwas stärkeren Grashalm ausgelöst wird, kann durch den hier Eingestellten Wert dieses einmalige Ereignis unterdrückt werden. Steht der Wert z.B. auf 200, so würde kein Hindernis erkannt werden, wenn der Bumper kürzer als 200ms betätigt wird. Wird die Zeit überschritten, wird die Hindernisumfahrung aktiviert.

---
### SKALIERUNG DER STANLEY-PARAMETER von MrTreeBark  
Mit dem Parameter **MAP_STANLEY_CONTROL** auf true werden die Stanley-Parameter für die langsame (**DOCKANGULARSPEED**) und die normale (**MOTOR_MAX_SPEED**) linear über den gesamten Geschwindigkeitsbereich von **MOTOR_MIN_SPEED** bis **MOTOR_MAX_SPEED** im Verhältnis zugeordnet. Dieses sollte zu einem deutlich besserem Regelverhalte führen, da die Stanley-Werte der Geschwindigkeit entsprechend skaliert werden.  
**Beispiel:**  
**STANLEY_CONTROL_K_SLOW**   0.1  
**STANLEY_CONTROL_K_NORMAL** 1.0  
**MAP_STANLEY_CONTROL** true 
**MOTOR_MIN_SPEED** 0.05  
**MOTOR_MAX_SPEED** 0.50  
**DOCKANGULARSPEED** 0.15  
**Speed --> Stanley-Wert --> Stanley-Wert mit MAP_STANLEY_CONTROL false**   
 0.05 ---> 0.1 ---> 1.0  
 0.15 ---> 0.3 ---> 0.1  
 0.25 ---> 0.5 ---> 1.0  
 0.50 ---> 1.0 ---> 1.0  

---
### MÄHMOTORTEST
Der Mähmotortest kann per Serial monitor mit Kommando AT+D aktiviert werden. Über den Serial monitor werden dabei Informationen für den Ablauf des Testes ausgegeben.
Beschreibung des Ablaufs:
- AT+D in der Konsole eingeben
- START/STOP Taste mind. 5 Sekunden gedrückt halten, bis ein akustisches Signal ertönt
- In weniger als 10 Sekunden wird der Mähmotor gestartet. Akustische Signale weisen als Warnung darauf hin.
- der Mähmotor wird auf einen PWM-Wert von 100 langsam hochbeschleunigt
- nach erreichen der Geschwindigkeit ertönt ein kurzer Signalton
- alle 10 Sekunden wird jetzt der PWM-Wert um 5 erhöht, bis maximal 255. Bei jeder Erhöhung ertönt ein kurzer Signalton.
- der Test kann durch drücken der START/STOP Taste jederzeit gestoppt werden. Der Mähmotor läuft dann an einer kurzen Rampe bis zum stopp runter
- falls man die Verdindung zum Serial monitor unterbrochen hatte, kann man diese jetzt wieder herstellen um den zuletzt getesteten PWM-Wert auslesen zu können.
Dafür hat man ca. 2 Minuten Zeit. Sobald man die Verbindung wieder hergestellt hat, kann man durch drücken der START/STOP-Taste die Wartezeit unterbrechen.

Dieser Test ist hauptsächlich dafür gedacht, die Lautstärke des Mähers beim Mähen testen zu können, um eine für sich (und die Nachbarn ;-) ) akzeptable Geschwindigkeit zu ermitteln. Der Ermittelte PWM-Wert kann dann in der **config.h** bei **MAX_MOW_RPM** eingetragen werden. 

---
### KALTSTART DES GPS-MODULS BEI GPS-REBOOT
Wenn das GPS neu gestartet werden soll, wird normaler weise ein Warmstart durchgeführt. Bei einem Warmstart bleiben die bisherigen Satelliteninformationen erhalten. Es wird nur die Software für das GNSS auf dem GPS-Empfänger neu gestartet.  
Bei einem Kaltstart werden gegenüber dem Warmstart auch alle Satellitenpositionen verworfen und müssen im Anschluss wieder neu erfasst werden. Wird der Parameter **GPS_COLD_REBOOT** auf true gesetzt, wird anstelle eines Warmstarts jedesmal ein Kaltstart durchgeführt.

---
### REBOOT DES GPS-MODULS BEI FIX TIMEOUT DURCH **GPS_REBOOT_RECOVERY_FLOAT_TIME**
Es kann vorkommen, dass der Mäher nach Stop durch überschreiten der Fix timeout Zeit den Zustand Float auch nach vielen Minuten nicht mehr verlässt. In der Vergangenheit hat sich gezeigt, dass ein Reboot des GPS-Receivers gelegentlich einen neuen Fix bringen kann. Mit dem Parameter **GPS_REBOOT_RECOVERY_FLOAT_TIME** ist es jetzt möglich eine maximale Wartezeit in Minuten im Zustand Float festzulegen. Wird diese Wartezeit überschritten, erfolgt ein Reboot des GPS-Receivers. Wird als Wartezeit eine 0 (Null) eingetragen, erfolgt kein GPS-Reboot.  
Für diese Funktion ist zusätzlich die Einstellung folgender Parameter erforderlich:
#define **GPS_REBOOT_RECOVERY**  true
#define **REQUIRE_VALID_GPS**  true

---
### KORREKTUR DER STROMVERBRAUCHSANZEIGE
Der in der App angezeigte Stromverbrauch berücksichtigt nur die von den Motortreibern gelieferten Stromverbrauchswerte, da die PCB1.3 bzw. 1.4 über keine direkte Strommessung zur Ermittelung des Gesamtverbrauchs verfügt. Die auf dem Board verbaute INA misst nur den Ladestrom. Um dennoch eine einigermaßen genaue Stromverbrauchsanzeige zu bekommen, kann eine Anpassung durch Verwendung der folgenden Parameter durchgeführt werden:  
- **GEAR_DRIVER_IDLE_CURRENT**  
Verbrauchsvert in Ampere eines Motortreibers für den Fahrantrieb bei stehendem Motor.  
- **MOW_DRIVER_IDLE_CURRENT**  
Verbrauchsvert in Ampere des Mähmotortreibers bei stehendem Motor.  
- **BOARD_IDLE_CURRENT**  
Grundverbrauch in Ampere, den das hat Boards, wenn alle installiereten Komponenten eingeschaltet sind, ohne die Motortreiber.  

Wenn man die Anpassung nicht vornehmen möchte, kann man die Werte einfach auf 0.00 lassen.  

Um die Werte bei Verwendung von BL-Treibern zu ermitteln, kann wie folgt vorgegangen werden:  
- Mäher spannungslos machen
- Versorgungsstecker (P15, P18 und P37) der BL-Treiber abziehen.
- Alle anderen Komponenten bleiben gesteckt.
- Mäher einschalten und den Strombedarf z.B. mit einem Zangenamperemeter ermitteln.
- Der ermittelte Stromwert kann bei **BOARD_IDLE_CURRENT** eingetragen werden.
- Mäher wieder Spannungslos machen.
- Versorgungsstecker P37 für den Mähmotortreiber wieder einstecken.
- Mäher einschalten und den Strombedarf z.B. mit einem Zangenamperemeter ermitteln.
- Den für **BOARD_IDLE_CURRENT** gemessenen Strom von dem soeben ermittelten Stromwert abziehen und bei **MOW_DRIVER_IDLE_CURRENT** eintragen.
- Mäher wieder Spannungslos machen.
- Versorgungsstecker P15 und P18 für die Fahrantriebe wieder anstecken.
- Mäher einschalten und den Gesamtstrom ermitteln. Von diesem werden die Werte für **BOARD_IDLE_CURRENT** und **MOW_DRIVER_IDLE_CURRENT** abgezogen. Den Wert, den man dabei erhält durch 2 Teilen und bei **GEAR_DRIVER_IDLE_CURRENT** eintragen.

Nachdem man anschließend das Projekt neu kompiliert und übertragen hat, sollte jetzt der in der App angezeigte Wert mit dem realen Wert übereinstimmen.

---



###Reboot GPS at a specific docking point
  - Bei Fahrt zur Docking-Station wird über den Parameter Wert von "DOCK_SLOW_ONLY_LAST_POINTS" die Position des Dockingpunktes angegeben (betrachtet aus Richtung Dockingstation), ab welchem mit langsamer Geschwindigkeit (linear = 0,1) weiter gefahren wird. Alle Dockingpunkte vorher werden mit der normalen (setspeed) Geschwindigkeit angefahren. Ein Wert von "Null" bewirkt, dass alle Punkte mit langsamer Geschwindigkeit angefahren werden.
  - Geht GPS-Fix auf dem Weg zwischen dem vorletzten Dockingpunkt und der Dockingstation verloren, wird die Fahrt mit IMU und ODO fortgesetzt.
  - Bei Fahrt aus der Docking-Station wird über den Parameter Wert von "DOCK_POINT_GPS_REBOOT" die Position des Dockingpunktes angegeben (betrachtet aus Richtung Dockingstation), ab welchem ein GPS-Reboot durchgeführt werden soll. Der Mäher wartet dann, bis ein GPS-Fix vorhanden ist, und setzt das undocking fort. Da der Mäher dann eine korrekte Position hat, wird der Rest der Dockingstrecke vorwärts gerichtet, mit normaler Geschwindigkeit und GPS-Unterstützung fortgesetzt. Bei aktiviertem "DOCK_IGNORE_GPS" wird nur bis zum GPS-Reset Punkt ohne GPS-Unterstützung gefahren. Ein Wert von "Null" bewirkt keinen GPS-Reboot beim undocking.
  - Der Dockingpunkt für den GPS-Reboot sollte so gewählt werden, dass dieser sich an einer Stelle befindet, wo der Mäher einfach einen guten GPS-FIX bekommen kann.

  - **WICHTIGE INFO:** Bitte min. 2 Dockingpunkte mehr anlegen als bei „DOCK_POINT_GPS_REBOOT“ eingestellt ist. Das liegt an der zusätzlichen retry Docking by obstacle Funktion, die in den Versionen > 1.0.230 eingeführt wurde. Trifft der Mäher beim Docking auf ein Hinderniss, fährt er wieder zurück bis zum GPSßReboot Punkt, rebootet das GPS und versucht es erneut.
 
  - **Detailierter Ablauf bei Verwendung von "DOCK_POINT_GPS_REBOOT" in Kombination mit "DOCK_IGNORE_GPS" = true:**
    - Mäher fährt rückwärts mit langsamer Geschwindigkeit und nur mit IMU/ODO bis zu dem bei "DOCK_POINT_GPS_REBOOT" eingestelltem Dockingpoint.
    - Dort angekommen wird ein GPS-Reset durchgeführt und der Mäher stoppt.
    - Während auf ein GPS-FIX gewartet wird, ertönt alle 5 Sek. ein kurzer Doppelton durch den Buzzer als akustisches Feedback.
    - Ist ein GPS-FIX vorhanden, muss dieses für mindestens 20 Sek. stabil bleiben. Ein einfacher Ton wird alle 5 Sek. abgespielt. Schwankt die GPS-Position zu stark, wird die Wartezeit wieder resetet und ein Doppelton mit längerer Impulsdauer wird abgespielt.
    - Ist die GPS-Position für mehr als 20 Sek. stabil, wird der Undockingprozess fortgesetzt. Da eine stabile Position besteht, erfolgt die weitere Fahrt vorwärts gerichtet, mit normaler Geschwindigkeit (setSpeed) und mit GPS-Unterstützung.

  Wer kein akustisches Feedback möchte, kann alle Zeilen die mit "if (!buzzer.isPlaying())" beginnen einfach auskommentieren.
  Zur Zeit sind auch noch Consolen-Ausgaben vorhanden, um eine bessere Kontrolle der Funktion beim testen zu haben.
