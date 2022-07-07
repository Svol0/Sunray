## Diese Version entspricht der Sunray Release Version 1.0.276 mit zusätzlichen Funktionen:
- Damit diese Version kompiliert werden kann, wird zusätzlich die Bibliothek „RunningMedian“ von Rob Tillaart benötigt. Diese kann in der Arduino IDE durch den Bibliotheksverwalter (Strg+Umschalt+I) installieren werden. Als Version habe ich die 0.3.6 genommen.
- Die Parameter, die in dieser Auflistung erwähnt werden sind in der **config_example.h** enthalten. Eine kurze Beschreibung und die Einheit des Parameters ist meistens vorhanden.

---
### REBOOT DES GPS-MODULS BEI FIX TIMEOUT DURCH **GPS_REBOOT_RECOVERY_FLOAT_TIME**
Es kann vorkommen, dass der Mäher nach Stop durch überschreiten der Fix timeout Zeit den Zustand Float auch nach vielen Minuten nicht mehr verlässt. In der Vergangenheit hat sich gezeigt, dass ein Reboot des GPS-Receivers gelegentlich einen neuen Fix bringen kann. Mit dem Parameter **GPS_REBOOT_RECOVERY_FLOAT_TIME** ist es jetzt möglich eine maximale Wartezeit in Minuten im Zustand Float festzulegen. Wird diese Wartezeit überschritten, erfolgt ein Reboot des GPS-Receivers. Wird als Wartezeit eine 0 (Null) eingetragen, erfolgt kein GPS-Reboot.  
Für diese Funktion ist zusätzlich die Einstellung folgender Parameter erforderlich:  
#define **GPS_REBOOT_RECOVERY**  true  
#define **REQUIRE_VALID_GPS**  true  
