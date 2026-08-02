# Test-Basis: PR-1498 - System Diagnostics UI & Telemetry REST API

Dieses Dokument beschreibt die BDD-Testszenarien für die Telemetrie-REST-API im `pib-backend` sowie den neuen "Diagnostics"-Tab im System-Menü des `cerebra`-Frontends.

---

## Qualitätsziele & Anforderungen

1. **Backend Telemetrie REST API (`pib-backend`):**
   - `GET /api/v1/diagnostics/summary`: Liefert eine Gesamtübersicht der Systemgesundheit inklusive Ampel-Status, CPU-Temperatur, Disk-Status, Container-Gesundheit und Bricklet-Status.
   - `GET /api/v1/diagnostics/bricklets`: Liefert Telemetriedaten aller konfigurierten Bricklets (Versorgungsspannung in Volt, Pin-Stromstärken in mA/A, Einzelstatus).
   - `GET /api/v1/diagnostics/system`: Liefert Host-Metriken (CPU-Temperatur in °C, Speicherbelegung auf Datenträger) sowie den Gesundheitsstatus aller verbundenen Docker-Container.

2. **Cerebra Frontend (`cerebra`):**
   - **Navigation & Tab:** Integration eines neuen "Diagnostics"-Tabs im System-Menü unter dem Pfad `/system/diagnostics`.
   - **Ampel-Indikatoren (Traffic-Light Badges):** Visuelle Statusanzeigen (OK/Grün, Warnung/Gelb, Fehler/Rot) für Gesamtstatus, CPU, Speicherplatz, Container und Bricklets.
   - **Bricklet-Karten:** Übersichtskarten für verbundene Bricklets mit Anzeige von Spannung, Gesamtstromstärke und Pin-Stromstärken.
   - **Host-Metriken & Container-Status:** Anzeige von CPU-Temperatur, Festplattenplatz und Container-Gesundheitszuständen.
   - **Refresh-Button:** Manuelles Aktualisieren aller Telemetriedaten auf Knopfdruck.

---

## BDD Szenarien

### Szenario 1: Abfragen der System-Diagnose-Zusammenfassung (GET /api/v1/diagnostics/summary)
- **Gegeben:** Das Flask-Backend läuft und der Diagnostics-Dienst ist bereit.
- **Wenn:** Ein HTTP GET-Request an `/api/v1/diagnostics/summary` gesendet wird.
- **Dann:** Antwortet das Backend mit Status Code 200 OK und einem JSON-Objekt, das `overallStatus`, `cpuTemperature`, `diskSpace`, `containersStatus` und `brickletsStatus` enthält.

### Szenario 2: Abfragen der Bricklet-Telemetriedaten (GET /api/v1/diagnostics/bricklets)
- **Gegeben:** Konfigurierte Bricklets existieren im System.
- **Wenn:** Ein HTTP GET-Request an `/api/v1/diagnostics/bricklets` gesendet wird.
- **Dann:** Antwortet das Backend mit Status Code 200 OK und einem Array `bricklets`, worin für jedes Bricklet `brickletNumber`, `uid`, `type`, `voltage`, `current`, `status` und `pins` enthalten sind.

### Szenario 3: Abfragen der System- und Container-Metriken (GET /api/v1/diagnostics/system)
- **Gegeben:** Das Backend läuft auf dem Host-System.
- **Wenn:** Ein HTTP GET-Request an `/api/v1/diagnostics/system` gesendet wird.
- **Dann:** Antwortet das Backend mit Status Code 200 OK und enthält `cpuTemperature`, `diskSpace` (total, used, free, percentUsed) sowie eine Liste von `containers` mit Name, Status und Health.

### Szenario 4: Navigation zum Diagnostics-Tab im Cerebra-Frontend
- **Gegeben:** Der Benutzer navigiert zum System-Bereich im Cerebra Frontend.
- **Wenn:** Der Benutzer auf den Navigations-Link "Diagnostics" klickt.
- **Dann:** Wird die Route `/system/diagnostics` aktiviert und die Diagnostics-Komponente gerendert.

### Szenario 5: Anzeige der Ampel-Indikatoren (Traffic-Light Badges)
- **Gegeben:** Die Diagnostics-Komponente ist geladen und hat Zusammenfassungsdaten empfangen.
- **Wenn:** `overallStatus` den Wert `"ok"` aufweist.
- **Dann:** Zeigt der Gesamtstatus-Badge eine grüne Ampel-Anzeige (z.B. `bg-success` mit Text `"OK"`).

### Szenario 6: Darstellung von Bricklet-Spannung und Stromstärken-Karten
- **Gegeben:** Telemetriedaten für ein Servo-Bricklet liegen vor (`voltage: 5.02`, `current: 120.5`).
- **Wenn:** Der Diagnostics-Tab die Bricklet-Karten rendert.
- **Dann:** Werden Spannung (`5.02 V`) und Stromstärke (`120.5 mA`) sowie Pin-Stromstärken in der entsprechenden Bricklet-Karte angezeigt.

### Szenario 7: Manuelle Aktualisierung über den Refresh-Button
- **Gegeben:** Der Benutzer befindet sich auf der Diagnostics-Seite.
- **Wenn:** Der Benutzer den Button "Refresh Diagnostics" anklickt.
- **Dann:** Werden die Endpunkte `/api/v1/diagnostics/summary`, `/bricklets` und `/system` erneut abgefragt und die Ansicht wird mit den aktuellen Werten aktualisiert.

---
*Dokumentiert im Rahmen von PR-1498.*
