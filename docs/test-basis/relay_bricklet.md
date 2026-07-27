# Test-Basis: PR-1495 - Relay Bricklet Toggle Aktivierung im Cerebra Frontend

Diese Test-Basis sichert die korrekte Aktivierung und Bedienbarkeit des Solid State Relay (SSR) Toggles im Cerebra-Frontend ab, sobald eine Hardware-ID (UID) für das Relay-Bricklet konfiguriert ist.

## Qualitätsziele & Anforderungen
1. **Verfügbarkeitsprüfung über Bricklet-Konfiguration:** Das Relay-Steuerungselement (`RelayControlComponent`) darf nicht dauerhaft deaktiviert bleiben, wenn eine gültige Bricklet-UID für das "Solid State Relay Bricklet" in den Systemeinstellungen hinterlegt ist.
2. **Dynamische Reaktivität:** Das Frontend muss bei Aktualisierung der Bricklet-UIDs über den `BrickletService` den Zustand des Toggle-Buttons sofort anpassen (aktivieren bei vorhandener UID, deaktivieren bei fehlender/leerer UID).
3. **Kompatibilität mit ROS-Status:** Sobald ein ROS-Status empfangen wird oder eine Relay-Bricklet-UID konfiguriert ist, ist das Toggle bedienbar.

## Verifizierte Testfälle (Test Basis TC-RELAY)

### TC-RELAY-1 — Relay Toggle Aktivierung bei konfigurierter Bricklet-UID
- **Gegeben:** Ein Solid State Relay Bricklet ist mit einer gültigen UID (z. B. `"XYZ123"`) im `BrickletService` konfiguriert.
- **Wenn:** Die `RelayControlComponent` initialisiert wird.
- **Dann:** Ist `isRelayAvailable` auf `true` gesetzt und der Toggle-Button `[disabled]` ist `false`.

### TC-RELAY-2 — Relay Toggle Deaktivierung ohne konfigurierte Bricklet-UID und ohne ROS-Status
- **Gegeben:** Kein Solid State Relay Bricklet bzw. ein Bricklet mit leerer UID (`""`) ist im `BrickletService` vorhanden und es liegt kein ROS-Status vor (`undefined`).
- **Wenn:** Die `RelayControlComponent` geladen wird.
- **Dann:** Ist `isRelayAvailable` auf `false` gesetzt und der Toggle-Button bleibt deaktiviert (`disabled`).

### TC-RELAY-3 — Statusübernahme und Toggle-Steuerung
- **Gegeben:** Der Relay Toggle ist verfügbar (`isRelayAvailable = true`).
- **Wenn:** Der Benutzer auf den Toggle-Button klickt.
- **Dann:** Wird `setSolidStateRelayState` im `RosService` mit dem umgeschalteten Zustand aufgerufen und das UI zeigt den neuen Zustand an.

---
*Dokumentiert im Rahmen von PR-1495.*
