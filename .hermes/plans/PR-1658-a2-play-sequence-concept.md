# A2 — `play_pose_sequence`: Konzept für flüssige, abgestimmte Sequenz-Bewegungen

Jira-Bezug: PR-1657 (get_hand_position_xyz) / PR-1658 (pose-Blöcke). Neuer Block wird vorgeschlagen als
Teil der Pose-Block-Liste (A2). Dieses Doku legt das Konzept frei, bewertet die Machbarkeit auf der
**realen pib-Hardware** und übernimmt Synchronisations-Prinzipien aus roboter-Industrie-Frameworks.

---

## TL;DR — Kernaussage

Der vom User vorgeschlagene Ansatz ("pro Übergang eine Übergangszeit festlegen, alle Motoren mit
einer Geschwindigkeit laufen lassen, sodass die Bewegung genau in dieser Zeit abläuft, am Ende die
Default-Zeit/Settings aus der DB wiederherstellen") ist **fachlich sinnvoll und umsetzbar** — mit
zwei Präzisierungen, die aus der Hardware-Architektur folgen:

1. **Dauer lässt sich nicht direkt per Zeit-Wert setzen.** `ApplyJointTrajectory` (das `pib_sdk.Write.move`
   nutzt) hat **kein** `time_from_start`/Dauer-Feld (siehe `.srv` unten). Die Robotik-Synchronisation
   läuft dadurch, dass alle Gelenke dieselbe **konfigurierte Bewegungsgeschwindigkeit/-beschleunigung**
   haben, sodass sie ein Segment zeitgleich beginnen und gleichzeitig beenden. "Zeit exact einhalten"
   heißt hier: die **Geschwindigkeit so wählen, dass der langsamste (am weitesten wandernde) Motor das
   Segment in der Wunsch-Zeit durchläuft** → alle anderen werden automatisch "proportional langsamer" und
   landen gleichzeitig. Das ist exakt das Industriestandard-Prinzip **"der langsamste Gelenk bestimmt die
   Segmentzeit"** (Time-scaling / slowest-joint controls the motion time).
2. **"Am Ende Default-Zeit wiederherstellen" ist richtig und wichtig.** Weil die `velocity/acceleration`
   pro Motor gespeichert/angewendet wird, muss nach der Sequenz der **MotorSettings-Stand wieder**
   angewendet werden, sonst bleiben die Motoren auf der Sequenz-Geschwindigkeit → blockly/joint-control
   verhält sich danach falsch. Das Konzept sieht explizit einen **Teardown/Settings-Restore** vor.

---

## Recherche: Wie synchronisieren Robotik-Frameworks Bewegungen?

### Industriell / Robotik-Einschlag (Quellen-konsolidiert)
- **Joint interpolation (MOVEJ) + ein gemeinsames Zeitsignal** ist der Standardweg für synchronisierte
  Multi-Joint-Bewegungen. Alle Gelenke folgen demselben Zeit-Skript; die **trajectory time scaling** legt
  die gemeinsamen velocity/accel-Limits so, dass ein Pfad (hier: Segment von Pose A zu Pose B) in einer
  vorgegebenen Zeit gefahren wird. Der **langsamste joint (max travel) bestimmt die Dauer**; die übrigen
  werden entsprechend zurückgestuft (sog. "time-optimal / time-scaling", z.B. 3-5-3 polynomial oder
  trapezoid). Nicht: jeden Joint mit maximaler Geschwindigkeit einzeln losjagern (das würde die Bewegung
  asynchron und abgehackt machen).
- **Trajectories (Robotics Toolbox, Peter Corke)** nutzen das gleiche Modell: `trap/SVous`-Profil pro
  Gelenk über eine **gemeinsame Zeitachse** `t`, mit optionaler polynomialer Blend an Segmentübergängen.
- **Arc/industrial flow (Konzept)**: Sequential pose targets werden in **einen** Trajektorien-Stream aus
  **via-points** (ein Winkel pro Gelenk AND *ein* `time_from_start`-Stempel pro Punkt) übersetzt. Der
  Controller interpoliert zwischen via-Punkten und blendet an den Ecken, statt dass `move`-Aufrufe einzeln
  ausgeführt werden. Genau diesen "zeitgestempelten via-Punkte"-Charakter braucht der pib-Block.

> **Übertragung auf pib:** pib hat keinen Trajektorien-Controller mit Timestamps (`.srv` hat sie nicht).
> Also wird die "gemeinsame Zeitachse" **ersetzt durch ein gemeinsames Geschwindigkeits-/Settings-Paket**:
> pro Segment `Wunsch-Dauer → velocity_k = travel_k / Wunsch-Dauer` für jeden Motor, alles im selben
> `Write`-Batch → synchroner Start, gemeinsames Ende.

### Warum das "flüssig abgestimmte" Ergebnis erzielt (und was es NICHT kann)
- **Wirkung:** Ein Segment beginnt für alle Motoren gleichzeitig und endet gleichzeitig (weil jedes Motor
  je eigene velocity hat, aber dieselbe Reisezeit). Der Arm "bewegt sich kontinuierlich" – kein
  Hintereinander-ticken wie `set_pose` + `sleep`.
- **Grenze (ehrlich):** Das Bricklet-`set_motion_configuration` hat ein **target velocity/accel/decel**
  pro Motor, aber pib's `set_position` fährt mit **gleichförmiger (konstanter) Sollgeschwindigkeit** zum
  Ziel; es gibt **keine Echtzeit-Interpolation mehrerer zwischenliegender Trajektorie-Punkte**. Das
  Ergebnis ist also ein **segmentweise beschleunigt-abgebremstes**, nicht polynomial-geblendetes Profil.
  Für „flüssig" über **mehrere Pose-Übergänge** heißt das: die **Joint-Bahnen am Übergang können minimal
  ecken**, weil kein Blend (polynomial) zwischen Segmenten existiert. Das mindert man durch genügend
  saubere Ziel-Posen + kurze Überlappung, aber man sollte es als Grenze dokumentieren (kein Echtzeit-
  Trajektorien-Planer eingebettet).

---

## Machbarkeit auf pib (verifiziert an Code)

- `ApplyJointTrajectory.srv` = `trajectory_msgs/JointTrajectory` (mit `joint_names`, `points[].positions[0]`).
  **Kein** `time_from_start`, `velocities` wird ignoriert (Node liest nur `positions[0]`, `motor_control.py:34`).
- `motor_control.apply_joint_trajectory` ruft für jeden Motor `motor.set_position(position)` → Bricklet
  `set_motion_configuration(velocity, acceleration, deceleration)` ist die **einzige** Steuerung der
  Bewegungsgeschwindigkeit (gespeichert pro Motor, DB-gestützt, via `motor_client.get_motor_settings`).
- `pib_sdk.control.Write.move(joint_names, angles)` baut `positions_internal` und sendet eine gemeinsame
  `ApplyJointTrajectory`. `Write.set(All, velocity=..., acceleration=...)` kann Settings anwenden.
  `Write.move`/`set` geht über `/apply_joint_trajectory` + `/apply_motor_settings`.
- `save_current_pose`/`list_poses`/`get_pose` (PR-1658) liefern die Posen in Grad/MotorMap.

→ **Dauer steuern = Velocity-Time-Trading:** pro Segment die `velocity` der Motoren so skalieren, dass
`travel_mm/velocity` = Wunsch-Dauer. Weil pib's velocity in "Bricklet-units" ist (nicht direkt mm/s
linear), wird `velocity_k = velocity_default_k * (travel_rel_k / travel_rel_max)` als Konservierung
verwendet: alle Motoren verhalten sich skaliert zur maximalen Reise → gleiche Segmentzeit.

---

## Block-Konzept (Vorschlag für Generator + Block-Def)

### Blockform (Vorschlag, erweiterbar)
```
[play pose sequence]
  step: [pose dropdown]  in [duration_s] s   →  (repeated via gear-mutator, siehe run-script-block)
```
- **Globaler Kontext im Programm-Einstieg:** `Write` + `BackendClient` werden geöffnet; Settings werden
  gelesen und am Ende wiederhergestellt.
- Pro Schritt: Pose laden, alle betroffenen Motoren (they Pose-spec list) holen, `velocity` skalieren,
  `Write.move(motors…)`, optional `sleep(rest)` falls Konfiguration.

### Generator-Pseudocode (das, was Cursor erzeugen würde — Konzept)
```
def play_pose_sequence(writer, backend, pose_names, durations_s, restore_settings=True):
    original = backend.get_motor_settings(All)          # Settings sichern (vel/accel/decel)
    try:
        for name, dur in zip(pose_names, durations_s):
            pose = get_pose(backend, name=name)
            seg = {m: abs(pose.motor_angles_deg[m] - current[m]) for m in pose.motors}
            max_travel = max(seg.values())
            scale = max_travel_duration_units(...)      # velocity-Skalierung via travel-ratio
            writer.set(List(pose.motors), velocity=_scaled(max_travel_velocity, seg))
            writer.move(pose.motors, pose.angles) ; wait_until_segment_done(dur)
    finally:
        if restore_settings:
            writer.set(All, **original)                 # Default bei DBB wieder herstellen
```
> `wait_until_segment_done` statt hartem `sleep(dur)`: am elegantesten `sleep(dur)` (Näherung) oder
> Poll der Zielposition per `Telemetry.get_position_deg` bis erreicht → robuster gegen Stall.

### Wichtig (aus dem Konzept festgehalten)
- **Settings-Restore zwingend** (`finally`), damit joint-control/blocks danach korrekt laufen (User-Punkt bestätigt).
- **Segmentzeit** = gemeinsame Wunschzeit; alle Motoren des Segments werden synchron gefahren.
- **Grenze dokumentieren**: keine polynomiale Blend zwischen Segmenten (Hardware-Grenze); `sleep`-basierte
  Takung. Für soften Übergang: kurz davor `velocity` sanft rampen (optional).