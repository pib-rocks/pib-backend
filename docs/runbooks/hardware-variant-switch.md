# Switching the hardware variant

Use this procedure only when an existing pib machine is deliberately converted
to another supported hardware layout. At present, seed profiles exist only for
`pib4edu` and `pib5edu`. The advanced and museum variants cannot be selected
until profiles are implemented for them.

## Before the switch

Export the current hardware IDs, mappings, and tuned limits:

```bash
curl --fail http://localhost:5000/api/system/hardware-config/export \
  --output hardware-config-before-variant-switch.json
```

Keep this JSON file together with the database backup produced by the command.

## Run the switch

From the pib backend checkout, select the required implemented variant:

```bash
sudo docker compose exec flask-app \
  flask --app run seed_hardware --variant pib4edu --force
```

The command asks you to type the variant name exactly. It then prints the
database backup path and a summary. A missing `--force`, a different
confirmation, an unknown variant, or a variant without a profile aborts before
the backup or any database change.

The switch:

- preserves controller addresses (hardware UIDs);
- preserves existing motor parameters and all pose position values;
- preserves poses, programs, chats, camera settings, and assistant data;
- updates controller types and supply voltages, motor mappings, and RGB button
  program assignments to the selected profile;
- creates missing profile hardware and removes hardware absent from the profile.

## Import the hardware IDs

Before importing, update the saved version 2 document to match the target
profile's controller numbers, controller kinds, supply voltages, and motor
mappings. Assign each physical controller's exported `address` to its new
controller number. Do not blindly import the old document: importing its old
mappings would undo the profile switch.

Import the resulting target-shaped document after the switch:

```bash
curl --fail \
  --header 'Content-Type: application/json' \
  --data-binary @hardware-config-before-variant-switch.json \
  http://localhost:5000/api/system/hardware-config/import
```

Version 1 hardware-config documents remain accepted for older backups.

## Verify

Check that the command summary names the intended variant and profile, reports
the backup path, and says `pose=yes, program=yes, chat=yes`. Review any warning
about pose positions referring to removed motors.

Export the active configuration and inspect controller numbers, supply voltages,
UIDs, motor `controllerNumber`/`channel` mappings, and limits:

```bash
curl --fail http://localhost:5000/api/system/hardware-config/export \
  --output hardware-config-after-variant-switch.json
```

Finally, exercise the motors and all three RGB buttons on the machine before
returning it to service.

## Roll back

The command prints a backup named like
`/app/pibdata.db.bak-20260917T194800000000Z`. To restore it, stop the API first,
copy that file over the live database, remove stale SQLite WAL files, and start
the API again:

```bash
sudo docker compose stop flask-app
sudo cp pib_api/flask/pibdata.db.bak-<UTC timestamp> pib_api/flask/pibdata.db
sudo rm -f pib_api/flask/pibdata.db-wal pib_api/flask/pibdata.db-shm
sudo docker compose start flask-app
```

Use the exact host-side backup filename corresponding to the `/app/...` path
printed by the command. Keep the failed/new database separately if it is needed
for diagnosis.
