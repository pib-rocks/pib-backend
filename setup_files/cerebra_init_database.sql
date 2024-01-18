CREATE TABLE IF NOT EXISTS motor (
   id INTEGER primary key AUTOINCREMENT NOT NULL, 
   name TEXT NOT NULL UNIQUE, 
   pulseWidthMin INTEGER NOT NULL, 
   pulseWidthMax INTEGER NOT NULL, 
   rotationRangeMin INTEGER NOT NULL, 
   rotationRangeMax INTEGER NOT NULL, 
   velocity INTEGER NOT NULL, 
   acceleration INTEGER NOT NULL, 
   deceleration INTEGER NOT NULL, 
   period INTEGER NOT NULL, 
   turnedOn BOOLEAN NOT NULL, 
   effort INTEGER, 
   active BOOLEAN NOT NULL,
   invert BOOLEAN NOT NULL);

INSERT INTO motor(name, pulseWidthMin, pulseWidthMax, rotationRangeMin, rotationRangeMax, velocity, acceleration, deceleration, period, turnedOn, effort, active, invert) VALUES
   ('tilt_forward_motor', 700, 2500, -4500, 4500, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('tilt_sideways_motor', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, FALSE, FALSE),
   ('turn_head_motor', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('thumb_left_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('thumb_left_opposition', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('index_left_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('middle_left_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('ring_left_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('pinky_left_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('thumb_right_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('thumb_right_opposition', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('index_right_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('middle_right_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('ring_right_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('pinky_right_stretch', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('upper_arm_left_rotation', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('elbow_left', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('lower_arm_left_rotation', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('wrist_left', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('shoulder_vertical_left', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('shoulder_horizontal_left', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('upper_arm_right_rotation', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('elbow_right', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('lower_arm_right_rotation', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('wrist_right', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('shoulder_vertical_right', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE),
   ('shoulder_horizontal_right', 700, 2500, -9000, 9000, 16000, 10000, 5000, 19500, TRUE, NULL, TRUE, FALSE);

CREATE TABLE IF NOT EXISTS personality (
   id INTEGER primary key AUTOINCREMENT NOT NULL, 
   personalityId Text UNIQUE NOT NULL, 
   name Text NOT NULL, 
   gender Text NOT NULL, 
   description Text, 
   pauseThreshold Numeric NOT NULL);

INSERT INTO personality(personalityId, name, gender, description, pauseThreshold) VALUES
   ("8f73b580-927e-41c2-98ac-e5df070e7288", "Eva", "Female", NULL, 0.8),
   ("8b310f95-92cd-4512-b42a-d3fe29c4bb8a", "Thomas", "Male", NULL, 0.8);

CREATE TABLE IF NOT EXISTS cameraSettings (
   id INTEGER primary key AUTOINCREMENT NOT NULL, 
   resolution Text NOT NULL, 
   refreshRate Numeric NOT NULL, 
   qualityFactor INTEGER NOT NULL, 
   resX Integer NOT NULL, 
   resY Integer NOT Null);

INSERT INTO cameraSettings(resolution, refreshRate, qualityFactor, resX, resY) VALUES
   ("SD", 0.1, 80, 640, 480);

CREATE TABLE IF NOT EXISTS bricklet (
   id INTEGER primary key AUTOINCREMENT NOT NULL, 
   brickletId TEXT NOT NULL, 
   brickletNumber INTEGER NOT NULL
);

INSERT INTO bricklet(brickletId, brickletNumber) VALUES
   ("XYZ", 1),
   ("XYZ", 2),
   ("XYZ", 3);

CREATE TABLE IF NOT EXISTS motorBrickletPin (
   id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, 
   brickletId INTEGER NOT NULL, 
   motorId INTEGER, 
   pin INTEGER NOT NULL, 
   FOREIGN KEY (brickletId) REFERENCES bricklet(id), 
   FOREIGN KEY (motorId) REFERENCES motor(id)
);

INSERT INTO motorBrickletPin(motorId, brickletId, pin) VALUES
   ((SELECT id FROM motor WHERE name = "turn_head_motor"), 1, 0),
   ((SELECT id FROM motor WHERE name = "tilt_forward_motor"), 1, 1),
   ((SELECT id FROM motor WHERE name = "tilt_sideways_motor"), 2, 8),
   ((SELECT id FROM motor WHERE name = "thumb_left_opposition"), 1, 9),
   ((SELECT id FROM motor WHERE name = "thumb_left_stretch"), 2, 0),
   ((SELECT id FROM motor WHERE name = "index_left_stretch"), 2, 1),
   ((SELECT id FROM motor WHERE name = "middle_left_stretch"), 2, 2),
   ((SELECT id FROM motor WHERE name = "ring_left_stretch"), 2, 3),
   ((SELECT id FROM motor WHERE name = "pinky_left_stretch"), 2, 4),
   ((SELECT id FROM motor WHERE name = "thumb_right_opposition"), 1, 3),
   ((SELECT id FROM motor WHERE name = "thumb_right_stretch"), 1, 4),
   ((SELECT id FROM motor WHERE name = "index_right_stretch"), 1, 5),
   ((SELECT id FROM motor WHERE name = "middle_right_stretch"), 1, 6),
   ((SELECT id FROM motor WHERE name = "ring_right_stretch"), 1, 7),
   ((SELECT id FROM motor WHERE name = "pinky_right_stretch"), 1, 8),
   ((SELECT id FROM motor WHERE name = "upper_arm_left_rotation"), 2, 5),
   ((SELECT id FROM motor WHERE name = "elbow_left"), 2, 6),
   ((SELECT id FROM motor WHERE name = "lower_arm_left_rotation"), 2, 7),
   ((SELECT id FROM motor WHERE name = "wrist_left"), 1, 2),
   ((SELECT id FROM motor WHERE name = "shoulder_vertical_left"), 3, 7),
   ((SELECT id FROM motor WHERE name = "shoulder_vertical_left"), 3, 9),
   ((SELECT id FROM motor WHERE name = "shoulder_horizontal_left"), 3, 0),
   ((SELECT id FROM motor WHERE name = "upper_arm_right_rotation"), 3, 1),
   ((SELECT id FROM motor WHERE name = "elbow_right"), 3, 2),
   ((SELECT id FROM motor WHERE name = "lower_arm_right_rotation"), 3, 3),
   ((SELECT id FROM motor WHERE name = "wrist_right"), 3, 4),
   ((SELECT id FROM motor WHERE name = "shoulder_vertical_right"), 3, 5),
   ((SELECT id FROM motor WHERE name = "shoulder_vertical_right"), 3, 8),
   ((SELECT id FROM motor WHERE name = "shoulder_horizontal_right"), 3, 6);

CREATE TABLE IF NOT EXISTS program (
   id INTEGER primary key AUTOINCREMENT NOT NULL, 
   name TEXT NOT NULL, 
   codeVisual TEXT NOT NULL, 
   programNumber TEXT UNIQUE NOT NULL);
   
INSERT INTO program(name, codeVisual, programNumber) VALUES
   ("hello_world", '{"blocks":{"languageVersion":0,"blocks":[{"type":"text_print","id":"]l,+vC{q$rZPVdSfyx=4","x":229,"y":67,"inputs":{"TEXT":{"shadow":{"type":"text","id":"v,}3JGN5d7og[X_/KJ)|","fields":{"TEXT":"hello world"}}}}}]}}', "e1d46e2a-935e-4e2b-b2f9-0856af4257c5");

CREATE TABLE IF NOT EXISTS chat (
   id INTEGER primary key AUTOINCREMENT NOT NULL, 
   chatid TEXT NOT NULL, 
   topic TEXT NOT NULL, 
   personalityId TEXT NOT NULL, 
   FOREIGN KEY (personalityId) REFERENCES personality(personalityId));

INSERT INTO chat(chatid, topic, personalityId) VALUES
   ("12345-12333-45345-456456", "Nuernberg", "8f73b580-927e-41c2-98ac-e5df070e7288"),
   ("12345-12333-45345-456567", "Home-Office", "8b310f95-92cd-4512-b42a-d3fe29c4bb8a");