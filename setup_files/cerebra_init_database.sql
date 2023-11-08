create table IF NOT EXISTS motor (id INTEGER primary key AUTOINCREMENT NOT NULL, name TEXT NOT NULL UNIQUE, pulseWidthMin INTEGER NOT NULL, pulseWidthMax INTEGER NOT NULL, rotationRangeMin INTEGER NOT NULL, rotationRangeMax INTEGER NOT NULL, velocity INTEGER NOT NULL, acceleration INTEGER NOT NULL, deceleration INTEGER NOT NULL, period INTEGER NOT NULL, turnedOn BOOLEAN NOT NULL, effort INTEGER);
insert into
   motor 
values
   (
      NULL, 'tilt_forward_motor', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'tilt_sideways_motor', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'turn_head_motor', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'thumb_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'thumb_left_opposition', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'index_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'middle_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'ring_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'pinky_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'thumb_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'thumb_right_opposition', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'index_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'middle_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'ring_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'pinky_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'upper_arm_left_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'elbow_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'lower_arm_left_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'wrist_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_vertical_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_horizontal_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'upper_arm_right_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'elbow_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'lower_arm_right_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'wrist_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_vertical_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_horizontal_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500, TRUE, NULL
   )
;
create table IF NOT EXISTS personality (id INTEGER primary key AUTOINCREMENT NOT NULL, personalityId Text UNIQUE NOT NULL, name Text NOT NULL, gender Text NOT NULL, description Text, pauseThreshold Numeric NOT NULL);
insert into
   personality
values
   (
      NULL, "8f73b580-927e-41c2-98ac-e5df070e7288", "Eva", "Female", NULL, 0.8
   )
;
insert into
   personality
values
   (
      NULL, "8b310f95-92cd-4512-b42a-d3fe29c4bb8a", "Thomas", "Male", NULL, 0.8
   )
;
create table IF NOT EXISTS cameraSettings (id INTEGER primary key AUTOINCREMENT NOT NULL, resolution Text NOT NULL, refreshRate Numeric NOT NULL, qualityFactor INTEGER NOT NULL, isActive BOOLEAN NOT NULL, resX Integer NOT NULL, resY Integer NOT Null);
insert into
   cameraSettings
values
   (
      NULL, "SD", 0.1, 80, FALSE, 640, 480
   )
;
create table IF NOT EXISTS program (id INTEGER primary key AUTOINCREMENT NOT NULL, name TEXT NOT NULL, program TEXT NOT NULL, programNumber INTEGER UNIQUE NOT NULL);