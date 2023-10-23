create table IF NOT EXISTS motor (id INTEGER primary key AUTOINCREMENT NOT NULL, name Text NOT NULL UNIQUE, pulseMin Integer NOT NULL, pulseMax Integer NOT NULL, degreeMin Integer NOT NULL, degreeMax Integer NOT NULL, velocity Integer NOT NULL, acceleration Integer NOT NULL, deceleration Integer NOT NULL, period Integer NOT NULL);
insert into
   motor 
values
   (
      NULL, 'tilt_forward_motor', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'tilt_sideways_motor', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'thumb_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'index_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'middle_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'ring_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'pinky_left_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'thumb_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'index_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'middle_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'ring_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'pinky_right_stretch', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'upper_arm_left_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'ellbow_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'lower_arm_left_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'wrist_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_vertical_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_horizontal_left', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'upper_arm_right_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'ellbow_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'lower_arm_right_rotation', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'wrist_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_vertical_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
insert into
   motor 
values
   (
      NULL, 'shoulder_horizontal_right', 700, 2500, -90, 90, 10000, 10000, 10000, 19500
   )
;
create table IF NOT EXISTS personality (id INTEGER primary key AUTOINCREMENT NOT NULL, personality_id Text NOT NULL UNIQUE, name Text NOT NULL, gender Text NOT NULL, description Text NOT NULL, pause_threshold Numeric NOT NULL);
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
create table IR NOT EXISTS camera_settings (id INTEGER primary key AUTOINCREMENT NOT NULL, resolution Text NOT NULL, refrash_rate Numeric NOT NULL, quality_factor INTEGER NOT NULL, is_active Boolean NOT NULL);
insert into
   camera_settings
values
   (
      NULL, "480p (SD)", 0.1, 80, false;
   )
;