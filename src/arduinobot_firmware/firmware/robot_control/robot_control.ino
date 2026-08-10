#include <Servo.h>

#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

#define STEP_INTERVAL_MS 50  // time between each 1-degree step for command-driven movement

Servo base;
Servo shoulder;
Servo elbow;
Servo gripper;

uint8_t idx = 0;
uint8_t value_idx = 0;
char value[4] = "000";

int current_pos[4];
int target_pos[4];
unsigned long last_step_time[4];

// Blocking, smooth single-joint move -- used only once, during the boot homing sequence
void reach_goal(Servo& motor, int goal)
{
  if (goal >= motor.read())
  {
    for (int pos = motor.read(); pos <= goal; pos++)
    {
      motor.write(pos);
      delay(100);
    }
  }
  else
  {
    for (int pos = motor.read(); pos >= goal; pos--)
    {
      motor.write(pos);
      delay(100);
    }
  }
}

// Non-blocking, single-step move -- used for all command-driven movement after boot
void update_joint(Servo& motor, int joint_index)
{
  if (current_pos[joint_index] == target_pos[joint_index])
  {
    return;  // already at target, nothing to do
  }

  unsigned long now = millis();
  if (now - last_step_time[joint_index] < STEP_INTERVAL_MS)
  {
    return;  // not time for the next step yet
  }

  if (target_pos[joint_index] > current_pos[joint_index])
  {
    current_pos[joint_index]++;
  }
  else
  {
    current_pos[joint_index]--;
  }

  motor.write(current_pos[joint_index]);
  last_step_time[joint_index] = now;
}

void setup() {
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN);

  delay(2000);  // pause so you can start recording before anything moves

  // Smooth, blocking homing sequence -- this is the boot-up animation
  reach_goal(base, 90);
  reach_goal(shoulder, 90);
  reach_goal(elbow, 90);
  reach_goal(gripper, 0);

  // Sync our tracking variables to match where the homing sequence left each joint,
  // so the non-blocking system below picks up from the correct state
  current_pos[0] = 90;
  current_pos[1] = 90;
  current_pos[2] = 90;
  current_pos[3] = 0;
  target_pos[0] = 90;
  target_pos[1] = 90;
  target_pos[2] = 90;
  target_pos[3] = 0;

  for (int i = 0; i < 4; i++)
  {
    last_step_time[i] = millis();
  }

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();

    if (chr == 'b')
    {
      idx = 0;
      value_idx = 0;
    }
    else if (chr == 's')
    {
      idx = 1;
      value_idx = 0;
    }
    else if (chr == 'e')
    {
      idx = 2;
      value_idx = 0;
    }
    else if (chr == 'g')
    {
      idx = 3;
      value_idx = 0;
    }
    else if (chr == ',')
    {
      int val = atoi(value);
      target_pos[idx] = val;

      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    else if (chr == '\n' || chr == '\r')
    {
      // Ignore line-ending characters
    }
    else
    {
      value[value_idx] = chr;
      value_idx++;
    }
  }

  update_joint(base, 0);
  update_joint(shoulder, 1);
  update_joint(elbow, 2);
  update_joint(gripper, 3);
}