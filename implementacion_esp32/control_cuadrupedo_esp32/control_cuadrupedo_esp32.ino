// Add libraries 
#include <Arduino.h>
#include <math.h>
#include <vector>
#include <ESP32Servo.h>
#define PI 3.14159265358979323846
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
uint8_t receivedData = 0x00;

// Declaración de parámetros
Servo servos[8];
float real_q1, real_q2, real_q1_op, real_q2_op;
 // caminata
const float step_length = 0.5;
const float step_height = 2.0;
const int gait_steps = 50; // frames per half cycle
const int gait_delay = 20; // ms per frame


      // Neutral position for the foot of leg 0
  float footX = 2;   // forward/back relative to shoulder
  float footZ = -6.5;  // down is negative in your IK
  float q1, q2;



// Parámetros del robot
const float l1 = 4.0;
const float l2 = 6.3;
const float L_b = 12.0;
const float W_b = 10.0;
const float H_b = 0.0;

const float link_lengths[2] = {l1, l2};

// Origen de las patas
const float leg_origins[4][3] = {
  { L_b/2,  W_b/2, 0 },   // ULL
  { L_b/2, -W_b/2, 0 },   // URL
  {-L_b/2,  W_b/2, 0 },   // LLL
  {-L_b/2, -W_b/2, 0 }    // LRL
};

// Base pose (position & orientation)
float base_pos[3] = {0.0, 0.0, 7.5};   // x, y, z in cm
float rpy_base[3] = {0.0, 0.0, 0.0};   // roll, pitch, yaw in radians


// Posiciones deseadas de las patas en el mundo
const float foot_world_targets[4][3] = {
  { L_b/2,  W_b/2, 0 },
  { L_b/2, -W_b/2, 0 },
  {-L_b/2,  W_b/2, 0 },
  {-L_b/2, -W_b/2, 0 }
};

void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if(event == ESP_SPP_DATA_IND_EVT){
    receivedData = SerialBT.read();
  } 
}

//-------------------------------------------------------------------
// Mathematic functions
void eul2rotm(const float rpy[3], float R[3][3]) {
  float roll = rpy[0], pitch = rpy[1], yaw = rpy[2];
  
  float cx = cos(roll), sx = sin(roll);
  float cy = cos(pitch), sy = sin(pitch);
  float cz = cos(yaw), sz = sin(yaw);

  R[0][0] = cz * cy;
  R[0][1] = cz * sy * sx - sz * cx;
  R[0][2] = cz * sy * cx + sz * sx;

  R[1][0] = sz * cy;
  R[1][1] = sz * sy * sx + cz * cx;
  R[1][2] = sz * sy * cx - cz * sx;

  R[2][0] = -sy;
  R[2][1] = cy * sx;
  R[2][2] = cy * cx;
}

// Resultado de la cinemática inversa
float Q[4][2]; // ángulos q1 y q2 por pata

void legIK(float x, float z, float l1, float l2, float &q1, float &q2) {
  float r = sqrt(x * x + z * z);
  float cos_theta2 = (r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  cos_theta2 = constrain(cos_theta2, -1.0, 1.0);  // Evita NaNs
  float sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);

  float theta2 = atan2(sin_theta2, cos_theta2);
  float phi = atan2(-z, -x);
  float beta = atan2(l2 * sin_theta2, l1 + l2 * cos_theta2);
  float theta1 = phi - beta;

  q1 = theta1 - PI / 9.0;
  q2 = theta2 + PI / 9.0;
}

void ik_leg_floating(const float foot_world[3], const float rpy[3], int legID, const float base_pos[3],
                     const float leg_origins[4][3], const float link_lengths[2], float angles[2]) {
  // Obtener matriz de rotación del cuerpo
  float R[3][3];
  eul2rotm(rpy, R);

  // Calcular origen de la pierna en el mundo
  float origin[3];
  for (int i = 0; i < 3; i++) {
    origin[i] = R[i][0] * leg_origins[legID][0] +
                R[i][1] * leg_origins[legID][1] +
                R[i][2] * leg_origins[legID][2] +
                base_pos[i];
  }

  // Vector pie respecto al origen de la pierna, en el marco del cuerpo
  // (R^T * (pie - origen)): la pierna gira con el cuerpo
  float p_leg[3];
  for (int i = 0; i < 3; i++) {
    p_leg[i] = R[0][i] * (foot_world[0] - origin[0]) +
               R[1][i] * (foot_world[1] - origin[1]) +
               R[2][i] * (foot_world[2] - origin[2]);
  }

  // Solo plano X-Z
  float x = p_leg[0];
  float z = p_leg[2];

  // Resolver IK
  legIK(x, z, link_lengths[0], link_lengths[1], angles[0], angles[1]);
}

//-----------------------------------------------------------------------------//
// Function to set angle to servomotors
int angle2duty(float angle) {
    int min_us = 500;
    int max_us = 2500;
    float us = min_us + (angle / 180.0f) * (max_us - min_us);
    int duty = int((us / 20000.0f) * 1023);
    return duty;
}

// radian to degree
float rad2deg(float angle_rad) {
    return angle_rad * 180.0f / PI;
}

// Real value to servos
void set_servos(float angle1, float angle2, float &real_q1, float &real_q2, int legID) {
    real_q1 = angle1;
    real_q2 = abs(angle2 - PI);

    if (legID == 0 || legID == 2) {
        // real_q1 and real_q2 stay the same
    }
    else if (legID == 1 || legID == 3) {
        real_q1 = abs(real_q1 - PI);
        real_q2 = abs(real_q2 - PI);
    }
}
//-----------------------------------------------------------------------------//

// Acciones
// Liying
void liying(float Q[4][2], Servo servos[8]) {
  const int steps = 50;
  const int delay_ms = 25;

  for (int step = 0; step <= steps; step++) {
    float ratio = (float)step / steps;

    for (int legID = 0; legID < 4; legID++) {
      float q1_current = Q[legID][0];
      float q2_current = Q[legID][1];

      float q1_interp = q1_current * (1.0 - ratio);
      float q2_interp = q2_current * (1.0 - ratio) + PI * ratio;

      float real_q1 = q1_interp;
      float real_q2 = abs(q2_interp - PI);

      if (legID == 1 || legID == 3) {
        real_q1 = abs(real_q1 - PI);
        real_q2 = abs(real_q2 - PI);
      }

      float angle1 = rad2deg(real_q1);
      float angle2 = rad2deg(real_q2);

      servos[2 * legID].write(angle1);
      servos[2 * legID + 1].write(angle2);
    }

    delay(delay_ms);
  }

  // Actualizamos Q a [0, PI]
  for (int i = 0; i < 4; i++) {
    Q[i][0] = 0;
    Q[i][1] = PI;
  }
}

// standing up
void standingUp(float Q[4][2], Servo servos[8], float L1, float L2,const float foot_world_targets[4][3],const float leg_origins[4][3]) {
  const int steps = 50;
  const int delay_ms = 20;

  float lengths[2] = {L1, L2};
  float base_pos[3] = {0, 0, 7.5};
  float rpy[3] = {0, 0, 0};
  float base_rot[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };

  float targetQ[4][2];

  // Step 1: compute IK for each leg
  for (int legID = 0; legID < 4; legID++) {
    float q[2];

    ik_leg_floating(
      foot_world_targets[legID],
      rpy,
      legID,
      base_pos,
      leg_origins,
      lengths,
      q
    );

    targetQ[legID][0] = q[0];
    targetQ[legID][1] = q[1];
  }

  // Step 2: interpolate from current Q to targetQ
  for (int step = 0; step <= steps; step++) {
    float ratio = (float)step / steps;

    for (int legID = 0; legID < 4; legID++) {
      float q1_interp = Q[legID][0] * (1.0 - ratio) + targetQ[legID][0] * ratio;
      float q2_interp = Q[legID][1] * (1.0 - ratio) + targetQ[legID][1] * ratio;

      float real_q1 = q1_interp;
      float real_q2 = abs(q2_interp - PI);

      if (legID == 1 || legID == 3) {
        real_q1 = abs(real_q1 - PI);
        real_q2 = abs(real_q2 - PI);
      }

      float angle1 = rad2deg(real_q1);
      float angle2 = rad2deg(real_q2);

      servos[2 * legID].write(angle1);
      servos[2 * legID + 1].write(angle2);
    }

    delay(delay_ms);
  }

  // Step 3: update Q to final joint angles
  for (int i = 0; i < 4; i++) {
    Q[i][0] = targetQ[i][0];
    Q[i][1] = targetQ[i][1];
  }
}

// Estirarse
void stretching(float Q[4][2], Servo servos[8]) {
  const int steps = 50;
  const int delay_ms = 20;

  // Target Q values
  float Q_target[4][2] = {
    {PI , 50.0 * PI / 180.0},
    {PI , 50.0 * PI / 180.0},
    {PI / 6.0,           1.9},
    {PI / 6.0,           1.9}
  };

  for (int step = 0; step <= steps; step++) {
    float ratio = (float)step / steps;

    for (int legID = 0; legID < 4; legID++) {
      float q1_current = Q[legID][0];
      float q2_current = Q[legID][1];

      float q1_target = Q_target[legID][0];
      float q2_target = Q_target[legID][1];

      float q1_interp = q1_current * (1.0 - ratio) + q1_target * ratio;
      float q2_interp = q2_current * (1.0 - ratio) + q2_target * ratio;

      float real_q1 = q1_interp;
      float real_q2 = abs(q2_interp - PI);

      if (legID == 1 || legID == 3) {
        real_q1 = abs(real_q1 - PI);
        real_q2 = abs(real_q2 - PI);
      }

      float angle1 = rad2deg(real_q1);
      float angle2 = rad2deg(real_q2);

      servos[2 * legID].write(angle1);
      servos[2 * legID + 1].write(angle2);
    }

    delay(delay_ms);
  }

  // Update Q to match the final target state
  for (int i = 0; i < 4; i++) {
    Q[i][0] = Q_target[i][0];
    Q[i][1] = Q_target[i][1];
  }
}


// Sentarse
void sitting(float Q[4][2], Servo servos[8]) {
  const int steps = 50;
  const int delay_ms = 20;

  // Target Q values
  float Q_target[4][2] = {
    {30.0 * PI / 180.0 , 45.0 * PI / 180.0},
    {30.0 * PI / 180.0 , 45.0 * PI / 180.0},
    {0,           5.0 * PI / 6.0},
    {0,           5.0 * PI / 6.0}
  };

  for (int step = 0; step <= steps; step++) {
    float ratio = (float)step / steps;

    for (int legID = 0; legID < 4; legID++) {
      float q1_current = Q[legID][0];
      float q2_current = Q[legID][1];

      float q1_target = Q_target[legID][0];
      float q2_target = Q_target[legID][1];

      float q1_interp = q1_current * (1.0 - ratio) + q1_target * ratio;
      float q2_interp = q2_current * (1.0 - ratio) + q2_target * ratio;

      float real_q1 = q1_interp;
      float real_q2 = abs(q2_interp - PI);

      if (legID == 1 || legID == 3) {
        real_q1 = abs(real_q1 - PI);
        real_q2 = abs(real_q2 - PI);
      }

      float angle1 = rad2deg(real_q1);
      float angle2 = rad2deg(real_q2);

      servos[2 * legID].write(angle1);
      servos[2 * legID + 1].write(angle2);
    }

    delay(delay_ms);
  }

  // Update Q to match the final target state
  for (int i = 0; i < 4; i++) {
    Q[i][0] = Q_target[i][0];
    Q[i][1] = Q_target[i][1];
  }
}

// Caminar

void moveLegPair(Servo servos[], float x, float z, float l1, float l2,
                 int legIndexA, int legIndexB, int delayMs) {
  float q1, q2;
  float real_q1, real_q2, real_q1_op, real_q2_op;

  // Run IK for the given target
  legIK(x, z, l1, l2, q1, q2);

  real_q1 = q1;
  real_q2 = abs(q2 - PI);

  // Opposite leg’s mirrored motion
  real_q1_op = abs(real_q1 - PI);
  real_q2_op = abs(real_q2 - PI);

  // Write main leg servos
  servos[legIndexA].write(rad2deg(real_q1));
  servos[legIndexA + 1].write(rad2deg(real_q2));

  // Write opposite leg servos
  servos[legIndexB].write(rad2deg(real_q1_op));
  servos[legIndexB + 1].write(rad2deg(real_q2_op));

  delay(delayMs);
}

//-----------------------------------------------------------------------------//

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Bluetooth");
  SerialBT.register_callback(BT_EventHandler);

  int pins[] = {18, 19, 21, 22, 33, 25, 26, 27};
  for (int i = 0; i < 8; ++i) {
    servos[i].attach(pins[i]);
  }

  // Initial pose
  for (int legID = 0; legID < 4; legID++) {
    float angles[2];
    ik_leg_floating(foot_world_targets[legID], rpy_base, legID, base_pos, leg_origins, link_lengths, angles);

    Q[legID][0] = angles[0];
    Q[legID][1] = angles[1];

    real_q1 = angles[0];
    real_q2 = abs(angles[1] - PI);

    if (legID == 1 || legID == 3) {
      real_q1 = abs(real_q1 - PI);
      real_q2 = abs(real_q2 - PI);
    }

    float angle1 = rad2deg(real_q1);
    float angle2 = rad2deg(real_q2);

    servos[2 * legID].write(angle1);
    servos[2 * legID + 1].write(angle2);

    Serial.printf("Leg %d: q1=%.2f, q2=%.2f -> angles=(%.1f, %.1f)\n",
                  legID, real_q1, real_q2, angle1, angle2); 
  }

  delay(5000);



}

// ===== Loop =====
void loop() {

  // Leer caracter
    //Serial.println(receivedData);
    if(receivedData==4){
      liying(Q, servos);
      Serial.println("liying");
    } 
    else if(receivedData==5){
      stretching(Q, servos);
      Serial.println("stretching"); 
    }
    else if(receivedData==3){
      standingUp(Q, servos, l1, l2, foot_world_targets, leg_origins);
      Serial.println("standingUp");
    }
    else if(receivedData==6){
      sitting(Q, servos);
      Serial.println("sitting");
    }
    else if(receivedData==1){
    // Neutral position
      legIK(0, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[0].write(rad2deg(real_q1));
      servos[1].write(rad2deg(real_q2));
      servos[6].write(rad2deg(real_q1_op));
      servos[7].write(rad2deg(real_q2_op));
      delay(150);

      // Lift leg
      legIK(1.75, -6, l1, l2, q1, q2); // z up by 1
      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[0].write(rad2deg(real_q1));
      servos[1].write(rad2deg(real_q2));
      servos[6].write(rad2deg(real_q1_op));
      servos[7].write(rad2deg(real_q2_op));
      delay(150);

        // Neutral position
      legIK(3.5, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[0].write(rad2deg(real_q1));
      servos[1].write(rad2deg(real_q2));
      servos[6].write(rad2deg(real_q1_op));
      servos[7].write(rad2deg(real_q2_op));
      delay(150);

      // Neutral position
      legIK(1.75, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[0].write(rad2deg(real_q1));
      servos[1].write(rad2deg(real_q2));
      servos[6].write(rad2deg(real_q1_op));
      servos[7].write(rad2deg(real_q2_op));
      delay(150);

        // Neutral position
      legIK(-1.5, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[0].write(rad2deg(real_q1));
      servos[1].write(rad2deg(real_q2));
      servos[6].write(rad2deg(real_q1_op));
      servos[7].write(rad2deg(real_q2_op));
      delay(150);

    // ---------------------------------------------------------------------------
        // Neutral position
      legIK(0, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[4].write(rad2deg(real_q1));
      servos[5].write(rad2deg(real_q2));
      servos[2].write(rad2deg(real_q1_op));
      servos[3].write(rad2deg(real_q2_op));
      delay(150);

      // Lift leg
      legIK(1.75, -6, l1, l2, q1, q2); // z up by 1
      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);
      servos[4].write(rad2deg(real_q1));
      servos[5].write(rad2deg(real_q2));
      servos[2].write(rad2deg(real_q1_op));
      servos[3].write(rad2deg(real_q2_op));
      delay(150);

          // Neutral position
      legIK(3.5, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[4].write(rad2deg(real_q1));
      servos[5].write(rad2deg(real_q2));
      servos[2].write(rad2deg(real_q1_op));
      servos[3].write(rad2deg(real_q2_op));
      delay(150);

      legIK(1.75, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[4].write(rad2deg(real_q1));
      servos[5].write(rad2deg(real_q2));
      servos[2].write(rad2deg(real_q1_op));
      servos[3].write(rad2deg(real_q2_op));
      delay(150);

        legIK(-1.5, -7.5, l1, l2, q1, q2);

      real_q1 = q1;
      real_q2 = abs(q2 - PI);

      real_q1_op = abs(real_q1 - PI);
      real_q2_op = abs(real_q2 - PI);

      servos[4].write(rad2deg(real_q1));
      servos[5].write(rad2deg(real_q2));
      servos[2].write(rad2deg(real_q1_op));
      servos[3].write(rad2deg(real_q2_op));
      delay(150);

    }
  delay(100);
}
