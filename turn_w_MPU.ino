#include <Wire.h>
#include <Adafruit_MPU6050.h> // ใช้ไลบรารี Adafruit ที่คุณมีอยู่
#include <Adafruit_Sensor.h> // จำเป็นสำหรับไลบรารี Adafruit_MPU6050

Adafruit_MPU6050 mpu;

// ตัวแปร Offset (จาก Calibration)
float gyroZ_offset = 0.0293;

// กำหนดขา PWM สำหรับมอเตอร์ (เหมือนเดิม)
const int leftMotorPin1 = 9; // Left Motor IN1 (PWM)
const int leftMotorPin2 = 3; // Left Motor IN2 (Direction)
const int rightMotorPin1 = 11;  // Right Motor IN1 (PWM)
const int rightMotorPin2 = 10;  // Right Motor IN2 (Direction)

// ตัวแปรสำหรับ Complementary Filter
float yaw = 0; // ค่า Yaw ที่ผ่านการฟิวชั่น
unsigned long lastUpdateTime = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ตั้งค่าขาควบคุมมอเตอร์
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // ตรวจสอบการเชื่อมต่อ MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 found!");

  // ตั้งค่า Gyro และ Accel Range (ค่าเริ่มต้นมักจะเหมาะสมอยู่แล้ว)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // สามารถปรับได้: 2, 4, 8, 16 G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);     // สามารถปรับได้: 250, 500, 1000, 2000 DPS
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);  // สามารถปรับได้
  delay(5000);
  lastUpdateTime = micros(); // เริ่มจับเวลา
}


/**
 * @brief ฟังก์ชันควบคุมมอเตอร์
 * @param leftSpeed ความเร็วล้อซ้าย (-255 ถึง 255, ลบคือถอยหลัง)
 * @param rightSpeed ความเร็วล้อขวา (-255 ถึง 255, ลบคือถอยหลัง)
 */
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // มอเตอร์ซ้าย
  if (leftSpeed >= 0) {
    digitalWrite(leftMotorPin2, LOW); // เดินหน้า
    analogWrite(leftMotorPin1, leftSpeed);
  } else {
    digitalWrite(leftMotorPin2, HIGH); // ถอยหลัง
    analogWrite(leftMotorPin1, -leftSpeed); // ใช้ค่าบวกสำหรับ PWM
  }

  // มอเตอร์ขวา
  if (rightSpeed >= 0) {
    digitalWrite(rightMotorPin2, LOW); // เดินหน้า
    analogWrite(rightMotorPin1, rightSpeed);
  } else {
    digitalWrite(rightMotorPin2, HIGH); // ถอยหลัง
    analogWrite(rightMotorPin1, -rightSpeed); // ใช้ค่าบวกสำหรับ PWM
  }
}

/**
 * @brief ฟังก์ชันอัปเดตค่า Roll, Pitch, Yaw ด้วย Complementary Filter
 */
void updateIMUData() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  g.gyro.z -= gyroZ_offset;

  // คำนวณ delta time
  unsigned long currentMicros = micros();
  float dt = (float)(currentMicros - lastUpdateTime) / 1000000.0; // แปลงเป็นวินาที
  lastUpdateTime = currentMicros;

  // อ่านค่า Gyroscope (ในหน่วย rad/s โดย Adafruit library)
  float gyroZ = g.gyro.z * 180 / PI; // Gyro Z สำหรับ Yaw

  // สำหรับ Yaw (ใช้แค่ Gyro เพราะ Accel ไม่สามารถบอก Yaw ได้โดยตรง)
  // ค่า Yaw นี้จะยังคงมี Drift ในระยะยาว หากไม่มี Magnetometer
  yaw += (gyroZ * dt);

  // ทำให้ค่า Yaw อยู่ในช่วง -180 ถึง 180 องศา
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

/**
 * @brief ฟังก์ชันทำให้หุ่นยนต์เลี้ยวตามมุมที่กำหนด
 * @param targetAngle มุมที่ต้องการเลี้ยว (องศา, บวกคือเลี้ยวขวา, ลบคือเลี้ยวซ้าย)
 * @param turnSpeed ความเร็วในการเลี้ยว (ค่า PWM, 0-255)
 */
void turnRobot(float targetAngle, int turnSpeed) {
  Serial.print("Turning ");
  Serial.print(targetAngle);
  Serial.println(" degrees.");

  // อัปเดตค่า IMU ครั้งแรกก่อนเริ่มเลี้ยว
  updateIMUData();
  float initialYaw = yaw; // บันทึกค่า Yaw เริ่มต้นก่อนเลี้ยว

  // กำหนดทิศทางการเลี้ยว
  if (targetAngle > 0) { // เลี้ยวขวา
    setMotorSpeed(turnSpeed, -turnSpeed); // ล้อซ้ายเดินหน้า, ล้อขวาถอยหลัง
  } else { // เลี้ยวซ้าย
    setMotorSpeed(-turnSpeed, turnSpeed); // ล้อซ้ายถอยหลัง, ล้อขวาเดินหน้า
  }

  // วนลูปจนกว่าจะถึงมุมที่ต้องการ
  while (true) {
    updateIMUData(); // อัปเดตค่ามุม

    float currentDeltaYaw = yaw - initialYaw; // คำนวณการเปลี่ยนแปลงมุม

    // จัดการการหมุนข้ามจาก 180 ไป -180 หรือกลับกัน
    if (currentDeltaYaw > 180) currentDeltaYaw -= 360;
    if (currentDeltaYaw < -180) currentDeltaYaw += 360;

    Serial.print("Current Yaw: ");
    Serial.println(currentDeltaYaw);

    if (targetAngle > 0 && currentDeltaYaw >= targetAngle) {
      break; // เลี้ยวขวาถึงมุมที่ต้องการ
    } else if (targetAngle < 0 && currentDeltaYaw <= targetAngle) {
      break; // เลี้ยวซ้ายถึงมุมที่ต้องการ
    }
    
    delayMicroseconds(10); // หน่วงเวลาเล็กน้อย
  }

  setMotorSpeed(0, 0); // หยุดมอเตอร์เมื่อเลี้ยวเสร็จ
  Serial.println("Turn complete.");
}

// ตัวอย่างการใช้งานใน loop (เรียกใช้เมื่อต้องการเลี้ยว)
void loop() {
  turnRobot(90, 100); // เลี้ยวขวา 90 องศา ด้วยความเร็ว 150
  delay(5000);
  turnRobot(-90, 100); // เลี้ยวซ้าย 90 องศา ด้วยความเร็ว 150

  while(true); // หยุดการทำงานหลังจากเลี้ยวเสร็จ
}