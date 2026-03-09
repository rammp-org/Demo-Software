void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
}

void loop() {
  // assuming time to complete application in minutes
  float fake_appTime = 40.0, fake_speed = 3.0, fake_accel = 1.0,
        fake_acc_x = 7.0, fake_acc_y = 7.7, fake_acc_z = 0.34,
        fake_seatAngle_pitch = 30.0, fake_seatAngle_roll = 33.0,
        fake_tilt = 0.2, fake_height = 4.2;
  float fake_arr[] = {fake_appTime,         fake_speed,          fake_accel,
                      fake_acc_x,           fake_acc_y,          fake_acc_z,
                      fake_seatAngle_pitch, fake_seatAngle_roll, fake_tilt,
                      fake_height};

  String data = "[";
  for (int i = 0; i < 10; i++) // change 10 to size of fake_arr
  {
    data = data + fake_arr[i] + ",";
  }

  data = data + "]";

  Serial.println(data);

  delay(5000);
}
