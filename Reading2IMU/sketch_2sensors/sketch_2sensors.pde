import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

Serial myPort;

final int PACKET_SIZE = 20;

// ============================
// Gyro 1 data
// ============================
float qw1 = 1, qx1 = 0, qy1 = 0, qz1 = 0;
long timestamp1 = 0;

// ============================
// Gyro 2 data
// ============================
float qw2 = 1, qx2 = 0, qy2 = 0, qz2 = 0;
long timestamp2 = 0;

void setup() {
  size(1200, 700, P3D);
  textSize(18);
  smooth(8);

  myPort = new Serial(this, "COM9", 460800);
  myPort.clear();
}

void draw() {
  background(25);
  lights();

  readPackets();

  fill(255);

  // ----------------------------
  // Text info: gyro 1
  // ----------------------------
  text("Gyro 1", 20, 30);
  text("timestamp: " + timestamp1 + " ms", 20, 55);
  text("qw: " + nf(qw1, 1, 4), 20, 80);
  text("qx: " + nf(qx1, 1, 4), 20, 105);
  text("qy: " + nf(qy1, 1, 4), 20, 130);
  text("qz: " + nf(qz1, 1, 4), 20, 155);

  // ----------------------------
  // Text info: gyro 2
  // ----------------------------
  text("Gyro 2", 20, 230);
  text("timestamp: " + timestamp2 + " ms", 20, 255);
  text("qw: " + nf(qw2, 1, 4), 20, 280);
  text("qx: " + nf(qx2, 1, 4), 20, 305);
  text("qy: " + nf(qy2, 1, 4), 20, 330);
  text("qz: " + nf(qz2, 1, 4), 20, 355);

  // ============================
  // Draw Gyro 1
  // ============================
  pushMatrix();
  translate(width * 0.33, height * 0.58, 0);

  // IMU frame -> Processing display frame
  rotateX(-HALF_PI);

  applyQuaternion(qw1, qx1, qy1, qz1);
  drawBox(220, 120, 70);
  popMatrix();

  // ============================
  // Draw Gyro 2
  // ============================
  pushMatrix();
  translate(width * 0.72, height * 0.58, 0);

  // IMU frame -> Processing display frame
  rotateX(-HALF_PI);

  applyQuaternion(qw2, qx2, qy2, qz2);
  drawBox(220, 120, 70);
  popMatrix();
}

// ===========================================================
// SERIAL INPUT
// Assumes data arrives as:
//   packet for gyro1, then packet for gyro2
// ===========================================================
void readPackets() {
  while (myPort.available() >= 2 * PACKET_SIZE) {
    readOnePacketInto(1);
    readOnePacketInto(2);
  }
}

void readOnePacketInto(int gyroIndex) {
  byte[] buf = new byte[PACKET_SIZE];
  int n = myPort.readBytes(buf);

  if (n != PACKET_SIZE) {
    return;
  }

  ByteBuffer bb = ByteBuffer.wrap(buf);
  bb.order(ByteOrder.LITTLE_ENDIAN);

  long ts = ((long)bb.getInt()) & 0xFFFFFFFFL;
  float w = bb.getFloat();
  float x = bb.getFloat();
  float y = bb.getFloat();
  float z = bb.getFloat();

  if (gyroIndex == 1) {
    timestamp1 = ts;
    qw1 = w;
    qx1 = x;
    qy1 = y;
    qz1 = z;
  } else {
    timestamp2 = ts;
    qw2 = w;
    qx2 = x;
    qy2 = y;
    qz2 = z;
  }
}

// ===========================================================
// APPLY QUATERNION
// ===========================================================
void applyQuaternion(float w, float x, float y, float z) {
  float norm = sqrt(w*w + x*x + y*y + z*z);
  if (norm == 0) return;

  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  w = constrain(w, -1.0, 1.0);

  float angle = 2.0 * acos(w);
  float s = sqrt(max(0.0, 1.0 - w*w));

  if (s < 0.001) {
    rotate(angle, 1, 0, 0);
  } else {
    rotate(angle, x/s, y/s, z/s);
  }
}

// ===========================================================
// DRAW BOX
// ===========================================================
void drawBox(float w, float h, float d) {
  float hw = w/2, hh = h/2, hd = d/2;
  noStroke();

  beginShape(QUADS);
  fill(255,0,0);
  vertex(-hw,-hh, hd); vertex(hw,-hh, hd); vertex(hw,hh,hd); vertex(-hw,hh,hd);
  endShape();

  beginShape(QUADS);
  fill(0,255,0);
  vertex(hw,-hh,-hd); vertex(-hw,-hh,-hd); vertex(-hw,hh,-hd); vertex(hw,hh,-hd);
  endShape();

  beginShape(QUADS);
  fill(0,0,255);
  vertex(hw,-hh,hd); vertex(hw,-hh,-hd); vertex(hw,hh,-hd); vertex(hw,hh,hd);
  endShape();

  beginShape(QUADS);
  fill(255,255,0);
  vertex(-hw,-hh,-hd); vertex(-hw,-hh,hd); vertex(-hw,hh,hd); vertex(-hw,hh,-hd);
  endShape();

  beginShape(QUADS);
  fill(255,0,255);
  vertex(-hw,hh,hd); vertex(hw,hh,hd); vertex(hw,hh,-hd); vertex(-hw,hh,-hd);
  endShape();

  beginShape(QUADS);
  fill(0,255,255);
  vertex(-hw,-hh,-hd); vertex(hw,-hh,-hd); vertex(hw,-hh,hd); vertex(-hw,-hh,hd);
  endShape();
}
