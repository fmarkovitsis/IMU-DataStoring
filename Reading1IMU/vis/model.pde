import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

Serial myPort;

// Quaternion from IMU
float qw = 1, qx = 0, qy = 0, qz = 0;
long timestamp = 0;

final int PACKET_SIZE = 20;

void setup() {
  size(900, 700, P3D);
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
  text("Quaternion", 10, 25);
  text("timestamp: " + timestamp + " us", 10, 50);
  text("qw: " + nf(qw,1,4), 10, 75);
  text("qx: " + nf(qx,1,4), 10, 100);
  text("qy: " + nf(qy,1,4), 10, 125);
  text("qz: " + nf(qz,1,4), 10, 150);

  pushMatrix();
  translate(width/2, height/2, 0);

  // IMU frame -> Processing display frame
  rotateX(-HALF_PI);

  applyQuaternion(qw, qx, qy, qz);

  drawBox(260, 140, 80);
  popMatrix();
}

// ===========================================================
// SERIAL INPUT
// ===========================================================
void readPackets() {
  while (myPort.available() >= PACKET_SIZE) {
    byte[] buf = new byte[PACKET_SIZE];
    myPort.readBytes(buf);

    ByteBuffer bb = ByteBuffer.wrap(buf);
    bb.order(ByteOrder.LITTLE_ENDIAN);

    timestamp = ((long)bb.getInt()) & 0xFFFFFFFFL;
    qw = bb.getFloat();
    qx = bb.getFloat();
    qy = bb.getFloat();
    qz = bb.getFloat();
    
  }
  println(timestamp, qw, qx, qy, qz);
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
