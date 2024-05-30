import processing.serial.*;

Serial serialPort;
float rotationX = 0;
float rotationY = 0;
float rotationZ = 0;

void setup() {
  size(800, 600, P3D);
  serialPort = new Serial(this, "COM7", 115200);  // Replace "COM4" with the appropriate serial port
}

void draw() {
  if (serialPort.available() > 0) {
    String data = serialPort.readStringUntil('\n');
    if (data != null) {
      data = data.trim();
      String[] angles = data.split("\t");
      if (angles.length == 4) {
        float pitch = float(angles[0]);
        float yaw = float(angles[2]);
        rotationY = radians(pitch);
        rotationZ= radians(yaw);
      }
    }
  }
  
  background(240); // Set light gray background
  
  // Draw background grid
  drawGrid(40, 40, color(220)); // Set grid color to a slightly darker gray
  
  translate(width / 2, height / 2); // Center the rocket
  
  // Apply rotations
  rotateX(rotationX);
  rotateY(rotationY);
  rotateZ(rotationZ);
  
  // Draw the rocket body (cylinder)
  fill(lerpColor(color(255, 0, 0), color(0, 255, 0), rotationX/TWO_PI)); // Use color gradient based on rotationX
  float bodyRadius = 50;
  float bodyHeight = 300;
  int bodyDetail = 24;
  drawCylinder(bodyRadius, bodyHeight, bodyDetail);
  
  // Draw the rocket nose (cone)
  translate(0, -bodyHeight / 2 - bodyRadius); // Move up to the top of the cylinder
  fill(lerpColor(color(0), color(79, 66, 65), rotationX/TWO_PI)); // Use color gradient based on rotationX
  float noseRadius = 50;
  float noseHeight = 150;
  int noseDetail = 24;
  drawCone(noseRadius, noseHeight, noseDetail);
  
  // Draw the rocket fins
  translate(0, bodyHeight / 2 + bodyRadius); // Move back down to the bottom of the cylinder
  
  // Calculate the position of the fins
  float finSize = 100; // Size of the fins (width and height)
  float finOffset = bodyRadius * 1.5; // Offset from the center of the rocket body
  float finAngle = 360 / 3; // Angle between each fin (120 degrees)
  
  // Draw each fin
  for (int i = 0; i < 3; i++) {
    float angle = radians(i * finAngle);
    pushMatrix();
    rotateY(angle);
    translate(0, finOffset);
    fill(lerpColor(color(0, 0, 255), color(255, 0, 255), rotationX/TWO_PI)); // Use color gradient based on rotationX
    rect(-finSize / 2, 0, finSize, finSize);
    popMatrix();
  }
}

void drawCylinder(float radius, float height, int detail) {
  float angle = 360 / detail;
  
  // Draw the body of the cylinder
  for (int i = 0; i < detail; i++) {
    float x = cos(radians(i * angle)) * radius;
    float z = sin(radians(i * angle)) * radius;
    float nextX = cos(radians((i + 1) * angle)) * radius;
    float nextZ = sin(radians((i + 1) * angle)) * radius;
    
    beginShape(QUAD_STRIP);
    vertex(x, -height / 2, z);
    vertex(x, height / 2, z);
    vertex(nextX, -height / 2, nextZ);
    vertex(nextX, height / 2, nextZ);
    endShape(CLOSE);
  }
}

void drawCone(float radius, float height, int detail) {
  float angle = 360 / detail;
  
  // Draw the cone
  beginShape(TRIANGLE_FAN);
  vertex(0, -height / 2, 0); // Apex of the cone
  for (int i = 0; i <= detail; i++) {
    float x = cos(radians(i * angle)) * radius;
    float z = sin(radians(i * angle)) * radius;
    vertex(x, height / 2, z);
  }
  endShape();
}

void drawGrid(float spacing, int count, color gridColor) {
  stroke(gridColor);
  for (int i = -count; i <= count; i++) {
    line(i * spacing, -count * spacing, i * spacing, count * spacing);
    line(-count * spacing, i * spacing, count * spacing, i * spacing);
  }
}
