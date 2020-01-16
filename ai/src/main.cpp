#include <string>
#include <cstring>
#include "GL/glut.h"
#include "scs.h"

unsigned char graph[GRAPH_HEIGHT][GRAPH_WIDTH];

void Display() {
  glTranslated(-0.5, -0.5, 0.0);
  glScaled(0.2, 0.2, 0.2);
  glColor3d(1.0, 0.0, 0.0);

  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < 5; i++) {
    glVertex2d((double)i, 1.0);
    glVertex2d((double)i, 0.0);
  }
  glEnd();

  glLoadIdentity();
  glTranslated(0.0, 0.5, 0.0);
  glColor3d(0.0, 1.0, 0.0);
  glRectd(-0.1, -0.1, 0.1, 0.1);
}

void AI_Camera() {
  int i = GRAPH_HEIGHT / 2, j;
  int left, right, middle, dir, voltage, Threshold = 100;
  double speed;

  sGetGraph(graph);

  for (j = GRAPH_WIDTH / 2; j > 0; j--)
    if (graph[i][j] < Threshold) break;
  left = j;
  for (j = GRAPH_WIDTH / 2; j < GRAPH_WIDTH - 1; j++)
    if (graph[i][j] < Threshold) break;
  right = j;
  middle = (left + right) / 2;

  dir = (middle - GRAPH_WIDTH / 2) * 300 / GRAPH_WIDTH;
  sSetServoDir(dir);

  speed = sGetSpeed();
  voltage = (int)((1.5 - speed) * 10.0 + 5.0);
  sSetMotor(voltage);
}

void AI_Electromagnetic() {
  int dir, voltage;
  double speed;
  double left, right;

  sVector pos(-0.1, 0.25, 0.05);
  left = sGetMagnetic(pos).GetX();

  pos.set(0.1, 0.25, 0.05);
  right = sGetMagnetic(pos).GetX();

  dir = (int)(right - left) * 5;
  sSetServoDir(dir);

  speed = sGetSpeed();
  voltage = (int)((1.0 - speed) * 10.0 + 5.0);
  sSetMotor(voltage);
}

unsigned char line[GRAPH_WIDTH];

void AI_Balance() {
  static double Angle = 0.0;
  static double s = 0.0;
  int i, left, right, middle, dir, Threshold = 100;
  double voltage, speed;
  sVector AngularSpeed = sGetAngularSpeed();
  Angle += AngularSpeed.GetX();

  speed = sGetSpeed();
  s += speed;
  s -= 0.3;
  voltage =
      Angle * 10.0 + AngularSpeed.GetX() * 10.0 + s * -20.0 + speed * -10.0;

  sGetLine(line);
  for (i = GRAPH_WIDTH / 2; i > 0; i--)
    if (line[i] < Threshold) break;
  left = i;

  for (i = GRAPH_WIDTH / 2; i < GRAPH_WIDTH - 1; i++)
    if (line[i] < Threshold) break;
  right = i;
  middle = (left + right) / 2;

  dir = (middle - GRAPH_WIDTH / 2) / 10;

  sSetMotorL(+dir - (int)voltage);
  sSetMotorR(-dir - (int)voltage);
}

int main(int argc, char *argv[]) {
  // printf("\n\nargc=%d, argv[0]=%s\n\n", argc, argv[0]);
  int type = 0;
  std::string = "final8.trk";
  if (argc > 1) {
    type = atoi(argv[1]);
    if (argc > 2) {
      track = argv[2];
    }
  }

  switch (type) {
    default:
    case 0:
      sSetCar(camera);
      sSetAiFunc(AI_Camera);
      sEnableCustomWindow();
      sSetDisplayFunc(Display);
      break;
    case 1:
      sSetCar(balance);
      sSetAiFunc(AI_Balance);
      sSetDepressionAngle(60.0);
      break;
    case 2:
      sSetCar(electromagnetic);
      sSetAiFunc(AI_Electromagnetic);
      break;
  }
  sSetTrack((std::string("../track/") + std::string(track)).c_str());
  sRegister("I've read the license. And I accept it.");
  scsMainLoop(&argc, argv);

  return 0;
}
