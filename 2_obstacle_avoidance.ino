#include <Servo.h>
#include <FlexiTimer2.h>

#define TRIG A0
#define ECHO A1
#define OBS_DIST 20

Servo servo[4][3];
const int pin[4][3] = {{3,4,2},{6,7,5},{9,8,10},{12,11,13}};

const float La = 55, Lb = 77.5, Lc = 27.5, Ls = 71;
const float Zabs = -28, Zdef = -50, Zup = -30, Zboot = Zabs;
const float Xdef = 62, Xoff = 0;
const float Ys = 0, Ystep = 40;
const float KEEP = 255, PI_VAL = 3.1415926;
const float spd_turn = 4, spd_leg = 8, spd_body = 3, spd_seat = 1;

volatile float sNow[4][3], sExp[4][3];
float tSpd[4][3], mSpd, spdMul = 1;
volatile int restCnt;

const float tA = sqrt(pow(2*Xdef+Ls,2)+pow(Ystep,2));
const float tB = 2*(Ys+Ystep)+Ls;
const float tC = sqrt(pow(2*Xdef+Ls,2)+pow(2*Ys+Ystep+Ls,2));
const float tAlpha = acos((tA*tA+tB*tB-tC*tC)/2/tA/tB);
const float tx1 = (tA-Ls)/2, ty1 = Ys+Ystep/2;
const float tx0 = tx1-tB*cos(tAlpha);
const float ty0 = tB*sin(tAlpha)-ty1-Ls;

long getDist() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long d = pulseIn(ECHO, HIGH, 25000);
  return d == 0 ? 999 : d * 0.034 / 2;
}

void cart2pol(volatile float &a, volatile float &b, volatile float &g,
             volatile float x, volatile float y, volatile float z) {
  float w = (x >= 0 ? 1 : -1) * sqrt(x*x + y*y);
  float v = w - Lc;
  a = atan2(z,v) + acos((La*La - Lb*Lb + v*v + z*z) / 2/La/sqrt(v*v+z*z));
  b = acos((La*La + Lb*Lb - v*v - z*z) / 2/La/Lb);
  g = (w >= 0) ? atan2(y,x) : atan2(-y,-x);
  a = a/PI_VAL*180; b = b/PI_VAL*180; g = g/PI_VAL*180;
}

void pol2srv(int leg, float a, float b, float g) {
  if (leg == 0)      { a = 90-a; g += 90; }
  else if (leg == 1) { a += 90; b = 180-b; g = 90-g; }
  else if (leg == 2) { a += 90; b = 180-b; g = 90-g; }
  else               { a = 90-a; g += 90; }
  servo[leg][0].write(a);
  servo[leg][1].write(b);
  servo[leg][2].write(g);
}

void srvISR(void) {
  sei();
  static float a, b, g;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (abs(sNow[i][j]-sExp[i][j]) >= abs(tSpd[i][j]))
        sNow[i][j] += tSpd[i][j];
      else sNow[i][j] = sExp[i][j];
    }
    cart2pol(a, b, g, sNow[i][0], sNow[i][1], sNow[i][2]);
    pol2srv(i, a, b, g);
  }
  restCnt++;
}

void setSite(int leg, float x, float y, float z) {
  float dx=0, dy=0, dz=0;
  if (x!=KEEP) dx = x-sNow[leg][0];
  if (y!=KEEP) dy = y-sNow[leg][1];
  if (z!=KEEP) dz = z-sNow[leg][2];
  float len = sqrt(dx*dx + dy*dy + dz*dz);
  tSpd[leg][0] = dx/len*mSpd*spdMul;
  tSpd[leg][1] = dy/len*mSpd*spdMul;
  tSpd[leg][2] = dz/len*mSpd*spdMul;
  if (x!=KEEP) sExp[leg][0]=x;
  if (y!=KEEP) sExp[leg][1]=y;
  if (z!=KEEP) sExp[leg][2]=z;
}

void waitLeg(int l) {
  while (sNow[l][0]!=sExp[l][0] || sNow[l][1]!=sExp[l][1] || sNow[l][2]!=sExp[l][2]);
}

void waitAll() { for (int i=0;i<4;i++) waitLeg(i); }

void stand()  { mSpd=spd_seat; for(int i=0;i<4;i++) setSite(i,KEEP,KEEP,Zdef); waitAll(); }
void sit()    { mSpd=spd_seat; for(int i=0;i<4;i++) setSite(i,KEEP,KEEP,Zboot); waitAll(); }

void stepFwd(unsigned int s) {
  mSpd = spd_leg;
  while (s-- > 0) {
    if (sNow[2][1]==Ys) {
      setSite(2,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zdef); waitAll();
      mSpd=spd_body;
      setSite(0,Xdef+Xoff,Ys,Zdef); setSite(1,Xdef+Xoff,Ys+2*Ystep,Zdef);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef); setSite(3,Xdef-Xoff,Ys+Ystep,Zdef); waitAll();
      mSpd=spd_leg;
      setSite(1,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(1,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(1,Xdef+Xoff,Ys,Zdef); waitAll();
    } else {
      setSite(0,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zdef); waitAll();
      mSpd=spd_body;
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef); setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys,Zdef); setSite(3,Xdef+Xoff,Ys+2*Ystep,Zdef); waitAll();
      mSpd=spd_leg;
      setSite(3,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(3,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(3,Xdef+Xoff,Ys,Zdef); waitAll();
    }
  }
}

void stepBack(unsigned int s) {
  mSpd = spd_leg;
  while (s-- > 0) {
    if (sNow[3][1]==Ys) {
      setSite(3,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(3,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(3,Xdef+Xoff,Ys+2*Ystep,Zdef); waitAll();
      mSpd=spd_body;
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zdef); setSite(1,Xdef+Xoff,Ys,Zdef);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef); setSite(3,Xdef-Xoff,Ys+Ystep,Zdef); waitAll();
      mSpd=spd_leg;
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(0,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(0,Xdef+Xoff,Ys,Zdef); waitAll();
    } else {
      setSite(1,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(1,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(1,Xdef+Xoff,Ys+2*Ystep,Zdef); waitAll();
      mSpd=spd_body;
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef); setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zdef); setSite(3,Xdef+Xoff,Ys,Zdef); waitAll();
      mSpd=spd_leg;
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zup); waitAll();
      setSite(2,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(2,Xdef+Xoff,Ys,Zdef); waitAll();
    }
  }
}

void turnLeft(unsigned int s) {
  mSpd = spd_turn;
  while (s-- > 0) {
    if (sNow[3][1]==Ys) {
      setSite(3,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(0,tx1-Xoff,ty1,Zdef); setSite(1,tx0-Xoff,ty0,Zdef);
      setSite(2,tx1+Xoff,ty1,Zdef); setSite(3,tx0+Xoff,ty0,Zup); waitAll();
      setSite(3,tx0+Xoff,ty0,Zdef); waitAll();
      setSite(0,tx1+Xoff,ty1,Zdef); setSite(1,tx0+Xoff,ty0,Zdef);
      setSite(2,tx1-Xoff,ty1,Zdef); setSite(3,tx0-Xoff,ty0,Zdef); waitAll();
      setSite(1,tx0+Xoff,ty0,Zup); waitAll();
      setSite(0,Xdef+Xoff,Ys,Zdef); setSite(1,Xdef+Xoff,Ys,Zup);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef); setSite(3,Xdef-Xoff,Ys+Ystep,Zdef); waitAll();
      setSite(1,Xdef+Xoff,Ys,Zdef); waitAll();
    } else {
      setSite(0,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(0,tx0+Xoff,ty0,Zup); setSite(1,tx1+Xoff,ty1,Zdef);
      setSite(2,tx0-Xoff,ty0,Zdef); setSite(3,tx1-Xoff,ty1,Zdef); waitAll();
      setSite(0,tx0+Xoff,ty0,Zdef); waitAll();
      setSite(0,tx0-Xoff,ty0,Zdef); setSite(1,tx1-Xoff,ty1,Zdef);
      setSite(2,tx0+Xoff,ty0,Zdef); setSite(3,tx1+Xoff,ty1,Zdef); waitAll();
      setSite(2,tx0+Xoff,ty0,Zup); waitAll();
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef); setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys,Zup); setSite(3,Xdef+Xoff,Ys,Zdef); waitAll();
      setSite(2,Xdef+Xoff,Ys,Zdef); waitAll();
    }
  }
}

void turnRight(unsigned int s) {
  mSpd = spd_turn;
  while (s-- > 0) {
    if (sNow[2][1]==Ys) {
      setSite(2,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(0,tx0-Xoff,ty0,Zdef); setSite(1,tx1-Xoff,ty1,Zdef);
      setSite(2,tx0+Xoff,ty0,Zup); setSite(3,tx1+Xoff,ty1,Zdef); waitAll();
      setSite(2,tx0+Xoff,ty0,Zdef); waitAll();
      setSite(0,tx0+Xoff,ty0,Zdef); setSite(1,tx1+Xoff,ty1,Zdef);
      setSite(2,tx0-Xoff,ty0,Zdef); setSite(3,tx1-Xoff,ty1,Zdef); waitAll();
      setSite(0,tx0+Xoff,ty0,Zup); waitAll();
      setSite(0,Xdef+Xoff,Ys,Zup); setSite(1,Xdef+Xoff,Ys,Zdef);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef); setSite(3,Xdef-Xoff,Ys+Ystep,Zdef); waitAll();
      setSite(0,Xdef+Xoff,Ys,Zdef); waitAll();
    } else {
      setSite(1,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(0,tx1+Xoff,ty1,Zdef); setSite(1,tx0+Xoff,ty0,Zup);
      setSite(2,tx1-Xoff,ty1,Zdef); setSite(3,tx0-Xoff,ty0,Zdef); waitAll();
      setSite(1,tx0+Xoff,ty0,Zdef); waitAll();
      setSite(0,tx1-Xoff,ty1,Zdef); setSite(1,tx0-Xoff,ty0,Zdef);
      setSite(2,tx1+Xoff,ty1,Zdef); setSite(3,tx0+Xoff,ty0,Zdef); waitAll();
      setSite(3,tx0+Xoff,ty0,Zup); waitAll();
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef); setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys,Zdef); setSite(3,Xdef+Xoff,Ys,Zup); waitAll();
      setSite(3,Xdef+Xoff,Ys,Zdef); waitAll();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  setSite(0,Xdef-Xoff,Ys+Ystep,Zboot); setSite(1,Xdef-Xoff,Ys+Ystep,Zboot);
  setSite(2,Xdef+Xoff,Ys,Zboot); setSite(3,Xdef+Xoff,Ys,Zboot);
  for (int i=0;i<4;i++) for(int j=0;j<3;j++) sNow[i][j]=sExp[i][j];
  FlexiTimer2::set(20, srvISR);
  FlexiTimer2::start();
  for (int i=0;i<4;i++) for(int j=0;j<3;j++) { servo[i][j].attach(pin[i][j]); delay(50); }
  stand();
  Serial.println(F("OBS_AVOID_READY"));
}

void loop() {
  long d = getDist();
  if (d < OBS_DIST) { stepBack(3); turnRight(2); }
  else stepFwd(1);
}
