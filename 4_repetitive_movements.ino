#include <Servo.h>
#include <FlexiTimer2.h>

Servo servo[4][3];
const int pin[4][3] = {{3,4,2},{6,7,5},{9,8,10},{12,11,13}};

const float La=55, Lb=77.5, Lc=27.5, Ls=71;
const float Zabs=-28, Zdef=-50, Zup=-30, Zboot=Zabs;
const float Xdef=62, Xoff=0, Ys=0, Ystep=40, Ydef=Xdef;
const float KEEP=255, PI_V=3.1415926;
const float spd_turn=4, spd_leg=8, spd_body=3, spd_seat=1;

volatile float sNow[4][3], sExp[4][3];
float tSpd[4][3], mSpd, spdMul=1;
volatile int restCnt;

const float tA=sqrt(pow(2*Xdef+Ls,2)+pow(Ystep,2));
const float tB=2*(Ys+Ystep)+Ls;
const float tC=sqrt(pow(2*Xdef+Ls,2)+pow(2*Ys+Ystep+Ls,2));
const float tAlpha=acos((tA*tA+tB*tB-tC*tC)/2/tA/tB);
const float tx1=(tA-Ls)/2, ty1=Ys+Ystep/2;
const float tx0=tx1-tB*cos(tAlpha), ty0=tB*sin(tAlpha)-ty1-Ls;

void cart2pol(volatile float &a, volatile float &b, volatile float &g,
             volatile float x, volatile float y, volatile float z) {
  float w=(x>=0?1:-1)*sqrt(x*x+y*y), v=w-Lc;
  a=atan2(z,v)+acos((La*La-Lb*Lb+v*v+z*z)/2/La/sqrt(v*v+z*z));
  b=acos((La*La+Lb*Lb-v*v-z*z)/2/La/Lb);
  g=(w>=0)?atan2(y,x):atan2(-y,-x);
  a=a/PI_V*180; b=b/PI_V*180; g=g/PI_V*180;
}

void pol2srv(int l, float a, float b, float g) {
  if(l==0){a=90-a;g+=90;} else if(l==1){a+=90;b=180-b;g=90-g;}
  else if(l==2){a+=90;b=180-b;g=90-g;} else{a=90-a;g+=90;}
  servo[l][0].write(a); servo[l][1].write(b); servo[l][2].write(g);
}

void srvISR(void) {
  sei(); static float a,b,g;
  for(int i=0;i<4;i++){
    for(int j=0;j<3;j++){
      if(abs(sNow[i][j]-sExp[i][j])>=abs(tSpd[i][j])) sNow[i][j]+=tSpd[i][j];
      else sNow[i][j]=sExp[i][j];
    }
    cart2pol(a,b,g,sNow[i][0],sNow[i][1],sNow[i][2]);
    pol2srv(i,a,b,g);
  }
  restCnt++;
}

void setSite(int l, float x, float y, float z) {
  float dx=0,dy=0,dz=0;
  if(x!=KEEP) dx=x-sNow[l][0]; if(y!=KEEP) dy=y-sNow[l][1]; if(z!=KEEP) dz=z-sNow[l][2];
  float len=sqrt(dx*dx+dy*dy+dz*dz);
  tSpd[l][0]=dx/len*mSpd*spdMul; tSpd[l][1]=dy/len*mSpd*spdMul; tSpd[l][2]=dz/len*mSpd*spdMul;
  if(x!=KEEP) sExp[l][0]=x; if(y!=KEEP) sExp[l][1]=y; if(z!=KEEP) sExp[l][2]=z;
}

void waitLeg(int l){while(sNow[l][0]!=sExp[l][0]||sNow[l][1]!=sExp[l][1]||sNow[l][2]!=sExp[l][2]);}
void waitAll(){for(int i=0;i<4;i++) waitLeg(i);}
void stand(){mSpd=spd_seat;for(int i=0;i<4;i++)setSite(i,KEEP,KEEP,Zdef);waitAll();}
void sit(){mSpd=spd_seat;for(int i=0;i<4;i++)setSite(i,KEEP,KEEP,Zboot);waitAll();}

void stepFwd(unsigned int s) {
  mSpd=spd_leg;
  while(s-->0){
    if(sNow[2][1]==Ys){
      setSite(2,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zdef);waitAll();
      mSpd=spd_body;
      setSite(0,Xdef+Xoff,Ys,Zdef);setSite(1,Xdef+Xoff,Ys+2*Ystep,Zdef);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef);setSite(3,Xdef-Xoff,Ys+Ystep,Zdef);waitAll();
      mSpd=spd_leg;
      setSite(1,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(1,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(1,Xdef+Xoff,Ys,Zdef);waitAll();
    } else {
      setSite(0,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zdef);waitAll();
      mSpd=spd_body;
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef);setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys,Zdef);setSite(3,Xdef+Xoff,Ys+2*Ystep,Zdef);waitAll();
      mSpd=spd_leg;
      setSite(3,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(3,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(3,Xdef+Xoff,Ys,Zdef);waitAll();
    }
  }
}

void stepBack(unsigned int s) {
  mSpd=spd_leg;
  while(s-->0){
    if(sNow[3][1]==Ys){
      setSite(3,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(3,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(3,Xdef+Xoff,Ys+2*Ystep,Zdef);waitAll();
      mSpd=spd_body;
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zdef);setSite(1,Xdef+Xoff,Ys,Zdef);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef);setSite(3,Xdef-Xoff,Ys+Ystep,Zdef);waitAll();
      mSpd=spd_leg;
      setSite(0,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(0,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(0,Xdef+Xoff,Ys,Zdef);waitAll();
    } else {
      setSite(1,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(1,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(1,Xdef+Xoff,Ys+2*Ystep,Zdef);waitAll();
      mSpd=spd_body;
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef);setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zdef);setSite(3,Xdef+Xoff,Ys,Zdef);waitAll();
      mSpd=spd_leg;
      setSite(2,Xdef+Xoff,Ys+2*Ystep,Zup);waitAll();
      setSite(2,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(2,Xdef+Xoff,Ys,Zdef);waitAll();
    }
  }
}

void turnLeft(unsigned int s) {
  mSpd=spd_turn;
  while(s-->0){
    if(sNow[3][1]==Ys){
      setSite(3,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(0,tx1-Xoff,ty1,Zdef);setSite(1,tx0-Xoff,ty0,Zdef);
      setSite(2,tx1+Xoff,ty1,Zdef);setSite(3,tx0+Xoff,ty0,Zup);waitAll();
      setSite(3,tx0+Xoff,ty0,Zdef);waitAll();
      setSite(0,tx1+Xoff,ty1,Zdef);setSite(1,tx0+Xoff,ty0,Zdef);
      setSite(2,tx1-Xoff,ty1,Zdef);setSite(3,tx0-Xoff,ty0,Zdef);waitAll();
      setSite(1,tx0+Xoff,ty0,Zup);waitAll();
      setSite(0,Xdef+Xoff,Ys,Zdef);setSite(1,Xdef+Xoff,Ys,Zup);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef);setSite(3,Xdef-Xoff,Ys+Ystep,Zdef);waitAll();
      setSite(1,Xdef+Xoff,Ys,Zdef);waitAll();
    } else {
      setSite(0,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(0,tx0+Xoff,ty0,Zup);setSite(1,tx1+Xoff,ty1,Zdef);
      setSite(2,tx0-Xoff,ty0,Zdef);setSite(3,tx1-Xoff,ty1,Zdef);waitAll();
      setSite(0,tx0+Xoff,ty0,Zdef);waitAll();
      setSite(0,tx0-Xoff,ty0,Zdef);setSite(1,tx1-Xoff,ty1,Zdef);
      setSite(2,tx0+Xoff,ty0,Zdef);setSite(3,tx1+Xoff,ty1,Zdef);waitAll();
      setSite(2,tx0+Xoff,ty0,Zup);waitAll();
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef);setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys,Zup);setSite(3,Xdef+Xoff,Ys,Zdef);waitAll();
      setSite(2,Xdef+Xoff,Ys,Zdef);waitAll();
    }
  }
}

void turnRight(unsigned int s) {
  mSpd=spd_turn;
  while(s-->0){
    if(sNow[2][1]==Ys){
      setSite(2,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(0,tx0-Xoff,ty0,Zdef);setSite(1,tx1-Xoff,ty1,Zdef);
      setSite(2,tx0+Xoff,ty0,Zup);setSite(3,tx1+Xoff,ty1,Zdef);waitAll();
      setSite(2,tx0+Xoff,ty0,Zdef);waitAll();
      setSite(0,tx0+Xoff,ty0,Zdef);setSite(1,tx1+Xoff,ty1,Zdef);
      setSite(2,tx0-Xoff,ty0,Zdef);setSite(3,tx1-Xoff,ty1,Zdef);waitAll();
      setSite(0,tx0+Xoff,ty0,Zup);waitAll();
      setSite(0,Xdef+Xoff,Ys,Zup);setSite(1,Xdef+Xoff,Ys,Zdef);
      setSite(2,Xdef-Xoff,Ys+Ystep,Zdef);setSite(3,Xdef-Xoff,Ys+Ystep,Zdef);waitAll();
      setSite(0,Xdef+Xoff,Ys,Zdef);waitAll();
    } else {
      setSite(1,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(0,tx1+Xoff,ty1,Zdef);setSite(1,tx0+Xoff,ty0,Zup);
      setSite(2,tx1-Xoff,ty1,Zdef);setSite(3,tx0-Xoff,ty0,Zdef);waitAll();
      setSite(1,tx0+Xoff,ty0,Zdef);waitAll();
      setSite(0,tx1-Xoff,ty1,Zdef);setSite(1,tx0-Xoff,ty0,Zdef);
      setSite(2,tx1+Xoff,ty1,Zdef);setSite(3,tx0+Xoff,ty0,Zdef);waitAll();
      setSite(3,tx0+Xoff,ty0,Zup);waitAll();
      setSite(0,Xdef-Xoff,Ys+Ystep,Zdef);setSite(1,Xdef-Xoff,Ys+Ystep,Zdef);
      setSite(2,Xdef+Xoff,Ys,Zdef);setSite(3,Xdef+Xoff,Ys,Zup);waitAll();
      setSite(3,Xdef+Xoff,Ys,Zdef);waitAll();
    }
  }
}

void bodyLeft(int i){
  setSite(0,sNow[0][0]+i,KEEP,KEEP);setSite(1,sNow[1][0]+i,KEEP,KEEP);
  setSite(2,sNow[2][0]-i,KEEP,KEEP);setSite(3,sNow[3][0]-i,KEEP,KEEP);waitAll();
}

void bodyRight(int i){
  setSite(0,sNow[0][0]-i,KEEP,KEEP);setSite(1,sNow[1][0]-i,KEEP,KEEP);
  setSite(2,sNow[2][0]+i,KEEP,KEEP);setSite(3,sNow[3][0]+i,KEEP,KEEP);waitAll();
}

void handWave(int i){
  float xt,yt,zt; mSpd=1;
  if(sNow[3][1]==Ys){
    bodyRight(15); xt=sNow[2][0];yt=sNow[2][1];zt=sNow[2][2]; mSpd=spd_body;
    for(int j=0;j<i;j++){setSite(2,tx1,ty1,50);waitAll();setSite(2,tx0,ty0,50);waitAll();}
    setSite(2,xt,yt,zt);waitAll(); mSpd=1; bodyLeft(15);
  } else {
    bodyLeft(15); xt=sNow[0][0];yt=sNow[0][1];zt=sNow[0][2]; mSpd=spd_body;
    for(int j=0;j<i;j++){setSite(0,tx1,ty1,50);waitAll();setSite(0,tx0,ty0,50);waitAll();}
    setSite(0,xt,yt,zt);waitAll(); mSpd=1; bodyRight(15);
  }
}

void handShake(int i){
  float xt,yt,zt; mSpd=1;
  if(sNow[3][1]==Ys){
    bodyRight(15); xt=sNow[2][0];yt=sNow[2][1];zt=sNow[2][2]; mSpd=spd_body;
    for(int j=0;j<i;j++){setSite(2,Xdef-30,Ys+2*Ystep,55);waitAll();setSite(2,Xdef-30,Ys+2*Ystep,10);waitAll();}
    setSite(2,xt,yt,zt);waitAll(); mSpd=1; bodyLeft(15);
  } else {
    bodyLeft(15); xt=sNow[0][0];yt=sNow[0][1];zt=sNow[0][2]; mSpd=spd_body;
    for(int j=0;j<i;j++){setSite(0,Xdef-30,Ys+2*Ystep,55);waitAll();setSite(0,Xdef-30,Ys+2*Ystep,10);waitAll();}
    setSite(0,xt,yt,zt);waitAll(); mSpd=1; bodyRight(15);
  }
}

void headUp(int i){
  setSite(0,KEEP,KEEP,sNow[0][2]-i);setSite(1,KEEP,KEEP,sNow[1][2]+i);
  setSite(2,KEEP,KEEP,sNow[2][2]-i);setSite(3,KEEP,KEEP,sNow[3][2]+i);waitAll();
}

void headDown(int i){
  setSite(0,KEEP,KEEP,sNow[0][2]+i);setSite(1,KEEP,KEEP,sNow[1][2]-i);
  setSite(2,KEEP,KEEP,sNow[2][2]+i);setSite(3,KEEP,KEEP,sNow[3][2]-i);waitAll();
}

void bodyDance(int i){
  float ds=2; sit(); mSpd=1;
  setSite(0,Xdef,Ydef,KEEP);setSite(1,Xdef,Ydef,KEEP);
  setSite(2,Xdef,Ydef,KEEP);setSite(3,Xdef,Ydef,KEEP);waitAll();
  setSite(0,Xdef,Ydef,Zdef-20);setSite(1,Xdef,Ydef,Zdef-20);
  setSite(2,Xdef,Ydef,Zdef-20);setSite(3,Xdef,Ydef,Zdef-20);waitAll();
  mSpd=ds; headUp(30);
  for(int j=0;j<i;j++){
    if(j>i/4) mSpd=ds*2; if(j>i/2) mSpd=ds*3;
    setSite(0,KEEP,Ydef-20,KEEP);setSite(1,KEEP,Ydef+20,KEEP);
    setSite(2,KEEP,Ydef-20,KEEP);setSite(3,KEEP,Ydef+20,KEEP);waitAll();
    setSite(0,KEEP,Ydef+20,KEEP);setSite(1,KEEP,Ydef-20,KEEP);
    setSite(2,KEEP,Ydef+20,KEEP);setSite(3,KEEP,Ydef-20,KEEP);waitAll();
  }
  mSpd=ds; headDown(30);
}

void setup() {
  Serial.begin(115200);
  setSite(0,Xdef-Xoff,Ys+Ystep,Zboot);setSite(1,Xdef-Xoff,Ys+Ystep,Zboot);
  setSite(2,Xdef+Xoff,Ys,Zboot);setSite(3,Xdef+Xoff,Ys,Zboot);
  for(int i=0;i<4;i++) for(int j=0;j<3;j++) sNow[i][j]=sExp[i][j];
  FlexiTimer2::set(20,srvISR); FlexiTimer2::start();
  for(int i=0;i<4;i++) for(int j=0;j<3;j++){servo[i][j].attach(pin[i][j]);delay(50);}
  Serial.println(F("REPEAT_MOVE_READY"));
}

void loop() {
  stand(); delay(2000);
  stepFwd(5); delay(2000);
  stepBack(5); delay(2000);
  turnLeft(5); delay(2000);
  turnRight(5); delay(2000);
  handWave(3); delay(2000);
  handShake(3); delay(2000);
  bodyDance(10); delay(2000);
  sit(); delay(5000);
}
