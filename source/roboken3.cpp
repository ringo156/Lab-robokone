#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "uart_mcu.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD

#define BAUD_RATE B19200

#endif
#define NUM 4                          // リンク数

dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointID      joint[NUM];              // ジョイントのID番号
dJointID      etc[2];
dsFunctions   fn;                      // ドロースタッフの描画関数

int fd;

typedef struct {
  dBodyID body;                        // ボディのID番号
  dGeomID geom;                        // ジオメトリのID番号
} MyObject;                            // MyObject構造体

union UFloatLong {
  long	lLong;
  float	fFloat;
};

MyObject rlink[NUM], tsuba, shinai;                   // リンク
dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節
int           ANSWER = 1;              // 逆運動学の解

dReal P[3] = {1.0, 1.0, 1.5};             // 先端の位置
dReal a[3], b[3];                         // 有顔ベクトル(a,b)
dReal  THETA[NUM] = {0.0};             // 関節の目標角度[rad]
static const dReal BASE1[3] = {0.041, 0.031, 0.3};

void tx_angle(float angle, int link)
{
  union UFloatLong func;
  unsigned char tx[6];
  func.fFloat = angle;
  tx[0] = ID;
  tx[1] = link;
  tx[2] = func.lLong >> 24;
  tx[3] = func.lLong >> 16;
  tx[4] = func.lLong >> 8;
  tx[5] = func.lLong;
  printf("id = %x, link = %x\n", tx[0], tx[1]);
  write(fd, tx, 6);

}

/*** ロボットアームの生成 ***/
void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dReal x[NUM]      = {0.01, 0.01, 0.00, 0.00};  // 重心 x
  dReal y[NUM]      = {0.00, 0.00, 0.00, 0.00};  // 重心 y
  dReal z[NUM]      = {0.15, 0.31, 0.3365, 0.377};  // 重心 z
  dReal weight[NUM] = {9.00, 0.10, 0.10, 0.10};  // 質量
  dReal c_x[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 x
  dReal c_y[NUM]    = {0.00, 0.00, 0.00, 0.00};  // 関節中心点 y
  dReal c_z[NUM]    = {0.30, 0.3, 0.311, 0.342};  // 関節中心点 z
  dReal axis_x[NUM] = {0, 0, 0, 0};              // 関節回転軸 x
  dReal axis_y[NUM] = {0, 0, 1, 1};              // 関節回転軸 y
  dReal axis_z[NUM] = {1, 1, 0, 0};              // 関節回転軸 z
  
  dReal xl[NUM] = {0.041, 0.041, 0.018, 0.028};
  dReal yl[NUM] = {0.031, 0.035, 0.045, 0.031};
  dReal zl[NUM] = {0.3, 0.021, 0.031, 0.05};

  // リンクの生成
  for (int i = 0; i < NUM; i++) {
    rlink[i].body = dBodyCreate(world);
    dBodySetPosition(rlink[i].body, x[i], y[i], z[i]);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, weight[i], xl[i], yl[i], zl[i]);
    dBodySetMass(rlink[i].body, &mass);
    rlink[i].geom = dCreateBox(space, xl[i], yl[i], zl[i]);
    dGeomSetBody(rlink[i].geom,rlink[i].body);
  }
  tsuba.body = dBodyCreate(world);
  dBodySetPosition(tsuba.body, 0.00, 0.00, 0.4);
  dMassSetZero(&mass);
  dMassSetCylinder(&mass, 0.5, 3, 0.04, 0.005);
  dBodySetMass(tsuba.body, &mass);
  tsuba.geom = dCreateCylinder(space, 0.04, 0.005);
  dGeomSetBody(tsuba.geom, tsuba.body);

  shinai.body = dBodyCreate(world);
  dBodySetPosition(shinai.body, 0.00, 0.00, 0.5);
  dMassSetZero(&mass);
  dMassSetCylinder(&mass, 0.5, 3, 0.003, 0.2);//r, h
  dBodySetMass(shinai.body, &mass);
  shinai.geom = dCreateCylinder(space, 0.003, 0.2);
  dGeomSetBody(shinai.geom, shinai.body);

  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], rlink[0].body, 0);
  dJointSetFixed(joint[0]);

  for (int j = 1; j < NUM; j++) {
    joint[j] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(joint[j], rlink[j].body, rlink[j-1].body);
    dJointSetHingeAnchor(joint[j], c_x[j], c_y[j], c_z[j]);
    dJointSetHingeAxis(joint[j], axis_x[j], axis_y[j],axis_z[j]);
  }
  
  etc[0] = dJointCreateFixed(world, 0);// ヒンジジョイントの生成
  dJointAttach(etc[0], tsuba.body, rlink[3].body);// 玉と円柱のボディをジョイントで結合
  dJointSetFixed(etc[0]);

  etc[1] = dJointCreateFixed(world, 0);
  dJointAttach(etc[1], shinai.body, rlink[3].body);
  dJointSetFixed(etc[1]);
  
}

/*** ロボットアームの描画 ***/
void drawArm()
{
  const dReal *pos1, *R1;
    dReal r,length;
    dReal *p[4];
    dReal length1[NUM][3] = {{0.021, 0.031, 0.3},
                             {0.041, 0.035, 0.021},
                             {0.018, 0.045, 0.031},
                             {0.028, 0.031, 0.050}};  // 長さ

    for(int i=0;i<NUM;i++){
        p[i] = length1[i];
    }

   for (int i = 0; i < NUM; i++ ) {       // カプセルの描画
     dsDrawBox(dGeomGetPosition(rlink[i].geom), dGeomGetRotation(rlink[i].geom), &length1[i][0]);
   }
   // 円柱の描画
   pos1 = dBodyGetPosition(tsuba.body);// 位置の取得
   R1   = dBodyGetRotation(tsuba.body);// 姿勢の取得
   dsDrawCylinder(pos1,R1, 0.005, 0.03);// 鍔の描画
   pos1 = dBodyGetPosition(shinai.body);
   R1   = dBodyGetRotation(shinai.body);
   dsDrawCylinder(pos1,R1, 0.2, 0.003);//竹刀の描画
}

/*** P制御 ***/
void Pcontrol()
{
  dReal k =  30.0, fMax = 200.0;                   // 比例ゲイン，最大トルク
  static float angle_now[NUM], angle_old[NUM];

  for (int j = 1; j < NUM; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    angle_now[j] = tmp;
    if(angle_old[j] != angle_now[j]){//更新したら送信する
      printf("joint%d : %f\n", j, angle_now[j]);
      if(j == 3)angle_now[j] *= -1;
      tx_angle(angle_now[j]*-1, j);
    }
    dReal z = THETA[j] - tmp;                      // 残差
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
    angle_old[j] = angle_now[j];
  }
}

/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    1.5f, 1.3f, 0.8f};          // 視点[m]
  float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]
  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}

/*** キー入力関数 ***/
void command(int cmd)
{
    if(cmd == 'j'){
      THETA[1] += M_PI/180;
    }else
    if(cmd == 'f'){
      THETA[1] -= M_PI/180;
    }else
    if(cmd == 'k'){
      THETA[2] += M_PI/180;
    }else
    if(cmd == 'd'){
      THETA[2] -= M_PI/180;
    }else
    if(cmd == 'l'){
      THETA[3] += M_PI/180;
    }else
    if(cmd == 's'){
      THETA[3] -= M_PI/180;
    }
    //角度の極限の設定
    if(THETA[1] > 90 * M_PI / 180){
      THETA[1] = 90 * M_PI / 180;
    }else
    if(THETA[1] < -90 * M_PI / 180){
      THETA[1] = -90 * M_PI / 180;
    }
  
}

/*** キー入力関数 ***/
void command2(int cmd)
{
  switch (cmd) {
  case '1':  ANSWER = 1; break;    // 1キーを押すと姿勢１
  case '2':  ANSWER = 2; break;    // 2キーを押すと姿勢２
  case '3':  ANSWER = 3; break;    // 3キーを押すと姿勢３
  case '4':  ANSWER = 4; break;    // 4キーを押すと姿勢４
  case 'j':  P[0] += 0.1; break;   // jキーを押すと先端のx座標が増加
  case 'f':  P[0] -= 0.1; break;   // fキーを押すと先端のx座標が減少
  case 'k':  P[1] += 0.1; break;   // kキーを押すと先端のy座標が増加
  case 'd':  P[1] -= 0.1; break;   // dキーを押すと先端のy座標が減少
  case 'l':  P[2] += 0.1; break;   // lキーを押すと先端のz座標が増加
  case 's':  P[2] -= 0.1; break;   // sキーを押すと先端のz座標が減少
  }
}

/*** シミュレーションループ ***/
void simLoop(int pause)
{
  Pcontrol();                                  // P制御
  dWorldStep(world, 0.01);                     // 動力学計算
  drawArm();                                   // ロボットの描画
}

/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command;                       // command関数
  fn.path_to_textures = "drawstuff/textures";
}

int main(int argc, char **argv)
{
  serial_init(BAUD_RATE, &fd);
  dInitODE();                                     // ODEの初期化
  setDrawStuff();
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  makeArm();                                      // アームの生成
  //makeSensor();                                   // センサの生成
  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE();                                    // ODEの終了
  return 0;
}
