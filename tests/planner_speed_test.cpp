// Real planner + the real kinematics: print a circle, a straight horizontal line and a vertical line at F=100 mm/s
#include <vector>
#include <algorithm>
#include "Marlin.h"
#include "planner.h"
int extrudemultiply=100; float volumetric_multiplier[1]={1}; int fanSpeed=0; uint8_t active_extruder=0;
void manage_inactivity(bool){} void manage_heater(){}
float degHotend(uint8_t){return 200;} float degTargetHotend(uint8_t){return 200;} void setTargetHotend0(float){}
void st_wake_up(){} void st_set_position(const long&,const long&,const long&,const long&){} void st_set_e_position(const long&){}
#define DELTA_RADIUS_V 124.0
static float tx[3],ty[3]; static float L=250.0f, delta_[3];
static void calc(const float c[3]){ for(int i=0;i<3;i++) delta_[i]=sqrt(L*L-(tx[i]-c[0])*(tx[i]-c[0])-(ty[i]-c[1])*(ty[i]-c[1]))+c[2]; }
// prepare_move as in Marlin_main.cpp: segments of the cartesian move
static void seg_move(const float a[3], const float b[3], float F /*mm/s*/, std::vector<double>& ratios, bool cart_len){
  double dist=sqrt((b[0]-a[0])*(b[0]-a[0])+(b[1]-a[1])*(b[1]-a[1])+(b[2]-a[2])*(b[2]-a[2]));
  float seconds=dist/F; int steps=((int(200*seconds))>1?int(200*seconds):1);
  float prev[3]={a[0],a[1],a[2]};
  for(int s=1;s<=steps;s++){
    float f=float(s)/steps; float p[3]={a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f,a[2]+(b[2]-a[2])*f}; calc(p);
    double cl=sqrt((p[0]-prev[0])*(p[0]-prev[0])+(p[1]-prev[1])*(p[1]-prev[1])+(p[2]-prev[2])*(p[2]-prev[2]));
    while(movesplanned()>=BLOCK_BUFFER_SIZE-2){ plan_discard_current_block(); }
    plan_buffer_line(delta_[0],delta_[1],delta_[2],0,F,0, cart_len? (float)cl:0.0f);
    block_t* last=&block_buffer[(block_buffer_head+BLOCK_BUFFER_SIZE-1)&(BLOCK_BUFFER_SIZE-1)];
    if(last->nominal_rate>0){ double t=(double)last->step_event_count/last->nominal_rate; ratios.push_back(cl/t/F); }
    prev[0]=p[0];prev[1]=p[1];prev[2]=p[2];
  }
}
static int failed=0;
static void report(const char*name,std::vector<double>& r,bool expect_exact){ double mn=1e9,mx=0,s=0; for(double v:r){if(v<mn)mn=v; if(v>mx)mx=v;s+=v;} printf("  %-34s nozzle speed / F : max %.3f  mean %.3f  (%zu segments)\n",name,mx,s/r.size(),r.size());
  if(expect_exact && !(s/r.size()>0.98 && mx<1.01)) { puts("  FAIL: the nozzle speed is not the feed rate"); failed=1; } }
int main(){
  const float S60=0.8660254f; tx[0]=-S60*124;ty[0]=-62;tx[1]=S60*124;ty[1]=-62;tx[2]=0;ty[2]=124;
  float spu[4]={80,80,80,836}; for(int i=0;i<4;i++){axis_steps_per_unit[i]=spu[i];}
  float mf[4]={500,500,500,25}; for(int i=0;i<4;i++) max_feedrate[i]=mf[i];
  unsigned long ma[4]={9000,9000,9000,10000}; for(int i=0;i<4;i++) max_acceleration_units_per_sq_second[i]=ma[i];
  acceleration=3000; retract_acceleration=3000; max_xy_jerk=20; max_z_jerk=20; max_e_jerk=5; reset_acceleration_rates(); plan_init();
  for(int mode=0;mode<2;mode++){
    printf("%s\n",mode==0?"without the Cartesian length (the planner measures the move in tower space, speeds are wrong):":"with the Cartesian length of the segment (what the firmware does):");
    std::vector<double> r1,r2,r3,r4;
    float c0[3]={-60,0,50},c1[3]={60,0,50}; plan_init(); seg_move(c0,c1,100,r1,mode); report("line through the centre, F=100",r1,mode==1);
    float d0[3]={-80,-70,50},d1[3]={-80,70,50}; plan_init(); seg_move(d0,d1,100,r2,mode); report("line near the edge, F=100",r2,mode==1);
    float v0[3]={0,0,20},v1[3]={0,0,120}; plan_init(); seg_move(v0,v1,100,r3,mode); report("vertical line, F=100",r3,mode==1);
    plan_init(); std::vector<double> rc; for(int a=0;a<360;a+=10){ float p0[3]={(float)(60*cos(a*M_PI/180)),(float)(60*sin(a*M_PI/180)),50}; float p1[3]={(float)(60*cos((a+10)*M_PI/180)),(float)(60*sin((a+10)*M_PI/180)),50}; seg_move(p0,p1,100,rc,mode);} report("circle r=60, F=100",rc,mode==1);
  }
  if(!failed) puts("planner nozzle speed OK"); return failed; }
