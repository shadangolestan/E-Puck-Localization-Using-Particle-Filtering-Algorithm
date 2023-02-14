#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <pthread.h>
#include <windows.h>


#define E = 2.7182818284590452353602874713
#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)
#define HEIGHT 120
#define WIDTH 180
#define SCALE 2
#define PARTICLE_SIZE 8000
#define DIS_STEP 8
#define MAX_READING 7
#define RAND_MAX 32767
#define pi 3.14159265358979323846
#define W_Size 20
#define alpha_fast 0.4
#define alpha_slow 0.02
#define MAX_READING 5
#define STOP_THRESHOLD 80
FILE *f2;
double distance;
double speed[2];
double degree;
double w_fast = 0;
double w_slow = 0;

double step_value[DIS_STEP] = {0, 1, 2, 3, 4, 5, 6, 7 }; 
double sensors_value[8];
double sensors_dis_milestones[8][DIS_STEP] ={ { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27},
                                              { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27},
                                              { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27},
                                              { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27},
                                              { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27},
                                              { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27},
                                              { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27},
                                              { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27}                                    
                                            };

//----------------------------------------------- STRUCTS:
struct odometry {
  double distance;
  double orientation;
};

struct Particle 
{
  double x;
  double y;
  double tetha;
  double weight;
};

struct Distance_Sensors
{
  double distance[8];
};

struct MV
{
  double m[8];
  double v[8];
};

struct Point
{
  int x;
  int y;
  int num;
};

int map[HEIGHT][WIDTH];
struct Particle particles[PARTICLE_SIZE];
int turning;
int update=0;
int direction=0;

//----------------------------------------------- STRUCTS END
//----------------------------------------------- FUNCTIONS:

//----------------------------------------------- READ MAP:
static void ReadMap()
{
  int i,j;
  FILE *f;
  f = fopen("c://shadan//p_files//map.txt","r");
  for(i =0; i<HEIGHT; i++)
    for(j=0; j<WIDTH; j++)
    {    char ch = getc(f);
         while(ch==' ' || ch=='\n')
            ch = getc(f);
            
         map[i][j]=ch-48;
            
    }
}

//----------------------------------------------- INITIALIZE PARTICLES:
static void Init_Particle()
{  
  w_fast = 0;
  w_slow = 0;
  
  int i, x, y;
  srand(time(NULL));
  
  for(i=0; i<PARTICLE_SIZE; i++)
  {
    //particles[i].x = 60;
    //particles[i].y = 90;
    //particles[i].tetha = pi/2;
    //particles[i].weight = 0;
    
    
    int ran = rand() % 18;
    double orientation;
    
    if      (ran == 0)    orientation = (double)(0);
    else if (ran == 1)    orientation = (double)(pi/8);
    else if (ran == 2)    orientation = (double)(2*(double)(pi/8));
    else if (ran == 3)    orientation = (double)(3*(double)(pi/8));
    else if (ran == 4)    orientation = (double)(4*(double)(pi/8));
    else if (ran == 5)    orientation = (double)(5*(double)(pi/8));
    else if (ran == 6)    orientation = (double)(6*(double)(pi/8));
    else if (ran == 7)    orientation = (double)(7*(double)(pi/8));
    else if (ran == 8)    orientation = (double)(pi);
    else if (ran == 9)    orientation = (double)(9*(double)(pi/8));
    else if (ran == 10)   orientation = (double)(10*(double)(pi/8));
    else if (ran == 11)   orientation = (double)(11*(double)(pi/8));
    else if (ran == 12)   orientation = (double)(12*(double)(pi/8));
    else if (ran == 13)   orientation = (double)(13*(double)(pi/8));
    else if (ran == 14)   orientation = (double)(14*(double)(pi/8));
    else if (ran == 15)   orientation = (double)(15*(double)(pi/8));
    else if (ran == 16)   orientation = (double)(16*(double)(pi/8));
    else if (ran == 17)   orientation = (double)(17*(double)(pi/8));
    
    do
    {
      x =  rand() % HEIGHT;
      y =  rand() % WIDTH;
      
      particles[i].x = x;
      particles[i].y = y;
      particles[i].tetha = orientation;
      //particles[i].tetha = pi;
      //particles[i].x = 110;
      //particles[i].y = 10;
      
      //((double)(rand() % 360)/180)*(pi);
      particles[i].weight = 0.00000;
      //printf("P[%d]--> x=%d y=%d t=%f w=%f\n",x,y,particles[i].tetha,particles[i].weight);
    } while (map[x][y] == 1);
    //printf("i=%d x=%d y=%d t=%f\n",i,x,y,particles[i].tetha);
  }
}

//----------------------------------------------- MEASURE SENSORS (DREAM FUNCTION):

static struct Distance_Sensors MeasureSensors (struct Particle particle)
{
  bool f[8]={0, 0, 0, 0, 0, 0, 0, 0};
  int i, j;
  double rayx[8];
  double rayy[8];
  
  double tethas[8]= {1.27-pi/2, 0.77-pi/2, 0.00-pi/2, 5.21-pi/2, 	4.21-pi/2, 3.14159-pi/2, 	2.37-pi/2, 1.87-pi/2};
  //double tethas[8]= {1.27, 0.77, 0.00, 5.21, 	4.21, 3.14159, 	2.37, 1.87};
  double r = 3.7;
  struct Distance_Sensors ds;
  
  for(i=1; (!f[0] || !f[1] || !f[2] || !f[3] || !f[4] || !f[5] || !f[6] || !f[7]); i++)
  {
      //printf("\n\n\ni=%d\n",i);
      for(j=0; j<8; j++)
      {
        if(f[j]!=1)
        {
          //printf("cal rayx[%d] start",j);
          rayx[j] = particle.x + cos(particle.tetha + tethas[j])*(SCALE*r + i);
          //printf("\tcompleted=%d\n",j,(int)rayx[j]);
          //printf("cal rayy[%d]",j);
          rayy[j] = particle.y + sin(particle.tetha + tethas[j])*(SCALE*r + i);
          //printf("\tcompleted=%d\n",j,(int)rayy[j]);
          if(map[(int)(rayx[j])][(int)(rayy[j])]==1)
            f[j]=1;
            
          if((int)(rayx[j])<3 || (int)(rayx[j])>HEIGHT-3 || (int)(rayy[j])<3 || (int)(rayy[j])>WIDTH-3 )
          {  
            f[j]=1;
          }
          //printf(" j = %d    Rx=%f , Ry=%f\n", j, rayx[j], rayy[j]);
        }
      }
      //printf("\nin ray casting i = %d\n\n",i);
  }
  //printf("part222222\n");
  //calculate distance for each sensor
  for(i=0; i<8; i++)
  {
    ds.distance[i] = (double)(sqrt( pow(rayx[i]-particle.x,2) + pow(rayy[i]-particle.y,2) )/SCALE) - r;
    if(ds.distance[i] >= MAX_READING)
      ds.distance[i] = MAX_READING;
    //printf("\n DS(%d) = %f\n", i, ds.distance[i]);
  }
  return ds;
} 

//----------------------------------------------- COMPUTER ODOMETRY:

static struct odometry compute_odometry() {
  double l = wb_differential_wheels_get_left_encoder();
  double r = wb_differential_wheels_get_right_encoder();
  double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
  double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH; // delta orientation
  
  struct odometry data ;
  data.distance = dl;
  data.orientation = asin(da);
  
  return data;
}

//----------------------------------------------- MOVE ROBOT:


struct odometry MoveRobot(double sensors_value[])
{
  double braitenberg_coefficients[8][2] =
    { 
      {150, -35}, {100, -15}, {80 , -10}, {-10, -10}, {-10, -10}, {-10,  80}, {-30, 100}, {-20, 150} 
    };
  int i;
  struct odometry data;
  struct odometry action;
  wb_differential_wheels_set_encoders(0, 0);
  srand(time(NULL));
  if(direction==0)
  {
    int r = rand()%3;
    if(r==0 || r == 1)
      direction = +35;
            
    else
      direction = -35;
  }
  
  if( sensors_value[0]>800 ||
      sensors_value[7]>800 ||
      sensors_value[1]>800 ||
      sensors_value[6]>800
    )
  {
    if (turning == 1)
    {
      
      if (direction > 0)
        direction += 0;
      
      else if (direction < 0)
        direction += 0;
    }   
    
    if(direction > 0)
    {
      
      speed[0] = +35;
      speed[1] = -35;
      direction--;
    }
    else
    {
      speed[0] = -35;
      speed[1] = +35;
      direction++;
    }
    
    data = compute_odometry();
    while(-(double)(pi/10) <= data.orientation && data.orientation  <= (double)(pi/10))
    { 
      wb_differential_wheels_set_speed(speed[0],speed[1]);
      wb_robot_step(TIME_STEP);
      data = compute_odometry();
    }
    turning = 1;
  }
  else
  {
    speed[0] = 120;
    speed[1] = 120;
    
    data = compute_odometry();
    
    while(data.distance < 0.01)
    { 
      wb_differential_wheels_set_speed(speed[0],speed[1]);
      wb_robot_step(TIME_STEP);
      data = compute_odometry();
    }
    turning = 0;
  }
  
  if(turning == 1)
  { 
    action.orientation = data.orientation;
    action.distance = 0;
  } 
  else
  {
    action.orientation = 0;
    action.distance = data.distance;
  }
  
  speed[0] = 0;
  speed[1] = 0;
  wb_differential_wheels_set_speed(speed[0],speed[1]);
  wb_robot_step(TIME_STEP);
  
  return action;
}

//----------------------------------------------- MVNPDF:

static double mvnpdf(double x, double mean, double var)
{
  return (1/sqrt(2*3.1415*var))*pow(2.7,-(pow(x-mean,2) / (2*var)));
}

//----------------------------------------------- RNDG:

double rndg(double mu, double sigma)
{
 const double epsilon = 0.00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000285;
 const double two_pi = 2.0*3.14159265358979323846;
 
 static double z0, z1;
 static bool generate;
 generate = !generate;
 
 if (!generate)
 return z1 * sigma + mu;
 
 double u1, u2;
 
 //printf("while in!-----");
 do
 {
   u1 = rand() * (1.0 / RAND_MAX);
   u2 = rand() * (1.0 / RAND_MAX);
 } while ( u1 <= epsilon );
 //printf("while out!\n");
 
 z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
 z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
 return z0 * sigma + mu;
}

//----------------------------------------------- DISTANCE TO MV:

static struct MV Distance2MV(struct Distance_Sensors distances)
{
  struct MV temp;
  int i;
  for(i=0; i < 8; i++)
  {
    int index = 0;
    while(distances.distance[i] >= step_value[index] && index<DIS_STEP)
      index++;
      
    if(index==0)
    index++;
    
    //printf("INDEX FOR SENSOR %d IN DIST2MV IS: %d\n",i, index);
    
    //printf("index=%d, d=%f\n",index,distances.distance[i]);
    if(index<DIS_STEP)
    {
      //y=a(x-x0)+b
      double a = (sensors_dis_milestones[i][index] - sensors_dis_milestones[i][index-1] ) / (step_value[index] - step_value[index-1]);
      temp.m[i] = a*(distances.distance[i] - step_value[index-1] ) + sensors_dis_milestones[i][index-1];
      temp.v[i] = temp.m[i] * 0.20;
      //printf("d=%f, sens(M)=%f\n",distances.distance[i],temp.m[i]);
    }
    else
    {
      temp.m[i] = sensors_dis_milestones[i][index-1];
      temp.v[i] = temp.m[i] * 0.2;
      //printf("d=%f, sens(M)=%f\n",distances.distance[i],temp.m[i]);
    }
  }
    

  return temp;

}


//----------------------------------------------- CONVERT SENSOR TO DINSTANCE:

static double S2D(double sens, int sensor_index)
{
  int i;
  int index = 0;
  
  //sensor_index = 0;
  //double lc [1][DIS_STEP] = { 3212.2, 1273.3, 447.56, 221.99, 123.31, 65.85, 44.44, 32.27};
  
  //roooz
  /*
  double lc [8][DIS_STEP] = { 
                              {3000, 1600,     550,    720,    630,    595,    550,     310},
                              {3000, 1600,     530,    360,    290,    239,    210,     190},
                              {3000, 1800,     955,    802,    695,    665,    625,     608},
                              {3000, 1390,     710,    520,    445,    420,    395,     370},
                              {3000, 1400,     850,    720,    680,    640,    622,	    610},
                              {3000, 1560,     825,    640,    670,    530,    505,	    485},
                              {3000, 1210,     540,    345,    275,    230,    202,     185},
                              {3000, 1150,     650,    500,    427,    390,    370,     350}
                            };
                            
  */
  
  
  
  //22222222222
    double lc [8][DIS_STEP] = { 
                              {3400, 1420,     870,    720,    670,    620,    600,     590},
                              {3520, 1080,     500,    330,    270,    230,    210,     190},
                              {3570, 1560,     960,    772,    705,    665,    635,     615},
                              {3640, 1070,     666,    510,    445,    420,    395,     370},
                              {3620, 1255,     830,    720,    680,    640,    622,	    610},
                              {3575, 1390,     810,    640,    550,    510,    500,	    448},
                              {3600, 1045,     530,    351,    268,    222,    202,     190},
                              {3560, 1000,      575,    439,    370,    334,    313,     295}
                            };
                            
  
  /*
  //Gorgo mish
  
  double lc [8][DIS_STEP] = { 
                              {3745, 1500,     720,    600,    508,    490,    400,     350},
                              {3760, 1100,     440,    245,    180,    150,    135,     120},
                              {3680, 1600,     850,    665,    590,    560,    530,     520},
                              {3400, 1010,     512,    370,    325,    300,    285,     270},
                              {3770, 1100,     760,    620,    570,    540,    525,	    515},
                              {3750, 1460,     750,    540,    475,    440,    411,	    400},
                              {3780, 1140,     390,    240,    170,    150,    140,     130},
                              {3100, 1030,     730,    424,    365,    342,    330,     320}
                            };
  
  */
  
  
  for (i = 1; i < DIS_STEP; i++)
  {
    if (sens < lc[sensor_index][i])
      index++;
  }
  
  double y1 = lc[sensor_index][index];
  double y2 = lc[sensor_index][index + 1];
  
  double x1 = index;
  double x2 = index + 1;
  
  
  double a = (double)((y2 - y1)/(x2 - x1));
  double x = (double)((sens - y1 + a*x1) / a);
  
    
  //for(i=0; i<DIS_STEP; i++)
  //  if(abs(sens - lc[sensor_index][index]) > abs(sens - lc[sensor_index][i]))
  //    index=i;
  return x;
}

//----------------------------------------------- WEIGHT PARTICLES:

static void weight_particle()
{
  int i;
  for(i = 0; i < PARTICLE_SIZE; i++)
  {
    bool cond = particles[i].x >= 0 && particles[i].x <= HEIGHT && particles[i].y >= 0 && particles[i].y <= WIDTH;
    if(!cond || map[(int)particles[i].x][(int)particles[i].y]==1 || particles[i].weight == -1 )
    {
      particles[i].weight = -1;
      continue;
    }
    
    struct Distance_Sensors distances = MeasureSensors(particles[i]);
    //struct MV particle_mv = Distance2MV(distances);
    
    int k = 0;
    int j;
    double sum = 1;
    float p_max = 0, p_rand, p_hit;
    float z_rand = 0.4, z_max = 0.9, z_hit = 4;
    
    for (j = 0; j < 8; j++)
    {
      if (distances.distance[j] > MAX_READING)
      {
        p_max = 1;
        p_hit = 0;
        p_rand = 0.2;
        //printf("HERE!");
      }
      
      else
      {
        p_max = 0;
        //p_hit = (mvnpdf(sensors_value[j],particle_mv.m[j], particle_mv.v[j]*10));
        p_hit = mvnpdf(S2D(sensors_value[j], j), distances.distance[j], 1);
        p_rand = 0;
        //printf("P_hit=%f\n",p_hit);
      }
      sum *= (p_max * z_max + p_rand * z_rand + p_hit * z_hit);
    }
    
    particles[i].weight = sum;
  }
  
  int min_index = FindMinIndex();
  int max_index = FindMaxIndex();
  
  //printf("max weight is(%d) = %f\n", max_index, particles[max_index].weight);
  //printf("min weight is(%d) = %f\n\n", min_index, particles[min_index].weight);
  
  
  //for(i=0; i<PARTICLE_SIZE; i++)
  //   if(particles[i].weight!=-1)
  //      particles[i].weight += particles[max_index].weight + abs(rndg(0,10*particles[max_index].weight/1));
  
  
  //for (i = 0; i < PARTICLE_SIZE; i++)
  //{
  //  if(particles[i].weight!=-1)
  //  {
  //    particles[i].weight = (particles[i].weight - particles[min_index].weight) / (particles[max_index].weight - particles[min_index].weight); 
  //  }
  //}
  
  for (i = 0; i < PARTICLE_SIZE; i++)
  {
    if (particles[i].weight != -1)
      particles[i].weight = particles[i].weight + (double)(particles[max_index].weight/50);
  }
  
  //for (i = 0; i < PARTICLE_SIZE; i++)
  //  printf("weight(%d) = %f \n", i, particles[i].weight);
}
//----------------------------------------------- FIND MAX INDEX:

int FindMaxIndex()
{
  int i, j;
  int max_index = 0;
  for (i = 1; i < PARTICLE_SIZE; i++)
  {
    if (particles[i].weight >= particles[max_index].weight)
      max_index = i;
  }
  
  return max_index;
}

//----------------------------------------------- FIND MIN INDEX:

int FindMinIndex()
{
  int i, j;
  int min_index = 0;
  for (i = 1; i < PARTICLE_SIZE; i++)
  {
    if (particles[i].weight < particles[min_index].weight && particles[i].weight!=-1)
      min_index = i;
  }
  
  return min_index;
}

//----------------------------------------------- MOVE PARTICLES:

static void move_particles(struct odometry action)
{
  //printf("\n----for entered");
  int i;
  //srand(time(NULL));
  for(i=0; i<PARTICLE_SIZE; i++)
  {      
    if(turning==0)
    {
      double dx =  SCALE * (double)(action.distance) * (double)cos(particles[i].tetha) + rndg(0,0.01);
      double dy =  SCALE * (double)(action.distance) * (double)sin(particles[i].tetha) + rndg(0,0.01);
      double dt = rndg(0,0.001);
      
      particles[i].tetha += dt;
      
      //if(particles[i].tetha >= 2*pi)
      //  particles[i].tetha = particles[i].tetha - 2*pi;
        
      //else if (particles[i].tetha <= -2*pi)
      //  particles[i].tetha = particles[i].tetha + 2*pi;
      
      
      int b=0;
      while( b<5 && particles[i].weight!=-1 ) //abs(particles[i].x + dx/b) > 1  &&  abs(particles[i].y + dy/b) ) 
      {
        particles[i].x += (double)(dx/5);
        particles[i].y += (double)(dy/5);
        
        bool cond1 = particles[i].x >= 0 && particles[i].x <= HEIGHT && particles[i].y >= 0 && particles[i].y <= WIDTH;
        bool cond2 = (map[(int)particles[i].x][(int)particles[i].y]==0);
      
        if(!cond1 || !cond2)
          particles[i].weight=-1;
        
        b++;
      }
    }
    else
    {
      double dx = rndg(0,0.00001 );
      double dy = rndg(0,0.00001 );
      double dt = action.orientation + rndg(0,0.1);
      
      particles[i].x += dx;
      particles[i].y += dy;
      particles[i].tetha += dt; 
      
      //if(particles[i].tetha >= 2*pi)
      //  particles[i].tetha = particles[i].tetha - 2*pi;
        
      //else if (particles[i].tetha <= -2*pi)
      //  particles[i].tetha = particles[i].tetha + 2*pi;
    }
  }
}

//----------------------------------------------- KILL AND BIRTH PARTICLES:

static void kill_birth()
{ 
  struct Particle p_temp[PARTICLE_SIZE]; 
  int i, j;
  double p_roulette[PARTICLE_SIZE];
  
  for (i = 0; i < PARTICLE_SIZE; i++)
    p_roulette[i] = 0;
    
    
    //----- sorting particles based on their weights
  
  for(i=0; i< PARTICLE_SIZE; i++)
  {
    int max_index = i;
    for(j=i+1; j<PARTICLE_SIZE; j++)
    {
        if(particles[j].weight > particles[max_index].weight)
            max_index = j;
    }
    
    double x,y,tetha,weight;
    
    x = particles[i].x;
    y = particles[i].y;
    tetha = particles[i].tetha;
    weight = particles[i].weight;
    
    particles[i].x = particles[max_index].x;
    particles[i].y = particles[max_index].y;
    particles[i].tetha = particles[max_index].tetha;
    particles[i].weight = particles[max_index].weight;
    
    particles[max_index].x = x;
    particles[max_index].y = y;
    particles[max_index].tetha = tetha;
    particles[max_index].weight = weight;
  }
  
  //----- sorting done!
    
  int threshold = 0;
  
  while(particles[threshold].weight >= 0 && threshold < PARTICLE_SIZE)
      threshold++;
  
  for(i = 0; i < threshold; i++)
  {
    if(i == 0)
      p_roulette[i] = particles[i].weight;
          
    else
      p_roulette[i] = p_roulette[i - 1] + particles[i].weight;
  }
  
  for(i = 0; i < threshold; i++)
  {
    p_roulette[i] = (double)(p_roulette[i] / p_roulette[threshold - 1]);
  }

  srand((unsigned)time(NULL));
  double slice_size = 0.000125;
  double slice_line_1 = 0;
  double slice_line_2 = slice_size;
  double arrow;
  struct Particle new_particle;
  
  double w_avg = 0;
  for (j = 0; j < PARTICLE_SIZE; j++)
  {
    if (particles[j].weight > 0)
      w_avg += particles[j].weight;
      
    else
      w_avg += 0;
  }
  w_avg = (double)(w_avg/PARTICLE_SIZE);
  
  w_fast = w_fast + alpha_fast * (w_avg - w_fast);
  w_slow = w_slow + alpha_slow * (w_avg - w_slow);
  
  double probability = (double)((double)(1 - (double)(w_fast/(w_slow ))) / 3);
  probability = 0.07;
  
  if (probability >= 0.3)
  {
    Init_Particle();
    return;
  }
  
  //probability =0.01;
  //printf("\nthe probability is : %f", probability);
 
  for (i = 0; i < PARTICLE_SIZE; i++)
  {
    double rnd = (double)((double)(rand() % 100) / 100);
    if (rnd < (double)(probability))
    {
      int x ;
      int y ;
      double orientation;
      int ran = rand() % 18;

      if      (ran == 0)    orientation = (double)(0);
      else if (ran == 1)    orientation = (double)(pi/8);
      else if (ran == 2)    orientation = (double)(2*(double)(pi/8));
      else if (ran == 3)    orientation = (double)(3*(double)(pi/8));
      else if (ran == 4)    orientation = (double)(4*(double)(pi/8));
      else if (ran == 5)    orientation = (double)(5*(double)(pi/8));
      else if (ran == 6)    orientation = (double)(6*(double)(pi/8));
      else if (ran == 7)    orientation = (double)(7*(double)(pi/8));
      else if (ran == 8)    orientation = (double)(pi);
      else if (ran == 9)    orientation = (double)(9*(double)(pi/8));
      else if (ran == 10)   orientation = (double)(10*(double)(pi/8));
      else if (ran == 11)   orientation = (double)(11*(double)(pi/8));
      else if (ran == 12)   orientation = (double)(12*(double)(pi/8));
      else if (ran == 13)   orientation = (double)(13*(double)(pi/8));
      else if (ran == 14)   orientation = (double)(14*(double)(pi/8));
      else if (ran == 15)   orientation = (double)(15*(double)(pi/8));
      else if (ran == 16)   orientation = (double)(16*(double)(pi/8));
      else if (ran == 17)   orientation = (double)(17*(double)(pi/8));

      do
      {
        x = rand()%HEIGHT;
        y = rand()%WIDTH;
        new_particle.x = x;
        new_particle.y = y;
        new_particle.tetha = orientation;
        new_particle.weight = 0.00000;
      } while (map[x][y]==1);
      
      p_temp[i].x = new_particle.x;
      p_temp[i].y = new_particle.y;
      p_temp[i].tetha = new_particle.tetha;
      p_temp[i].weight = new_particle.weight;
    }
    
    else
    {
      double random = ((double)((double)rand()) / (double)RAND_MAX);
      double diff = slice_line_2 - slice_line_1;
      double r = random * diff;
      arrow = (double)(slice_line_1 + r);
      
  
      int index = 0;
      if(p_roulette[index] < arrow)
      {
        index = 1;
        while(!(p_roulette[index - 1] < arrow  &&  arrow < p_roulette[index]))
        { 
          index++;
        }
      }
  
      p_temp[i].x = rndg(particles[index].x, 1);
      p_temp[i].y = rndg(particles[index].y, 1);
      p_temp[i].tetha = particles[index].tetha; //rndg(p_temp[index].tetha,0);
      p_temp[i].weight = particles[index].weight;

      int a=(int)p_temp[i].x;
      int b=(int)p_temp[i].y;
      
      if (map[a][b] == 1) 
        p_temp[i].weight = -1;
    }
    
    slice_line_1 = slice_line_1 + slice_size;
    slice_line_2 = slice_line_2 + slice_size;
  }
    
  for(i = 0; i<PARTICLE_SIZE; i++)
  {
    particles[i].x = p_temp[i].x;
    particles[i].y = p_temp[i].y;
    particles[i].tetha = p_temp[i].tetha;
    particles[i].weight = p_temp[i].weight;
  }
}

//----------------------------------------------- PRINT THE MAP:

static void print(struct Point robot)
{
  int k=0,a,b;
    //for(k=0; k<PARTICLE_SIZE; k++)
    //  printf(" weight=%f\n",particles[k].weight);
  
    f2 = fopen("c://shadan//p_files//matmap.txt","w+");
  //if(f2!=NULL)
  {
    int i, j,map2p[HEIGHT][WIDTH+1];
    
    for(i = 0; i<HEIGHT; i++)
      for(j = 0; j<WIDTH; j++)
        map2p[i][j]=map[i][j];
    //printf("A");
    for(i = 0; i < PARTICLE_SIZE; i++)
    {
      bool cond = particles[i].x > 0 && particles[i].x < HEIGHT && particles[i].y > 0 && particles[i].y < WIDTH;

      if(!cond)
          continue;
        
      a = (int)particles[i].x;
      b = (int)particles[i].y;
      //printf("\n %d %d",a,b);
      map2p[a][b] = 2;
      //map2p[i][i]=2;
    }
    
    //add robot to map
    map2p[robot.x][robot.y]=3;
    
    
    
    //printf("B");
    for(i = 0; i<HEIGHT; i++)
    {  
      for(j = 0; j<WIDTH; j++)
      {
        fputc(map2p[i][j]+48,  f2);
        fputc(' ',  f2);
      }
      fputs("\n", f2);    
    }
    fclose(f2);
    //printf("C");
  }
}

//----------------------------------------------- CLUSTERING
static struct Point clustering()
{
  //int W_Size = 8;
  int i, j, p, q, cmap[HEIGHT][WIDTH];
  //int window_array[HEIGHT-W_Size][WIDTH-W_Size];
  int window_array[HEIGHT][WIDTH];
  for(i=0; i<HEIGHT; i++)
    for(j=0; j<WIDTH; j++)
      if(map[i][j]==0)
        cmap[i][j]=0;
      else
        cmap[i][j]=-1;
  
  for(i = 0; i<PARTICLE_SIZE; i++)
  {
    int x = (int)particles[i].x;
    int y = (int)particles[i].y;
    
    if(cmap[x][y] != -1)
      cmap[x][y]++;
  }
  /*
  FILE *fx; 
  fx = fopen("c://shadan//p_files//cmap.txt","w+");
  for(i = 0; i<HEIGHT; i++)
    {  
      for(j = 0; j<WIDTH; j++)
      {
        fputc(cmap[i][j]+48,  fx);
        fputc(' ',  fx);
      }
      fputs("\n", fx);    
    }
    fclose(fx);  
  */  
    
    
  for(i=0; i<HEIGHT; i++)
    for(j=0; j<WIDTH; j++)
      window_array[i][j] = 0;
  
  int max_indexi=0, max_indexj=0;
  for(i = W_Size/2; i < HEIGHT - W_Size/2; i++)
  {
    for(j = W_Size/2; j < WIDTH - W_Size/2; j++)
    {
      
      for(p = i - W_Size/2; p < i + W_Size/2 ; p++)
      {
        for(q = j - W_Size/2; q < j + W_Size/2; q++)
        {
          if(cmap[i][j]>0)
            window_array[i][j]+=cmap[p][q];
        }
      }
      if(window_array[max_indexi][max_indexj]<window_array[i][j])
      {
        max_indexi=i;
        max_indexj=j;
      }
    }
  }
  struct Point t_point;
  t_point.x = max_indexi;
  t_point.y = max_indexj;
  t_point.num = window_array[max_indexi][max_indexj];
  return t_point;
}


//----------------------------------------------- FUNTIONS END:
//----------------------------------------------- MAIN FUNCTION:

int main(int argc, char *argv[]) {
  ReadMap();
  double distance;
  int exit = 0;
  WbDeviceTag distance_sensor[8];
  int i,j;
  wb_robot_init();
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer,TIME_STEP*4);
  wb_differential_wheels_enable_encoders(TIME_STEP*4);

  for (i = 0; i < 8; i++) {
    char device_name[4];
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*4);
  }

  int counter = 0;
  struct odometry action;
  Init_Particle();
  struct Point p_null;
  p_null.x = HEIGHT/2;
  p_null.y = WIDTH/2;
  p_null.num = 0;
  print(p_null);
      
  int c=0;
  struct odometry action_step;
  
  //HDC x = GetDC(0x0007045C);
  
  int q=0;
  
  double tt=0;
  while (1) 
  {
    c++;
    //printf("C = %d\n", c);
    //for(i=0;i<1000;i++)
    //  SetPixel(x,i,i,123);
  
    for (i = 0; i < 8; i++) 
    {
        sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
        //if(sensors_value[i]<0)
        //  sensors_value[i]=10;
        
    }  
    const double *a = wb_accelerometer_get_values(accelerometer);
    
    action = MoveRobot(sensors_value);
    action.distance = action.distance * 100;
    
    if ((action.orientation != 0) || action.distance >= 1)
    {
      move_particles(action);
      weight_particle();
      
      if (turning != 1)
      {
        kill_birth();
      }
     
      wb_differential_wheels_set_encoders(0, 0);
    }
    
    if(c % 1 == 0)
    {
      
      //printf("---- PRINTED ----\n");
      struct Point cluster = clustering();
      print(cluster);
      printf("\n\tANS = (%d,%d)-->%d\n",cluster.x/2, cluster.y/2, cluster.num);
      
      double percentage = 0.9;
      if(cluster.num >= percentage * PARTICLE_SIZE)
      {
        speed[0]=0;
        speed[1]=0;
        wb_differential_wheels_set_speed(speed[0],speed[1]);
        wb_robot_step(TIME_STEP);
        printf("\n\tLocalization Done!    %f\n",percentage * PARTICLE_SIZE);
        return;
      }
      //if(cluster>0)
      //{
      //  printf("ANS = (%d,%d)",cluster%WIDTH, cluster/WIDTH);
      //  return;
      //}
      
      c = 0;
    }
  }
  return 0;
}
