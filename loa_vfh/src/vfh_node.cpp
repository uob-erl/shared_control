#include <ros/ros.h>
#include <sys/time.h>
#include <vector>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#define DTOR(d) ((d) * M_PI / 180)

#define TIMESUB(a, b, result)                                                 \
  do {                                                                        \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;                             \
    (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;                          \
    if ((result)->tv_usec < 0) {                                              \
      --(result)->tv_sec;                                                     \
      (result)->tv_usec += 1000000;                                           \
    }                                                                         \
  } while (0)


class VFH_Algorithm
{
public:
    VFH_Algorithm( double cell_size,
                   int window_diameter,
                   int sector_angle,
                   double safety_dist_0ms,
                   double safety_dist_1ms,
                   int max_speed,
                   int max_speed_narrow_opening,
                   int max_speed_wide_opening,
                   int max_acceleration,
                   int min_turnrate,
                   int max_turnrate_0ms,
                   int max_turnrate_1ms,
                   double min_turn_radius_safety_factor,
                   double free_space_cutoff_0ms,
                   double obs_cutoff_0ms,
                   double free_space_cutoff_1ms,
                   double obs_cutoff_1ms,
                   double weight_desired_dir,
                   double weight_current_dir );

    ~VFH_Algorithm();

    int Init();

    // Choose a new speed and turnrate based on the given laser data and current speed.
    //
    // Units/Senses:
    //  - goal_direction in degrees, 0deg is to the right.
    //  - goal_distance  in mm.
    //  - goal_distance_tolerance in mm.
    //
    int Update_VFH( double laser_ranges[361][2],
                    int current_speed,
                    float goal_direction,
                    float goal_distance,
                    float goal_distance_tolerance,
                    int &chosen_speed,
                    int &chosen_turnrate );

    // Get methods
    int   GetMinTurnrate() { return MIN_TURNRATE; }
    // Angle to goal, in degrees.  0deg is to our right.
    float GetDesiredAngle() { return Desired_Angle; }
    float GetPickedAngle() { return Picked_Angle; }

    // Max Turnrate depends on speed
    int GetMaxTurnrate( int speed );
    int GetCurrentMaxSpeed() { return Current_Max_Speed; }

    // Set methods
    void SetRobotRadius( float robot_radius ) { this->ROBOT_RADIUS = robot_radius; }
    void SetMinTurnrate( int min_turnrate ) { MIN_TURNRATE = min_turnrate; }
    void SetCurrentMaxSpeed( int Current_Max_Speed );

    // The Histogram.
    // This is public so that monitoring tools can get at it; it shouldn't
    // be modified externally.
    // Sweeps in an anti-clockwise direction.
    float *Hist;

    void Print_Cells_Mag();

private:

    // Functions

    int VFH_Allocate();

    float Delta_Angle(int a1, int a2);
    float Delta_Angle(float a1, float a2);
    int Bisect_Angle(int angle1, int angle2);

    bool Cant_Turn_To_Goal();

    // Returns 0 if something got inside the safety distance, else 1.
    int Calculate_Cells_Mag( double laser_ranges[361][2], int speed );
    // Returns 0 if something got inside the safety distance, else 1.
    int Build_Primary_Polar_Histogram( double laser_ranges[361][2], int speed );
    int Build_Binary_Polar_Histogram(int speed);
    int Build_Masked_Polar_Histogram(int speed);
    int Select_Candidate_Angle();
    int Select_Direction();
    int Set_Motion( int &speed, int &turnrate, int current_speed );

    // AB: This doesn't seem to be implemented anywhere...
    // int Read_Min_Turning_Radius_From_File(char *filename);

    void Print_Cells_Dir();
    void Print_Cells_Dist();
    void Print_Cells_Sector();
    void Print_Cells_Enlargement_Angle();
    void Print_Hist();

    // Returns the speed index into Cell_Sector, for a given speed in mm/sec.
    // This exists so that only a few (potentially large) Cell_Sector tables must be stored.
    int Get_Speed_Index( int speed );

    // Returns the safety dist in mm for this speed.
    int Get_Safety_Dist( int speed );

    float Get_Binary_Hist_Low( int speed );
    float Get_Binary_Hist_High( int speed );

    int GetTimeDouble(double* time);

    // Data

    float ROBOT_RADIUS;           // millimeters
    int CENTER_X;                 // cells
    int CENTER_Y;                 // cells
    int HIST_SIZE;                // sectors (over 360deg)

    float CELL_WIDTH;             // millimeters
    int WINDOW_DIAMETER;          // cells
    int SECTOR_ANGLE;             // degrees
    float SAFETY_DIST_0MS;        // millimeters
    float SAFETY_DIST_1MS;        // millimeters
    int Current_Max_Speed;        // mm/sec
    int MAX_SPEED;                // mm/sec
    int MAX_SPEED_NARROW_OPENING; // mm/sec
    int MAX_SPEED_WIDE_OPENING;   // mm/sec
    int MAX_ACCELERATION;         // mm/sec/sec
    int MIN_TURNRATE;             // deg/sec -- not actually used internally

    int NUM_CELL_SECTOR_TABLES;

    // Scale turnrate linearly between these two
    int MAX_TURNRATE_0MS;       // deg/sec
    int MAX_TURNRATE_1MS;       // deg/sec
    double MIN_TURN_RADIUS_SAFETY_FACTOR;
    float Binary_Hist_Low_0ms, Binary_Hist_High_0ms;
    float Binary_Hist_Low_1ms, Binary_Hist_High_1ms;
    float U1, U2;
    float Desired_Angle, Dist_To_Goal, Goal_Distance_Tolerance;
    float Picked_Angle, Last_Picked_Angle;
    int   Max_Speed_For_Picked_Angle;

    // Radius of dis-allowed circles, either side of the robot, which
    // we can't enter due to our minimum turning radius.
    float Blocked_Circle_Radius;

    std::vector<std::vector<float> > Cell_Direction;
    std::vector<std::vector<float> > Cell_Base_Mag;
    std::vector<std::vector<float> > Cell_Mag;
    std::vector<std::vector<float> > Cell_Dist;      // millimetres
    std::vector<std::vector<float> > Cell_Enlarge;

    // Cell_Sector[x][y] is a vector of indices to sectors that are effected if cell (x,y) contains
    // an obstacle.
    // Cell enlargement is taken into account.
    // Acess as: Cell_Sector[speed_index][x][y][sector_index]
    std::vector<std::vector<std::vector<std::vector<int> > > > Cell_Sector;
    std::vector<float> Candidate_Angle;
    std::vector<int> Candidate_Speed;

    double dist_eps;
    double ang_eps;

    float *Last_Binary_Hist;

    // Minimum turning radius at different speeds, in millimeters
    std::vector<int> Min_Turning_Radius;

    // Keep track of last update, so we can monitor acceleration
    timeval last_update_time;

    int last_chosen_speed;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//vfh_algorithm.cc
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
VFH_Algorithm::VFH_Algorithm( double cell_size,
                              int window_diameter,
                              int sector_angle,
                              double safety_dist_0ms,
                              double safety_dist_1ms,
                              int max_speed,
                              int max_speed_narrow_opening,
                              int max_speed_wide_opening,
                              int max_acceleration,
                              int min_turnrate,
                              int max_turnrate_0ms,
                              int max_turnrate_1ms,
                              double min_turn_radius_safety_factor,
                              double free_space_cutoff_0ms,
                              double obs_cutoff_0ms,
                              double free_space_cutoff_1ms,
                              double obs_cutoff_1ms,
                              double weight_desired_dir,
                              double weight_current_dir )
    : CELL_WIDTH(cell_size),
      WINDOW_DIAMETER(window_diameter),
      SECTOR_ANGLE(sector_angle),
      SAFETY_DIST_0MS(safety_dist_0ms),
      SAFETY_DIST_1MS(safety_dist_1ms),
      Current_Max_Speed(max_speed),
      MAX_SPEED(max_speed),
      MAX_SPEED_NARROW_OPENING(max_speed_narrow_opening),
      MAX_SPEED_WIDE_OPENING(max_speed_wide_opening),
      MAX_ACCELERATION(max_acceleration),
      MIN_TURNRATE(min_turnrate),
      MAX_TURNRATE_0MS(max_turnrate_0ms),
      MAX_TURNRATE_1MS(max_turnrate_1ms),
      MIN_TURN_RADIUS_SAFETY_FACTOR(min_turn_radius_safety_factor),
      Binary_Hist_Low_0ms(free_space_cutoff_0ms),
      Binary_Hist_High_0ms(obs_cutoff_0ms),
      Binary_Hist_Low_1ms(free_space_cutoff_1ms),
      Binary_Hist_High_1ms(obs_cutoff_1ms),
      U1(weight_desired_dir),
      U2(weight_current_dir),
      Desired_Angle(90),
      Picked_Angle(90),
      Last_Picked_Angle(Picked_Angle),
      last_chosen_speed(0)
{
    this->Last_Binary_Hist = NULL;
    this->Hist = NULL;
    if ( SAFETY_DIST_0MS == SAFETY_DIST_1MS )
    {
        // For the simple case of a fixed safety_dist, keep things simple.
        NUM_CELL_SECTOR_TABLES = 1;
    }
    else
    {
        // AB: Made this number up...
        NUM_CELL_SECTOR_TABLES = 20;
    }
}

/**
 * Class destructor
 */
VFH_Algorithm::~VFH_Algorithm()
{
    if(this->Hist)
        delete[] Hist;
    if(this->Last_Binary_Hist)
        delete[] Last_Binary_Hist;
}

/**
 * Get the max turn rate at the given speed
 * @param speed current speed
 * @return max turn rate
 */
int VFH_Algorithm::GetMaxTurnrate( int speed )
{
    int val = ( MAX_TURNRATE_0MS - (int)(speed*( MAX_TURNRATE_0MS-MAX_TURNRATE_1MS )/1000.0) );

    if ( val < 0 )
        val = 0;

    return val;
}

/**
 * Set the current max speed
 * @param max_speed current max speed
 */
void VFH_Algorithm::SetCurrentMaxSpeed( int max_speed )
{
    this->Current_Max_Speed = MIN( max_speed, this->MAX_SPEED );
    this->Min_Turning_Radius.resize( Current_Max_Speed+1 );

    // small chunks of forward movements and turns-in-place used to
    // estimate turning radius, coz I'm too lazy to screw around with limits -> 0.
    double dx, dtheta;

    //
    // Calculate the turning radius, indexed by speed.
    // Probably don't need it to be precise (changing in 1mm increments).
    //
    // WARNING: This assumes that the max_turnrate that has been set for VFH is
    //          accurate.
    //
    for(int x=0;x<=Current_Max_Speed;x++)
    {
        dx = (double) x / 1e6; // dx in m/millisec
        dtheta = ((M_PI/180)*(double)(GetMaxTurnrate(x))) / 1000.0; // dTheta in radians/millisec
        Min_Turning_Radius[x] = (int) ( ((dx / tan( dtheta ))*1000.0) * MIN_TURN_RADIUS_SAFETY_FACTOR ); // in mm
    }
}


// Doesn't need optimization: only gets called once per update.
/**
 * Get the speed index ( for the current local map )
 * @param speed given speed
 * @return the index speed
 */
int
VFH_Algorithm::Get_Speed_Index( int speed )
{
    int val = (int) floor(((float)speed/(float)Current_Max_Speed)*NUM_CELL_SECTOR_TABLES);

    if ( val >= NUM_CELL_SECTOR_TABLES )
        val = NUM_CELL_SECTOR_TABLES-1;

//     printf("Speed_Index at %dmm/s: %d\n",speed,val);

    return val;
}

// Doesn't need optimization: only gets called on init plus once per update.
/**
 * Get the safety distance at the given speed
 * @param speed given speed
 * @return the safety distance
 */
int
VFH_Algorithm::Get_Safety_Dist( int speed )
{
    int val = (int) ( SAFETY_DIST_0MS + (int)(speed*( SAFETY_DIST_1MS-SAFETY_DIST_0MS )/1000.0) );

    if ( val < 0 )
        val = 0;

//    printf("Safety_Dist at %dmm/s: %d\n",speed,val);

    return val;
}

// AB: Could optimize this with a look-up table, but it shouldn't make much
//     difference: only gets called once per sector per update.
/**
 * Get the current low binary histogram threshold
 * @param speed given speed
 * @return the threshold
 */
float
VFH_Algorithm::Get_Binary_Hist_Low( int speed )
{
    return ( Binary_Hist_Low_0ms - (speed*( Binary_Hist_Low_0ms-Binary_Hist_Low_1ms )/1000.0) );
}

// AB: Could optimize this with a look-up table, but it shouldn't make much
//     difference: only gets called once per sector per update.
/**
 * Get the current high binary histogram threshold
 * @param speed given speed
 * @return the threshold
 */
float
VFH_Algorithm::Get_Binary_Hist_High( int speed )
{
    return ( Binary_Hist_High_0ms - (speed*( Binary_Hist_High_0ms-Binary_Hist_High_1ms )/1000.0) );
}

/**
 * Start up the VFH+ algorithm
 * @return 1
 */
int VFH_Algorithm::Init()
{
  int x, y, i;
  float plus_dir=0, neg_dir=0, plus_sector=0, neg_sector=0;
  bool plus_dir_bw, neg_dir_bw, dir_around_sector;
  float neg_sector_to_neg_dir=0, neg_sector_to_plus_dir=0;
  float plus_sector_to_neg_dir=0, plus_sector_to_plus_dir=0;
  int cell_sector_tablenum, max_speed_this_table;
  float r;

  CENTER_X = (int)floor(WINDOW_DIAMETER / 2.0);
  CENTER_Y = CENTER_X;
  HIST_SIZE = (int)rint(360.0 / SECTOR_ANGLE);

  // it works now; let's leave the verbose debug statement out
  /*
  printf("CELL_WIDTH: %1.1f\tWINDOW_DIAMETER: %d\tSECTOR_ANGLE: %d\tROBOT_RADIUS: %1.1f\tSAFETY_DIST: %1.1f\tMAX_SPEED: %d\tMAX_TURNRATE: %d\tFree Space Cutoff: %1.1f\tObs Cutoff: %1.1f\tWeight Desired Dir: %1.1f\tWeight Current_Dir:%1.1f\n", CELL_WIDTH, WINDOW_DIAMETER, SECTOR_ANGLE, ROBOT_RADIUS, SAFETY_DIST, MAX_SPEED, MAX_TURNRATE, Binary_Hist_Low, Binary_Hist_High, U1, U2);
  */

  VFH_Allocate();

  for(x=0;x<HIST_SIZE;x++) {
    Hist[x] = 0;
    Last_Binary_Hist[x] = 1;
  }

  // For the following calcs:
  //   - (x,y) = (0,0)   is to the front-left of the robot
  //   - (x,y) = (max,0) is to the front-right of the robot
  //
  for(x=0;x<WINDOW_DIAMETER;x++) {
    for(y=0;y<WINDOW_DIAMETER;y++) {
      Cell_Mag[x][y] = 0;
      Cell_Dist[x][y] = sqrt(pow((CENTER_X - x), 2) + pow((CENTER_Y - y), 2)) * CELL_WIDTH;

      Cell_Base_Mag[x][y] = pow((3000.0 - Cell_Dist[x][y]), 4) / 100000000.0;

      // Set up Cell_Direction with the angle in degrees to each cell
      if (x < CENTER_X) {
        if (y < CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(CENTER_Y - y) / (float)(CENTER_X - x));
          Cell_Direction[x][y] *= (360.0 / 6.28);
          Cell_Direction[x][y] = 180.0 - Cell_Direction[x][y];
        } else if (y == CENTER_Y) {
          Cell_Direction[x][y] = 180.0;
        } else if (y > CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(y - CENTER_Y) / (float)(CENTER_X - x));
          Cell_Direction[x][y] *= (360.0 / 6.28);
          Cell_Direction[x][y] = 180.0 + Cell_Direction[x][y];
        }
      } else if (x == CENTER_X) {
        if (y < CENTER_Y) {
          Cell_Direction[x][y] = 90.0;
        } else if (y == CENTER_Y) {
          Cell_Direction[x][y] = -1.0;
        } else if (y > CENTER_Y) {
          Cell_Direction[x][y] = 270.0;
        }
      } else if (x > CENTER_X) {
        if (y < CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(CENTER_Y - y) / (float)(x - CENTER_X));
          Cell_Direction[x][y] *= (360.0 / 6.28);
        } else if (y == CENTER_Y) {
          Cell_Direction[x][y] = 0.0;
        } else if (y > CENTER_Y) {
          Cell_Direction[x][y] = atan((float)(y - CENTER_Y) / (float)(x - CENTER_X));
          Cell_Direction[x][y] *= (360.0 / 6.28);
          Cell_Direction[x][y] = 360.0 - Cell_Direction[x][y];
        }
      }

      // For the case where we have a speed-dependent safety_dist, calculate all tables
      for ( cell_sector_tablenum = 0;
            cell_sector_tablenum < NUM_CELL_SECTOR_TABLES;
            cell_sector_tablenum++ )
      {
        max_speed_this_table = (int) (((float)(cell_sector_tablenum+1)/(float)NUM_CELL_SECTOR_TABLES) *
                                      (float) MAX_SPEED);

        // printf("cell_sector_tablenum: %d, max_speed: %d, safety_dist: %d\n",
        // cell_sector_tablenum,max_speed_this_table,Get_Safety_Dist(max_speed_this_table));

        // Set Cell_Enlarge to the _angle_ by which a an obstacle must be
        // enlarged for this cell, at this speed
        if (Cell_Dist[x][y] > 0)
        {
          r = ROBOT_RADIUS + Get_Safety_Dist(max_speed_this_table);
          // Cell_Enlarge[x][y] = (float)atan( r / Cell_Dist[x][y] ) * (180/M_PI);
          Cell_Enlarge[x][y] = (float)asin( r / Cell_Dist[x][y] ) * (180/M_PI);
        }
        else
        {
          Cell_Enlarge[x][y] = 0;
        }

        Cell_Sector[cell_sector_tablenum][x][y].clear();
        plus_dir = Cell_Direction[x][y] + Cell_Enlarge[x][y];
        neg_dir  = Cell_Direction[x][y] - Cell_Enlarge[x][y];

        for(i=0;i<(360 / SECTOR_ANGLE);i++)
        {
            // Set plus_sector and neg_sector to the angles to the two adjacent sectors
            plus_sector = (i + 1) * (float)SECTOR_ANGLE;
            neg_sector = i * (float)SECTOR_ANGLE;

            if ((neg_sector - neg_dir) > 180) {
                neg_sector_to_neg_dir = neg_dir - (neg_sector - 360);
            } else {
                if ((neg_dir - neg_sector) > 180) {
                    neg_sector_to_neg_dir = neg_sector - (neg_dir + 360);
                } else {
                    neg_sector_to_neg_dir = neg_dir - neg_sector;
                }
            }

            if ((plus_sector - neg_dir) > 180) {
                plus_sector_to_neg_dir = neg_dir - (plus_sector - 360);
            } else {
                if ((neg_dir - plus_sector) > 180) {
                    plus_sector_to_neg_dir = plus_sector - (neg_dir + 360);
                } else {
                    plus_sector_to_neg_dir = neg_dir - plus_sector;
                }
            }

            if ((plus_sector - plus_dir) > 180) {
                plus_sector_to_plus_dir = plus_dir - (plus_sector - 360);
            } else {
                if ((plus_dir - plus_sector) > 180) {
                    plus_sector_to_plus_dir = plus_sector - (plus_dir + 360);
                } else {
                    plus_sector_to_plus_dir = plus_dir - plus_sector;
                }
            }

            if ((neg_sector - plus_dir) > 180) {
                neg_sector_to_plus_dir = plus_dir - (neg_sector - 360);
            } else {
                if ((plus_dir - neg_sector) > 180) {
                    neg_sector_to_plus_dir = neg_sector - (plus_dir + 360);
                } else {
                    neg_sector_to_plus_dir = plus_dir - neg_sector;
                }
            }

            plus_dir_bw = 0;
            neg_dir_bw = 0;
            dir_around_sector = 0;

            if ((neg_sector_to_neg_dir >= 0) && (plus_sector_to_neg_dir <= 0)) {
                neg_dir_bw = 1;
            }

            if ((neg_sector_to_plus_dir >= 0) && (plus_sector_to_plus_dir <= 0)) {
                plus_dir_bw = 1;
            }

            if ((neg_sector_to_neg_dir <= 0) && (neg_sector_to_plus_dir >= 0)) {
                dir_around_sector = 1;
            }

            if ((plus_sector_to_neg_dir <= 0) && (plus_sector_to_plus_dir >= 0)) {
                plus_dir_bw = 1;
            }

            if ((plus_dir_bw) || (neg_dir_bw) || (dir_around_sector)) {
                Cell_Sector[cell_sector_tablenum][x][y].push_back(i);
            }
        }
      }
    }
  }

//  assert( GlobalTime->GetTime( &last_update_time ) == 0 );

  gettimeofday(&last_update_time,0);

  return(1);
}

/**
 * Allocate the VFH+ memory
 */
int VFH_Algorithm::VFH_Allocate()
{
  std::vector<float> temp_vec;
  std::vector<int> temp_vec3;
  std::vector<std::vector<int> > temp_vec2;
  std::vector<std::vector<std::vector<int> > > temp_vec4;
  int x;

  Cell_Direction.clear();
  Cell_Base_Mag.clear();
  Cell_Mag.clear();
  Cell_Dist.clear();
  Cell_Enlarge.clear();
  Cell_Sector.clear();

  temp_vec.clear();
  for(x=0;x<WINDOW_DIAMETER;x++) {
    temp_vec.push_back(0);
  }

  temp_vec2.clear();
  temp_vec3.clear();
  for(x=0;x<WINDOW_DIAMETER;x++) {
    temp_vec2.push_back(temp_vec3);
  }

  for(x=0;x<WINDOW_DIAMETER;x++) {
    Cell_Direction.push_back(temp_vec);
    Cell_Base_Mag.push_back(temp_vec);
    Cell_Mag.push_back(temp_vec);
    Cell_Dist.push_back(temp_vec);
    Cell_Enlarge.push_back(temp_vec);
    temp_vec4.push_back(temp_vec2);
  }

  for(x=0;x<NUM_CELL_SECTOR_TABLES;x++)
  {
    Cell_Sector.push_back(temp_vec4);
  }

  Hist = new float[HIST_SIZE];
  Last_Binary_Hist = new float[HIST_SIZE];
  this->SetCurrentMaxSpeed( MAX_SPEED );

  return(1);
}

/**
 * Update the VFH+ state using the laser readings and the robot speed
 * @param laser_ranges the laser (or sonar) readings
 * @param current_speed the current robot speed
 * @param goal_direction the desired direction
 * @param goal_distance the desired distance
 * @param goal_distance_tolerance the distance tolerance from the goal
 * @param chosen_speed the chosen speed to drive the robot
 * @param chosen_turnrate the chosen turn rathe to drive the robot
 * @return 1
 */
int VFH_Algorithm::Update_VFH( double laser_ranges[361][2],
                               int current_speed,
                               float goal_direction,
                               float goal_distance,
                               float goal_distance_tolerance,
                               int &chosen_speed,
                               int &chosen_turnrate )
{
  int print = 0;

  this->Desired_Angle = goal_direction;
  this->Dist_To_Goal  = goal_distance;
  this->Goal_Distance_Tolerance = goal_distance_tolerance;

  //
  // Set current_pos_speed to the maximum of
  // the set point (last_chosen_speed) and the current actual speed.
  // This ensures conservative behaviour if the set point somehow ramps up beyond
  // the actual speed.
  // Ensure that this speed is positive.
  //
  int current_pos_speed;
  if ( current_speed < 0 )
  {
      current_pos_speed = 0;
  }
  else
  {
      current_pos_speed = current_speed;
  }


  if ( current_pos_speed < last_chosen_speed )
  {
      current_pos_speed = last_chosen_speed;
  }
//  printf("Update_VFH: current_pos_speed = %d\n",current_pos_speed);


  // Work out how much time has elapsed since the last update,
  // so we know how much to increase speed by, given MAX_ACCELERATION.
  timeval now;
  timeval diff;
  double  diffSeconds;
//  assert( GlobalTime->GetTime( &now ) == 0 );
  assert( gettimeofday(&now, 0) == 0 );

  TIMESUB( &now, &last_update_time, &diff );
  diffSeconds = diff.tv_sec + ( (double)diff.tv_usec / 1000000 );

  last_update_time.tv_sec = now.tv_sec;
  last_update_time.tv_usec = now.tv_usec;
//  printf("Update_VFH: Build_Primary_Polar_Histogram\n");
  if ( Build_Primary_Polar_Histogram(laser_ranges,current_pos_speed) == 0)
  {
      // Something's inside our safety distance: brake hard and
      // turn on the spot
      Picked_Angle = Last_Picked_Angle;
      Max_Speed_For_Picked_Angle = 0;
      Last_Picked_Angle = Picked_Angle;
  }
  else
  {
    print = 0;
    if (print) {
      printf("Primary Histogram\n");
      Print_Hist();
    }

    Build_Binary_Polar_Histogram(current_pos_speed);
    if (print) {
      printf("Binary Histogram\n");
      Print_Hist();
    }

    Build_Masked_Polar_Histogram(current_pos_speed);
    if (print) {
      printf("Masked Histogram\n");
      Print_Hist();
    }

    // Sets Picked_Angle, Last_Picked_Angle, and Max_Speed_For_Picked_Angle.
    Select_Direction();

  }

//  printf("Picked Angle: %f\n", Picked_Angle);

  //
  // OK, so now we've chosen a direction.  Time to choose a speed.
  //

  // How much can we change our speed by?
  int speed_incr;
  if ( (diffSeconds > 0.3) || (diffSeconds < 0) )
  {
      // Either this is the first time we've been updated, or something's a bit screwy and
      // update hasn't been called for a while.  Don't want a sudden burst of acceleration,
      // so better to just pick a small value this time, calculate properly next time.
      speed_incr = 10;
  }
  else
  {
      speed_incr = (int) (MAX_ACCELERATION * diffSeconds);
  }

  if ( Cant_Turn_To_Goal() )
  {
//      printf("The goal's too close -- we can't turn tightly enough to get to it, so slow down...");
      speed_incr = -speed_incr;
  }

  // Accelerate (if we're not already at Max_Speed_For_Picked_Angle).
  chosen_speed = MIN( last_chosen_speed + speed_incr, Max_Speed_For_Picked_Angle );

//  printf("Max Speed for picked angle: %d\n",Max_Speed_For_Picked_Angle);

  // Set the chosen_turnrate, and possibly modify the chosen_speed
  Set_Motion( chosen_speed, chosen_turnrate, current_pos_speed );

  last_chosen_speed = chosen_speed;

  if (print)
    printf("CHOSEN: SPEED: %d\t TURNRATE: %d\n", chosen_speed, chosen_turnrate);

  return(1);
}


/**
 * The robot going too fast, such does it overshoot before it can turn to the goal?
 * @return true if the robot cannot turn to the goal
 */
bool VFH_Algorithm::Cant_Turn_To_Goal()
{
    // Calculate this by seeing if the goal is inside the blocked circles
    // (circles we can't enter because we're going too fast).  Radii set
    // by Build_Masked_Polar_Histogram.

    // Coords of goal in local coord system:
    float goal_x = this->Dist_To_Goal * cos( DTOR(this->Desired_Angle) );
    float goal_y = this->Dist_To_Goal * sin( DTOR(this->Desired_Angle) );

// AlexB: Is this useful?
//     if ( goal_y < 0 )
//     {
//         printf("Goal behind\n");
//         return true;
//     }

    // This is the distance between the centre of the goal and
    // the centre of the blocked circle
    float dist_between_centres;

//     printf("Cant_Turn_To_Goal: Dist_To_Goal = %f\n",Dist_To_Goal);
//     printf("Cant_Turn_To_Goal: Angle_To_Goal = %f\n",Desired_Angle);
//     printf("Cant_Turn_To_Goal: Blocked_Circle_Radius = %f\n",Blocked_Circle_Radius);

    // right circle
    dist_between_centres = hypot( goal_x - this->Blocked_Circle_Radius, goal_y );
    if ( dist_between_centres+this->Goal_Distance_Tolerance < this->Blocked_Circle_Radius )
    {
//        printf("Goal close & right\n");
        return true;
    }

    // left circle
    dist_between_centres = hypot( -goal_x - this->Blocked_Circle_Radius, goal_y );
    if ( dist_between_centres+this->Goal_Distance_Tolerance < this->Blocked_Circle_Radius )
    {
//        printf("Goal close & left.\n");
        return true;
    }

    return false;
}

/**
 * Difference between two integer angle
 * @param a1 first angle
 * @param a2 second angle
 * @return the difference
 */
float VFH_Algorithm::Delta_Angle(int a1, int a2)
{
  return(Delta_Angle((float)a1, (float)a2));
}

/**
 * Difference between two float angle
 * @param a1 first angle
 * @param a2 second angle
 * @return the difference
 */
float VFH_Algorithm::Delta_Angle(float a1, float a2)
{
  float diff;

  diff = a2 - a1;

  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }

  return(diff);
}

/**
 * Calculate the bisector between two angle
 * @param angle1 first angle
 * @param angle2 second angle
 * @return the bisector angle
 */
int VFH_Algorithm::Bisect_Angle(int angle1, int angle2)
{
  float a;
  int angle;

  a = Delta_Angle((float)angle1, (float)angle2);

  angle = (int)rint(angle1 + (a / 2.0));
  if (angle < 0) {
    angle += 360;
  } else if (angle >= 360) {
    angle -= 360;
  }

  return(angle);
}

/**
 * Select the candidate angle to decide the direction using the given weights
 * @return 1
 */
int VFH_Algorithm::Select_Candidate_Angle()
{
  unsigned int i;
  float weight, min_weight;

  if (Candidate_Angle.size() == 0)
  {
      // We're hemmed in by obstacles -- nowhere to go,
      // so brake hard and turn on the spot.
      Picked_Angle = Last_Picked_Angle;
      Max_Speed_For_Picked_Angle = 0;
      Last_Picked_Angle = Picked_Angle;
      return(1);
  }

  Picked_Angle = 90;
  min_weight = 10000000;

  for(i=0;i<Candidate_Angle.size();i++)
  {
      //printf("CANDIDATE: %f\n", Candidate_Angle[i]);
      weight = U1 * fabs(Delta_Angle(Desired_Angle, Candidate_Angle[i])) +
          U2 * fabs(Delta_Angle(Last_Picked_Angle, Candidate_Angle[i]));
      if (weight < min_weight)
      {
          min_weight = weight;
          Picked_Angle = Candidate_Angle[i];
          Max_Speed_For_Picked_Angle = Candidate_Speed[i];
      }
  }

  Last_Picked_Angle = Picked_Angle;

  return(1);
}

/**
 * Select the used direction
 * @return 1
 */
int VFH_Algorithm::Select_Direction()
{
  int start, i, left;
  float angle, new_angle;
  std::vector<std::pair<int,int> > border;
  std::pair<int,int> new_border;

  Candidate_Angle.clear();
  Candidate_Speed.clear();

  //
  // set start to sector of first obstacle
  //
  start = -1;

  // only look at the forward 180deg for first obstacle.
  for(i=0;i<HIST_SIZE/2;i++)
  {
      if (Hist[i] == 1)
      {
          start = i;
          break;
      }
  }

  if (start == -1)
  {
      Picked_Angle = Desired_Angle;
      Last_Picked_Angle = Picked_Angle;
      Max_Speed_For_Picked_Angle = Current_Max_Speed;

//      printf("No obstacles detected in front of us: full speed towards goal: %f, %f, %d\n",
//    		  Picked_Angle, Last_Picked_Angle, Max_Speed_For_Picked_Angle);

      return(1);
  }

  //
  // Find the left and right borders of each opening
  //

  border.clear();

  //printf("Start: %d\n", start);
  left = 1;
  for(i=start;i<=(start+HIST_SIZE);i++) {
    if ((Hist[i % HIST_SIZE] == 0) && (left)) {
      new_border.first = (i % HIST_SIZE) * SECTOR_ANGLE;
      left = 0;
    }

    if ((Hist[i % HIST_SIZE] == 1) && (!left)) {
      new_border.second = ((i % HIST_SIZE) - 1) * SECTOR_ANGLE;
      if (new_border.second < 0) {
        new_border.second += 360;
      }
      border.push_back(new_border);
      left = 1;
    }
  }

  //
  // Consider each opening
  //
  for(i=0;i<(int)border.size();i++)
  {
//    printf("BORDER: %d %d\n", border[i].first, border[i].second);
    angle = Delta_Angle(border[i].first, border[i].second);

    if (fabs(angle) < 10)
    {
        // ignore very narrow openings
        continue;
    }

    if (fabs(angle) < 80)
    {
        // narrow opening: aim for the centre

        new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;

        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_NARROW_OPENING));
    }
    else
    {
        // wide opening: consider the centre, and 40deg from each border

        new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;

        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(Current_Max_Speed);

        new_angle = (float)((border[i].first + 40) % 360);
        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));

        new_angle = (float)(border[i].second - 40);
        if (new_angle < 0)
            new_angle += 360;
        Candidate_Angle.push_back(new_angle);
        Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));

        // See if candidate dir is in this opening
        if ((Delta_Angle(Desired_Angle, Candidate_Angle[Candidate_Angle.size()-2]) < 0) &&
            (Delta_Angle(Desired_Angle, Candidate_Angle[Candidate_Angle.size()-1]) > 0)) {
            Candidate_Angle.push_back(Desired_Angle);
            Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
        }
    }
  }

  Select_Candidate_Angle();

  return(1);
}

/**
 * Print the cells directions
 */
void VFH_Algorithm::Print_Cells_Dir()
{
  int x, y;

  printf("\nCell Directions:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Direction[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the cells magnitude
 */
void VFH_Algorithm::Print_Cells_Mag()
{
  int x, y;

  printf("\nCell Magnitudes:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Mag[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the cells distances
 */
void VFH_Algorithm::Print_Cells_Dist()
{
  int x, y;

  printf("\nCell Distances:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Dist[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the cells sectors
 */
void VFH_Algorithm::Print_Cells_Sector()
{
  int x, y;
  unsigned int i;

  printf("\nCell Sectors for table 0:\n");
  printf("***************************\n");

  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      for(i=0;i<Cell_Sector[0][x][y].size();i++) {
        if (i < (Cell_Sector[0][x][y].size() -1 )) {
          printf("%d,", Cell_Sector[0][x][y][i]);
        } else {
          printf("%d\t\t", Cell_Sector[0][x][y][i]);
        }
      }
    }
    printf("\n");
  }
}

/**
 * Print the cells enlargement angles
 */
void VFH_Algorithm::Print_Cells_Enlargement_Angle()
{
  int x, y;

  printf("\nEnlargement Angles:\n");
  printf("****************\n");
  for(y=0;y<WINDOW_DIAMETER;y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      printf("%1.1f\t", Cell_Enlarge[x][y]);
    }
    printf("\n");
  }
}

/**
 * Print the histogram
 */
void VFH_Algorithm::Print_Hist()
{
  int x;
  printf("Histogram:\n");
  printf("****************\n");

  for(x=0;x<=(HIST_SIZE/2);x++) {
    printf("%d,%1.1f\n", (x * SECTOR_ANGLE), Hist[x]);
  }
  printf("\n\n");
}

/**
 * Calcualte the cells magnitude
 * @param laser_ranges laser (or sonar) readings
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Calculate_Cells_Mag( double laser_ranges[361][2], int speed )
{
  int x, y;

  float safeSpeed = (float) Get_Safety_Dist(speed);
  float r = ROBOT_RADIUS +  safeSpeed;

//printf("Laser Ranges\n");
//printf("************\n");
//for(x=0;x<=360;x++) {
//printf("%d: %f %f\n", x, laser_ranges[x][0], r);
//}


  // AB: This is a bit dodgy...  Makes it possible to miss really skinny obstacles, since if the
  //     resolution of the cells is finer than the resolution of laser_ranges, some ranges might be missed.
  //     Rather than looping over the cells, should perhaps loop over the laser_ranges.

  // Only deal with the cells in front of the robot, since we can't sense behind.
  for(x=0;x<WINDOW_DIAMETER;x++)
  {
      for(y=0;y<(int)ceil(WINDOW_DIAMETER/2.0);y++)
      {
//    	  printf("Cell %d,%d: Cell_Dist is %f, range is %f i: %d (minimum is %f)\n",
//    	                      x,
//    	                      y,
//    	                      Cell_Dist[x][y] + CELL_WIDTH / 2.0,
//    	                      laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0],
//    	                      (int)rint(Cell_Direction[x][y] * 2.0),
//    	                      r);
        // controllo se il laser passa attraverso la cella
          if ((Cell_Dist[x][y] + CELL_WIDTH / 2.0) >
              laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0])
          {
              if ( Cell_Dist[x][y] < r && !(x==CENTER_X && y==CENTER_Y) )
              {
//                   printf("Cell %d,%d: Cell_Dist is %f, range is %f (minimum is %f): too close...\n",
//                          x,
//                          y,
//                          Cell_Dist[x][y] + CELL_WIDTH / 2.0,
//                          laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0],
//                          r);

//                   printf("ROBOT_RADIUS %f, Get_Safety_Dist(speed) %f\n", ROBOT_RADIUS, safeSpeed);

                  // Damn, something got inside our safety_distance...
                  // Short-circuit this process.
                  return(0);
              }
              else
              {
                // cella piena quindi:
                // assegno alla cella il peso che dipende dalla distanza
                  Cell_Mag[x][y] = Cell_Base_Mag[x][y];
              }
          } else {
            // è vuota perchè il laser ci passa oltre!!!!
              Cell_Mag[x][y] = 0.0;
          }
      }
  }

  return(1);
}

/**
 * Build the primary polar histogram
 * @param laser_ranges laser (or sonar) readings
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Build_Primary_Polar_Histogram( double laser_ranges[361][2], int speed )
{
  int x, y;
  unsigned int i;
  // index into the vector of Cell_Sector tables
  int speed_index = Get_Speed_Index( speed );

//  printf("Build_Primary_Polar_Histogram: speed_index %d %d\n", speed_index, HIST_SIZE);

  for(x=0;x<HIST_SIZE;x++) {
    Hist[x] = 0;
  }

  if ( Calculate_Cells_Mag( laser_ranges, speed ) == 0 )
  {
      // set Hist to all blocked
      for(x=0;x<HIST_SIZE;x++) {
          Hist[x] = 1;
      }
      return 0;
  }

//  Print_Cells_Dist();
//  Print_Cells_Dir();
//  Print_Cells_Mag();
//  Print_Cells_Sector();
//  Print_Cells_Enlargement_Angle();

  // Only have to go through the cells in front.
  for(y=0;y<=(int)ceil(WINDOW_DIAMETER/2.0);y++) {
    for(x=0;x<WINDOW_DIAMETER;x++) {
      for(i=0;i<Cell_Sector[speed_index][x][y].size();i++) {
        Hist[Cell_Sector[speed_index][x][y][i]] += Cell_Mag[x][y];
      }
    }
  }

  return(1);
}

/**
 * Build the binary polar histogram
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Build_Binary_Polar_Histogram( int speed )
{
  int x;

  for(x=0;x<HIST_SIZE;x++) {
      if (Hist[x] > Get_Binary_Hist_High(speed)) {
      Hist[x] = 1.0;
      } else if (Hist[x] < Get_Binary_Hist_Low(speed)) {
      Hist[x] = 0.0;
    } else {
      Hist[x] = Last_Binary_Hist[x];
    }
  }

  for(x=0;x<HIST_SIZE;x++) {
    Last_Binary_Hist[x] = Hist[x];
  }

  return(1);
}

//
// This function also sets Blocked_Circle_Radius.
//
/**
 * Build the masked polar histogram
 * @param speed robot speed
 * @return 1
 */
int VFH_Algorithm::Build_Masked_Polar_Histogram(int speed)
{
  int x, y;
  float center_x_right, center_x_left, center_y, dist_r, dist_l;
  float angle_ahead, phi_left, phi_right, angle;

  // center_x_[left|right] is the centre of the circles on either side that
  // are blocked due to the robot's dynamics.  Units are in cells, in the robot's
  // local coordinate system (+y is forward).
  center_x_right = CENTER_X + (Min_Turning_Radius[speed] / (float)CELL_WIDTH);
  center_x_left = CENTER_X - (Min_Turning_Radius[speed] / (float)CELL_WIDTH);
  center_y = CENTER_Y;

  angle_ahead = 90;
  phi_left  = 180;
  phi_right = 0;

  Blocked_Circle_Radius = Min_Turning_Radius[speed] + ROBOT_RADIUS + Get_Safety_Dist(speed);

  //
  // This loop fixes phi_left and phi_right so that they go through the inside-most
  // occupied cells inside the left/right circles.  These circles are centred at the
  // left/right centres of rotation, and are of radius Blocked_Circle_Radius.
  //
  // We have to go between phi_left and phi_right, due to our minimum turning radius.
  //

  //
  // Only loop through the cells in front of us.
  //
  for(y=0;y<(int)ceil(WINDOW_DIAMETER/2.0);y++)
  {
    for(x=0;x<WINDOW_DIAMETER;x++)
    {
        if (Cell_Mag[x][y] == 0)
            continue;

        if ((Delta_Angle(Cell_Direction[x][y], angle_ahead) > 0) &&
            (Delta_Angle(Cell_Direction[x][y], phi_right) <= 0))
        {
            // The cell is between phi_right and angle_ahead

            dist_r = hypot(center_x_right - x, center_y - y) * CELL_WIDTH;
            if (dist_r < Blocked_Circle_Radius)
            {
                phi_right = Cell_Direction[x][y];
            }
        }
        else if ((Delta_Angle(Cell_Direction[x][y], angle_ahead) <= 0) &&
                 (Delta_Angle(Cell_Direction[x][y], phi_left) > 0))
        {
            // The cell is between phi_left and angle_ahead

            dist_l = hypot(center_x_left - x, center_y - y) * CELL_WIDTH;
            if (dist_l < Blocked_Circle_Radius)
            {
                phi_left = Cell_Direction[x][y];
            }
        }
    }
  }

  //
  // Mask out everything outside phi_left and phi_right
  //
  for(x=0;x<HIST_SIZE;x++)
  {
      angle = x * SECTOR_ANGLE;
      if ((Hist[x] == 0) && (((Delta_Angle((float)angle, phi_right) <= 0) &&
                              (Delta_Angle((float)angle, angle_ahead) >= 0)) ||
                             ((Delta_Angle((float)angle, phi_left) >= 0) &&
                              (Delta_Angle((float)angle, angle_ahead) <= 0))))
      {
          Hist[x] = 0;
      }
      else
      {
          Hist[x] = 1;
      }
  }

  return(1);
}

/**
 * Set the motion commands
 * @param speed the desire speed
 * @param turnrate the desire turn rate
 * @param actual_speed the current speed
 * @return 1
 */
int VFH_Algorithm::Set_Motion( int &speed, int &turnrate, int actual_speed )
{
  // This happens if all directions blocked, so just spin in place
  if (speed <= 0)
  {
    //printf("stop\n");
      turnrate = GetMaxTurnrate( actual_speed );
      speed = 0;
  }
  else
  {
//    printf("Picked %f\n", Picked_Angle);
    if ((Picked_Angle > 270) && (Picked_Angle < 360)) {
        turnrate = -1 * GetMaxTurnrate( actual_speed );
    } else if ((Picked_Angle < 270) && (Picked_Angle > 180)) {
      turnrate = GetMaxTurnrate( actual_speed );
    } else {
      turnrate = (int)rint(((float)(Picked_Angle - 90) / 75.0) * GetMaxTurnrate( actual_speed ));
//    	turnrate = (int)rint(((float)(Picked_Angle) / 75.0) * GetMaxTurnrate( actual_speed ));
//      printf("GetMaxTurnrate( actual_speed ): %d, Picked_Angle: %f, turnrate %d\n",
//    		  GetMaxTurnrate( actual_speed ),
//    		  Picked_Angle,
//    		  turnrate);

      if (turnrate > GetMaxTurnrate( actual_speed )) {
        turnrate = GetMaxTurnrate( actual_speed );
      } else if (turnrate < (-1 * GetMaxTurnrate( actual_speed ))) {
        turnrate = -1 * GetMaxTurnrate( actual_speed );
      }

//      if (abs(turnrate) > (0.9 * GetMaxTurnrate( actual_speed ))) {
//        speed = 0;
//      }
    }
  }

//  speed and turnrate have been set for the calling function -- return.

  return(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//vfh_algorithm.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define DEG2RAD(a) ((a) * M_PI / 180.0)


std::string scan_topic_  = "scan_with_noise";
std::string odom_topic_  = "odometry/filtered";

class VFH_node
{
public:
  VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~VFH_node();
  void update();
private:
  VFH_Algorithm *m_vfh;

  double m_cell_size;			// 100 mm
  int m_window_diameter;		// cells
  int m_sector_angle;			// in deg
  double m_safety_dist_0ms;
  double m_safety_dist_1ms;
  int m_max_speed;
  int m_max_speed_narrow_opening;
  int m_max_speed_wide_opening;
  int m_max_acceleration;
  int m_min_turnrate;
  int m_max_turnrate_0ms;
  int m_max_turnrate_1ms;
  double m_min_turn_radius_safety_factor;
  double m_free_space_cutoff_0ms;
  double m_obs_cutoff_0ms;
  double m_free_space_cutoff_1ms;
  double m_obs_cutoff_1ms;
  double m_weight_desired_dir;
  double m_weight_current_dir;

  double m_robot_radius;
  double m_robotVel;
    double m_laser_ranges[361][2];

  int chosen_speed,chosen_turnrate;

  // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Publisher  vel_publisher_;

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//vfh_node.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

VFH_node::VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private):
nh_(nh), nh_private_(nh_private)
{
  ROS_INFO("Starting VFH");

  m_cell_size = 100;							// mm, cell dimension
  m_window_diameter = 60;						// number of cells
  m_sector_angle = 5;							// deg, sector angle

  if (!nh_private_.getParam ("m_safety_dist_0ms", m_safety_dist_0ms))
    m_safety_dist_0ms = 100; 				// mm, double, safe distance at 0 m/s

  if (!nh_private_.getParam ("m_safety_dist_1ms", m_safety_dist_1ms))
    m_safety_dist_1ms = 100; 				// mm, double, safe distance at 1 m/s

  if (!nh_private_.getParam ("m_max_speed", m_max_speed))
    m_max_speed= 1000; //200 originally, should be 1000   // mm/sec, int, max speed

  if (!nh_private_.getParam ("m_max_speed_narrow_opening", m_max_speed_narrow_opening))
    m_max_speed_narrow_opening= 200; 		// mm/sec, int, max speed in the narrow opening

  if (!nh_private_.getParam ("m_max_speed_wide_opening", m_max_speed_wide_opening))
    m_max_speed_wide_opening= 300; 			// mm/sec, int, max speed in the wide opening

  if (!nh_private_.getParam ("m_max_acceleration", m_max_acceleration))
    m_max_acceleration = 200;    			// mm/sec^2, int, max acceleration

  if (!nh_private_.getParam ("m_min_turnrate", m_min_turnrate))
    m_min_turnrate = 40;	 				// deg/sec, int, min turn rate <--- not used

  if (!nh_private_.getParam ("m_max_turnrate_0ms", m_max_turnrate_0ms))
    m_max_turnrate_0ms = 40;				// deg/sec, int, max turn rate at 0 m/s

  if (!nh_private_.getParam ("m_max_turnrate_1ms", m_max_turnrate_1ms))
    m_max_turnrate_1ms = 40;				// deg/sec, int, max turn rate at 1 m/s

  m_min_turn_radius_safety_factor = 1.0; 		// double ????

  if (!nh_private_.getParam ("m_free_space_cutoff_0ms", m_free_space_cutoff_0ms))
    m_free_space_cutoff_0ms = 2000000.0; 	//double, low threshold free space at 0 m/s

  if (!nh_private_.getParam ("m_obs_cutoff_0ms", m_obs_cutoff_0ms))
    m_obs_cutoff_0ms = 4000000.0;			//double, high threshold obstacle at 0 m/s

  if (!nh_private_.getParam ("m_free_space_cutoff_1ms", m_free_space_cutoff_1ms))
    m_free_space_cutoff_1ms = 2000000.0; 	//double, low threshold free space at 1 m/s

  if (!nh_private_.getParam ("m_obs_cutoff_1ms", m_obs_cutoff_1ms))
    m_obs_cutoff_1ms = 4000000.0;			//double, high threshold obstacle at 1 m/s

  if (!nh_private_.getParam ("m_weight_desired_dir", m_weight_desired_dir))
    m_weight_desired_dir = 5.0;				//double, weight desired direction

  if (!nh_private_.getParam ("m_weight_current_dir", m_weight_current_dir))
    m_weight_current_dir = 1.0;				//double, weight current direction

  if (!nh_private_.getParam ("m_robot_radius", m_robot_radius))
    m_robot_radius = 300.0;					// robot radius in mm

  m_vfh = new VFH_Algorithm(m_cell_size, m_window_diameter, m_sector_angle,
      m_safety_dist_0ms, m_safety_dist_1ms, m_max_speed,
      m_max_speed_narrow_opening, m_max_speed_wide_opening,
      m_max_acceleration, m_min_turnrate, m_max_turnrate_0ms,
      m_max_turnrate_1ms, m_min_turn_radius_safety_factor,
      m_free_space_cutoff_0ms, m_obs_cutoff_0ms, m_free_space_cutoff_1ms,
      m_obs_cutoff_1ms, m_weight_desired_dir, m_weight_current_dir);

  m_vfh->SetRobotRadius(m_robot_radius);
  m_vfh->Init();

  // subscribe to topics
  scan_subscriber_ = nh_.subscribe(
      scan_topic_, 1, &VFH_node::scanCallback, this);
  odom_subscriber_ = nh_.subscribe(
      odom_topic_, 1, &VFH_node::odomCallback, this);
  // cmd_vel publisher
  vel_publisher_= nh_.advertise<geometry_msgs::Twist>("cmd_vel",5);

}

VFH_node::~VFH_node()
{
  // stop the robot
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x=0.0;
  cmd_vel.angular.z=0.0;
  vel_publisher_.publish(cmd_vel);
  delete m_vfh;
}

void VFH_node::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  ROS_DEBUG("odomCallback(): received odometry");
  m_robotVel = odom_msg->twist.twist.linear.x * 1000.0;
}


void VFH_node::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  ROS_DEBUG("scanCallback(): received scan, ranges %d",scan_msg->ranges.size());

  unsigned int n = scan_msg->ranges.size();
  for (unsigned i = 0; i < 361; i++)
    m_laser_ranges[i][0] = -1;

  int step=1;
  int startIndex=0;
  float laserSpan = scan_msg->angle_max - scan_msg->angle_min;

  if(laserSpan > M_PI || n>180) // in case we are using HOKUYO
  {
    startIndex = (- M_PI/2 - scan_msg->angle_min) /scan_msg->angle_increment;
    float rays_per_degree = (M_PI/180.0)/scan_msg->angle_increment;
    ROS_DEBUG("scanCallback(): startIndex %d, raysxdeg %f", startIndex, rays_per_degree);
    for (unsigned i = 0; i<180; i++)
    {
      step = int(rays_per_degree * i);
      // calculate position in laser frame
      if (startIndex+step > n-1) // probably this is not necessary :/
        step = step-1;

      double r = scan_msg->ranges[startIndex+step]*1000.0;

      if (r<10)
        r = scan_msg->range_max *1000.0;

      ROS_DEBUG("%d:%f\n",i,r);
      m_laser_ranges[i*2][0] = r;
      m_laser_ranges[i*2 + 1][0] = r;
    }
  }
  else
  {
    for (unsigned i = 0; i<180; i++) // in case we are using SICK
    {
      // calculate position in laser frame
      double r = scan_msg->ranges[i]*1000.0;
      m_laser_ranges[i*2][0] = r;
      m_laser_ranges[i*2 + 1][0] = r;
    }
  }

  // perform vfh+
  update();
}


void VFH_node::update()
{
  float desiredAngle=0.0;
  float desiredDist=100000.0;
  float currGoalDistanceTolerance=250;

  m_vfh->Update_VFH(m_laser_ranges, (int) (m_robotVel), desiredAngle + 90.0,
      desiredDist, currGoalDistanceTolerance, chosen_speed,
      chosen_turnrate);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x=(float)(chosen_speed)/1000.0;
  cmd_vel.angular.z= DEG2RAD(chosen_turnrate);
  vel_publisher_.publish(cmd_vel);

  ROS_DEBUG("chosen_speed %d, chosen_turnrate %d", chosen_speed,
      chosen_turnrate);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "vfh_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  VFH_node vfh_node(nh,nh_private);
  ros::spin();

  return 0;
}
