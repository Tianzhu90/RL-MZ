/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Example version with a very simple driver model.                        */
/*                                                                          */
/*  Version of 2012-08-28                                   Lukas Kautzsch  */
/*==========================================================================*/
#include<cmath>
#include "DriverModel.h"
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>


#pragma data_seg("Shared")  
//float M[1000][8] = { 0.0 };
int p_id = -1;
double p_a = 0.0;
//double A[2000] = { 0.0 };
//double S[2000][8] = { 0.0 };
#pragma data_seg()  
#pragma comment(linker,"/section:Shared,rws")  


using namespace std;



	
/*==========================================================================*/
float test1 = 3;
int  timestep = 0.0;
double  current_speed        = 0.0;
double  current_length = 0;
double  desired_acceleration = 0.0;
double  desired_lane_angle   = 0.0;
long    active_lane_change   = 0;
long    rel_target_lane      = 0;
double  desired_velocity     = 0.0;
long    turning_indicator    = 0;
long    vehicle_color        = RGB(0,0,0);
double  lead_vehicle_distance         = 300.0;
double  lead_vehicle_speed_difference = 40.0;
double  lead_vehicle_length           =   0.0;


double  nlead_vehicle_distance = 300.0;
double  nlead_vehicle_speed_difference = 40.0;
double  nlead_vehicle_length = 0.0;
double  nfollow_vehicle_distance = 300.0;
double  nfollow_vehicle_speed_difference = 40.0;
double  nfollow_vehicle_length = 0.0;

long	laneNum = 0.0;
double  xPos = 0.0;
double  distance_end = 9999;
double lead_vehicle_acceleration = 0.0;
double nlead_vehicle_acceleration = 0.0;
double nfollow_vehicle_acceleration = 0.0;
double current_acc = 0.0;
int   id = 0;
long   nvehid = 0.0;
//ofstream output("E:\\Python\\DQN\\data.txt", ios::trunc);
int count2 = 0;



double seedrandom(long seed)
{
	double x = sin(seed) * 10000;
	return x - floor(x);
}





double wiedman99(double x)
{
	double cc0 = 4.92*0.3048; //Standstill Distance - m
	double cc1 = x; //Spacing Time - second 1.7 in Vissim
	double cc2 = 13.12*0.3048; //Following Variation ("max drift") - m
	double cc3 = -8.00; //Threshold for Entering 'Following' - s
	double cc4 = -0.35; //Negative 'Following' Threshold - m/s
	double cc5 = 0.35; //Positive 'Following' Threshold - m/s
	double cc6 = 11.44/10000; //Speed Dependency of Oscillation - 10^-4 rad/s
	double cc7 = 0.82*0.3048; //Oscillation Acceleration - m/s^2
	double cc8 = 11.48*0.3048; //Standstill Acceleration - m/s^2
	double cc9 = 4.92*0.3048; //Acceleration at 80km/h - m/s^2
	double v_slower;
	double dx = lead_vehicle_distance - lead_vehicle_length;
	double dv = lead_vehicle_speed_difference;
	double lead_vehicle_speed = current_speed + lead_vehicle_speed_difference;
	double sdxc;
	if (lead_vehicle_speed <= 0) {
		sdxc = cc0;
	}
	else {
		if (dv >= 0 || lead_vehicle_acceleration < -1) {
			v_slower = current_speed;
			
		}
		else {
			v_slower = lead_vehicle_speed + dv*(seedrandom(id)-0.5);
			
		}
		sdxc = cc0 + cc1*v_slower;
	}
	double sdxo = sdxc + cc2;
	double sdxv = sdxo + cc3*(dv - cc4);
	double sdv = cc6*dx*dx;  //distance driver starts perceiving speed differences when approaching slower leader
	double sdvc = (lead_vehicle_speed > 0) ? cc4 - sdv : 0; //minimum closing dv
	double sdvo = (current_speed > cc5) ? sdv + cc5 : sdv; //minimum opening dv
	double acc = 0.0;
	if ((dv < sdvo) && (dx <= sdxc)) {
		if (current_speed > 0) {
			if (dv < 0) {
				if (dx > cc0) {
					acc = min(lead_vehicle_acceleration + dv*dv / (cc0 - dx), current_acc);
				}
				else {
					acc = min(lead_vehicle_acceleration + 0.5*(dv - sdvo), current_acc);
				}
			}
			if (acc > -cc7) {
				acc = -cc7;
			}
			else {
				acc = max(acc, -10 + 0.5*sqrt(current_speed));
			}
		}
	}
	else if ((dv < sdvc) && (dx < sdxv)) {
		acc = max(0.5*dv*dv / (-dx + sdxc - 0.1), -10);
	}
	else if ((dv < sdvo) && (dx < sdxo)) {
		if (current_acc <= 0) {
			acc = min(current_acc, -cc7);
		}
		else {
			acc = max(current_acc, cc7);
			//if (follower.length >= 6.5) {follower_a *= 0.5}
			acc = min(acc, desired_velocity - current_speed);
		}
	}


	else {
		if (dx > sdxc) {
			double a_max = cc8 + cc9*min(current_speed, 80 * 1000 / 3600) + seedrandom(id); //capped at 80km/h
			if (dx < sdxo) {
				acc = min(dv*dv / (sdxo - dx), a_max);
			}
				else {
					acc = a_max;
				}
			}
			
		//if (follower.length >= 6.5) {follower_a *= 0.5}
		acc = min(acc, desired_velocity - current_speed);
	}
	//output11 << id << "	" << sdvo<<"	"<< sdxc << "	" << sdvc<< "	"<< dv << "	" << dx << "	" << acc << endl;
	//output11 << id << "	" << nvehid << "	" << distance_end << "	" << acc << "	dx=" << dx << "	" << sdxc<< "	dv=" << dv << "	" << sdvo << endl;
	return acc;
}

double rightlane()
{
	double speedreduction = 1 / 3;
	double xhead = nlead_vehicle_distance - nlead_vehicle_length;
	double xfollow = nfollow_vehicle_distance - current_length;
	double vhead = current_speed + nlead_vehicle_speed_difference;
	double vfollow = current_speed + nfollow_vehicle_speed_difference;
	double ahead = nlead_vehicle_acceleration;
	double afollow = nfollow_vehicle_acceleration;
	double xheadbetween = lead_vehicle_distance - lead_vehicle_length;

	double newXfollow = -xfollow + (vfollow + afollow / 2);
	double newXhead = xhead + (vhead + ahead / 2);


	double acc = ((newXhead + newXfollow) - 2 * current_speed) / 2;

	//output11 << id << "||" << newXfollow << "||" << newXhead << "||" << current_speed << "||" << acc << endl;





	if (xhead > xfollow && current_speed < (vhead + vfollow) / 2 + 1) {
		acc = 2;
		//output11 << id << "||" << "xh>xf&cv<v" << "||" << (vhead + vfollow) / 2 << "||" << current_speed << "||" << acc << endl;
	}
	else if (xhead > xfollow && current_speed >= (vhead + vfollow) / 2 + 3) {
		acc = max(-sqrt(xfollow), (vhead + vfollow) / 2 - current_speed);
		//output11 << id << "||" << "xh>xf&cv>v" << "||" << (vhead + vfollow) / 2 << "||" << current_speed << "||" << acc << endl;
	}
	else if (xhead <= xfollow && current_speed < (vhead + vfollow) / 2 - 1) {
		acc = min(sqrt((xfollow - xhead) / 2), (vhead + vfollow) / 2 - current_speed);
		//output11 << id << "||" << "xh<xf&cv<v" << "||" << (vhead + vfollow) / 2 << "||" << current_speed << "||" << acc << endl;
	}
	else if (xhead <= xfollow && current_speed >= (vhead + vfollow) / 2 - 3) {
		acc = -2;
		//output11 << id << "||" << "xh<xf&cv>v" << "||" << (vhead + vfollow) / 2 << "||" << current_speed << "||" << acc << endl;
	}
	else {
		acc = 0;
	}



	if (acc > 0) {
		acc = min(acc, 2);
	}
	else {
		acc = max(-2, acc);
	}

	if (lead_vehicle_distance < nlead_vehicle_distance) {
		if (nlead_vehicle_distance + nfollow_vehicle_distance < 100) {
			acc = wiedman99(10);
		}
		else {
			acc = wiedman99(1.2);
		}
	}

	if (xhead > 30 && xfollow > 30) {
		acc = wiedman99(3);
	}

	return acc;
}

/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule,
                       DWORD   ul_reason_for_call,
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (long   type,
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <long_value>, <double_value> or            */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_PATH                   :
    case DRIVER_DATA_TIMESTEP               :
		return 1;
    case DRIVER_DATA_TIME                   :
		timestep = (int)round(double_value);
      return 1;
    case DRIVER_DATA_VEH_ID                 :
      /* reset leading vehicle's data for this new vehicle */
		id = (int)round(long_value);
      lead_vehicle_distance         = 200.0;
      lead_vehicle_speed_difference = 40.0;
      lead_vehicle_length           =   0.0;
	  /*for (int i = 0; i <= 7; i++) {
		  M[id][i] = 0;
		  S[id][i] = 0;
	  }
	  for (int i = 3; i <= 7; i = i + 2) {
		  S[id][i] = 200;
	  }*/
      return 1;
    case DRIVER_DATA_VEH_LANE               :
		laneNum = long_value;
		return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
      return 1;
    case DRIVER_DATA_VEH_VELOCITY           :
      current_speed = double_value;
	  /*M[id][0] = current_speed;
	  if (xPos > 400 && xPos <= 1120) {
		  if (laneNum == 1) {
			  S[id][3] = current_speed;
		  }
	  }*/
      return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
		current_acc = double_value;
		return 1;
    case DRIVER_DATA_VEH_LENGTH             :
		current_length = double_value;
		return 1;
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
      turning_indicator = long_value;
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      desired_velocity = double_value;
      return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
		/*M[id][1] = 1023.0 - double_value;
		if (xPos > 400 && xPos <= 1120) {
			if (laneNum == 1) {
				S[id][1] = 1120 - double_value;
			}
		}*/
		xPos = double_value;
		return 1;
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_TYPE               :
      return 1;
    case DRIVER_DATA_VEH_COLOR              :
      vehicle_color = long_value;
      return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
      return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
    case DRIVER_DATA_NVEH_ID                :
		if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
			nvehid = long_value;
		}
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
      return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
      if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
        lead_vehicle_distance = double_value;
      }
	  if (index1 == 1 && index2 == 1) { 
		  nlead_vehicle_distance = double_value;
	  }
	  if (index1 == 1 && index2 == -1) { 
		  nfollow_vehicle_distance = -double_value;
	  }

      return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
      if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
        lead_vehicle_speed_difference = -double_value;
		/*M[id][2] = lead_vehicle_speed_difference;
		if (xPos > 400 && xPos <= 1120) {
			if (laneNum == 1) {
				S[id][2] = lead_vehicle_speed_difference;
			}
		}*/
      }
	  if (index1 == 1 && index2 == 1) {
		  nlead_vehicle_speed_difference = -double_value;
		  /*M[id][4] = nlead_vehicle_speed_difference;
		  if (xPos > 400 && xPos <= 1120) {
			  if (laneNum == 1) {
				  S[id][4] = nlead_vehicle_speed_difference;
			  }
		  }*/
	  }
	  if (index1 == 1 && index2 == -1) { 
		  nfollow_vehicle_speed_difference = -double_value;
		  /*M[id][6] = nfollow_vehicle_speed_difference;
		  if (xPos > 400 && xPos <= 1120) {
			  if (laneNum == 1) {
				  S[id][6] = nfollow_vehicle_speed_difference;
			  }
		  }*/

	  }
      return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
	  if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
		lead_vehicle_acceleration = double_value;
	  }
	  if (index1 == 1 && index2 == 1) { 
		  nlead_vehicle_acceleration = double_value;
	  }
	  if (index1 == 1 && index2 == -1) {
		  nfollow_vehicle_acceleration = double_value;
	  }
      return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
      if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
        lead_vehicle_length = double_value;
		/*M[id][3] = lead_vehicle_distance - lead_vehicle_length;
		if (xPos > 400 && xPos <= 1120) {
			if (laneNum == 1) {
				S[id][3] = lead_vehicle_distance - lead_vehicle_length;
			}
		}*/
      }
	  if (index1 == 1 && index2 == 1) {
		  nlead_vehicle_length = double_value;
		  /*M[id][5] = nlead_vehicle_distance - nlead_vehicle_length;
		  if (xPos > 400 && xPos <= 1120) {
			  if (laneNum == 1) {
				  S[id][5] = nlead_vehicle_distance - nlead_vehicle_length;
			  }
		  }*/
	  }
	  if (index1 == 1 && index2 == -1) {
		  nfollow_vehicle_length = double_value;
		  /*M[id][7] = nfollow_vehicle_distance - current_length;
		  if (xPos > 400 && xPos <= 1120) {
			  if (laneNum == 1) {
				  S[id][7] = nfollow_vehicle_distance - current_length;
			  }
		  }*/
	  }
      return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
    case DRIVER_DATA_NO_OF_LANES            :
    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
		if (double_value > 3.8) {
			distance_end = double_value;
		}
		if (distance_end < 70 && laneNum == 1 && nvehid < 0) {
			lead_vehicle_distance = distance_end-4;
lead_vehicle_speed_difference = -current_speed;
		}
		return 1;
    case DRIVER_DATA_RADIUS:
	case DRIVER_DATA_MIN_RADIUS:
	case DRIVER_DATA_DIST_TO_MIN_RADIUS:
	case DRIVER_DATA_SLOPE:
	case DRIVER_DATA_SLOPE_AHEAD:
	case DRIVER_DATA_SIGNAL_DISTANCE:
	case DRIVER_DATA_SIGNAL_STATE:
	case DRIVER_DATA_SIGNAL_STATE_START:
	case DRIVER_DATA_SPEED_LIMIT_DISTANCE:
	case DRIVER_DATA_SPEED_LIMIT_VALUE:
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:
		desired_acceleration = double_value;
		return 1;
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		desired_lane_angle = double_value;
		return 1;
	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		active_lane_change = long_value;
		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		rel_target_lane = long_value;
		return 1;
	default:
		return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue(long   type,
	long   index1,
	long   index2,
	long   *long_value,
	double *double_value,
	char   **string_value)
{
	/* Gets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, and writes that value to <*double_value>,     */
	/* <*float_value> or <**string_value> (object and value selection       */
	/* depending on <type>).                                                */
	/* Return value is 1 on success, otherwise 0.                           */




	switch (type) {
	case DRIVER_DATA_STATUS:
		*long_value = 0;
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		*long_value = turning_indicator;
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		*double_value = desired_velocity;
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		*long_value = vehicle_color;
		return 1;
	case DRIVER_DATA_WANTS_SUGGESTION:
		*long_value = 1;
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:
		if (xPos <= 400 && xPos > -406) {
			*double_value = wiedman99(3.5);
		}

		else if (xPos > 400 && xPos <= 1120) {
			if (laneNum == 1) {
				//output << id << '|' << timestep << '|' << A[id] << endl;
				if (id == p_id){
					*double_value = p_a;
				}
				else {
					*double_value = wiedman99(3.5);
				}
				
			}
			else {
				*double_value = wiedman99(3.5);
			}
		}
		else {
			*double_value = wiedman99(1.2);
		}	
		return 1;

	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		*double_value = desired_lane_angle;
		return 1;
	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		*long_value = active_lane_change;
		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		*long_value = rel_target_lane;
		return 1;
	case DRIVER_DATA_SIMPLE_LANECHANGE:
		*long_value = 1;
		return 1;
	default:
		return 0;
	}
}

//DRIVERMODEL_API float *array(int x)
//{
//	float *information = new float[8];
//	information[0] = M[x][0];
//	information[1] = M[x][1];
//	information[2] = M[x][2];
//	if (M[x][3] == 0) {
//		information[3] = 200;
//	}
//	else {
//		information[3] = M[x][3];
//	}
//	information[4] = M[x][4];
//	if (M[x][5] == 0) {
//		information[5] = 200;
//	}
//	else {
//		information[5] = M[x][5];
//	}
//	information[6] = M[x][6];
//	if (M[x][7] == 0) {
//		information[7] = 200;
//	}
//	else {
//		information[7] = M[x][7];
//	}
//	return information;
//}
//
//DRIVERMODEL_API double *envinput(int id)
//{
//	double *input = new double [8];
//
//	for (int i = 0; i < 8; i++) {
//		input[i] = S[id][i];
//	}
//	return input;
//}

DRIVERMODEL_API double** ls2() {
	double** information = new double*[2000];
	for (int i = 0; i < 3; ++i)
	{
		information[i] = new double[8];
	}
	for (int k = 0; k<3; k++) {
		for (int j = 0; j < 3; j++)
			information[k][j] = 1;
	}
	return information;
}


//DRIVERMODEL_API void acc_python(int x, double y) {
//	//output << x << '|' << y << endl;
//	A[x] = y;
//}
//
//DRIVERMODEL_API void acc_clear() {
//	//output << "sucess" << endl;
//	for (int i = 0; i < 2000; i++) {
//		A[i] = 0;
//	}
//}


DRIVERMODEL_API void sendpython(int x, double y) {
	p_id = x;
	p_a = y;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand(long number)
{
	/* Executes the command <number> if that is available in the driver */
	/* module. Return value is 1 on success, otherwise 0.               */

	switch (number) {
	case DRIVER_COMMAND_INIT:
      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
      return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
//		for (int i = 0; i <= 7; i++) {
//			output << M[id][i] << '|';
//		}
//		output << endl;
      return 1;
    default :
      return 0;
  }
}



/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/
