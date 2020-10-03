//LIBRERIE ROS
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/SetBool.h"
#include "signal.h"

//LIBRERIE C++
#include <cmath>


//PUBLISHER
ros::Publisher motion_pub_ = {};

//ARRAY DEGLI OSTACOLI A DISTANZA MINIMA NELLE 5 REGIONI
float min_regions_[5] = {0};

//LAG DI ATTIVAZIONE NODO
bool active_ = false;

//MESSAGGIO DI TIPO Twist PER LA MOVIMENTAZIONE 
geometry_msgs::Twist msg;

//DISTANZA DI SICUREZZA DA OSTACOLI
float sft_dist = 2.7;

//VARIABILE DI STATO
int state_ = 0;
/*
	FIND_THE_WALL = 0, 
	TURN_LEFT = 1  
	FOLLOW_WALL = 2
*/

/* ************************************************************** */
/* -- FUNCTIONS DECLARATIONS ------------------------------------ */
bool wall_follower_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg);
void change_state(int state);
void take_actions(void);
void find_wall(void);
void turn_left(void);
void follow_the_wall(void);
float get_min(const sensor_msgs::LaserScan::ConstPtr &msg, int index1, int index2, float max_dist);
/* ************************************************************** */

/* ************************************************************** */
/* -- MAIN ------------------------------------- */
int main(int argc, char **argv)
{
	//INIZIALIZZAZIONE NODO
	ros::init(argc, argv, "wall_follower_node");
	ros::NodeHandle n;

	//DICHIARAZIONI SUBSCRIBER E SERVIZI
	ros::Subscriber laser_sub;
	ros::ServiceServer service;

	//SUBSCRIBER
	laser_sub = n.subscribe("/laser/scan", 100, clbk_laser);
	//PUBLISHER
	motion_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//SERVIZI
	service = n.advertiseService("wall_follower_switch", wall_follower_switch);

	ros::Rate loop_rate(20);
	while (ros::ok())
	{	
		//SE IL FLAG DI ATTIVAZIONE NON E' ATTIVO IL NODO E' IN LOOP INFINITO
		if (!active_)
		{
		}
		//SE IL FLAG E' ATTIVO IL NODO ALTERNA FASI DI RIERCA OSTACOLO, VIRATA A SINISTRA
        //E NAVIGAZIONE ATTORNO ALL'OSTACOLO
		else
		{
			if (state_ == 0)
				find_wall();
			else if (state_ == 1)
				turn_left();
			else if (state_ == 2)
				follow_the_wall();
			else
				ROS_INFO("Unknow_state");

			motion_pub_.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

/* ************************************************************** */
/* -- FUNCTIONS DEFINITIONS ------------------------------------- */

/*
wall_follower_switch
CALLBACK FUNCTION DEL SERIVIZIO service, CHIAMATA OGNI VOLTA CHE IL MOVEMENT_MANAGER RICHIEDE L'ATTIVAIZIONE
DI QUESTO NODO
*/
bool wall_follower_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	active_ = req.data;
	res.success = true;
	res.message = "Done!";
	return true;
}

/*
clbk_laser
CALLBACK FUNCTION DEL SUBSCRIBER laser_sub, CHIAMATA OGNI VOLTA CHE ARRIVA UN MESSAGGIO
DAL SENSORE LASER.
LA FUNZIONE DIVIDE I DATI IN 5 RANGE PER OGNUNO DEI QUALI RICAVA L'OSTACOLO A DISTANZA MINIMA
*/
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	int k = 0;
	float max_dist = 10;

	//calculates the minimum for each of the 5 regions
	for (k = 0; k < 5; k++)
	{
		min_regions_[k] = get_min(msg, k * 144, (k + 1) * 144, max_dist);
	}
	//now we have the position of the closes object for each region in the min_regions[] array
	take_actions();
}

/*
change_state
FUNZIONE PER CAMBIARE LO STATO
*/
void change_state(int state)
{
	if (state != state_)
	{
		state_ = state;
		ROS_INFO("State changed");
	}
}

/*
take_actions
FUNZIONE CHE A SECONDA DELE VALORE DELLE DISTANZE PRESENTI NELL'ARRAY min_regions_ RICHIEDE
IL CAMBIO DA UNO STATO ALL'ALTRO
*/
void take_actions(void)
{
	//std_msgs::String state_description;

	float fright = min_regions_[1];
	float front = min_regions_[2];
	float fleft = min_regions_[3];
	float left = min_regions_[4];

	if ((fright > sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		//state_description.data = "case 1 - nothing ";
		change_state(0);
	}
	else if ((fright > sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		//state_description.data = "case 2 - fleft ";
		change_state(0);
	}
	else if ((fright > sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		//state_description.data = "case 3 - front ";
		change_state(1);
	}
	else if ((fright > sft_dist) && (front < sft_dist) && (fleft < sft_dist))
	{
		//state_description.data = "case 4 - front and fleft ";
		change_state(1);
	}
	else if ((fright < sft_dist) && (front > sft_dist) && (fleft > sft_dist))
	{
		//state_description.data = "case 5 - fright ";
		change_state(2);
	}
	else if ((fright < sft_dist) && (front > sft_dist) && (fleft < sft_dist))
	{
		//state_description.data = "case 6 - fright and fleft ";
		change_state(0);
	}
	else if ((fright < sft_dist) && (front < sft_dist) && (fleft > sft_dist))
	{
		//state_description.data = "case 7 - fright and front ";
		change_state(1);
	}
	else if ((fright < sft_dist) && (front < sft_dist) && (fleft < sft_dist))
	{
		//state_description.data = "case 8 - fright and front and fleft ";
		change_state(1);
	}
	else
	{
		//state_description.data = "unknown case";
		ROS_INFO("CASO SCONOSCIUTO");
	}
}

/*
find_wall
FUNZIONE CHE RIEMPIE IL MESSAGGIO Twist	CON VALORI OPPORTUNI
*/
void find_wall(void)
{
	msg.linear.x = 0.7;
	msg.angular.z = -0.3;
}

/*
turn_left
FUNZIONE CHE RIEMPIE IL MESSAGGIO Twist	CON VALORI OPPORTUNI
*/
void turn_left(void)
{
	msg.linear.x = 0;
	msg.angular.z = 0.6;
}

/*
follow_the_wall
FUNZIONE CHE RIEMPIE IL MESSAGGIO Twist	CON VALORI OPPORTUNI
*/
void follow_the_wall(void)
{
	msg.linear.x = 0.7;
	msg.angular.z = 0.1;
}

/*
get_min
FUNZIONE CHE CALCOLA IL MINIMO DI UN ARRAY IN UNA FINESTRA COMPRESA TRA index1 	E inde2
*/
float get_min(const sensor_msgs::LaserScan::ConstPtr &msg, int index1, int index2, float max_dist)
{
	int i;
	float temp_min = msg->ranges[index1];

	for (i = index1; i < index2; i++)
	{
		if (msg->ranges[i] < temp_min)
		{
			temp_min = msg->ranges[i];
		}
		if (temp_min > max_dist)
		{
			temp_min = max_dist;
		}
	}
	return temp_min;
}

