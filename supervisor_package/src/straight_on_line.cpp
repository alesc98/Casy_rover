//LIBRERIE ROS
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_listener.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include "std_srvs/SetBool.h"
#include "mavros_msgs/CommandHome.h"

//LIBRERIE C++
#include <cmath>

//PUBLISHER
ros::Publisher motion_pub_ = {};

//FLAG DI ATTIVAZIONE
bool active_ = false;

bool arrived_ = true;

//VARIABILI DI ORIENTAMENTO
double roll_ = 0.0;
double pitch_ = 0.0;
double yaw_ = 0.0;

//ERRORI DI ORIENTAMENTO E POSIZIONE
double err_yaw_;
double err_pos_ = 1000;

//PRECISIONE DI ORIENTAMENTO E POSIZIONE
double yaw_precision_ = M_PI / 87;
double dist_precision_ = 0.3;

//VARIABILE DI STATO
int state_ = 0;

//VARIABILI DI NAVGAZIONE
float TURN_LEFT_ANGULAR_ = 0.3;
float TURN_RIGHT_ANGULAR_ = -0.3;
float GO_STRAIGHT_LINEAR_ = 0.7;

//MESSAGGIO Twist PER CONTROLLARE IL differential_drive plugin
geometry_msgs::Twist twist_msg = {};

//POSIZIONE DESIDERATA
geometry_msgs::PointStamped map_point;
//POSIZIONE ATTUALE ROVER
geometry_msgs::Point position_;

/* ************************************************************** */
/* -- FUNCTIONS DECLARATIONS ------------------------------------ */
void go_straight(geometry_msgs::PointStamped des_pos, geometry_msgs::Twist twist_msg);
void fix_yaw(geometry_msgs::PointStamped des_pos, geometry_msgs::Twist twist_msg);
void clbk_imu_gps(const nav_msgs::Odometry::ConstPtr &imu_gps_msg);
int change_state(int state);
void done(geometry_msgs::PointStamped des_pos, geometry_msgs::Twist twist_msg);
double normalize_angle(double angle);
bool straight_on_line_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void set_new_dest(const geometry_msgs::Point::ConstPtr new_dest);
void fix_yaw(geometry_msgs::PointStamped des_pos, geometry_msgs::Twist twist_msg);
/* ************************************************************** */

/* ************************************************************** */
/* -- MAIN ------------------------------------- */
int main(int argc, char **argv)
{
    //INIZIALIZZAZIONE NODO
    ros::init(argc, argv, "straight_on_line_node");
    ros::NodeHandle n;

    //DICIARAZIONI SUBSCRIBER E SERVIZI
    ros::Subscriber gps_sub, dest_sub;
    ros::ServiceServer service;
    //SUBSCRIBER
    gps_sub = n.subscribe("/odometry/filtered_map", 100, clbk_imu_gps);
    dest_sub = n.subscribe("/gtp_dest", 10, set_new_dest);
    //PUBLISHER
    motion_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //SERVII
    service = n.advertiseService("straight_on_line_switch", straight_on_line_switch);

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        //SE IL FLAG DI ATTIVAZIONE NON E' ATTIVO IL NODO E' IN LOOP INFINITO
        if (!active_)
        {
        }
        //SE IL FLAG E' ATTIVO IL NODO ALTERNA FASI DI CORREZIONE ORIENTAMENTO E MOVIMENTAZIONE
        //A SECONDA DELLO STATO IN CUI SI TROVA
        else
        {

            if (state_ == 0)
                fix_yaw(map_point, twist_msg);
            else if (state_ == 1)
                go_straight(map_point, twist_msg);
            else if (state_ == 2)
            {
                //QUANDO L'ERRORE DI POSIZIONE E'SUFFICENTEMENTE BASSO SI PASSA ALLO STATO 2
                //(POSIZIONE RAGGIUNTA)
                done(map_point, twist_msg);
                arrived_ = true;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/* ************************************************************** */
/* -- FUNCTIONS DEFINITIONS ------------------------------------- */

/*
fix_yaw
FUNZIONE CHE PRENDE IN INGRESSO LA DESTINAZIONE DA RAGGIUNGERE dest_pos ED UN MESSAGGIO Twist
DA PUBBLICARE
*/
void fix_yaw(geometry_msgs::PointStamped des_pos, geometry_msgs::Twist twist_msg)
{
    double desired_yaw;
    //CALCOLO DELLO YAW DESIDERATO 
    desired_yaw = atan2(des_pos.point.y - position_.y, des_pos.point.x - position_.x);
    //CALCOLO DELL'ERRORE DI YAW
    err_yaw_ = normalize_angle(desired_yaw - yaw_);
    ROS_INFO("Current Yaw = %f, desired yaw = %f,Yaw error = %f", yaw_, desired_yaw, err_yaw_);

    //SE L'ERRORE DI YAW E MAGGIORE DELLA PRECISIONE SETTATA VIENE PUBBLICATO IL MESSAGGIO Twist
    //CHE CORREGGERA' L'ORIENTAMENTO
    if (fabs(err_yaw_) > yaw_precision_)
    {
        if (err_yaw_ > 0)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = TURN_LEFT_ANGULAR_;
        }
        else
        {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = TURN_RIGHT_ANGULAR_;
        }

        motion_pub_.publish(twist_msg);
    }
    //QUANDO L'ERRORE E' CONTENUTO SI PASSA ALLO STATO GO_STRAIGHT
    if (fabs(err_yaw_) <= yaw_precision_)
    {
        ROS_INFO("Yaw error = %f, changing state to : Go straight", err_yaw_);
        change_state(1);
    }
}

/*
go_straight
FUNZONE CHE PERMETTE DI FAR MUOVERE IL ROVER SU LINEE RETTE
*/
void go_straight(geometry_msgs::PointStamped des_pos, geometry_msgs::Twist twist_msg)
{
    double desired_yaw;
    //CALCOLO YAW DESIDERATO
    desired_yaw = atan2(des_pos.point.y - position_.y, des_pos.point.x - position_.x);
    //CALCOLO ERRORI DI POSIZIONE E DI YAW
    err_yaw_ = normalize_angle(desired_yaw - yaw_);
    err_pos_ = sqrt(pow(des_pos.point.y - position_.y, 2) + pow(des_pos.point.x - position_.x, 2));

    //QUANDO L'ERRORE DI POSIZIONE E' MAGGIORE DELLA PRECISIONE DESIDERATA SI PUBBLICA UN MESSAGGIO
    //DI NAVIGAZIONE LINEARE
    if (err_pos_ > dist_precision_)
    {
        twist_msg.linear.x = GO_STRAIGHT_LINEAR_;
        twist_msg.angular.z = 0.0;
        motion_pub_.publish(twist_msg);
    }
    //QUANDO L'ERRORE E' SUFFICENTEMENTE BASSO SI PASSA ALLO STATO DI DONE
    else
        change_state(2);

    //CONTROLLO DELL'ERRORE DI YAW, SE TROPPO ALTO SI PASSA ALLO STATO FIX_YAW
    if (fabs(err_yaw_) > yaw_precision_)
    {
        ROS_INFO("Yaw error = %f", err_yaw_);
        change_state(0);
    }
}

/*
clbk_imu_gps
CALLBACK FUNCTION DEL SUBSCRIBER gps_sub SOTTOSCRITTO AL TOPIC /Odometry/filtered_map
FRUTTO DELL'UNIONE DI INFORMAZIONI DI GPS, IMU E ODOMETRIA DEL ROVER
*/
void clbk_imu_gps(const nav_msgs::Odometry::ConstPtr &imu_gps_msg)
{
    //AGGIORNAMENTO POSIZIONE ATTUALE 
    position_ = imu_gps_msg->pose.pose.position;
    //AGGIORNAMENTO ORIENTAMENTO 
    tf::Quaternion q(imu_gps_msg->pose.pose.orientation.x,
                     imu_gps_msg->pose.pose.orientation.y,
                     imu_gps_msg->pose.pose.orientation.z,
                     imu_gps_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
}

double normalize_angle(double angle)
{
    if (std::abs(angle) > M_PI)
        angle = angle - (2 * M_PI * angle) / (std::abs(angle));
    return angle;
}

/*
change_state
FUNZIONE PER CAMBIARE LO STATO
*/
int change_state(int state)
{
    state_ = state;
    ROS_INFO("State changed to %d", state);
}


/*
done
FUNZIONE CHE PUBBLICA UN MESSAGGIO Twist VUOTO PER FERMARE IL ROVER QUANDO QUESTO
RAGGIUNGE LA POSIZIONE DESIDERATA
*/
void done(geometry_msgs::PointStamped des_pos, geometry_msgs::Twist twist_msg)
{
    double desired_yaw;

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    motion_pub_.publish(twist_msg);

    //ANCHE IN STATO DI DONE SI CONTROLLA L'ORIENTAMENTO DEL ROVER IN MODO CHE SI POSSA RAGGIUNGERE UNA NUOVA
    //DESTINAZIONE 
    desired_yaw = atan2(des_pos.point.y - position_.y, des_pos.point.x - position_.x);
    err_yaw_ = normalize_angle(desired_yaw - yaw_);
    err_pos_ = sqrt(pow(des_pos.point.y - position_.y, 2) + pow(des_pos.point.x - position_.x, 2));

    if (fabs(err_yaw_) > yaw_precision_)
    {
        ROS_INFO("Yaw error = %f", err_yaw_);
        change_state(0);
    }
}

/*
straight_on_line_switch
CALLBACK FUNCTION DEL SERIVIZIO service, CHIAMATA OGNI VOLTA CHE IL MOVEMENT_MANAGER RICHIEDE L'ATTIVAIZIONE
DI QUESTO NODO
*/
bool straight_on_line_switch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    active_ = req.data;
    res.success = true;
    res.message = "Done!";
    return true;
}

/*
set_new_dest
CALLBACK FUNCTION DEL SUBSCRIBER dest_sub, CHIAMATA OGNI VOLTA CHE IL MOVEMENT_MANAGER TRASMETTE LA 
POSIZIONE DA RAGGIUNGERE
*/
void set_new_dest(const geometry_msgs::Point::ConstPtr new_dest)
{
    map_point.point.x = new_dest->x;
    map_point.point.y = new_dest->y;
    map_point.point.z = new_dest->z;
}