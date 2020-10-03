//LIBRERIE ROS
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include <mavros_msgs/Waypoint.h>
#include "std_srvs/SetBool.h"
#include "mavros_msgs/CommandHome.h"
#include <tf/transform_listener.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>

//LIBRERIE C++
#include <cmath>

//CLIENT DEI SERVIZI INIZIALIZZATI NEI NODI FOLLOW_WALL E STRAIGHT_ON_LINE
ros::ServiceClient srv_client_wall_follower_, srv_client_go_to_point_, waypoint_request_client_, home_request_client_, waypoint_reached;
ros::Publisher set_dest_go_to_point;

//VARIABILI DI POSIZIONE DEL ROVER
double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;
geometry_msgs::Point position_;

//LATITUDINE E LONGITUDINE DELLA HOME
double home_position_lat_ = 0;
double home_position_lon_ = 0;

//LATITUDINE E LONGITUDINE INIZIALI DEL ROVER
double initial_position_lat_;
double initial_position_lon_;

//MASSIMO ERRORE DI YAW AMMESSO 5 GRADI
double yaw_error_allowed_ = 5 * (M_PI / 180);

//FLAG PER STABILIRE LA POSIZIONE INIZIALE DEL ROVER NELLA MAPPA
bool first_pos_read_ = false;

//FLAG DI RESET MISSIONE
bool plan_reset_ = true;

//LATITUDINE E LONGITUDINE DEL WAYPOINT DA RAGGIUNGERE
double desired_postion_lat_;
double desired_position_lon_;

//NUMERO DI WAYPOINT RICEVUTI
int n_waypoint_recived_ = 0;
int n_total_waypoint = 0;
//INDICE
int i = 0;


//VARIABILI DI POSIZIONE INIZIALE E DESIDERATA DEL ROVER NELLA MAPPA
geometry_msgs::PointStamped initial_map_position_;
geometry_msgs::PointStamped home_map_position_;
geometry_msgs::PointStamped desired_map_position_;
// VARIABILI PER TRASFORMAZIONE DI COORDINATE GPS-->MAP
geometry_msgs::PointStamped UTM_point, UTM_point_home;
std::string utm_zone;

//VARIABILI DI STATO E CONTEGGIO TEMPI
/* 0 = go to point
   1 = follow wall */
int state_ = 0;
int count_state_time_ = 0;
int count_loop_ = 0;

//VETTORE DELLE DIZTANZE MINIME DEGLI OSTACOLI DAL ROVER
float min_regions_[5] = {0};

//MESSAGGIO MAVROS PER LA COMUNICAZIONE DELLE COORDINATE DEL WAYPOINT DA RAGGIUNGERE AL NODO DI NAVIGAZIONE LINEARE
geometry_msgs::Point dest_pos_;

//FLAGS DI RICHIESTA WAYPOINT E POSIZIONE DI HOME
std_srvs::SetBool req_waypoint, req_home;

//FLAGS DI ARRIVO PRIMO WAYPOINT E HOME SETTATA
bool first_waypoint_arrived_ = false;
bool home_setted_ = false;

/* ************************************************************** */
/* -- FUNCTIONS DECLARATIONS ------------------------------------ */
void clbk_imu_gps(const nav_msgs::Odometry::ConstPtr &imu_gps_msg);
void clbk_laser(const sensor_msgs::LaserScan::ConstPtr &msg);
void clbk_nav(const mavros_msgs::Waypoint::ConstPtr &waypoint_msg);
void clbk_home(const mavros_msgs::Waypoint::ConstPtr &waypoint_msg);
int change_state(int state);
void done(geometry_msgs::Twist twist_msg);
double normalize_angle(double angle);
float distance_to_line(geometry_msgs::Point p0);
float get_min(const sensor_msgs::LaserScan::ConstPtr &msg, int index1, int index2, float max_dist);
geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input);
geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input);
/* ************************************************************** */

/* ************************************************************** */
/* -- MAIN ------------------------------------- */
int main(int argc, char **argv)
{
    //INIZIALIZZAZIONE NODO
    ros::init(argc, argv, "rover_movement_manager_node");
    ros::NodeHandle n;

    //DICHIARAZIONI DI SUBSCRIBERS
    ros::Subscriber laser_sub, gps_sub, mavros_waypoint, mavros_home;

    //SOTTOSCRIZIONI A TOPIC
    gps_sub = n.subscribe("/odometry/filtered_map", 100, clbk_imu_gps);
    laser_sub = n.subscribe("/laser/scan", 100, clbk_laser);
    mavros_waypoint = n.subscribe("/mavros/waypoint", 100, clbk_nav);
    mavros_home = n.subscribe("/mavros/home", 100, clbk_home);

    //CLIENT DEI SERVIZI
    srv_client_wall_follower_ = n.serviceClient<std_srvs::SetBool>("wall_follower_switch");
    srv_client_go_to_point_ = n.serviceClient<std_srvs::SetBool>("straight_on_line_switch");
    waypoint_request_client_ = n.serviceClient<std_srvs::SetBool>("waypoint_request");
    home_request_client_ = n.serviceClient<std_srvs::SetBool>("home_request");
    waypoint_reached = n.serviceClient<std_srvs::SetBool>("waypoint_reached");

    //PUBLISHER
    set_dest_go_to_point = n.advertise<geometry_msgs::Point>("/gtp_dest", 1);

    //VARIABILI DI NAVIGAZIONE
    float distance_position_to_line = 0;
    float err_pos_tollerance = 0.4;
    double err_pos, err_home;

    //FLAG DI UPDATE DELLA POSIZIONE INIZIALE DEL ROVER
    bool initial_pos_updated = false;

    //FLAG DI HOMING
    bool homing_done = false;

    //FLAG DI INIZIALIZZAZIONE NAVIGAZIONE AL RAGGIUNGIMENTO DI UN WAYPOINT
    bool initialized = false;

    //NUMERO DI POSIZIONE DI FERMATA DEL ROVER
    int n_pos_reached = 0;

    //FLAG DI RAGGIUNGIMENTO WAYPOINT (COMUNICATO AL SUPERVISORE)
    std_srvs::SetBool arrived_to_waypoint;
    arrived_to_waypoint.request.data = false;

    //VARIABILI PER LA LETTURA DELLA POSIZIONE INIZIALE DEL ROVER NELLA MAPPA
    boost::shared_ptr<nav_msgs::Odometry const> shared_first_position_message;
    nav_msgs::Odometry first_position_message;

    //ATTESA ESISTENA SERVIZI
    waypoint_request_client_.waitForExistence();

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        //OGNI VOLTA CHE VIENE RICHESTO IL RESET DEL PIANO SI REGISTRA LA POSIZIONE INIZIALE DEL ROBOT E SI RICHIEDE
        //LA POSIZIONE DI HOME DEL PIANO
        if (plan_reset_)
        {
            plan_reset_ = false;
            first_waypoint_arrived_ = false;
            n_waypoint_recived_ = 0;
            n_pos_reached = 0;
            ROS_INFO("In attesa dell'arrivo del primo messaggio posizionale del rover");

            //ATTESA RICEZIONE POSIZIONE INIZIALE DEL ROVER NEL MAP_FRAME
            shared_first_position_message = ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered_map", n);
            if (shared_first_position_message != NULL)
                first_position_message = *shared_first_position_message;
            initial_map_position_.point.x = first_position_message.pose.pose.position.x;
            initial_map_position_.point.y = first_position_message.pose.pose.position.y;
            ROS_INFO("La posizione iniziale del rover e' [%f, %f]", initial_map_position_.point.x, initial_map_position_.point.y);

            //INVIO SEGNALE DI RICHIESTA COORDINATE HOME AL SUPERVISORE
            req_home.request.data = true;
            if (home_request_client_.call(req_home))
                ROS_INFO("Richiesta home effettuata");
            else
                ROS_INFO("Impossibile comunicare con il supervisore");
        }
        /*
        FINCHE' LA HOME NON E' SETTATA CON SUCCESSO ALL'INTETRNO DEL CICLO WHILE NON ACCADE NULLA
        (TUTTE LE CONDIZIONI DEGLI IF SONO FALSE)
        */
        if (home_setted_)
        {

            //QUANDO LE COORDINATE DI HOME SONO RICEVUTE E CONVERTITE NEL MAP_FRAME
            //SI VERIFICA SE LA POSIZIONE INIZIALE DEL ROBOT COINCIDE CON QUELLA DI HOME
            //IN CASO CONTRARIO LA POSIZIONE DI HOME VIENE IMPOSTATA COME PROSSIMA DESTINAZIONE E RAGGIUNTA
            err_home = sqrt(pow(home_map_position_.point.x - initial_map_position_.point.x, 2) + pow(home_map_position_.point.y - initial_map_position_.point.y, 2));

            if (err_home > 1.5)
            {
                ROS_INFO("La posizione iniziale del rover [%f, %f] non corrisponde alla home pianificata", initial_map_position_.point.x, initial_map_position_.point.y);
                ROS_INFO("Spostamento alla posizione di home: [%f, %f]", home_map_position_.point.x, home_map_position_.point.y);
                desired_map_position_.point.x = home_map_position_.point.x;
                desired_map_position_.point.y = home_map_position_.point.y;

                dest_pos_.x = home_map_position_.point.x;
                dest_pos_.y = home_map_position_.point.y;
                dest_pos_.z = 0;
                set_dest_go_to_point.publish(dest_pos_);
                ROS_INFO("Destinazione trasmessa al nodo go_to_point");
                homing_done = true;
            }
            else
            {
                //SE LA POSIZIONE INIZIALE DEL ROVER COINCIDE CON QUELLA DI HOME
                //VIENE RICHIESTO UN WAYPOINT AL SUPERVISORE
                req_waypoint.request.data = true;
                if (waypoint_request_client_.call(req_waypoint))
                    ROS_INFO("Richiesta di un waypoint effettuata");
            }

            home_setted_ = false;
        }
        //DOPO LA FASE DI "HOMING" O L'ARRIVO DEL PRIMO WAYPOINT
        //VIENE GESTITA LA MOVIMENTAZIONE DEL ROVER
        //QUESTO, IN ASSENZA DI OSTACOLI, PROCEDERA' SU UNA LINEA RETTA CHE PASSA PER IL PUNTO DI PARTENZA E QUELLO DI ARRIVO
        //GLI EVENTUALI OSTACOLI PRESENTI NEL TRAGITTO SARANNO CIRCUMNAVIGATI
        //STATO 0 = NAVIGAZIONE RETTILINEA
        //STATO 1 = CIRCUMNAVIGAZIONE

        if (first_waypoint_arrived_ || homing_done)
        {
            //L'INIZIALIZZAZIONE DELLO STATO GO TO POINT E' NECESSARIO AFFINCHE IL ROVER POSSA MUOVERSI
            if (!initialized)
            {
                change_state(0);
                initialized = true;
            }

            //ALGORITMO DI NAVIGAZIONE E CRITERI DI CAMBIO STATO
            err_pos = sqrt(pow(desired_map_position_.point.x - position_.x, 2) + pow(desired_map_position_.point.y - position_.y, 2));
            //LA NAVIGAZIONE CONTINUA FINTANTO CHE L'ERRORE DI POSIZIONE E' MAGGIORE DI 0.4
            if (err_pos > 0.4)
            {
                arrived_to_waypoint.request.data = false;
                initial_pos_updated = false;
                distance_position_to_line = distance_to_line(position_);

                if (state_ == 0)
                    if (min_regions_[2] > 0.2 && min_regions_[2] < 1.5)
                        change_state(1);

                if (state_ == 1)
                    if (count_state_time_ > 3 && distance_position_to_line < 0.6)
                    {
                        change_state(0);
                    }

                count_loop_++;

                if (count_loop_ == 20)
                {
                    count_state_time_++;
                    count_loop_ = 0;
                }
            }
            //QUANDO L'ERRORE DI POSIZIONE E' <=0.45 E IL FLAG DI ARRIVO AL WAYPOINT NON E' ANCORA STATO SETTATO
            //SI EFFETTUA L'AGGIORNAMENTO DELLA POSIZIONE INIZIALE DEL ROVER E LA RICHIESTA DI UN NUOVO WAYPOINT

            else if (!arrived_to_waypoint.request.data && err_pos - err_pos_tollerance <= 0.05)
            {
                n_pos_reached++;
                arrived_to_waypoint.request.data = true;
                waypoint_reached.call(arrived_to_waypoint);

                //LA POSIZIONE INIZIALE DEL ROVER DEVE ESSERE AGGIORNATA ALLA SUA POSIZIONE ATTUALE
                //OGNI VOLTA CHE QUESTO RAGGIUNGE UN WAYPOINT ( NECESSARIO PER IL FUNZIONAMENTO DELL'ALGORITMO)
                if (!initial_pos_updated)
                {
                    initial_pos_updated = true;
                    initial_map_position_.point.x = position_.x;
                    initial_map_position_.point.y = position_.y;
                    ROS_INFO("Posizione desiderata [%f, %f] raggiunta", desired_map_position_.point.x, desired_map_position_.point.y);
                }
                //LA SEGUENTE CONDIZIONE E' VERIFICATA SOLO QUANDO, A FINE MISSIONE, IL ROVER E' TORNATO ALLA POSIZIONE DI HOME
                //VIENE DUNQUE SETTATO IL FLAG CHE PERMETTE DI RICHIEDERE LA POSIZIONE DI HOME DI UN EVENTUALE NUOVO PIANO
                err_home = sqrt(pow(home_map_position_.point.x - initial_map_position_.point.x, 2) + pow(home_map_position_.point.y - initial_map_position_.point.y, 2));
                ROS_INFO("err_home %f", err_home);
                if (err_home < 1.5 && n_pos_reached >= n_total_waypoint +1)
                {
                    plan_reset_ = true;
                }
                else
                {
                    //RICHIESTA DI UN WAYPOINT
                    req_waypoint.request.data = true;
                    if (waypoint_request_client_.call(req_waypoint))
                        ROS_INFO("Richiesta di un nuovo waypoint effettuata");
                }
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
clbk_imu_gps
FUNZIONE CHIAMATA OGNI VOLTA CHE VIENE RICEVUTO UN MESSAGGIO POSIZIONALE DEL ROVER NEL MAP FRAME 
(dal topic /odometry/filtered_map). QUESTI MESSAGGI DERIVANO DALL'INTERSEZIONE DELLE INFORMAZIONI
DI GPS, SENSORE IMU E ODOMETRIA DEL ROVER.
*/
void clbk_imu_gps(const nav_msgs::Odometry::ConstPtr &imu_gps_msg)
{
    //Aggiornare la posizione del robot
    position_ = imu_gps_msg->pose.pose.position;

    //Aggiornare l'angolo del robot (yaw)
    tf::Quaternion q(imu_gps_msg->pose.pose.orientation.x,
                     imu_gps_msg->pose.pose.orientation.y,
                     imu_gps_msg->pose.pose.orientation.z,
                     imu_gps_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
}
/*
clbk_laser
FUNZIONE CHIAMATA OGNI VOLTA CHE VIENE RICEVUTO UN MESSAGGIO DAL SENSORE LASER.
I RAGGI (ESTESI ANGOLARMENTE DI 180 GRADI) SONO SUDDIVISI IN 5 RANGE, PER OGNUNO 
VIENE CALCOLATO IL VALORE MINIMO (OSTACOLO PIU' VICINO AL ROVER NEL RANGE)
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
}

/*
get_min
FUNZIONE NECESSARIA PER CALOLARE IL VAOLRE MINIMO DI UN ARRAY IN UNA FINESTRA
COMPRESA TRA index1 E index2
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

/*
change_state
FUNZIONE DI CAMBIO STATO CHE ATTIVA E DISATTIVA I NODI GO_TO_POINT E FOLLOW_WALL
*/
int change_state(int state)
{
    count_state_time_ = 0;
    state_ = state;
    std_srvs::SetBool srv_go_to_point, srv_wall_follower;
    ROS_INFO("State changed to: %d", state);

    if (state_ == 0)
    {
        srv_go_to_point.request.data = true;
        srv_wall_follower.request.data = false;
    }
    if (state_ == 1)
    {
        srv_go_to_point.request.data = false;
        srv_wall_follower.request.data = true;
    }

    if (srv_client_go_to_point_.call(srv_go_to_point))
        ROS_INFO("Servizio go_to_point avvisato con successo");

    else
        ROS_INFO("Comunicazione con il servizio go_to_point fallita");

    if (srv_client_wall_follower_.call(srv_wall_follower))
        ROS_INFO("Servizio wall_follower avvisato con successo");

    else
        ROS_INFO("Comunicazione con il servizio wall_follower fallita");
}

/*
distance_to_line
FUNZIONE CHE CALCOLA LA DISTANZA DEL ROVER DALLA RETTA DI NAVIGAZIONE 
*/
float distance_to_line(geometry_msgs::Point p0)
{
    geometry_msgs::Point p1, p2;
    float up_eq, lo_eq, distance;

    p1 = initial_map_position_.point;
    p2 = desired_map_position_.point;
    up_eq = fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x));
    lo_eq = sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));
    distance = up_eq / lo_eq;

    return distance;
}

double normalize_angle(double angle)
{
    if (std::abs(angle) > M_PI)
        angle = angle - (2 * M_PI * angle) / (std::abs(angle));
    return angle;
}

/*LATLONGtoUTM FUNCTION*/
geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}
/*END*/

/*UTMtoMAPPOINT FUNCTION*/
geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while (notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
        }
    }
    return map_point_output;
}
/*END*/

/*
clbk_nav
CALLBACK FUNCTION DEL SUBSCRIBER mavros_waypoint, CHIAMATA ALLA RICEZIONE DI UN WAYPOINT 
*/

void clbk_nav(const mavros_msgs::Waypoint::ConstPtr &waypoint_msg)
{
    //IL param4 DEI WAYPOINT E' SETTATO A 0 DAL SUPERVISORE SE SI TRATTA DI UN QUALUNQUE WAYPOINT
    //VIENE SETTATO AD 1 SE IL WAYPOINT E' UTILIZZATO PER TRASMETTERE LA POSIZIONE DI HOME
    if (waypoint_msg->param4 == 0)
    {
        n_waypoint_recived_++;
        //TRASFORMAZIONE DELLE COORDINATE DEL WAYPOIT IN COORDINATE DEL MAP_FRAME
        //TRAMISSIONE DELLE COORDINATE RICAVATE AL NODO GO_TO_POINT
        UTM_point = latLongtoUTM(waypoint_msg->x_lat, waypoint_msg->y_long);
        desired_map_position_ = UTMtoMapPoint(UTM_point);
        ROS_INFO("Posizione ricevuta [%f, %f] ", desired_map_position_.point.x, desired_map_position_.point.y);
        dest_pos_.x = desired_map_position_.point.x;
        dest_pos_.y = desired_map_position_.point.y;
        dest_pos_.z = 0;
        set_dest_go_to_point.publish(dest_pos_);
        ROS_INFO("Destinazione trasmessa al nodo go_to_point");
        first_waypoint_arrived_ = true;
    }
    //param4 != 0, IL WAYPOINT RICEVUTO CONTIENE LE COORDINATE DI HOME
    else
    {
        //DATO CHE LE COORDINATE DI HOME SONO GIA' STATE RICEVUTE E TRASFORMATE AD INIZIO MISSIONE
        //SI PREFERISCE ASSEGNARLE DIRETTAMENTE PIUTTOSTO CHE TRASFORMARE NUOVAMENTE LE COORDINATE GEOGRAFICHE
        desired_map_position_.point = home_map_position_.point;
        desired_map_position_.point = home_map_position_.point;
        dest_pos_.x = home_map_position_.point.x;
        dest_pos_.y = home_map_position_.point.y;
        dest_pos_.z = 0;
        set_dest_go_to_point.publish(dest_pos_);
        ROS_INFO("Destinazione trasmessa al nodo go_to_point");
    }
    //OGNI VOLTA CHE VIENE RICEVUTO UN WAYPOINT E' INIZIALIZZATO LO STATO 0
    change_state(0);
}

/*
clbk_home
CALLBACK FUNCTION DEL SUBSCRIBER mavro_home UTILIZZATO PER LA RICEZIONE DELLE COORDINATE DI HOME AD
INIZIO MISSIONE
*/
void clbk_home(const mavros_msgs::Waypoint::ConstPtr &home_msg)
{
    home_position_lat_ = home_msg->x_lat;
    home_position_lon_ = home_msg->y_long;
    n_total_waypoint = home_msg->param3;

    UTM_point_home = latLongtoUTM(home_position_lat_, home_position_lon_);
    home_map_position_ = UTMtoMapPoint(UTM_point_home);

    ROS_INFO("Posizione di Home settata con successo");
    home_setted_ = true;
}
