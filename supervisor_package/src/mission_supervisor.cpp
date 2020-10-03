//LIBRERIE ROS
#include "ros/ros.h"
#include <nlohmann/json.hpp>
#include <mavros_msgs/Waypoint.h>
#include "mavros_msgs/CommandHome.h"
#include "std_srvs/SetBool.h"
#include "signal.h"

//LIBRERIE C++
#include <fstream>
#include <iostream>
#include <iomanip>

using nlohmann::json;

//MESSAGGI DI TIPO mavros_msgs::Waypoint AI QUALI SARANNO ASSEGNATI I VALORI DEI MISSION ITEMS
//E DELLA PLANNED HOME POSITION LETTI DAL PLAN FILE
mavros_msgs::Waypoint wayoint_array_[20];
mavros_msgs::Waypoint waypoint_home_;
//DICHIARAZIONI DI PUBLISHERS, CLIENTS E SERVIZI
ros::Publisher waypoint_pub_, home_coordinate_pub_;
ros::ServiceClient plan_reset_client;
ros::ServiceServer waypoint_request, home_request, waypoint_reached;
//CONTATORI EROGAZIONE
int count_loop_ = 0;
int count_state_time_ = 0;

int number_of_waypoints;
//INDICE
int j = 0;
//FLAG DI RICHIESTA WAYPOINT E HOME DA PARTE DEL MOVMENT_MANAGER
bool waypoint_pub_flag_ = false;
bool home_pub_flag_ = false;
//FLAG DI FILE LETTO
bool file_read = false;
//SRV DI TIPO SetBool PER EFFETTUARE IL RESET DI VARIABILI NEL MOVEMENT_MANAGER A FINE MISSIONE
std_srvs::SetBool plan_reset_message;

/* ************************************************************** */
/* -- FUNCTIONS DECLARATIONS ------------------------------------ */
bool exists(const json &j, const std::string &key);
bool waypoint_request_function(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
bool home_request_function(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
bool waypoint_reached_function(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
/* ************************************************************** */


/* ************************************************************** */
/* -- MAIN ------------------------------------- */
int main(int argc, char **argv)
{
    //INIZIALIZZAZIONE NODO
    ros::init(argc, argv, "mission_supervisor_node");
    ros::NodeHandle n;
    //PUBLISHER
    waypoint_pub_ = n.advertise<mavros_msgs::Waypoint>("/mavros/waypoint", 1);
    home_coordinate_pub_ = n.advertise<mavros_msgs::Waypoint>("/mavros/home", 1);
    //SERVIZI
    waypoint_request = n.advertiseService("waypoint_request", waypoint_request_function);
    home_request = n.advertiseService("home_request", home_request_function);
    waypoint_reached = n.advertiseService("waypoint_reached", waypoint_reached_function);
    //CLIENT
    plan_reset_client = n.serviceClient<std_srvs::SetBool>("plan_reset");
    //FLAG DI RICHIESTA HOMING
    bool homed = false;

    //CONTENITORI DEI DATI RILEVANTI PROVENIENTI DAL FILE.PLAN
    std::string file_name;
    std::string file_type;
    int vachile_type;
    std::string ground_station;
    std::ifstream i;

    //INIZIALIZZAZIONE DEI CONTENITORI DEI DATI RELATIVI AI WAYPOINTS
    json command, frame;
    json param_hold;
    json param_lat;
    json param_lon;
    json mission;
    json items_array;

    double planned_home_pos_[3] = {};
    //INDICE
    int k = 0;

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        //SE NON E' STATO LETTO ALCUN FILE QUESTO VERRA' RICHIESTO E DUNQUE APERTO
        if (!file_read)
        {
            k = 0;
            j = 0;
            homed = false;
            ros::Duration(2.0).sleep();
            ROS_INFO("/--------------- ROVER CASY --------------- */");
            ROS_INFO("Inserire il percorso del file.plan da caricare");
            ROS_INFO("ESEMPIO: /home/user/file.plan");
            ROS_INFO("/------------------------------------------ */");
            std::cin >> file_name;
            ros::Duration(2.0).sleep();

            //LETTURA FILE MISSIONE E PARSING DEI DATI
            i.open(file_name, std::ios::in);
            // std::ifstream i("/home/alessandro/casy_rover_plan.plan");
            json plan = json::parse(i);
            i.close();

            //VERIFICA CORRETTEZZA TIPO E CAMPI RILEVANTI DEL FILE
            if (plan.find("fileType") != plan.end())
            {
                if (plan["fileType"] != "Plan")
                {
                    ROS_INFO("Il file aperto non è di tipo plan");
                    ROS_INFO("VERIFICARE LA CORRETTEZZA DEL FILE.PLAN, PREMERE CTRL+C PER USCIRE");
                    i.close();
                    while (ros::ok)
                        ;
                }
            }
            else
            {
                ROS_INFO("Il file aperto non è corretto");
                ROS_INFO("VERIFICARE LA CORRETTEZZA DEL FILE.PLAN, PREMERE CTRL+C PER USCIRE");
                while (ros::ok)
                    ;
            }

            if (!exists(plan, "groundStation"))
            {
                ROS_INFO("ERRORE: Il file aperto non contiene il campo 'groundstation'");
                ROS_INFO("VERIFICARE LA CORRETTEZZA DEL FILE.PLAN, PREMERE CTRL+C PER USCIRE");
                while (ros::ok)
                    ;
            }
            if (!exists(plan, "mission"))
            {
                ROS_INFO("ERRORE: Il file aperto non contiene il campo 'mission'");
                ROS_INFO("VERIFICARE LA CORRETTEZZA DEL FILE.PLAN, PREMERE CTRL+C PER USCIRE");
                while (ros::ok)
                    ;
            }
            if (!exists(plan["mission"], "items"))
            {
                ROS_INFO("ERRORE: Il file aperto non contiene l'array 'mission/items'");
                ROS_INFO("VERIFICARE LA CORRETTEZZA DEL FILE.PLAN, PREMERE CTRL+C PER USCIRE");
                while (ros::ok)
                    ;
            }
            if (!exists(plan["mission"], "plannedHomePosition"))
            {
                ROS_INFO("ERRORE: Il file aperto non contiene l'array 'mission/plannedHomePosition'");
                ROS_INFO("VERIFICARE LA CORRETTEZZA DEL FILE.PLAN, PREMERE CTRL+C PER USCIRE");
                while (ros::ok)
                    ;
            }
            if (!exists(plan["mission"], "vehicleType"))
            {
                ROS_INFO("ERRORE: Il file aperto non contiene l'oggetto 'mission/vehicleType'");
                ROS_INFO("VERIFICARE LA CORRETTEZZA DEL FILE.PLAN, PREMERE CTRL+C PER USCIRE");
                while (ros::ok)
                    ;
            }

            //ISOLAMENTO DELL'OGGETTO MISSIONE E DELL'ARRAY DI ITEM IN ESSO CONTENUTO
            mission = plan.at("/mission"_json_pointer);
            items_array = plan.at("/mission/items"_json_pointer);

            //CALCOLO NUMERO DI WAYPOINTS PRESENTI NEL PLAN
            number_of_waypoints = plan.at("/mission/items"_json_pointer).size();

            //ESTRAZIONE DEI DATI PIU' RILEVANTI
            file_type = plan.at("fileType").get<std::string>();
            ground_station = plan.at("groundStation").get<std::string>();
            vachile_type = mission.at("vehicleType").get<int>();

            //LETTURA DELLE COORDINATE DI HOME PIANIFICATA
            for (const auto &planned_home_pos_ition_array : mission["plannedHomePosition"])
            {
                planned_home_pos_[k] = planned_home_pos_ition_array.get<double>();
                k++;
            }
            k = 0;

            //RIEMPIMENTO DEL WAYPOINT MESSAGE PER COMUNICARE LA POSIZIONE DI HOME
            waypoint_home_.x_lat = planned_home_pos_[0];
            waypoint_home_.y_long = planned_home_pos_[1];
            waypoint_home_.z_alt = planned_home_pos_[2];
            waypoint_home_.param3 = number_of_waypoints;

            for (const auto &item_relevant_element : items_array)
            {

                //ISOLAMENTO DELLE VOCI FRAME E COMMAND DI OGNI WAYPOINT
                frame = item_relevant_element["frame"];
                command = item_relevant_element["command"];

                //ASSEGNAMENTO DEI CAMPI FRAME E COMMAND NELLE STRUTTURE DATI MAVROS_MSGS/WAYPOINT
                wayoint_array_[j].frame = frame.get<uint8_t>();
                wayoint_array_[j].command = command.get<uint8_t>();

                for (const auto &item_parameter : item_relevant_element["params"])
                {
                    //ESTRAZIONE DI PARAMETRI E COORDINATE DI OGNI SINGOLO SIMPLE ITEMS DEL PIANO, ASSEGNAMENTO ALLE RISPETTIVE STRUTTURE DATI MAVROS_MSGS/WAYPOINT

                    if (k == 0)
                    {
                        if (item_parameter.is_null())
                            wayoint_array_[j].param1 = 0;
                        else
                            wayoint_array_[j].param1 = item_parameter.get<float>();
                    }
                    if (k == 1)
                    {
                        if (item_parameter.is_null())
                            wayoint_array_[j].param2 = 0;
                        else
                            wayoint_array_[j].param2 = item_parameter.get<float>();
                    }
                    if (k == 2)
                    {
                        if (item_parameter.is_null())
                            wayoint_array_[j].param3 = 0;
                        else
                            wayoint_array_[j].param3 = item_parameter.get<float>();
                    }
                    if (k == 3)
                    {
                        if (item_parameter.is_null())
                            wayoint_array_[j].param4 = 0;
                        else
                            wayoint_array_[j].param4 = item_parameter.get<float>();
                    }
                    if (k == 4)
                    {
                        if (item_parameter.is_null())
                        {
                            ROS_INFO("ERRORE: La latitudine non può assumere il valore 'null', verificare la correttezza del file.plan");
                            ROS_INFO("ERRORE: La latitudine del waypoint numero %d sara' impostata al valore di home", j + 1);
                            wayoint_array_[j].x_lat = planned_home_pos_[0];
                        }
                        else
                            wayoint_array_[j].x_lat = item_parameter.get<double>();
                    }
                    if (k == 5)
                    {
                        if (item_parameter.is_null())
                        {
                            ROS_INFO("ERRORE: La longitudine non può assumere il valore 'null', verificare la correttezza del file.plan");
                            ROS_INFO("ERRORE: La longitudine del waypoint numero %d sara' impostata al valore di home", j + 1);
                            wayoint_array_[j].y_long = planned_home_pos_[1];
                        }
                        else
                            wayoint_array_[j].y_long = item_parameter.get<double>();
                    }
                    if (k == 6)
                    {
                        if (item_parameter.is_null())
                        {
                            ROS_INFO("L'altitudine non può assumere il valore 'null', verificare la correttezza del file.plan");
                            ROS_INFO("ERRORE: L'altitudine del waypoint numero %d sara' impostata al valore corretto: 0", j + 1);
                            wayoint_array_[j].z_alt = 0;
                        }
                        else
                            wayoint_array_[j].z_alt = item_parameter.get<double>();
                    }
                    k++;
                }
                k = 0;
                j++;
            }
            j = 0;
            //SETTAGGIO FLAG DI FILE LETTO E PRINT DEI DATI RILEVANTI SULLA CONSOLE
            file_read = true;
            ROS_INFO("/--------------- PLAN FILE --------------- */");
            ROS_INFO_STREAM("TIPO DI FILE: " << file_type);
            ROS_INFO_STREAM("GROUND STATION: " << ground_station);
            if (vachile_type == 10)
                ROS_INFO("TIPO DI VEICOLO: Rover");
            else
            {
                ROS_INFO("TIPO DI VEICOLO: Sconosciuto");
            }
            ROS_INFO("NUMERO WAYPOINTS: %d", number_of_waypoints);
            ROS_INFO("/------------------------------------------ */");
        }
        //LA CONDIZIONE SOTTOSTANTE E' VERIFICATA SOLO QUANDO VIENE RICHIESTO UN WAYPOINT DAL MOVEMENT_MANAGER
        //CIO' ACCADE AD INIZIO MISSIONE E OGNI VOLTA CHE SI RAGGIUNGE UN WAYPOINT
        if (waypoint_pub_flag_)
        {
            //LA CONDIZIONE j!=0 SERVE PER EVITARE CHE SI ENTRI ALL'INTERNO DELL'IF QUANDO SI RAGGIUNGE LA POSIZIONE
            //DI HOME AD INIZIO MISSIONE
            //IL PARAM1 DI OGNI WAYPOINT INDICA IL TEMPO DI SOSTA PER L'EROGAZIONE SE E' NULLO NON SI ENTRA
            //INOLTRE PER ENTRARE NELL'IF E' NECESSARIO CHE L'INDICE j SIA INFERIORE AL NUMERO DI WAYPOINT (CONDIZIONE DI MISSIONE IN CORSO)
            if (wayoint_array_[j].param1 != 0 && j < number_of_waypoints && j != 0)
            {
                ros::Rate loop_rate(20);
                ROS_INFO("Erogazione fitofarmaco in corso tempo restante: %.0f", wayoint_array_[j].param1);
                while (count_state_time_ != wayoint_array_[j].param1)
                {
                    count_loop_++;

                    if (count_loop_ == 20)
                    {
                        count_state_time_++;
                        count_loop_ = 0;
                        if(wayoint_array_[j].param1 - count_state_time_ != 0)
                            ROS_INFO("Erogazione fitofarmaco in corso tempo restante: %.0f", (wayoint_array_[j].param1 - count_state_time_));
                        else
                             ROS_INFO("Erogazione Completata");
                    }
                    loop_rate.sleep();
                }
                count_loop_ = 0;
                count_state_time_ = 0;
            }
            //QUANDO IL MOVEMENT_MANAGER RICHIEDE UN WAYPOINT, QUESTO SARA' PUBBLICATO DAL SUPERVISORE SOLO SE CI SONO ANCORA
            //WAYPOINTS DA PUBBLICARE
            if (j < number_of_waypoints)
            {
                waypoint_pub_.publish(wayoint_array_[j]);
                ROS_INFO("/-------------------- PUBBLICAZIONE -------------------- */");
                ROS_INFO("Waypoint %d pubblicato [LAT: %.8f, LONG: %.8f]", j + 1, wayoint_array_[j].x_lat, wayoint_array_[j].y_long);
                ROS_INFO("/------------------------------------------------------- */");
                j++;
            }
            //SE NON CI SONO ULTERIORI WAYPOINTS DA PUBBLICARE LA MISSIONE E' FINITA, IL ROVER PU0' RIPORTARSI ALLA POSIZIONE DI HOME
            else
            {
            
                if (!homed)
                {
                    ROS_INFO("/--------------------- MISSIONE COMPLETATA ---------------------- */");
                    ROS_INFO("TUTTI I WAYPOINT SONO STATI PUBBLICATI, RITORNO ALLA HOME");
                    ROS_INFO("/---------------------------------------------------------------- */");
                    waypoint_home_.param4 = 1;
                    
                    waypoint_pub_.publish(waypoint_home_);
                    homed = true;
                }
            }
            waypoint_pub_flag_ = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/* ************************************************************** */
/* -- FUNCTIONS DEFINITIONS ------------------------------------- */

/*
exists
FUNZIONE CHE VERIFICA LA PRESENZA DI UNA key (OGGETTO) ALL'INTERNO DEL FILE.PLAN
*/
bool exists(const json &j, const std::string &key)
{
    return j.find(key) != j.end();
}
/*
waypoint_request_function
CALLBACK FUNCTION DEL SERVIZIO WAYPOINT REQUEST, CHIAMATA OGNI VOLTA CHE IL MOVEMENT_MANAGER
RICHIEDE UN WAYPOINT
*/
bool waypoint_request_function(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    waypoint_pub_flag_ = req.data;
    res.success = true;
    res.message = "Done!";

    return true;
}
/*
home_request_function
CALLBACK FUNCTION DEL SERVIZIO HOME_REQUEST, CHIAMATA NEL MOMENTO IN CUI IL MOVEMENT MANAGER 
RICHIEDE LE COORDINATE DI HOME
*/
bool home_request_function(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    home_pub_flag_ = req.data;

    home_coordinate_pub_.publish(waypoint_home_);
    ROS_INFO("Coordinate della posizione di HOME pubblicate [LAT: %.8f, LONG: %.8f]", waypoint_home_.x_lat, waypoint_home_.y_long);
    res.success = true;
    res.message = "Done!";

    return true;
}

/*
waypoint_reached_function
CALLBACK FUNCTION DEL SERVIZIO WAYPOINT_REACHED, A SECONDA DEL VALORE DELL'INDICE j E' POSSIBILE 
STAMPARE IN CONSOLE MESSAGGI DIVERSI
QUANDO LA MISSIONE E IL RITORNO ALLA POSIZIONE DI HOME SONO COMPLETATI SI RESETTA IL FLAG DI FILE LETTO
PER POTER CARICARE UN NUOVO FILE
*/
bool waypoint_reached_function(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    static int n;

    if (req.data)
    {
        if (j == 0)
        {
            ROS_INFO("/---- POSIZIONAMENTO ALLA HOME COMPLETATO ----/");
        }
        else if (j <= number_of_waypoints)
        {

            ROS_INFO("/--- WAYPOINT %d RAGGIUNTO ---/", j);
            if (j == number_of_waypoints)
                j++;
        }
        else if (j > number_of_waypoints)
        {
            ROS_INFO("/---- RIENTRO ALLA POSIZIONE DI HOME AVVENUTO CON SUCCESSO ----/");
            file_read = false;
        }
    }

    res.success = true;
    res.message = "Done!";

    return true;
}
