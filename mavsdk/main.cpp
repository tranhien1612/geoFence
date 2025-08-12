#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "json.hpp"
#include "geojson.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mavlink_direct/mavlink_direct.h>

std::shared_ptr<mavsdk::Action> g_action;
float warning_distance = 0.5; //km
float break_distance = 0.1;
int current_mode = -1;
int old_mode = -1;

/* --------------------------- Geo ---------------------------*/
std::string folderPath = "/home/ubuntu/src/geoFence/geoData/countries/";
std::string fencePath = "/home/ubuntu/src/geoFence/geoData/fence/";
std::atomic<bool> hasGPS(false);
std::atomic<bool> rtkSent(false);
std::vector<PolygonData> polygons;
PolygonData nearestPolygon;
Point gpsRaw;

void send_rtk(){
    if(rtkSent.load(std::memory_order_acquire) == false){
        auto result = g_action->return_to_launch();
        if (result != mavsdk::Action::Result::Success) {
            std::cerr << "Failed to enter RTL mode: " << result << std::endl;
        }
        std::cout << "Send RTK message!" << std::endl;
        rtkSent.store(true);
    }
}

// void send_mode(){
//     auto result =  g_action->.set_flight_mode(static_cast<mavsdk::Action::FlightMode>(old_mode));   
// }

void checkPointInPolygons(Point gps){
    double minDist;
    bool foundInside;
    PolygonData* poly = findContainingOrNearestPolygon(gps, polygons, minDist, foundInside);

    if(minDist <= break_distance){
        std::cout << "[BREAK] Drone is inside polygon: " << poly->name << ", distance = " << minDist << " km\n";

        if(rtkSent.load(std::memory_order_acquire) == false){
            auto result = g_action->return_to_launch();
            if (result != mavsdk::Action::Result::Success) {
                std::cerr << "Failed to enter RTL mode: " << result << std::endl;
            }
            std::cout << "Change mode to RTK mode!" << std::endl;
            rtkSent.store(true);
        }

    }else if(minDist > break_distance && minDist <= warning_distance){
        std::cout << "[Warning] Drone is near polygon: " << poly->name << ", distance = " << minDist << " km\n";
        if(rtkSent.load(std::memory_order_acquire) == true){
            rtkSent.store(false);
        }
        if(old_mode != -1 && current_mode == 6){
            auto result = g_action->hold();  
            if (result != mavsdk::Action::Result::Success) {
                std::cerr << "Failed to enter Hold mode: " << result << std::endl;
            }
            std::cout << "Change mode to Hold mode!" << std::endl;
        }
    }else{
        // std::cout << "Nearest polygon: " << poly->name << ", distance = " << minDist << " km\n";
    }
}

void *geo_handle(void* args){
    while (!hasGPS.load(std::memory_order_acquire)){ //wait until has gps data
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // pthread_t tid = pthread_self();
    std::string folderPath = *(std::string *)args;
    auto [filePath, countryName] = pointInCountries(folderPath, gpsRaw);

    if (filePath.empty() || countryName.empty()){
        std::cout << "filePath or countryName is empty.\n";
        pthread_exit(nullptr);
    }

    std::cout << "Position of : " << filePath << ", countryName: " << countryName << std::endl;

    std::string path = fencePath + countryName + ".geojson";
    std::cout << "Read geofence file: " << path << std::endl;
    polygons = readGeoJson(path);
    if (polygons.empty()) {
        std::cout << "No polygon data found or file could not be read.\n";
        pthread_exit(nullptr);
    }

    while(1){
        checkPointInPolygons(gpsRaw); //in: 15.973020, 106.137830, out: 15.932643, 109.950595
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return nullptr;
}
/* --------------------------- Geo ---------------------------*/

void usage(const std::string& bin_name){
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP server: tcpin://<our_ip>:<port>\n"
              << " For TCP client: tcpout://<remote_ip>:<port>\n"
              << " For UDP server: udp://<our_ip>:<port>\n"
              << " For UDP client: udp://<remote_ip>:<port>\n"
              << " For Serial : serial://</path/to/serial/dev>:<baudrate>]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14550\n";
}

std::string mode2str(int mode){
    std::string str;
    switch(mode){
        case 0: 
            str = "STABILIZE"; break;
        case 1: 
            str = "ACRO"; break;
        case 2: 
            str = "ALT_HOLD"; break;
        case 3: 
            str = "AUTO"; break;
        case 4: 
            str = "GUIDED"; break;
        case 5: 
            str = "LOITER"; break;
        case 6: 
            str = "RTL"; break; 
        case 7: 
            str = "CIRCLE"; break;
        case 8: 
            str = "POSITION"; break;
        case 9: 
            str = "LAND"; break;
        case 10: 
            str = "OF_LOITER"; break;
        case 11: 
            str = "DRIFT"; break;
        case 13: 
            str = "SPORT"; break;
        case 14: 
            str = "FLIP"; break;
        case 15: 
            str = "AUTOTUNE"; break;
        case 16: 
            str = "POSHOLD"; break;
        case 17: 
            str = "BRAKE"; break;
        case 18: 
            str = "THROW"; break;
        case 19: 
            str = "AVOID_ADSB"; break;
        case 20: 
            str = "GUIDED_NOGPS"; break;
        case 21: 
            str = "SMART_RTL"; break;
        case 22: 
            str = "FLOWHOLD"; break;
        case 23: 
            str = "FOLLOW"; break;
        case 24: 
            str = "ZIGZAG"; break;
        case 25: 
            str = "SYSTEMID"; break;
        case 26: 
            str = "AUTOROTATE"; break;
        case 27: 
            str = "AUTO_RTL"; break;
        default:
            str = "Unknown";
    }
    return str;
}

int extract_value(const std::string& json_str, const std::string& key) {
    auto pos = json_str.find("\"" + key + "\":");
    if (pos == std::string::npos) return -1;
    pos += key.size() + 3; // skip over "key":
    size_t end_pos = json_str.find_first_of(",}", pos);
    return std::stoi(json_str.substr(pos, end_pos - pos));
}

void handle_message(const mavsdk::MavlinkDirect::MavlinkMessage& message){
    // std::cout << "** " << message.message_name << " **\n";
    // std::cout << message.fields_json << '\n';

    // std::cout << "MessageName: " << message.message_name << ", system_id: " << message.system_id << 
    //     ", component_id: " << message.component_id << ", target_system: " << message.target_system << 
    //     ", target_component: " << message.target_component << std::endl;
    
    int msgId = extract_value(message.fields_json, "message_id");

    switch(msgId){
        case 0: {//heartbeat
            // int base_mode = extract_value(message.fields_json, "base_mode");
            int custom_mode = extract_value(message.fields_json, "custom_mode");
            // std::cout << "Base mode: " << mode2str(base_mode) << ", custom_mode: " << mode2str(custom_mode) << std::endl;
            if (custom_mode != current_mode) {
                old_mode = current_mode;       // only update when mode changes
                current_mode = custom_mode;
                std::cout << "Mode changed!" << std::endl;
                std::cout << "Old mode: " << mode2str(old_mode) << ", New mode: " << mode2str(current_mode) << std::endl;
            }
            break;
        }
        case 24: {//gps
            double lat = (double)extract_value(message.fields_json, "lat") / 1e7;
            double lon = (double)extract_value(message.fields_json, "lon") / 1e7;
            // double alt = (double)extract_value(message.fields_json, "alt") / 1000.0 ;
            // std::cout << "Lat: " << lat << ", Lon: " << lon << ", Alt: " << alt << std::endl;
            gpsRaw.lat = lat;
            gpsRaw.lon = lon;
            if(gpsRaw.lat != 0.0 && gpsRaw.lon != 0 && hasGPS.load(std::memory_order_acquire) == false){
                hasGPS.store(true, std::memory_order_release);
            }

            break;
        }
        default:
            break;
    }

}

void handle_position(mavsdk::Telemetry::Position position){
    std::cout << "Vehicle is at lat: " << position.latitude_deg << ", lon: " << position.longitude_deg << 
        ", absolute_altitude_m: " << position.absolute_altitude_m << ", relative_altitude_m: " << position.relative_altitude_m << std::endl;
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        // return 1;
    }

    // Initialize MAVSDK with GroundStation component type
    mavsdk::Mavsdk mavsdk{
        mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}
    };

    // Add connection
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection("udpin://0.0.0.0:14550"); //argv[1]
    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    // Wait for the system to connect
    std::cout << "Waiting for system to connect..." << std::endl;
    while (mavsdk.systems().size() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Get the connected system
    auto system = mavsdk.systems().at(0);
    if (!system->is_connected()) {
        std::cerr << "System not connected" << std::endl;
        return 1;
    }
    g_action = std::make_shared<mavsdk::Action>(system);

    pthread_t geotThread; 
    if (pthread_create(&geotThread, nullptr, geo_handle, &folderPath) != 0) {
        perror("pthread_create failed");
        return -1;
    }

    // Instantiate the plugin
    auto mavlink_direct = mavsdk::MavlinkDirect{system};
    auto hb_handle = mavlink_direct.subscribe_message("HEARTBEAT", handle_message);
    auto gps_handle = mavlink_direct.subscribe_message("GPS_RAW_INT", handle_message);

    // auto telemetry = mavsdk::Telemetry{system};
    // telemetry.subscribe_position(handle_position);


    // auto action = mavsdk::Action{system};
    // auto result = action.return_to_launch();
    // if (result != mavsdk::Action::Result::Success) {
    //     std::cerr << "Failed to enter RTL mode: " << result << std::endl;
    // }

    std::cout << "Listening for GPS messages." << std::endl;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Unsubscribe from GPS updates
    mavlink_direct.unsubscribe_message(hb_handle);
    mavlink_direct.unsubscribe_message(gps_handle);
    std::cout << "Unsubscribed from GPS updates, exiting." << std::endl;

    pthread_cancel(geotThread);
    pthread_join(geotThread, nullptr);
    return 0;
}