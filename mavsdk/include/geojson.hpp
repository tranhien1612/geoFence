#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <cmath>
#include <limits>
#include <tuple>

#include "json.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

constexpr double EARTH_RADIUS_KM = 6371.0;
struct PolygonData {
    std::string name;
    std::vector<std::vector<double>> coordinates;
};

struct Point {
    double lat;
    double lon;
    // double alt;
};

std::vector<PolygonData> readGeoJson(const std::string& filePath) {
    std::vector<PolygonData> polygons;

    if(!std::filesystem::exists(filePath)){
        std::cout << filePath << " does not exist.\n";
        return polygons;
    }

    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filePath << std::endl;
        return polygons;
    }
    
    json geojson;
    file >> geojson;

    const auto& features = geojson["features"];
    int count = 0;
    for (const auto& feature : features) {
        if (feature["geometry"]["type"] != "Polygon") continue;

        PolygonData poly;
        poly.name = "polygon_" + std::to_string(count++);

        const auto& coords = feature["geometry"]["coordinates"];
        if (!coords.is_array() || coords.empty()) continue;

        for (const auto& point : coords[0]) {
            if (point.size() != 2) continue;
            poly.coordinates.push_back({point[0].get<double>(), point[1].get<double>()});
        }

        polygons.push_back(poly);
    }

    return polygons;
};

void printCoordinatesList(std::vector<PolygonData> polygons){
    for (const auto& poly : polygons) {
        std::cout << "Name: " << poly.name << "\n";
        for (const auto& coord : poly.coordinates) {
            std::cout << "[" << coord[0] << ", " << coord[1] << "] ";
        }
        std::cout << "\n\n";
    }
};

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
};

bool isPointInPolygon(Point p, const std::vector<std::vector<double>>& polygon) {
    int n = polygon.size();
    int count = 0;

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;

        double lat1 = polygon[i][1], lon1 = polygon[i][0];
        double lat2 = polygon[j][1], lon2 = polygon[j][0];

        if ((lat1 > p.lat) != (lat2 > p.lat)) {
            double intersectLon = (lon2 - lon1) * (p.lat - lat1) / (lat2 - lat1) + lon1;
            if (p.lon < intersectLon) {
                count++;
            }
        }
    }

    return (count % 2 == 1);
};

double haversine(Point p1, Point p2) {
    double dLat = deg2rad(p2.lat - p1.lat);
    double dLon = deg2rad(p2.lon - p1.lon);
    p1.lat = deg2rad(p1.lat);
    p2.lat = deg2rad(p2.lat);

    double a = sin(dLat/2)*sin(dLat/2) + cos(p1.lat)*cos(p2.lat)*sin(dLon/2)*sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS_KM * c;
};

double pointToSegmentDistance(Point p, Point p1, Point p2) {
    // Chuyển về không gian phẳng để tính gần đúng
    double x0 = p.lon, y0 = p.lat;
    double x1 = p1.lon, y1 = p1.lat;
    double x2 = p2.lon, y2 = p2.lat;

    double dx = x2 - x1;
    double dy = y2 - y1;
    if (dx == 0 && dy == 0)
        return haversine(p, p1);

    double t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy);
    t = std::max(0.0, std::min(1.0, t));

    double proj_x = x1 + t * dx;
    double proj_y = y1 + t * dy;
    Point temp = {proj_y, proj_x};
    return haversine(p, temp); // đảo lại vì proj_x là lon
;}

double minDistanceToPolygon(Point p, const std::vector<std::vector<double>>& polygon) {
    double minDist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t j = (i + 1) % polygon.size();
        double lon1 = polygon[i][0], lat1 = polygon[i][1];
        double lon2 = polygon[j][0], lat2 = polygon[j][1];
        Point p1 = {lat1, lon1};
        Point p2 = {lat2, lon2};

        double dist = pointToSegmentDistance(p, p1, p2);
        minDist = std::min(minDist, dist);
    }
    return minDist;
};

PolygonData* findContainingOrNearestPolygon(
    Point gps,
    const std::vector<PolygonData>& polygons,
    double& minDistanceOut,
    bool& foundInside
) {
    PolygonData* result = nullptr;
    minDistanceOut = std::numeric_limits<double>::max();
    foundInside = false;

    for (const auto& poly : polygons) {
        if (isPointInPolygon(gps, poly.coordinates)) {
            foundInside = true;
            minDistanceOut = 0.0;
            // std::cout << "Point is inside polygon: " << poly.name << std::endl;
            return new PolygonData(poly);
        }

        double dist = minDistanceToPolygon(gps, poly.coordinates);
        if (dist < minDistanceOut) {
            minDistanceOut = dist;
            // std::cout << "Point is not inside polygon: " << poly.name << ", distance: " << minDistanceOut << std::endl;
            result = new PolygonData(poly);
        }
    }
    return result;
};

std::tuple<std::string, std::string> pointInCountries(std::string folderPath, Point gps){
    if (!fs::exists(folderPath)) {
        std::cout << "Path does not exist: " << folderPath << std::endl;
        return {"", ""};
    }

    for (const auto& entry : fs::directory_iterator(folderPath)) {
        if (entry.path().extension() == ".json") {
            std::string filename = entry.path().string();
            std::vector<PolygonData> polygons = readGeoJson(filename);
            double minDist;
            bool foundInside;
            PolygonData* poly = findContainingOrNearestPolygon(gps, polygons, minDist, foundInside);
            if (foundInside) {
                std::string countryName = entry.path().stem().string();
                return {filename, countryName};
            }
        }
    }
    return {"", ""};
};
