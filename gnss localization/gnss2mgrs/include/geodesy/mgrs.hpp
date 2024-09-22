#ifndef GEODESY_MGRS_H
#define GEODESY_MGRS_H

#include "geodesy/utm.h"
#include <cmath>
#include <string>

namespace geodesy {

class MGRSPoint {
public:
    std::string gridZone;  // Grid Zone Designator (GZD)
    std::string squareID;  // 100km Square Identifier
    int easting;           // Easting value within the 100km square
    int northing;          // Northing value within the 100km square

    // Constructor from UTMPoint
    MGRSPoint(const UTMPoint& utm_point) {
        gridZone = std::to_string(utm_point.zone) + getLatitudeBand(utm_point.northing);
        squareID = get100kmSquareID(utm_point.zone, utm_point.easting, utm_point.northing);
        easting = static_cast<int>(fmod(utm_point.easting, 100000.0));
        northing = static_cast<int>(fmod(utm_point.northing, 100000.0));
    }

    // Convert MGRS point to string representation
    std::string toString() const {
        return gridZone + " " + squareID + " " + std::to_string(easting) + " " + std::to_string(northing);
    }

private:
    // Calculate latitude band based on northing value
    char getLatitudeBand(double northing) {
        const std::string bands = "CDEFGHJKLMNPQRSTUVWX";
        int bandIndex = static_cast<int>((northing / 10000000.0) + 10); // Normalize to correct band
        return bands[bandIndex];
    }

    // Calculate the 100km square ID based on the zone, easting, and northing
    std::string get100kmSquareID(int zone, double easting, double northing) {
        const std::string e100k = "ABCDEFGHJKLMNPQRSTUV";  // Easting 100km grid squares
        const std::string n100k = (zone % 2 == 0) ? "FGHJKLMNPQRSTUVABCDE" : "ABCDEFGHJKLMNPQRSTUV";  // Northing 100km grid squares

        int eIndex = static_cast<int>(easting / 100000.0) % 20;
        int nIndex = static_cast<int>(northing / 100000.0) % 20;

        return std::string(1, e100k[eIndex]) + std::string(1, n100k[nIndex]);
    }
};

} 

#endif  
