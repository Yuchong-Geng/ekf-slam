
#pragma once

#include "./common.h"

// Define a LandMark on the map
// Consists of ID, x position, and y position
struct MapPoint{
    uint64_t id;
    float x;
    float y;
};

// Mapper Class
// data: Vector containing MapPoints which represent all landmarks
// initialize: Function to read in landmarks from specified filename
class Mapper {
 public:
    Mapper() {}
    void initialize(const string& filename);
    vector<MapPoint> data;
};

void Mapper::initialize(const string& filename) {
    ifstream in_file(filename, ifstream::in);
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << filename << endl;
        exit(EXIT_FAILURE);
    }

    string line;
    int index = 0;
    while (getline(in_file, line)){
        istringstream ss(line);
        ss >> MapPoint.id;
        ss >> MapPoint.x;
        ss >> MapPoint.y;
        data.push_back(MapPoint)
        if (debug && index <50)
        cout << data.back().id << ": " << data.back().x << ": "
            << data.back().y << endl;
    }
    // TODO: Complete initialize()

    if (in_file.is_open()) {
        in_file.close();
    }
}
