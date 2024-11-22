#include "kalman_filter.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "hls_math.h"
#include <map>
#include <cmath>

// Define tolerance for verification
// Had to make tolerance bigger when changed bits of types
const pose_t HEIGHT_TOLERANCE = 0.4;
const int POINTS = 10;

int main() {
    MapCell map[MAP_WIDTH][MAP_HEIGHT];

    // pointcloud file has points' x y and z position
    // map file has rows with x and y positions followed by height and variance

    // Load file with pointcloud data
    std::ifstream map_file("map1.txt");
    if (!map_file.is_open()) {
        std::cerr << "Failed to open map file!" << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(map_file, line)) {
        std::istringstream iss(line);
        std::string token;

        // Parse the x, y, height, and variance values
        int x, y;
        double height, variance;

        std::getline(iss, token, ','); x = std::stoi(token);
        std::getline(iss, token, ','); y = std::stoi(token);
        std::getline(iss, token, ','); height = std::stod(token);
        std::getline(iss, token, ','); variance = std::stod(token);

        // Assign the values to the corresponding map cell
        if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
            map[x][y].height = height;
            map[x][y].variance = variance;
        }
        else {
            std::cerr << "Warning: Out-of-bounds map cell (" << x << ", " << y << ") in map file." << std::endl;
        }
    }

    map_file.close();

    // Load file with pointcloud data
    std::ifstream file("pointcloud_data.txt");
    if (!file.is_open()) {
        std::cerr << "Failed to open point cloud data file!" << std::endl;
        return 1;
    }

    hls::stream<Point3D> point_cloud_stream;
    Point3D points[POINTS];
    int count = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Point3D point;
        if (!(iss >> point.x >> point.y >> point.z)) {
            std::cerr << "Error reading line in point cloud file: " << line << std::endl;
            continue;
        }

        point_cloud_stream.write(point);
        points[count].x = point.x;
        points[count].y = point.y;
        points[count].z = point.z;
        count++;
    }

    file.close();

    // Load file with pointcloud data
    std::ifstream out_file("golden_output.txt");
    if (!out_file.is_open()) {
        std::cerr << "Failed to open point cloud data file!" << std::endl;
        return 1;
    }

    int key_x;
    int key_y;
    double height_cell;
    std::map<std::pair<int, int>, pose_t> cells_out;
    while (std::getline(out_file, line)) {
        std::istringstream iss(line);
        Point3D point;
        if (!(iss >> key_x >> key_y >> height_cell)) {
            std::cerr << "Error reading golden output file: " << line << std::endl;
            continue;
        }

        cells_out[{key_x, key_y}] = height_cell;
    }

    out_file.close();

    // Call the map update function with the point cloud
    process_pointcloud(point_cloud_stream, map);

    // Output the updated height and variance of each affected cell
    std::cout << "Updated map after processing point cloud:" << std::endl;
    for (int i = 0; i < POINTS; i++) {
        int cell_x = hls::ceil(static_cast<float>(points[i].x) * CELL_SIZE);
        int cell_y = hls::ceil(static_cast<float>(points[i].y) * CELL_SIZE);

        // Print only updated cells 
        std::cout << "Cell (" << cell_x << ", " << cell_y << "): "
            << "Height = " << map[cell_x][cell_y].height
            << ", Variance = " << map[cell_x][cell_y].variance << std::endl;
    }
   
    // Check each point for reasonable updates
    bool test_pass = true;
    for (int i = 0; i < POINTS; i++) {
        int cell_x = hls::ceil(static_cast<float>(points[i].x) * CELL_SIZE);
        int cell_y = hls::ceil(static_cast<float>(points[i].y) * CELL_SIZE);

        // Check that cell has been updated reasonably based on expected values
        if (hls::abs(map[cell_x][cell_y].height - cells_out[{cell_x, cell_y}]) > HEIGHT_TOLERANCE) {
            std::cout << "\nWrong height! \n"
                << "CELL(" << cell_x << ")(" << cell_y << ")"
                << " height: " << map[cell_x][cell_y].height
                << " expected: " << cells_out[{cell_x, cell_y}] << "\n";
            test_pass &= false;
        }
    }

    // Output the updated map to a .txt file
    std::ofstream output_file("updated_map1.txt");
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output file for writing!" << std::endl;
        return 1;
    }

    for (int i = 0; i < MAP_WIDTH; i++) {
        for (int j = 0; j < MAP_HEIGHT; j++) {
            output_file << i << "," << j << ","
                << map[i][j].height << ","
                << map[i][j].variance << "\n";
        }
    }

    output_file.close();

    // Display test results
    if (test_pass) {
        std::cout << "***********************************************" << std::endl;
        std::cout << "\nTest PASSED: Map updated correctly within tolerance.\n" << std::endl;
        std::cout << "***********************************************\n" << std::endl;
        return 0;
    }
    else {
        std::cout << "***********************************************" << std::endl;
        std::cout << "\nTest FAILED: Map updates incorrect.\n" << std::endl;
        std::cout << "***********************************************\n" << std::endl;
        return 1;
    }

}
