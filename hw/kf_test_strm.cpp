#include "kalman_filter_strm.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "hls_math.h"
#include <map>
#include <cmath>
#include <vector>

// Define tolerance for verification
// Had to make tolerance bigger when changed bits of types
const pose_t HEIGHT_TOLERANCE = 0.4;

void getMapData(MapCell *map, std::string filename){
   // Load file with pointcloud data
    std::ifstream map_file(filename);
    if (!map_file.is_open()) {
        std::cerr << "Failed to open map file!" << std::endl;
       exit(1);
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
            map[x * MAP_WIDTH + y].height = height;
            map[x * MAP_WIDTH + y].variance = variance;
        }
        else {
            std::cerr << "Warning: Out-of-bounds map cell (" << x << ", " << y << ") in map file." << std::endl;
        }
    }

    map_file.close();
}

void getPointCloud(std::string filename, pt_cloud& point_cloud_stream, std::vector <point3d> &points, idx &count){
    // Load file with pointcloud data
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open point cloud data file!" << std::endl;
        exit(1);
    }

    std::string line;
    axis_pt ax_pt;
    count = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        point3d point;
        if (!(iss >> point.x >> point.y >> point.z)) {
            std::cerr << "Error reading line in point cloud file: " << line << std::endl;
            continue;
        }

        points.push_back(point);
        count++;
    }
    
    for(idx i =0; i < count; i++){
        ax_pt.data = points.at(i);
        if(i == count - 1){
            ax_pt.last = 1;             
        } else{
            ax_pt.last = 0;
        }
        point_cloud_stream.write(ax_pt);
    }

    file.close();
}

void getGoldenOut(std::string filename, std::map<std::pair<int, int>, pose_t> &cells_out){
        // Load file with pointcloud data
    std::ifstream out_file(filename);
    if (!out_file.is_open()) {
        std::cerr << "Failed to open point cloud data file!" << std::endl;
        exit(1);
    }

    int key_x;
    int key_y;
    float height_cell;
    std::string line;
    while (std::getline(out_file, line)) {
        std::istringstream iss(line);
        point3d point;
        if (!(iss >> key_x >> key_y >> height_cell)) {
            std::cerr << "Error reading golden output file: " << line << std::endl;
            continue;
        }

        cells_out[{key_x, key_y}] = height_cell;
    }

    out_file.close();
}

int main() {
    MapCell map[MAP_WIDTH * MAP_HEIGHT];

    // map file has rows with x and y positions followed by height and variance
    getMapData(map, "map1.txt");

    // pointcloud file has points' x y and z position
    pt_cloud point_cloud_stream;
    #pragma HLS STREAM depth=16 variable=point_cloud_stream
    std::vector <point3d> points;
    idx point_num;
    getPointCloud("pointcloud_data.txt", point_cloud_stream, points, point_num);

    std::map<std::pair<int, int>, pose_t> cells_out;
    getGoldenOut("golden_output.txt", cells_out);

    // Call the map update function with the point cloud 
    kalman_filter(point_cloud_stream, map); 
    // Output the updated height and variance of each affected cell
    std::cout << "Updated map after processing point cloud:" << std::endl;
    for (int i = 0; i < point_num; i++) {
        idx cell_x = hls::ceil(static_cast<float>(points[i].x) * CELL_SIZE);
        idx cell_y = hls::ceil(static_cast<float>(points[i].y) * CELL_SIZE);
            std::cout << cell_x * MAP_WIDTH + cell_y << "\t"; 

        // Print only updated cells 
        std::cout << "Cell (" << cell_x << ", " << cell_y << "): "
            << "Height = " << map[cell_x * MAP_WIDTH + cell_y].height
            << ", Variance = " << map[cell_x * MAP_WIDTH + cell_y].variance << std::endl;
    }
   
    // Check each point for reasonable updates
    bool test_pass = true;
    for (int i = 0; i < point_num; i++) {
        int cell_x = hls::ceil(static_cast<float>(points[i].x) * CELL_SIZE);
        int cell_y = hls::ceil(static_cast<float>(points[i].y) * CELL_SIZE);

        // Check that cell has been updated reasonably based on expected values
        if (hls::abs(map[cell_x* MAP_WIDTH + cell_y].height - cells_out[{cell_x, cell_y}]) > HEIGHT_TOLERANCE) {
            std::cout << "\nWrong height! \n"
                << "CELL(" << cell_x << ")(" << cell_y << ")"
                << " height: " << map[cell_x * MAP_WIDTH + cell_y].height
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
                << map[i * MAP_WIDTH + j].height << ","
                << map[i * MAP_WIDTH + j].variance << "\n";
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
