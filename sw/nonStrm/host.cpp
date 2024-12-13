#include <CL/cl_ext_xilinx.h> 

#include "host.h"

#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>

// XRT includes
#include "xrt/xrt_bo.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_kernel.h"

#define DATA_SIZE 4096
#define ERROR_TOLERANCE 0.4
#define DEVICE_ID 0
#define MAX_SIZE_STREAM 10

// Load pointcloud values
void loadPointCloud(const std::string &filename, std::vector<point3d> &points, short &num_points);
void loadMap(const std::string &filename, MapCell map[MAP_WIDTH*MAP_HEIGHT]);
void loadGoldenOutput(const std::string &filename, std::map<std::pair<idx, idx>, float> &in);
xrt::bo ptCloudBuffer(xrt::device &device, std::vector<point3d> &point_cloud, 
    idx num_points, xrt::kernel &kernel, short id);
xrt::bo mapBuffer(xrt::device &device, MapCell map[MAP_WIDTH*MAP_HEIGHT], 
    idx num_points, xrt::kernel &kernel, idx id);
void writeMapToHost(xrt::bo &bo, MapCell map[MAP_WIDTH*MAP_HEIGHT]);
void saveMap(const std::string &filename, MapCell map[MAP_WIDTH*MAP_HEIGHT]);
bool testOutput(MapCell map[MAP_WIDTH*MAP_HEIGHT], 
                   std::vector<point3d> points, 
                   const std::map<std::pair<idx, idx>, float> &golden_output,
                   idx num_points, pose_t tolerance);

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <xclbin_file> <pointcloud_file> <map_file> <output_file>" << std::endl;
        return 1;
    }

    // Parse command-line arguments
    std::string binary_path = argv[1];
    std::string pointcloud_file = argv[2];
    std::string map_file = argv[3];
    std::string golden_output_file = argv[4];

    idx num_points = 0;  
    std::vector<point3d> point_cloud;  
    MapCell map[MAP_WIDTH*MAP_HEIGHT];

    std::cout << "Open the device " << DEVICE_ID << std::endl;
    auto device = xrt::device(DEVICE_ID);
    std::cout << "Load the xclbin " << binary_path << std::endl;
    auto uuid = device.load_xclbin(binary_path); 
    // auto kernel = xrt::kernel(device, uuid, "kalman_filter");  

    // // Load point cloud, map data, and golden output
    // std::cout << "Loading Point File" << std::endl;    
    // loadPointCloud(pointcloud_file, point_cloud, num_points); 
    // std::cout << "Loading Map File" << std::endl;  
    // loadMap(map_file, map); 
    // std::cout << "Loading Test File" << std::endl;  
    // std::map<std::pair<idx, idx>, float> golden_output;
    // loadGoldenOutput(golden_output_file, golden_output);
    // // 

    // // // Allocate Buffer in Global Memory
    // std::cout << "Allocating Pointclud buffer" << std::endl; 
    // xrt::bo point_cloud_bo = ptCloudBuffer(device, point_cloud, num_points, kernel, 0);
    // std::cout << "Allocating Map buffer" << std::endl;    
    // xrt::bo map_bo = mapBuffer(device, map, num_points, kernel, 1);

    // // Buffer for num_points
    // std::cout << "Allocating point size buffer" << std::endl; 
    // auto num_bo = xrt::bo(device, sizeof(short), kernel.group_id(2));
    // auto num_ptr = num_bo.map<short*>(); 
    // *num_ptr = num_points;
    // num_bo.write(num_ptr);
    // num_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);  

    // // // Execute kernel
    // std::cout << "Executing kernel" << std::endl;  
    // auto run = kernel(point_cloud_bo, map_bo, num_points);   
    // run.wait(); 

    // // // Retrieve and save the output map
    // std::cout << "Writing new map to host" << std::endl;  
    // writeMapToHost(map_bo, map);
    // std::cout << "Saving Map" << std::endl;  
    // std::string output_map_file = "updt_map.txt";
    // saveMap(output_map_file, map);

    // std::cout << "Kernel execution complete. Output map written to " << output_map_file << std::endl;

    // bool test_pass = testOutput(map, point_cloud, golden_output, num_points, ERROR_TOLERANCE);

    // return test_pass;
    return 0;
}

// Load pointcloud values
void loadPointCloud(const std::string &filename, std::vector<point3d> &points, idx &num_points) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open point cloud file: " << filename << std::endl;
        exit(1);
    }

    points.clear();
  
    // axis_data axi_pc;
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        point3d point;
        if (!(iss >> point.x >> point.y >> point.z)) {
            std::cerr << "Error reading line in point cloud file: " << line << std::endl;
            continue;
        }
        
        points.push_back(point);
//         axi_pc.data = point;
//         point_cloud.write(axi_pc);
    }
    num_points = points.size(); 
    infile.close();
}

// Load map values
void loadMap(const std::string &filename, MapCell map[MAP_WIDTH*MAP_HEIGHT]) {
    std::ifstream infile(filename);

    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open map file: " << filename << std::endl;
        exit(1);
    }

    std::string line;
    while(std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string token;

        // Parse the x, y, height, and variance values
        idx x, y;
        double height, variance;

        std::getline(iss, token, ','); x = std::stoi(token);
        std::getline(iss, token, ','); y = std::stoi(token);
        std::getline(iss, token, ','); height = std::stod(token);
        std::getline(iss, token, ','); variance = std::stod(token);

        // Assign the values to the corresponding map cell
        if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
            map[x*MAP_WIDTH+y].height = height;
            map[x*MAP_WIDTH+y].variance = variance;
        }
        else {
            std::cerr << "Warning: Out-of-bounds map cell (" << x << ", " << y << ") in map file." << std::endl;
        }
    }

    infile.close();
}

// Load pointcloud data ot a buffer
xrt::bo ptCloudBuffer(xrt::device &device, std::vector<point3d> &point_cloud, 
    idx num_points, xrt::kernel &kernel, short id) {
    size_t buffer_size = num_points * sizeof(point3d);
    auto point_cloud_bo = xrt::bo(device, buffer_size, kernel.group_id(id));

    // // Map host data to the buffer
    auto host_ptr = point_cloud_bo.map<point3d*>(); 
    for(auto i = 0; i < num_points; i++){
        host_ptr[i] = point_cloud.at(i);         
    }  
    point_cloud_bo.write(host_ptr);
    // Sync data to device memory
    point_cloud_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);    
    return point_cloud_bo;
}

// Load map data ot a buffer
xrt::bo mapBuffer(xrt::device &device, MapCell map[MAP_WIDTH*MAP_HEIGHT], 
    idx num_points, xrt::kernel &kernel, short id) {
    size_t buffer_size = sizeof(MapCell) * MAP_WIDTH * MAP_HEIGHT;
    auto map_bo = xrt::bo(device, buffer_size, kernel.group_id(id));

    // Map host data to the buffer
    auto host_ptr = map_bo.map<MapCell*>();
    for (idx i = 0; i < MAP_WIDTH; ++i) {
        for (idx j = 0; j < MAP_HEIGHT; ++j) {
            host_ptr[i * MAP_WIDTH + j] = map[i*MAP_WIDTH+j];
        }
    }
    map_bo.write(host_ptr);
    // Sync data to device memory
    map_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    return map_bo;
}

// 
void writeMapToHost(xrt::bo &bo, MapCell map[MAP_WIDTH*MAP_HEIGHT]) {
    // Sync device buffer to host
    bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE); 

    auto bo_data = bo.map<MapCell*>();
    bo.read(bo_data);
    for (idx i = 0; i < MAP_WIDTH; ++i) {
        for (idx j = 0; j < MAP_HEIGHT; ++j) {
            map[i*MAP_WIDTH+j].height = bo_data[i * MAP_WIDTH + j].height; 
            map[i*MAP_WIDTH+j].variance = bo_data[i * MAP_WIDTH + j].variance; 
        }
    }
}

// 
void saveMap(const std::string &filename, MapCell map[MAP_WIDTH*MAP_HEIGHT]) {
    std::ofstream outfile(filename);

    if (!outfile.is_open()) {
        std::cerr << "Error: Cannot open output file: " << filename << std::endl;
        exit(1);
    }

    for (idx i = 0; i < MAP_WIDTH; ++i) {
        for (idx j = 0; j < MAP_HEIGHT; ++j) {
            outfile << i << "," << j << ","
            << map[i*MAP_WIDTH+j].height << ", " 
            << map[i*MAP_WIDTH+j].variance << "\n";
        }
    }

    outfile.close();
}

// 
void loadGoldenOutput(const std::string &filename, std::map<std::pair<idx, idx>, float> &cells_out) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Failed to open golden output file: " << filename << std::endl;
        exit(1);
    }

    idx key_x, key_y;
    float height_cell;
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);

        if (!(iss >> key_x >> key_y >> height_cell)) {
            std::cerr << "Error reading line in golden output file: " << line << std::endl;
            continue;
        }

        cells_out[{key_x, key_y}] = height_cell;
    }
    infile.close();
}

// 
bool testOutput(MapCell map[MAP_WIDTH*MAP_HEIGHT], 
                   std::vector<point3d> points, 
                   const std::map<std::pair<idx, idx>, float> &golden_output, 
                   idx num_points, pose_t tolerance){

    bool test_pass = true;

    for (auto i = 0; i < num_points; i++) {
        point3d point = points.at(i);
        idx cell_x = ceil(static_cast<float>(point.x) * CELL_SIZE);
        idx cell_y = ceil(static_cast<float>(point.y) * CELL_SIZE);

        auto key = std::make_pair(cell_x, cell_y);
        if (golden_output.find(key) != golden_output.end()) {

            if (abs(map[cell_x*MAP_WIDTH+cell_y].height - golden_output.at(key)) > tolerance) {
                std::cout << "\nWrong height for CELL(" << cell_x << ", " << cell_y << ")"
                          << "\nComputed: " << map[cell_x*MAP_WIDTH+cell_y].height
                          << "\nExpected: " << golden_output.at(key) << "\n";
                test_pass = false;
            }
        }
    }

    // Output results
    if (test_pass) {
        std::cout << "\nTest PASSED: Map updated correctly within tolerance " << tolerance << std::endl;
    } else {
        std::cout << "\nTest FAILED: Map updates incorrect." << std::endl;
    }

    return test_pass;
}