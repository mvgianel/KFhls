#include <CL/cl_ext_xilinx.h> 

#include "kalman_filter.h"

#include <iostream>
#include <cstring>
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
void loadPointCloud(const std::string &filename, point3d *point_cloud, short num_points);
void loadMap(const std::string &filename, MapCell map[MAP_WIDTH][MAP_HEIGHT]);
void loadPointCloud(const std::string &filename, point3d *point_cloud, short num_points);
void loadGoldenOutput(const std::string &filename, std::map<std::pair<int, int>, float> &in);
xrt::bo ptCloudBuffer(xrt::device &device, point3d *point_cloud, 
    short num_points, xrt::kernel &kernel, short id);
xrt::bo mapBuffer(xrt::device &device, MapCell map[MAP_WIDTH][MAP_HEIGHT], 
    short num_points, xrt::kernel &kernel, short id);
void writeMapToHost(xrt::bo &bo, MapCell map[MAP_WIDTH][MAP_HEIGHT]);
void saveMap(const std::string &filename, MapCell map[MAP_WIDTH][MAP_HEIGHT]);
bool testOutput(MapCell map[MAP_WIDTH][MAP_HEIGHT], 
                   point3d *points, 
                   const std::map<std::pair<int, int>, float> &golden_output,
                   short num_points, pose_t tolerance);

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

    std::cout << "Open the device" << DEVICE_ID << std::endl;
    auto device = xrt::device(DEVICE_ID);
    std::cout << "Load the xclbin " << binary_path << std::endl;
    auto uuid = device.load_xclbin(binary_path); 

    auto kernel = xrt::kernel(device, uuid, "kalman_filter", xrt::kernel::cu_access_mode::exclusive);  
    short num_points;  
    point3d *point_cloud = new point3d[DATA_SIZE];  
    MapCell map[MAP_WIDTH][MAP_HEIGHT];
    
    // Load point cloud, map data, and golden output
    std::cout << "Loading Point File" << std::endl;    
    loadPointCloud(pointcloud_file, point_cloud, num_points); 
    std::cout << "Loading Map File" << std::endl;  
    loadMap(map_file, map); 
    std::cout << "Loading Test File" << std::endl;  
    std::map<std::pair<int, int>, float> golden_output;
    loadGoldenOutput(golden_output_file, golden_output);

    // Allocate Buffer in Global Memory
    std::cout << "Allocating Pointclud buffer" << std::endl; 
    // size_t pt_buffer_size = num_points * sizeof(point3d);
    // auto point_cloud_bo = xrt::bo(device, buffer_size, kernel.group_id(id));

    // // // Map host data to the buffer
    // auto host_ptr = point_cloud_bo.map<point3d*>(); 
    // for(auto i = 0; i < num_points; i++){
    //     host_ptr[i] = point_cloud[i];        
    // }
    // Sync data to device memory
    // point_cloud_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);          
    xrt::bo point_cloud_bo = ptCloudBuffer(device, point_cloud, num_points, kernel, 0);
    std::cout << "Allocating Map buffer" << std::endl;    
    xrt::bo map_bo = mapBuffer(device, map, num_points, kernel, 1);

    // Execute kernel
    auto run = kernel(point_cloud_bo, map_bo);
    run.wait(); 

    // Retrieve and save the output map
    writeMapToHost(map_bo, map);
    char output_map_file[] = "updt_map.txt";
    saveMap(output_map_file, map);

    std::cout << "Kernel execution complete. Output map written to " << output_map_file << std::endl;

    bool test_pass = testOutput(map, point_cloud, golden_output, num_points, ERROR_TOLERANCE);

    std::cout << "TEST PASSED\n";

    return test_pass;
    // return 0;
}

// Load pointcloud values
void loadPointCloud(const std::string &filename, point3d *points, short num_points) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open point cloud file: " << filename << std::endl;
        exit(1);
    }

    short count = 0;    
    // axis_data axi_pc;
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        float x,y,z;
        if (!(iss >> x >> y >> z)) {
            std::cerr << "Error reading line in point cloud file: " << line << std::endl;
            continue;
        }
        
        points[count].x = float_to_fixed(x);
        points[count].y = float_to_fixed(y);
        points[count].z = float_to_fixed(z);
        count++;
//         axi_pc.data = point;
//         point_cloud.write(axi_pc);
    }
    num_points = count; 
    infile.close();
}

// Load map values
void loadMap(const std::string &filename, MapCell map[MAP_WIDTH][MAP_HEIGHT]) {
    std::ifstream infile(filename);

    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open map file: " << filename << std::endl;
        exit(1);
    }

    for (int i = 0; i < MAP_WIDTH; ++i) {
        for (int j = 0; j < MAP_HEIGHT; ++j) {
            infile >> map[i][j].height >> map[i][j].variance;
        }
    }
    infile.close();
}

// Load pointcloud data ot a buffer
xrt::bo ptCloudBuffer(xrt::device &device, point3d *point_cloud, 
    short num_points, xrt::kernel &kernel, short id) {
    size_t buffer_size = num_points * sizeof(point3d);
    auto point_cloud_bo = xrt::bo(device, buffer_size, kernel.group_id(id));

    // // Map host data to the buffer
    auto host_ptr = point_cloud_bo.map<point3d*>(); 
    for(auto i = 0; i < num_points; i++){
        host_ptr[i] = point_cloud[i];        
    }

    // Sync data to device memory
    point_cloud_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);    
    return point_cloud_bo;
}

// Load map data ot a buffer
xrt::bo mapBuffer(xrt::device &device, MapCell map[MAP_WIDTH][MAP_HEIGHT], 
    short num_points, xrt::kernel &kernel, short id) {
    size_t buffer_size = sizeof(MapCell) * MAP_WIDTH * MAP_HEIGHT;
    auto map_bo = xrt::bo(device, buffer_size, kernel.group_id(id));

    // Map host data to the buffer
    // auto host_ptr = map_bo.map<MapCell*>();
    // for (int i = 0; i < MAP_WIDTH; ++i) {
    //     for (int j = 0; j < MAP_HEIGHT; ++j) {
    //         host_ptr[i * sizeof(MapCell) + j] = map[i][j];
    //     }
    // }
    map_bo.write(map);
    // Sync data to device memory
    map_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    return map_bo;
}

// 
void writeMapToHost(xrt::bo &bo, MapCell map[MAP_WIDTH][MAP_HEIGHT]) {
    // Sync device buffer to host
    bo.sync(XCL_BO_SYNC_BO_FROM_DEVICE); 
    auto bo_data = bo.map<MapCell*>();

    for (int i = 0; i < MAP_WIDTH; ++i) {
        for (int j = 0; j < MAP_HEIGHT; ++j) {
            map[i][j] = bo_data[i * sizeof(MapCell) + j];
        }
    }
}

// 
void saveMap(const std::string &filename, MapCell map[MAP_WIDTH][MAP_HEIGHT]) {
    std::ofstream outfile(filename);

    if (!outfile.is_open()) {
        std::cerr << "Error: Cannot open output file: " << filename << std::endl;
        exit(1);
    }

    for (int i = 0; i < MAP_WIDTH; ++i) {
        for (int j = 0; j < MAP_HEIGHT; ++j) {
            outfile << map[i][j].height << " " << map[i][j].variance << "\n";
        }
    }

    outfile.close();
}

// 
void loadGoldenOutput(const std::string &filename, std::map<std::pair<int, int>, float> &cells_out) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Failed to open golden output file: " << filename << std::endl;
        exit(1);
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int key_x, key_y;
        float height_cell;

        if (!(iss >> key_x >> key_y >> height_cell)) {
            std::cerr << "Error reading line in golden output file: " << line << std::endl;
            continue;
        }

        cells_out[{key_x, key_y}] = height_cell;
    }
    infile.close();
}

// 
bool testOutput(MapCell map[MAP_WIDTH][MAP_HEIGHT], 
                   point3d *points, 
                   const std::map<std::pair<int, int>, float> &golden_output, 
                   short num_points, pose_t tolerance){

    bool test_pass = true;

    for (auto i = 0; i < num_points; i++) {
        point3d point = points[i];
        int cell_x = ceil(static_cast<float>(point.x) * CELL_SIZE);
        int cell_y = ceil(static_cast<float>(point.y) * CELL_SIZE);

        auto key = std::make_pair(cell_x, cell_y);
        if (golden_output.find(key) != golden_output.end()) {
            pose_t computed_height = map[cell_x][cell_y].height;
            pose_t expected_height = golden_output.at(key);

            if (hls::abs(computed_height - expected_height) > tolerance) {
                std::cout << "\nWrong height for CELL(" << cell_x << ", " << cell_y << ")"
                          << "\nComputed: " << computed_height
                          << "\nExpected: " << expected_height << "\n";
                test_pass = false;
            }
        }
    }

    // Output results
    if (test_pass) {
        std::cout << "***********************************************" << std::endl;
        std::cout << "\nTest PASSED: Map updated correctly within tolerance.\n" << std::endl;
        std::cout << "***********************************************\n" << std::endl;
    } else {
        std::cout << "***********************************************" << std::endl;
        std::cout << "\nTest FAILED: Map updates incorrect.\n" << std::endl;
        std::cout << "***********************************************\n" << std::endl;
    }

    return test_pass;
}