#include "kalman_filter.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "hls_math.h"

// Define tolerance for verification
const pose_t HEIGHT_TOLERANCE = 0.3;

int main() {
    // Set sensor to map transformation and noise variance
    Point3D sensor_to_map_translation = {0.0, 0.0, 0.5};
    noise_t sensor_noise_variance = 0.1;
    MapCell map[MAP_WIDTH][MAP_HEIGHT];

    // Initialize the height and variance maps to zero
    for (int i = 0; i < MAP_WIDTH; i++) {
        for (int j = 0; j < MAP_HEIGHT; j++) {
        	map[i][j].height = 0.0;
        	map[i][j].variance = 1.0; // Initialize variance to 1.0 for baseline uncertainty
        }
    }

    // Load file with pointcloud data
    std::ifstream file("pointcloud_data.txt");
    if (!file.is_open()) {
		std::cerr << "Failed to open point cloud data file!" << std::endl;
		return 1;
    }

    std::string line;
    bool test_pass = true;
    hls::stream<Point3D> point_cloud_stream;
    Point3D points[10];
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
	}

	file.close();

    // Call the map update function with the point cloud
    process_pointcloud(point_cloud_stream, sensor_to_map_translation, sensor_noise_variance, map);

  for (int i = 0; i < 10; i++){
    // Check that cell has been updated reasonably based on expected values
    if (abs(map[int(points[i].x / CELL_SIZE)][int(points[i].y / CELL_SIZE)].height - points[i].z) > HEIGHT_TOLERANCE) {
        std::cout << "CELL(" <<int(points[i].x / CELL_SIZE) <<")(" << int(points[i].y / CELL_SIZE) << ")" << "height: "
        << map[int(points[i].x / CELL_SIZE)][int(points[i].y / CELL_SIZE)].height << " expected: " << points[i].z << "\n";
        test_pass &= false;
        }
    }

    // Output the updated height and variance of each affected cell
    std::cout << "Updated map after processing point cloud:" << std::endl;
    for (int i = 0; i < MAP_WIDTH; i++) {
        for (int j = 0; j < MAP_HEIGHT; j++) {
            if (map[i][j].height != 0.0 || map[i][j].variance != 1.0) {  // Print only updated cells
                std::cout << "Cell (" << i << ", " << j << "): "
                          << "Height = " << map[i][j].height
                          << ", Variance = " << map[i][j].variance << std::endl;

            }
        }
    }

    // Display test results
    if (test_pass) {
        std::cout << "Test PASSED: Map updated correctly within tolerance." << std::endl;
        return 0;
    } else {
        std::cout << "Test FAILED: Map updates incorrect." << std::endl;
        return 1;
    }

}
