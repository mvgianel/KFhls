#include <hls_math.h>
#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_stream.h>
#include <ap_fixed.h>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#define STATE_DIM 3

// typedef ap_fixed<32, 16> pose_t;
// typedef ap_fixed<32, 16> noise_t;
// typedef ap_fixed<16, 8> dat_t;
typedef ap_fixed<32, 16> pose_t;
typedef ap_fixed<32, 16> noise_t;
typedef ap_fixed<16, 8> dat_t;


// Sensor and map configurations
constexpr int MAP_CELLS_W = 10;    // Map cell sections width in meters
constexpr int MAP_CELLS_H = 10;   // Map cell sections height
constexpr int CELL_SIZE = 10;      // Cell divides meters into CELL_SIZE partitions
constexpr int MAP_WIDTH = MAP_CELLS_W * CELL_SIZE;    // Map width 
constexpr int MAP_HEIGHT = MAP_CELLS_H * CELL_SIZE;    // Map height


typedef struct { pose_t x, y, z; } Point3D;

// Set sensor to map transformation and noise variance
const Point3D sensor_to_map_translation = {0.0, 0.0, 0.5};
const noise_t sensor_noise_variance = 0.01;


struct MapCell {
	pose_t height;
	noise_t variance;
};

void process_pointcloud(hls::stream<Point3D> &point_cloud, MapCell map[MAP_WIDTH][MAP_HEIGHT]);

#endif