#include <hls_math.h>
#include <ap_fixed.h>
#include <ap_int.h>
#include <hls_stream.h>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#define STATE_DIM 3

// typedef ap_fixed<32, 16> pose_t;
// typedef ap_fixed<32, 16> noise_t;
// typedef ap_fixed<16, 8> dat_t;
typedef float pose_t;
typedef float noise_t;
typedef float dat_t;
typedef struct { pose_t x, y, z; } Point3D;

// Sensor and map configurations
constexpr int MAP_CELLS_W = 100;    // Map width in cells
constexpr int MAP_CELLS_H = 100;   // Map height in cells
constexpr float CELL_SIZE = 0.1; // Cell size in meters
constexpr int MAP_WIDTH = MAP_CELLS_W / CELL_SIZE;    // Map width in cells
constexpr int MAP_HEIGHT = MAP_CELLS_H / CELL_SIZE;   // Map height in cells

struct MapCell {
	pose_t height;
	noise_t variance;
};

void process_pointcloud(hls::stream<Point3D> &point_cloud, const Point3D &sensor_to_map_translation, 
noise_t sensor_noise_variance, MapCell map[MAP_WIDTH][MAP_HEIGHT]);

#endif
