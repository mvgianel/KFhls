#include <hls_math.h>
#include <ap_fixed.h>
#include <ap_int.h>
#include <ap_fixed.h>

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

extern "C"{    

constexpr short FR_BITS = 16;
constexpr short FR_BITS2 = 8;
constexpr short BT_WIDTH = 32;

// #ifdef BUILD_FOR_HARDWARE
// typedef ap_fixed<BT_WIDTH, FR_BITS> pose_t;
// typedef ap_fixed<BT_WIDTH, FR_BITS> noise_t;
// typedef ap_fixed<FR_BITS, FR_BITS2> dat_t;
typedef short idx;
// #else
typedef float pose_t;
typedef float noise_t;
typedef float dat_t;
// #endif

// Sensor and map configurations
constexpr int MAP_CELLS_W = 10;    // Map cell sections width in meters
constexpr int MAP_CELLS_H = 10;   // Map cell sections height
constexpr int CELL_SIZE = 10;      // Cell divides meters into CELL_SIZE partitions
constexpr int MAP_WIDTH = MAP_CELLS_W * CELL_SIZE;    // Map width 
constexpr int MAP_HEIGHT = MAP_CELLS_H * CELL_SIZE;    // Map height

typedef struct {pose_t x,y,z;} point3d;
// typedef hls::axis<point3d, 0,0,0> axis_pt;
// typedef hls::stream<axis_pt> pt_cloud;

// Set sensor to map transformation and noise variance
const noise_t sensor_noise_variance = 0.01;

struct MapCell {
	pose_t height;
	noise_t variance;
    // idx x;
    // idx y;
};

// typedef hls::axis<MapCell, 0,0,0> axi_map;
// typedef hls::stream<axi_map> cell_stm;

extern "C" void kalman_filter(point3d *point_cloud, MapCell *map, short num_points);
}
#endif