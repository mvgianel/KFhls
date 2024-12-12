#include <hls_math.h>
#include <ap_fixed.h>
#include <ap_int.h>
#include <ap_fixed.h>

#ifndef HOST_H
#define HOST_H

extern "C"{    

constexpr short FR_BITS = 16;
constexpr short FR_BITS2 = 8;
constexpr short BT_WIDTH = 32;

typedef float pose_t;
typedef float noise_t;
typedef float dat_t;
typedef short idx;

// Sensor and map configurations
constexpr idx MAP_CELLS_W = 10;    // Map cell sections width in meters
constexpr idx MAP_CELLS_H = 10;   // Map cell sections height
constexpr idx CELL_SIZE = 10;      // Cell divides meters into CELL_SIZE partitions
constexpr idx MAP_WIDTH = MAP_CELLS_W * CELL_SIZE;    // Map width 
constexpr idx MAP_HEIGHT = MAP_CELLS_H * CELL_SIZE;    // Map height

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

// Function to convert float to ap_fixed bit-wise
// pose_t float_to_fixed(float value) {
//     union {
//         float f;
//         uint32_t u;  // Bit-level representation of the float
//     } float_bits;

//     float_bits.f = value;  // Load the float value

//     // Extract components of IEEE 754 float
//     uint32_t sign = (float_bits.u >> 31) & 0x1;
//     int32_t exponent = ((float_bits.u >> 23) & 0xFF) - 127;  // Unbias exponent
//     uint32_t mantissa = float_bits.u & 0x7FFFFF;  // Extract mantissa (23 bits)
//     mantissa |= (1 << 23);  // Add implicit leading 1 for normalized floats

//     // Calculate fixed-point equivalent
//     int32_t fixed_value = mantissa;  // Start with mantissa
//     int shift_amount = exponent - 23;  // Exponent adjustment relative to mantissa bits

//     if (shift_amount > 0) {
//         fixed_value <<= shift_amount;  // Shift left for positive exponent
//     } else {
//         fixed_value >>= -shift_amount;  // Shift right for negative exponent
//     }

//     // Apply sign
//     if (sign) {
//         fixed_value = -fixed_value;
//     }

//     // Scale down to match apose_t
//     pose_t result = fixed_value / std::pow(2.0, 16);  // Adjust fractional bits
//     return result;
// }

void kalman_filter(point3d *point_cloud, MapCell *map, idx num_points);
}
#endif