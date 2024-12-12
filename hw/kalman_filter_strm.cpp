#include "kalman_filter_strm.h"

// Function to calculate measurement variance for a point (based on Section III-B)
dat_t calculate_measurement_variance(const point3d &point) {
    // TODO  
    // Define Jacobians
    dat_t J_s = hls::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    return sensor_noise_variance * (J_s * J_s);  // Propagate variance using the Jacobian
}

// Kalman Filter update for map cell height estimation
void kalman_update(MapCell &cell, pose_t p_meas, noise_t var_meas) {
    if(cell.variance == -1){
        cell.height = p_meas;
        cell.variance = var_meas;
    } else {
    // Calculate Kalman gain
    noise_t kalman_gain = cell.variance / (cell.variance + var_meas);
    // Update the height estimate
    cell.height = cell.height + kalman_gain * (p_meas - cell.height);
    // Update the variance
    cell.variance = (1 - kalman_gain) * cell.variance;
    }      
}

// void read_map(MapCell local_map[MAP_HEIGHT*MAP_WIDTH], MapCell *map_in){
//     // Copy data from map_in (PS) to BRAM
//     for (int i = 0; i < MAP_HEIGHT; i++) {
//         for (int j = 0; j < MAP_WIDTH; j++) {
//             local_map[i * MAP_WIDTH + j] = map_in[i * MAP_WIDTH + j];
//         }
//     }
// }

// void write_map(MapCell &local_map, MapCell *map_out){
//         // Copy data back to map_out (PS)
//     for (int i = 0; i < MAP_HEIGHT; i++) {
//         for (int j = 0; j < MAP_WIDTH; j++) {
//             map_out[i * MAP_WIDTH + j] = local_map[i*MAP_WIDTH+j];       
//         }
//     }
// }

// Process point cloud and update map
void kalman_filter(pt_cloud& point_cloud, MapCell *map) {
    #pragma HLS INTERFACE m_axi port=map offset=slave depth=1024 bundle=maps 
    #pragma HLS INTERFACE axis port=point_cloud
    // #pragma HLS interface ap_ctrl_none port = return
    // #pragma HLS PIPELINE II=1
    #pragma HLS DATAFLOW
    // #pragma HLS PIPELINE
    // Initialize buffers

    while (!point_cloud.empty()) {  
        #pragma HLS PIPELINE II=1 rewind off
    	// Read a point from the point cloud stream    
        axis_pt ax_pt = point_cloud.read();
        point3d sensor_point = ax_pt.data;   
             
        // Map the 3D map point to a 2D grid cell (x, y) and height measurement (z)
        int cell_x = hls::ceil(sensor_point.x * pose_t(CELL_SIZE));
        int cell_y = hls::ceil(sensor_point.y * pose_t(CELL_SIZE));

        // Ensure cell indices are within bounds
        if (cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT) {

             // Use the z-coordinate as the height measurement
            pose_t p_meas = sensor_point.z;  
            // Calculate measurement variance for this point
            noise_t var_meas = calculate_measurement_variance(sensor_point);
 
            // Apply Kalman filter update
            kalman_update(map[cell_x * MAP_WIDTH + cell_y], p_meas, var_meas);   
        }
        if(ax_pt.last)
            break; 
    }
}
 