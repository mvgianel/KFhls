#include "kalman_filter.h"



// Function that uses IMU data to calculate robot pose, and adapt map based on robot pose

// Function to calculate measurement variance for a point (based on Section III-B)
dat_t calculate_measurement_variance(const point3d &point) {
    // TODO  
    // Define Jacobians
    dat_t J_s = hls::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    return sensor_noise_variance * (J_s * J_s);  // Propagate variance using the Jacobian
}

// Kalman Filter update for map cell height estimation
void kalman_update(MapCell &cell, pose_t p_meas, noise_t var_meas) {
    // Calculate Kalman gain
    noise_t kalman_gain = cell.variance / (cell.variance + var_meas);

    auto temp_height = cell.height;
    auto temp_variance = cell.variance;
    
    // Update the height estimate
    auto new_height = temp_height + kalman_gain * (p_meas - temp_height);
    // Update the variance
    auto new_variance = (1 - kalman_gain) * temp_variance;
}

// Process point cloud and update map
void kalman_filter(point3d *point_cloud, MapCell *map, short num_points) {
    #pragma HLS INTERFACE m_axi port=map offset=slave depth=1024 bundle=gmem 
    #pragma HLS INTERFACE m_axi port=point_cloud offset=slave depth=1024 bundle=gmem
    #pragma HLS INTERFACE s_axilite port=return bundle=control
    #pragma HLS INTERFACE s_axilite port=num_points bundle=control
    #pragma HLS INTERFACE s_axilite port=map bundle=control
    #pragma HLS INTERFACE s_axilite port=point_cloud bundle=control
    // #pragma HLS interface ap_ctrl_none port = return
    // #pragma HLS PIPELINE II=2
    // #pragma HLS DATAFLOW
    // #pragma HLS PIPELINE
    // Initialize buffers
    
    for (short i = 0; i < num_points; i++) {  
        // #pragma HLS PIPELINE II=1 rewind off
        
    	// Read a point from the point cloud stream    
        point3d sensor_point = point_cloud[i];      
             
        // Map the 3D map point to a 2D grid cell (x, y) and height measurement (z)
        int cell_x = hls::ceil(sensor_point.x * pose_t(CELL_SIZE));
        int cell_y = hls::ceil(sensor_point.y * pose_t(CELL_SIZE));

        // Ensure cell indices are within bounds
        if (cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT) {
            MapCell cell = map[cell_x * MAP_WIDTH + cell_y];

             // Use the z-coordinate as the height measurement
            pose_t p_meas = sensor_point.z;  
            // Calculate measurement variance for this point
            noise_t var_meas = calculate_measurement_variance(sensor_point);
 
            // Apply Kalman filter update
            kalman_update(cell, p_meas, var_meas);   
                             
            map[cell_x * MAP_WIDTH + cell_y] = cell;            
        }
    }
}
 