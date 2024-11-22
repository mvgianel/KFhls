#include "kalman_filter.h"

// Function to calculate measurement variance for a point (based on Section III-B)
dat_t calculate_measurement_variance(const Point3D &point) {
    #pragma HLS INLINE
    // TODO  
    // Define Jacobians
    dat_t J_s = hls::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    return sensor_noise_variance * (J_s * J_s);  // Propagate variance using the Jacobian
}

// Kalman Filter update for map cell height estimation
void kalman_update(MapCell &cell, pose_t p_meas, noise_t var_meas) {
    #pragma HLS INLINE
    // Calculate Kalman gain
    noise_t kalman_gain = cell.variance / (cell.variance + var_meas);

    // Update the height estimate
    cell.height = cell.height + kalman_gain * (p_meas - cell.height);

    // Update the variance
    cell.variance = (1 - kalman_gain) * cell.variance;
}

// Process point cloud and update map
void process_pointcloud(hls::stream<Point3D> &point_cloud, MapCell map[MAP_WIDTH][MAP_HEIGHT]) {
    #pragma HLS ARRAY_PARTITION variable=map complete dim=2
    #pragma HLS INTERFACE ap_ctrl_hs port=return
    #pragma HLS INTERFACE axis port=point_cloud
    #pragma HLS INTERFACE bram port=map
    #pragma HLS STREAM depth=8 variable=point_cloud
    #pragma HLS DEPENDENCE variable=map inter false
    #pragma HLS DATAFLOW
    // Initialize buffers

	Point3D sensor_point;

	while (!point_cloud.empty()) {
        #pragma HLS PIPELINE II=1 rewind off
    	// Read a point from the point cloud stream
        sensor_point = point_cloud.read();

        // Map the 3D map point to a 2D grid cell (x, y) and height measurement (z)
        int cell_x = hls::ceil(sensor_point.x * pose_t(CELL_SIZE));
        int cell_y = hls::ceil(sensor_point.y * CELL_SIZE);

        // Ensure cell indices are within bounds
        if (cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT) {
            MapCell &cell = map[cell_x][cell_y];

            pose_t p_meas = sensor_point.z;   // Use the z-coordinate as the height measurement

            // Calculate measurement variance for this point
            noise_t var_meas = calculate_measurement_variance(sensor_point);
 
 
            // Apply Kalman filter update
            kalman_update(cell, p_meas, var_meas);
        }
    }

}