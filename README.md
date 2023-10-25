**Title:** Navigation with IMU and Magnetometer in ROS

**Description:**

In this project, the focus was on building a comprehensive navigation stack employing two distinct sensors: GPS and IMU. Through this process, the relative strengths, drawbacks, and the principles of sensor fusion were explored in detail.

1. **Data Collection in a Car (Teamwork)**:
   - Utilized both the GPS and IMU sensors, ensuring that the data from both sensors was synchronized and logged effectively. This was done by crafting a unified ROS launch file.
   - Followed a predetermined driving loop around Boston, starting and ending at Ruggles station. This drive incorporated various turns and movements to capture a diverse set of data.

2. **Analysis of Collected Data (Individual)**:
   - **Heading Estimation**:
     - Performed a magnetometer calibration to correct for "hard-iron" and "soft-iron" effects. The calibrated data was then employed to derive the yaw angle.
     - Used a complementary filter to fuse yaw measurements from the magnetometer and the gyro, providing a more accurate estimate of the vehicle's heading.
     - Compared the fused yaw measurements with those from the IMU and analyzed discrepancies.
   - **Velocity Estimation**:
     - Computed the forward velocity by integrating the forward acceleration data.
     - Plotted and compared velocity estimates from both the accelerometer and the GPS.
   - **Dead Reckoning with IMU**:
     - Utilized the derived yaw and velocity data to estimate the vehicle's trajectory. This was then juxtaposed with the trajectory data from the GPS.
     - As an additional step, attempted to estimate the value of \( x_c \) (the offset of the IMU from the vehicle's center of mass) based on discrepancies between the IMU and GPS data.

3. **Documentation and Report Structuring**: 
   - Composed a comprehensive report detailing the methodologies, findings, and conclusions derived from the analysis.
   - The report emphasized plots of various estimates (magnetometer calibration, yaw, velocity, etc.) and a thorough discussion on the observed discrepancies and their potential causes.
   - Addressed specific questions laid out in the project brief, providing insights into the performance of the VectorNav IMU in dead reckoning scenarios.

By the conclusion of this lab, I honed my skills in sensor fusion, dead reckoning, and navigation using both GPS and IMU data. This project shed light on the intricate dance of combining data from multiple sensors to derive more reliable and accurate navigation estimates.