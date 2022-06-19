# GNSS_INS_LooseCouple
A C++ Program that calculates GNSS/INS LooseCouple using Extended Kalman Filter.
Test datasets are included (GNSS_PLAYGROUND1.posT and IMU_PLAYGROUND1.imr)

INS State includes position (3d) / velocity (3d) / attitude (3d) / gyro's bias (3d) / accelerometer's bias (3d) / gyro's scale factor(3d) / accelerometer's scale factor(3d). The state vector is a 21 * 1 vector

We use the AsteRx SBi to collect dataset, which combines GPS/GNSS and an industry-grade IMU (Inertial Measurement Unit) to deliver precise positioning together with 3D attitude (heading, roll, pitch). You can check out this link https://www.directindustry-china.cn/prod/septentrio/product-183002-2282177.html for more info.

![image](https://user-images.githubusercontent.com/100557045/174484720-06c4a0ec-44f7-4b9a-97cd-0ef8e2689a3e.png)
