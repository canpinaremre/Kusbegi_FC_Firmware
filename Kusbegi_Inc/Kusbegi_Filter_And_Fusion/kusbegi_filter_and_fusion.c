/*
 *
 *
 *
 */

#include <Kusbegi_Inc/Kusbegi_Filter_And_Fusion/kusbegi_filter_and_fusion.h>

int8_t Complementary_filter_YPR(KSB_KUSBEGI *Kusbegi){

	float rad_to_deg = 180/M_PI;
	float elapsedtime = Kusbegi->imu.elapsed_time / 1000.0f; /* s */
	float compass_x_horizontal,compass_y_horizontal,compass_yaw;



	 /*---X axis angle---*/
	Kusbegi->attitude.euler[0] = COMPLEMENTARY_PR_ALFA *(Kusbegi->attitude.euler[0] + (Kusbegi->imu.gyro[0] *elapsedtime)) + (1.0f - COMPLEMENTARY_PR_ALFA)*((atan(Kusbegi->imu.accel[1]/sqrt(pow(Kusbegi->imu.accel[0],2) + pow(Kusbegi->imu.accel[2],2)))*rad_to_deg));


	/*---Y axis angle---*/
	Kusbegi->attitude.euler[1] = COMPLEMENTARY_PR_ALFA *(Kusbegi->attitude.euler[1] + (Kusbegi->imu.gyro[1] *elapsedtime)) + (1.0f - COMPLEMENTARY_PR_ALFA)*((atan(-1*Kusbegi->imu.accel[0]/sqrt(pow(Kusbegi->imu.accel[1],2) + pow(Kusbegi->imu.accel[2],2)))*rad_to_deg));

	Kusbegi->attitude.ypr[1] = Kusbegi->attitude.euler[1];
	Kusbegi->attitude.ypr[2] = Kusbegi->attitude.euler[0];

	float angle_pitch = Kusbegi->attitude.ypr[1];
	float angle_roll = -Kusbegi->attitude.ypr[2];

	compass_x_horizontal = (float)Kusbegi->imu.magno[0] * cos(angle_pitch * -0.0174533) + (float)Kusbegi->imu.magno[1] * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)Kusbegi->imu.magno[2] * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
	compass_y_horizontal = (float)Kusbegi->imu.magno[1] * cos(angle_roll * 0.0174533) + (float)Kusbegi->imu.magno[2] * sin(angle_roll * 0.0174533);


	if (compass_y_horizontal < 0)
		compass_yaw = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * rad_to_deg));
	else
		compass_yaw = (atan2(compass_y_horizontal, compass_x_horizontal)) * rad_to_deg;


	float yaw_from_gyro = (Kusbegi->attitude.euler[2] - Kusbegi->imu.gyro[3] *elapsedtime);

	if(yaw_from_gyro < 0)
		yaw_from_gyro += 360.0f;
	else if(yaw_from_gyro >= 360)
		yaw_from_gyro -= 360.0f;


	if((yaw_from_gyro - compass_yaw) > 180){
		compass_yaw += 360.0f;
	}
	else if((yaw_from_gyro - compass_yaw) < -180){
		compass_yaw -= 360.0f;
	}


	Kusbegi->attitude.euler[2] = COMPLEMENTARY_YAW_ALFA * yaw_from_gyro + (1.0f - COMPLEMENTARY_YAW_ALFA) * compass_yaw;

	if(Kusbegi->attitude.euler[2] >= 360){
		Kusbegi->attitude.euler[2] -= 360.0f;
	}
	else if(Kusbegi->attitude.euler[2] < 0){
		Kusbegi->attitude.euler[2] += 360.0f;
	}


	Kusbegi->attitude.ypr[0] = Kusbegi->attitude.euler[2];

	return 0;
}

