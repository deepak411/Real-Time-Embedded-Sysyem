#include<zephyr.h>
#include<misc/printk.h>
#include<stdio.h>
#include<stdlib.h>
#include<device.h>
#include<string.h>
#include<gpio.h>
#include<pinmux.h>
#include<sensor.h>
#include<uart.h>
#include<pinmux_galileo.h>
#include<math.h>

#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

/* pulse time for ultrasonic sensor */
#define PULSETIME 1
/* sleep time for ultrasonic sensor*/
#define SLEEPTIMEB 500
/* sleep time for MPU 6050 */
#define SLEEPTIMEC 500
/* ultrasonic sensor trigger pin */
#define OUTPUT_PIN 3
#define OUTPUT_PINMUX 0
/* ultrasonic sensor echo pin */
#define INPUT_PIN 4
#define INPUT_PINMUX 1 

/******Kalman filter settings******/ 
#define RAD_TO_DEG (180.0/3.14)

typedef struct kalman_struct{
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double rate;
	double bias;
	double P[2][2];
}kalman ;

kalman kalmanx;
kalman kalmany;


double accX, accY, accZ; //sensor reading
double gyroX, gyroY, gyroZ; //sensor reading
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Orientation values for Kalman Filter
float dt; // Time interval between two sensor samples
uint32_t timer;

/* enum for thread identification */
typedef enum threadID{
	THREAD_B, // thread for ultrasonic sensor
	THREAD_C  // thread for MPU 6050
}thread_ID;

typedef struct msg{  // message stucture for the message queue
	thread_ID tid;
	/* gyroscope velocity readings in different directions */
	int32_t gyro_x_val1;
	int32_t gyro_x_val2;
	int32_t gyro_y_val1;
	int32_t gyro_y_val2;
	int32_t gyro_z_val1;
	int32_t gyro_z_val2;
	/* gyroscope acceleration readings in different directions */
	int32_t gyro_acc_x_val1;
	int32_t gyro_acc_x_val2;
	int32_t gyro_acc_y_val1;
	int32_t gyro_acc_y_val2;
	int32_t gyro_acc_z_val1;
	int32_t gyro_acc_z_val2;
	uint32_t us_dist;	// ultrasonic distance
	
} msg;
struct k_msgq mq;    // message queue to handle values 
int32_t max = 2147483647;
struct device *dev;  // device instance for GPIO port
struct device *sdev; // device instance for MPU6050 port
struct device *pin;  // device instance for Pinmux

K_MSGQ_DEFINE(mq,sizeof(msg),10,4);	// defining message queue for UART communication

K_SEM_DEFINE(q_sem, 0, 2);	// defining semaphore for message queue access


/* get angle using Kalman filter */
double getAngle(int sid, double newAngle, double newRate, double dt) {

	if(sid==0){	
		kalmanx.rate = newRate - kalmanx.bias;
		kalmanx.angle += dt * kalmanx.rate;


		kalmanx.P[0][0] += dt * (dt*kalmanx.P[1][1] - kalmanx.P[0][1] - kalmanx.P[1][0] + kalmanx.Q_angle);
		kalmanx.P[0][1] -= dt * kalmanx.P[1][1];
		kalmanx.P[1][0] -= dt * kalmanx.P[1][1];
		kalmanx. P[1][1] += kalmanx.Q_bias * dt;


		double S = kalmanx.P[0][0] + kalmanx.R_measure; // Estimate error
		/* Step 5 */
		double K[2]; // Kalman gain - This is a 2x1 vector
		K[0] = kalmanx.P[0][0] / S;
		K[1] = kalmanx.P[1][0] / S;

		double y = newAngle -kalmanx.angle;
		kalmanx.angle += K[0] * y;
		kalmanx.bias += K[1] * y;


		double P00_temp = kalmanx.P[0][0];
		double P01_temp = kalmanx.P[0][1];

		kalmanx.P[0][0] -= K[0] * P00_temp;
		kalmanx.P[0][1] -= K[0] * P01_temp;
		kalmanx.P[1][0] -= K[1] * P00_temp;
		kalmanx.P[1][1] -= K[1] * P01_temp;

		return kalmanx.angle;
	}
	else{
		kalmany.rate = newRate - kalmany.bias;
		kalmany.angle += dt * kalmany.rate;

		kalmany.P[0][0] += dt * (dt*kalmany.P[1][1] - kalmany.P[0][1] - kalmany.P[1][0] + kalmany.Q_angle);
		kalmany.P[0][1] -= dt * kalmany.P[1][1];
		kalmany.P[1][0] -= dt * kalmany.P[1][1];
		kalmany. P[1][1] += kalmany.Q_bias * dt;

		double S = kalmany.P[0][0] + kalmany.R_measure; // Estimate error
		/* Step 5 */
		double K[2]; // Kalman gain - This is a 2x1 vector
		K[0] = kalmany.P[0][0] / S;
		K[1] = kalmany.P[1][0] / S;

		double y = newAngle - kalmany.angle;
		kalmany.angle += K[0] * y;
		kalmany.bias += K[1] * y;


		double P00_temp = kalmany.P[0][0];
		double P01_temp = kalmany.P[0][1];

		kalmany.P[0][0] -= K[0] * P00_temp;
		kalmany.P[0][1] -= K[0] * P01_temp;
		kalmany.P[1][0] -= K[1] * P00_temp;
		kalmany.P[1][1] -= K[1] * P01_temp;

		return kalmany.angle;	
	}
}

/* Do the filter update */
void filterData(){
 	dt=(((float)(k_cycle_get_32())- (float)(timer))/1000000000.0);
	timer=k_cycle_get_32();
	double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	printf("Roll ========= %f Pitch ======== %f\n",roll, pitch);

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

	if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
	    kalmany.angle=pitch;
	    kalAngleY = pitch;
	    gyroYangle = pitch;
	} 
	else
	    kalAngleY = getAngle(1, pitch, gyroYrate, dt);     // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
	    gyroXrate = -gyroXrate;     // Invert rate, so it fits the restriced accelerometer reading
	kalAngleX = getAngle(0, roll, gyroXrate, dt);

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	gyroYangle += gyroYrate * dt;

	if (gyroXangle < -180 || gyroXangle > 180)
	    gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
	    gyroYangle = kalAngleY;
}



void entryPoint(thread_ID tid){
	struct sensor_value x;
	struct sensor_value y;
	struct sensor_value z;
	struct sensor_value acc_x;
	struct sensor_value acc_y;
	struct sensor_value acc_z;
	int rc;
	uint32_t val;
	uint32_t start_time;
	uint32_t stop_time;
	uint32_t cycles_spent;
	uint32_t nanoseconds_spent;

	msg data;
	data.gyro_x_val1 = 0;
	data.gyro_x_val2 = 0;
	data.gyro_y_val1 = 0;
	data.gyro_y_val2 = 0;
	data.gyro_z_val1 = 0;
	data.gyro_z_val2 = 0;
	data.gyro_acc_x_val1 = 0;
	data.gyro_acc_x_val2 = 0;
	data.gyro_acc_y_val1 = 0;
	data.gyro_acc_y_val2 = 0;
	data.gyro_acc_z_val1 = 0;
	data.gyro_acc_z_val2 = 0;
	data.us_dist = 0;
	while (1) {

		data.tid=tid;
		/* ultrasonic sensor */
		if(tid==THREAD_B){

			int status = gpio_pin_write(dev,OUTPUT_PIN,1);
			k_sleep(PULSETIME);

			status = gpio_pin_write(dev,OUTPUT_PIN,0);
			gpio_pin_read(dev, INPUT_PIN, &val);
			
			while(val==0){
				gpio_pin_read(dev, INPUT_PIN, &val);
							
			}

			/* capture initial time stamp */
			start_time = k_cycle_get_32();

			/* reading the echo pulse */
			while(val==1){
				gpio_pin_read(dev, INPUT_PIN, &val);
							
			}

			/* capture final time stamp */
			stop_time = k_cycle_get_32();

			/* compute how long the work took (assumes no counter rollover) */
			cycles_spent = stop_time - start_time;
			nanoseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
			uint32_t d = nanoseconds_spent/58000;
			/* update structure for message queue */
			data.us_dist = d;	
			/* add message to queue */
			k_msgq_put(&mq,&data,K_NO_WAIT);
			k_sem_give(&q_sem);
			k_sleep(SLEEPTIMEB);
			
		}
		/* MPU 6050 */
		else if(tid==THREAD_C){
			/* read different channels */
			rc = sensor_sample_fetch(sdev);
			rc = sensor_channel_get(sdev, SENSOR_CHAN_GYRO_X,&x);
			rc = sensor_channel_get(sdev, SENSOR_CHAN_GYRO_Y,&y);
			rc = sensor_channel_get(sdev, SENSOR_CHAN_GYRO_Z,&z);
			rc = sensor_channel_get(sdev, SENSOR_CHAN_ACCEL_X,&acc_x);
			rc = sensor_channel_get(sdev, SENSOR_CHAN_ACCEL_Y,&acc_y);
			rc = sensor_channel_get(sdev, SENSOR_CHAN_ACCEL_Z,&acc_z);
			/* update structure for message queue */
			data.gyro_acc_x_val1 = acc_x.val1;
			data.gyro_acc_x_val2 = acc_x.val2;
			data.gyro_acc_y_val1 = acc_y.val1;
			data.gyro_acc_y_val2 = acc_y.val2;
			data.gyro_acc_z_val1 = acc_z.val1;
			data.gyro_acc_z_val2 = acc_z.val2;
			data.gyro_x_val1 = x.val1;
			data.gyro_x_val2 = x.val2;
			data.gyro_y_val1 = y.val1;
			data.gyro_y_val2 = y.val2;
			data.gyro_z_val1 = z.val1;
			data.gyro_z_val2 = z.val2;
			/* add message to queue */
			k_msgq_put(&mq,&data,K_NO_WAIT);
			k_sem_give(&q_sem);
			k_sleep(SLEEPTIMEC);
		}

	}
}

char __noinit __stack threadB_stack_area[STACKSIZE];

char __noinit __stack threadC_stack_area[STACKSIZE];

void threadB(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	/* routine to Read sensor values and pass them to UART */
	entryPoint(THREAD_B);
}

void threadC(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	/* routine to Read sensor values and pass them to UART */
	entryPoint(THREAD_C);
}

void threadA(void *dummy1, void *dummy2, void *dummy3)
{
	
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	// port device instant binding

	kalmanx.Q_angle = 0.001;
	kalmanx.Q_bias = 0.003;
	kalmanx.R_measure = 0.003;
	kalmanx.angle =0;
	kalmanx.bias =0;
	//kalmanx.P ={0.0};

	kalmany.Q_angle = 0.001;
	kalmany.Q_bias = 0.003;
	kalmany.R_measure = 0.003;
	kalmany.angle =0;
	kalmany.bias =0;
	//kalmany.P ={0.0};
	

	dev = device_get_binding(CONFIG_GPIO_DW_0_NAME);
	// pinmux binding
	pin = device_get_binding(CONFIG_PINMUX_NAME);
	if(!dev)
		printk("Errorrr binding GPIO.....\n");
	if(!pin)
		printk("Errorrr binding PINMUX.....\n");
	// MPU6050 sensor binding
	sdev = device_get_binding("MPU6050");
	if(!sdev)
		printk("Errorrr binding MPU6050.....\n");
	/* set sensor attributes */
	struct sensor_value val;
	val.val1 = 0;
	val.val2 = 0;
	sensor_attr_set(sdev, SENSOR_ATTR_CALIB_TARGET,SENSOR_CHAN_GYRO_ANY,&val);	
	
	// pinmux settings
	pinmux_pin_set(pin,OUTPUT_PINMUX,PINMUX_FUNC_A);
	pinmux_pin_set(pin,INPUT_PINMUX,PINMUX_FUNC_B);
	
	// spawn threads to read the sensors
	k_thread_spawn(threadB_stack_area, STACKSIZE, threadB, NULL, NULL, NULL,
	PRIORITY, 0, K_NO_WAIT);
	
	k_thread_spawn(threadC_stack_area, STACKSIZE, threadC, NULL, NULL, NULL,
	PRIORITY, 0, K_NO_WAIT);

	struct msg data;

	/* UART output */
	while(1){
		/* take message from queue */
		k_sem_take(&q_sem, K_FOREVER);
		k_msgq_get(&mq,&data,K_FOREVER);		
		if(data.tid == THREAD_B){
			/* ultrasonic sensor output on UART */
			printf("distance value is ...:%d\n", (int)data.us_dist);	
		}
		else{
			/* MPU 6050 output on UART */
			gyroX=((float)data.gyro_x_val1+(float)data.gyro_x_val2*.000001);
			gyroY=((float)data.gyro_y_val1+(float)data.gyro_y_val2*.000001);
			gyroZ=((float)data.gyro_z_val1+(float)data.gyro_z_val2*.000001);

			accX=((float)data.gyro_x_val1+(float)data.gyro_x_val2*.000001);
			accY=((float)data.gyro_y_val1+(float)data.gyro_y_val2*.000001);
			accZ=((float)data.gyro_z_val1+(float)data.gyro_z_val2*.000001);
			
			filterData();
			
			printf("Angular Velocity in X direction is==== %f\n",gyroX);
			printf("Angular Velocity in Y direction is====  is %f\n",gyroY);
			printf("Angular Velocity in Z direction is====  is %f\n",gyroZ);
			printf("Acceleration in X direction is==== %f\n",accX);
			printf("Acceleration in Y direction is==== %f\n",accY);
			printf("Acceleration in Z direction is==== %f\n",accZ);
			printf("Orientation in X direction is=== %f\n",kalAngleX);
			printf("Orientation in Y direction is=== %f\n",kalAngleY);	
		}
	}
}




K_THREAD_DEFINE(threadA_id, STACKSIZE, threadA, NULL, NULL, NULL,
PRIORITY, 0, K_NO_WAIT);


