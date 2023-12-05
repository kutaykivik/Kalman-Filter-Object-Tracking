/* Standard includes. */
#include <stdio.h>
#include <time.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Kalman.h"
#include "Kalman_math.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The times are converted from
milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define measureTASK_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 10UL )
#define filterTASK_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 10UL )
/* The number of items the queue can hold at once. */
#define mainQUEUE_LENGTH					( 10 )

//Measurement period
double dt = 0.01; 

int isFirstIteration = 1;

FILE *file;	

static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );


static QueueHandle_t xQueue = NULL;

TickType_t getRandomBlockTime() {
    // rand() fonksiyonu 0 ile RAND_MAX (genellikle 32767) arasında bir değer üretir.
    // Bu değeri istediğiniz aralığa sığacak şekilde ölçeklendiriyoruz.
    return pdMS_TO_TICKS(1UL + rand() % 10UL); 
}


static void prvQueueSendTask( void *pvParameters )
{
    TickType_t xNextWakeTime;
    //TickType_t xBlockTime = measureTASK_SEND_FREQUENCY_MS*10;
    //const uint32_t ulValueToSend = mainVALUE_SENT_FROM_TASK;


	/* Prevent the compiler warning about the unused parameter. */
	( void ) pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	double x = 10.0;    // Initial x position
    double y = 20.0;    // Initial y position
    double vx = 3.0;    // Initial x velocity
    double vy = 6.0;    // Initial y velocity
    double sigma_x2 = 0.1;  // Variance of Gaussian noise for x
    double sigma_y2 = 0.1;  // Variance of Gaussian noise for y

	double measured[2];

	srand(time(NULL));


	for( ;; )
	{
		// Generate ground truth data
		x = x + vx*dt;
		y = y + vy*dt;

		fprintf(file, "Ground truth pos_x: %f, Ground truth v_x: %f, Ground truth pos_y: %f, Ground truth v_y: %f, ",
			x, vx, y, vy);

		double noisy_x = x + randn(0, sqrt(sigma_x2));
    	double noisy_y = y + randn(0, sqrt(sigma_y2));
		measured[0] = noisy_x;
		measured[1] = noisy_y;

		fprintf(file, "Measurement pos_x: %f , Measurement pos_y: %f, ", measured[0], measured[1]);

		//TickType_t xBlockTime = pdMS_TO_TICKS(10UL + rand() % 91UL);
		TickType_t xBlockTime = measureTASK_SEND_FREQUENCY_MS;
		/* Place this task in the blocked state until it is time to run again.
		The block time is specified in ticks, pdMS_TO_TICKS() was used to
		convert a time specified in milliseconds into a time specified in ticks.
		While in the Blocked state this task will not consume any CPU time. */
		vTaskDelayUntil( &xNextWakeTime, xBlockTime );

		/* Send to the queue - causing the queue receive task to unblock and
		write to the console.  0 is used as the block time so the send operation
		will not block - it shouldn't need to block as the queue should always
		have at least one space at this point in the code. */
		xQueueSend( xQueue, &measured[0], 0U );

	}
}

static void prvQueueReceiveTask( void *pvParameters )
{
TickType_t xNextWakeTime;
TickType_t xBlockTime = filterTASK_SEND_FREQUENCY_MS;

double ulReceivedValue[2];
KalmanFilter kf;


	/* Prevent the compiler warning about the unused parameter. */
	( void ) pvParameters;

    /* Start to save the execution time*/
	clock_t start = clock();

	for( ;; )
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h.  It will not use any CPU time while it is in the
		Blocked state. */
		xQueueReceive( xQueue, &ulReceivedValue[0], portMAX_DELAY );


        /* Enter critical section to use printf. Not doing this could potentially cause
           a deadlock if the FreeRTOS simulator switches contexts and another task
           tries to call printf - it should be noted that use of printf within
           the FreeRTOS simulator is unsafe, but used here for simplicity. */
        taskENTER_CRITICAL();
        {
            /*  To get here something must have been received from the queue, but
            is it an expected value?  Normally calling printf() from a task is not
            a good idea.  Here there is lots of stack space and only one task is
            using console IO so it is ok.  However, note the comments at the top of
            this file about the risks of making Windows system calls (such as
            console output) from a FreeRTOS task. */

            if (isFirstIteration) {
                // Initialize Kalman filter only on the first iteration
                kalman_init(&kf);
                isFirstIteration = 0;  // Set the flag to false after initialization
            }

			printf("%f, %f\n", ulReceivedValue[0], ulReceivedValue[1]);

            //Log estimated datas and kalman gain
            fprintf(file,  "Estimated pos_x: %f, Estimated v_x: %f, Estimated pos_y: %f, Estimated v_y: %f, Kalman gain1: %f, Kalman gain2: %f, ", kf.est[0], kf.est[1], kf.est[2], kf.est[3], kf.KGData[0], kf.KGData[2]);
            printf("Estimated pos_x: %f\n Estimated v_x: %f\n Estimated pos_y: %f\n, Estimated v_y: %f\n", kf.est[0], kf.est[1], kf.est[2], kf.est[3]);
			
			kalmanFilter(&kf, &ulReceivedValue[0]);
			printf("Estimated pos_x: %f\n Estimated v_x: %f\n Estimated pos_y: %f\n, Estimated v_y: %f\n", kf.est[0], kf.est[1], kf.est[2], kf.est[3]);
			clock_t end = clock();
		
			// Log execution time
			fprintf(file, "Time taken for Kalman Filter: %f seconds\n", ((double)(end - start)) / CLOCKS_PER_SEC);
			


			//File closing time(sec)
			if(((double) (end - start)) / CLOCKS_PER_SEC > 60){
				fclose(file); 
			}  
        }
        taskEXIT_CRITICAL();

		vTaskDelayUntil( &xNextWakeTime, xBlockTime );
	}
}


int main(){

	printf("Hello world\n");

	file = fopen("output.txt", "w");


	if (file == NULL) {
		perror("Error opening file");
		exit(1);  // Terminate the program with an error code
	}

	xQueue = xQueueCreate( mainQUEUE_LENGTH, 2*sizeof( double ) );

	if( xQueue != NULL )
	{
		/* Start the two tasks as described in the comments at the top of this
		file. */
		xTaskCreate( prvQueueReceiveTask,			/* The function that implements the task. */
					"Rx", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 		/* The size of the stack to allocate to the task. */
					NULL, 							/* The parameter passed to the task - not used in this simple case. */
					mainQUEUE_RECEIVE_TASK_PRIORITY,/* The priority assigned to the task. */
					NULL );							/* The task handle is not required, so NULL is passed. */

		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );


		/* Start the tasks running. */
		vTaskStartScheduler();
	}
	fclose(file);  

	return 0;
}
