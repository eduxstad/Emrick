

/**
 * main.c
 */


#include <posix.h>
#include <pthread.h>
#include <stdio.h>

#define NUM_THREADS 4


int main_func() {

    // Essential threads for Emrick to function
    pthread_t battery_t;
    pthread_t send_t;
    pthread_t receive_t;
    pthread_t lights_t; // Might be merged into the main thread...

    // Thread array for main thread to wait for
    pthread_t pool[NUM_THREADS] =
    {battery_t, send_t, receive_t, lights_t};

    int thread_args[NUM_THREADS];
    int result = -1;

    // Create <first> thread
    // If thread fails to be created, stop creation
    // battery_microvolts
    result = pthread_create(&battery_t, NULL, /*func*/, &thread_args);

    if (result != 0) {
        fprintf(STDERR, "Error: Failed to create battery thread!");
        while (1);
    }

    // Create second thread
    result = -1;
    result = pthread_create(&send_t, NULL, /*func*/, &thread_args);
    if (result != 0) {
        fprintf(STDERR, "Error: Failed to create RX thread!");
        while (1);
    }

    // Create third thread
    result = -1;
    result = pthread_create(&receive_t, NULL, /*func*/, &thread_args);
    if (result != 0) {
        fprintf(STDERR, "Error: Failed to create TX thread!");
        while (1);
    }

    // Create fourth thread (once again, might be merged into the main thread)
    result = -1;
    result = pthread_create(&lights_t, NULL, /*func*/, &thread_args);
    if (result != 0) {
        fprintf(STDERR, "Error: Failed to create lights thread!");
        while (1);
    }

    // Every function is on a different thread, so
    // This thread is now useless...for now


    // Wait for threads to finish executing
    // If abnormal behavior occurs
    for (int i = 0; i < NUM_THREADS; i++) {
        pthread_join(pool[i], NULL);
    }


    return 0;
}

int main(void)
{
    main_func();
	return 0;
}
