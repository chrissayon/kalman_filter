#include <stdio.h>
#include "kalman_filter.h"


int main(){
    kalman_filter kalman;
    printf("%i", kalman.process_noise_dimensions[1]);    
}