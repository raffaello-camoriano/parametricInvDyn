#include <stdio.h>
#include <yarp/os/Time.h>

using namespace yarp::os;

int main()
{
    printf("Starting the application\n");
    int times=10;

    while(times--)
    {
        printf("Hello iCub\n");
        Time::delay(0.5); //wait 0.5 seconds
    }
    printf("Goodbye!\n");
}