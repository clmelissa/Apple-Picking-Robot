#include <cstdio>
#include <unistd.h>

int main()
{
    int data[] = {10,5,13};  //Random data we want to send
    FILE *file;
    file = fopen("/dev/ttyUSB0","w");  //Opening device file
    int i = 0;
    unsigned int sleep_time = 10;
    for(i = 0 ; i < 3 ; i++)
    {
        fprintf(file,"%d",data[i]); //Writing to the file
        fprintf(file,"%c",','); //To separate digits
        usleep(1);
    }
    fclose(file);
}