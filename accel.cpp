#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <fstream>


struct offsets{
        float xAccelOffset=0;
        float yAccelOffset=0;
        float zAccelOffset=0;
        float xGyroOffset=0;
        float yGyroOffset=0;
        float zGyroOffset=0;
    };
void calibrate(uint8_t fd,offsets &sensorError ,float accelLSBsensitivity=16384,float gyroLSBsensitivity=131, int16_t bufferSize = 500,uint8_t accelDeadzone = 8,uint8_t gyroDeadzone = 1){
    uint8_t raw[14];
    float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    int16_t n = 0;
    int16_t j=0;
    uint8_t reg = 0x3B;
    bool uncalibrated=true;
    float accelErrorTarget=16384/(4*9.81);// within 0.25m/s/s
    float axSum=0;
    float aySum=0;
    float azSum=0;
    std::cout << "Calibration started" << std::endl; 
    std::ofstream logFile("imu_calibrate.csv"); 
    while(true){
            write(fd,&reg,1);
            read(fd,raw,14);
            int16_t rawAx = (raw[0]<<8)|raw[1]; 
            int16_t rawAy = (raw[2]<<8)|raw[3];
            int16_t rawAz = (raw[4]<<8)|raw[5];
            int16_t rawGx = (raw[8]<<8)|raw[9];
            int16_t rawGy = (raw[10]<<8)|raw[11];
            int16_t rawGz = (raw[12]<<8)|raw[13];
            float ax = (static_cast<float>(rawAx)-sensorError.xAccelOffset);
            float ay = (static_cast<float>(rawAy)-sensorError.yAccelOffset);
            float az = (static_cast<float>(rawAz)-sensorError.zAccelOffset);
            float gx = (static_cast<float>(rawGx)-sensorError.xGyroOffset);
            float gy = (static_cast<float>(rawGy)-sensorError.yGyroOffset);
            float gz = (static_cast<float>(rawGz)-sensorError.zGyroOffset);
            //std::cout << ax  << " " << ay << " " << az << " " << gx << " " << gy << " " << gz << std::endl;
            axSum += ax;
            aySum += ay;
            azSum += az;
            logFile << ax << "," << ay << "," << az << ","
                    << gx << "," << gy << "," << gz << "\n";
            logFile.flush(); 
             
            n+=1;
            if(n>bufferSize){
                if(abs(axSum/bufferSize)<accelErrorTarget && abs(aySum/bufferSize)<accelErrorTarget && 9.81+accelErrorTarget<abs(azSum/bufferSize)<9.81+accelErrorTarget){
                        std::cout << "calibration succeeded"<< std::endl; 
                        return;
                }
                        
                sensorError.xAccelOffset+=axSum/bufferSize;
                sensorError.yAccelOffset+=aySum/bufferSize;
                sensorError.zAccelOffset+=azSum/bufferSize-accelLSBsensitivity;
                sensorError.xGyroOffset+=gx/bufferSize;
                sensorError.yGyroOffset+=gy/bufferSize;
                sensorError.zGyroOffset+=gz/bufferSize;
                axSum = 0;
                aySum = 0;
                azSum = 0;
                n=0;
                j+=1;
            }    
             
            if(j>5){
                std::cout << "calibration failed"<< std::endl; 
                return;
            }
            usleep(100);
    }
    std::cout << "while loop exited"<< std::endl; 
    
}

int main(){
        uint8_t fd=open("/dev/i2c-1",O_RDWR);
        if(fd<0){
                perror("failed to open I2C bus");
                return 1;
        }
        uint8_t addr = 0x68;
        if(ioctl(fd,I2C_SLAVE,addr)<0){
                perror("Failed to set I2C address");
                return 1;
        }
        uint8_t pwr[2]={0x6B, 0x00};
        write(fd,pwr,2);

        std::ofstream logFile("imu_data.csv");     
        logFile << "Time_ms,AX,AY,AZ,GX,GY,GZ\n"; 

        
        
        offsets sensorError{};
        float accelLSBsensitivity=16384;
        float gyroLSBsensitivity=131;
        calibrate(fd,sensorError,accelLSBsensitivity,gyroLSBsensitivity);

        uint8_t reg = 0x3B;
        float ax,ay,az,gx,gy,gz;
        int16_t n = 0;
        uint8_t raw[14];
        float g = 9.815;
        uint8_t config[4];
        uint8_t configReg=0x1c;
        write(fd,&configReg,1);
        read(fd,config,1);
        std::cout << sensorError.zAccelOffset << std::endl;

        while(true){
            write(fd,&reg,1);
            read(fd,raw,14);
            int16_t rawAx = (raw[0]<<8)|raw[1]; 
            int16_t rawAy = (raw[2]<<8)|raw[3];
            int16_t rawAz = (raw[4]<<8)|raw[5];
            int16_t rawGx = (raw[8]<<8)|raw[9];
            int16_t rawGy = (raw[10]<<8)|raw[11];
            int16_t rawGz = (raw[12]<<8)|raw[13];
            float ax = (static_cast<float>(rawAx)-sensorError.xAccelOffset)/accelLSBsensitivity;
            float ay = (static_cast<float>(rawAy)-sensorError.yAccelOffset)/accelLSBsensitivity;
            float az = (static_cast<float>(rawAz)-sensorError.zAccelOffset)/accelLSBsensitivity;
            float gx = (static_cast<float>(rawGx)-sensorError.xGyroOffset)/gyroLSBsensitivity;
            float gy = (static_cast<float>(rawGy)-sensorError.yGyroOffset)/gyroLSBsensitivity;
            float gz = (static_cast<float>(rawGz)-sensorError.zGyroOffset)/gyroLSBsensitivity;
            int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::high_resolution_clock::now().time_since_epoch()
                  ).count();
            logFile << now <<","<< ax << "," << ay << "," << az << ","
                    << gx << "," << gy << "," << gz << "\n";
            logFile.flush(); // ensures data is written immediately

            std::cout << "\r"
                  << "AX: " << std::setw(8) << ax << " "
                  << "AY: " << std::setw(8) << ay << " "
                  << "AZ: " << std::setw(8) << az << " "
                  << "GX: " << std::setw(8) << gx << " "
                  << "GY: " << std::setw(8) << gy << " "
                  << "GZ: " << std::setw(8) << gz 
                  << std::flush;
            std::cout << "\rax: " << ax  << " ay: " << ay << " az: " << az << " gx " << gx << " gy " << gy << " gz " << gz << std::flush;

            usleep(100000);
    }
        std::cout << sensorError.xAccelOffset;
        close(fd);
        
        return 0;
}
