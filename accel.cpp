#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <fstream>
#include <pigpio.h>
#include <termios.h>    // termios, tcgetattr, tcsetattr
#include <cstring>      // memset (or use {})
#include <vector>

// g++ accel.cpp -o accel -lpigpio -lrt -pthread

struct offsets
{
        float xAccelOffset = 0;
        float yAccelOffset = 0;
        float zAccelOffset = 0;
        float xGyroOffset = 0;
        float yGyroOffset = 0;
        float zGyroOffset = 0;
};

struct IMUdata
{
        float xAccel = 0;
        float yAccel = 0;
        float zAccel = 0;
        float xGyro = 0;
        float yGyro = 0;
        float zGyro = 0;
};

float accelLSBsensitivity = 16384;
float gyroLSBsensitivity = 131;

uint8_t I2CSetup(int LED = 5){
         // Connect to pigpio daemon
        if (gpioInitialise() < 0)
        {
                std::cerr << "pigpio init failed\n";
                return 1;
        }

        gpioSetMode(LED, PI_OUTPUT);
        gpioWrite(LED, 1);   // LED ON
                uint8_t fd = open("/dev/i2c-1", O_RDWR);
        if (fd < 0)
        {
                perror("failed to open I2C bus");
                return 1;
        }
        uint8_t addr = 0x68;
        if (ioctl(fd, I2C_SLAVE, addr) < 0)
        {
                perror("Failed to set I2C address");
                return 1;
        }
        uint8_t pwr[2] = {0x6B, 0x00};
        write(fd, pwr, 2);

        return fd;
}

int GPSsetup(){
        int GPSfd = open("/dev/serial0", O_RDWR | O_NOCTTY);
        if (GPSfd == -1) {
                perror("Unable to open serial port");
                return 1;
        }
        termios options{};
        tcgetattr(GPSfd,&options);
        cfsetispeed(&options,B9600); // set input speed 
        cfsetospeed(&options,B9600); // set output speed
        options.c_cflag &= ~PARENB; // no parity
        options.c_cflag &= ~CSTOPB; // 1 stop bit
        options.c_cflag &= ~CSIZE;  // clear current data bit size
        options.c_cflag |= CS8;     // 8 data bits
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN]  = 0;  // minimum number of characters to read
        options.c_cc[VTIME] = 10; // timeout in deciseconds (1 second)
        tcsetattr(GPSfd, TCSANOW, &options);
        

        return GPSfd;
}

void calibrate(uint8_t fd, offsets &sensorError, int16_t bufferSize = 500, uint8_t accelDeadzone = 8, uint8_t gyroDeadzone = 1){
        uint8_t raw[14];
        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        int16_t n = 0;
        int16_t j = 0;
        uint8_t reg = 0x3B;
        bool uncalibrated = true;
        float accelErrorTarget = 16384 / (4 * 9.81); // within 0.25m/s/s
        float axSum = 0;
        float aySum = 0;
        float azSum = 0;
        std::cout << "Calibration started" << std::endl;
        std::ofstream IMUlogFile("imu_calibrate.csv");
        while (true)
        {
                write(fd, &reg, 1);
                read(fd, raw, 14);
                int16_t rawAx = (raw[0] << 8) | raw[1];
                int16_t rawAy = (raw[2] << 8) | raw[3];
                int16_t rawAz = (raw[4] << 8) | raw[5];
                int16_t rawGx = (raw[8] << 8) | raw[9];
                int16_t rawGy = (raw[10] << 8) | raw[11];
                int16_t rawGz = (raw[12] << 8) | raw[13];
                float ax = (static_cast<float>(rawAx) - sensorError.xAccelOffset);
                float ay = (static_cast<float>(rawAy) - sensorError.yAccelOffset);
                float az = (static_cast<float>(rawAz) - sensorError.zAccelOffset);
                float gx = (static_cast<float>(rawGx) - sensorError.xGyroOffset);
                float gy = (static_cast<float>(rawGy) - sensorError.yGyroOffset);
                float gz = (static_cast<float>(rawGz) - sensorError.zGyroOffset);
                // std::cout << ax  << " " << ay << " " << az << " " << gx << " " << gy << " " << gz << std::endl;
                axSum += ax;
                aySum += ay;
                azSum += az;
                IMUlogFile << ax << "," << ay << "," << az << ","
                        << gx << "," << gy << "," << gz << "\n";
                IMUlogFile.flush();

                n += 1;
                if (n > bufferSize)
                {
                        if (abs(axSum / bufferSize) < accelErrorTarget && abs(aySum / bufferSize) < accelErrorTarget && 9.81 + accelErrorTarget < abs(azSum / bufferSize) < 9.81 + accelErrorTarget)
                        {
                                std::cout << "calibration succeeded" << std::endl;
                                return;
                        }

                        sensorError.xAccelOffset += axSum / bufferSize;
                        sensorError.yAccelOffset += aySum / bufferSize;
                        sensorError.zAccelOffset += azSum / bufferSize - accelLSBsensitivity;
                        sensorError.xGyroOffset += gx / bufferSize;
                        sensorError.yGyroOffset += gy / bufferSize;
                        sensorError.zGyroOffset += gz / bufferSize;
                        axSum = 0;
                        aySum = 0;
                        azSum = 0;
                        n = 0;
                        j += 1;
                }

                if (j > 5)
                {
                        std::cout << "calibration failed" << std::endl;
                        return;
                }
                usleep(100);
        }
        std::cout << "while loop exited" << std::endl;
}

void accelConvertData(uint8_t raw[14],int64_t now,IMUdata &IMUoutput,offsets &sensorError){
        int16_t rawAx = (raw[0] << 8) | raw[1];
        int16_t rawAy = (raw[2] << 8) | raw[3];
        int16_t rawAz = (raw[4] << 8) | raw[5];
        int16_t rawGx = (raw[8] << 8) | raw[9];
        int16_t rawGy = (raw[10] << 8) | raw[11];
        int16_t rawGz = (raw[12] << 8) | raw[13];
        IMUoutput.xAccel = (static_cast<float>(rawAx) - sensorError.xAccelOffset) / accelLSBsensitivity;
        IMUoutput.yAccel = (static_cast<float>(rawAy) - sensorError.yAccelOffset) / accelLSBsensitivity;
        IMUoutput.zAccel = (static_cast<float>(rawAz) - sensorError.zAccelOffset) / accelLSBsensitivity;
        IMUoutput.xGyro = (static_cast<float>(rawGx) - sensorError.xGyroOffset) / gyroLSBsensitivity;
        IMUoutput.yGyro = (static_cast<float>(rawGy) - sensorError.yGyroOffset) / gyroLSBsensitivity;
        IMUoutput.zGyro = (static_cast<float>(rawGz) - sensorError.zGyroOffset) / gyroLSBsensitivity;
}

void consoleOutput(IMUdata &IMUoutput){
        std::cout << "\r"
        << "AX: " << std::setw(8) << IMUoutput.xAccel << " "
        << "AY: " << std::setw(8) << IMUoutput.yAccel << " "
        << "AZ: " << std::setw(8) << IMUoutput.zAccel << " "
        << "GX: " << std::setw(8) << IMUoutput.xGyro << " "
        << "GY: " << std::setw(8) << IMUoutput.yGyro << " "
        << "GZ: " << std::setw(8) << IMUoutput.zGyro
        << std::flush;
        std::cout << "\rax: " << IMUoutput.xAccel << " ay: " << IMUoutput.yAccel << " az: " << IMUoutput.zAccel << " gx " << IMUoutput.xGyro << " gy " << IMUoutput.yGyro << " gz " << IMUoutput.zGyro << std::flush;

}
       
int main()
{
        int LED = 5;
        uint8_t fd = I2CSetup(LED);
        int GPSfd= GPSsetup();
        offsets sensorError{};
        calibrate(fd, sensorError);
        gpioWrite(LED, 0);

        // setup log file and add titles
        std::ofstream IMUlogFile("imu_data.csv");
        std::ofstream GPSlogFile("gps_data.csv");
        IMUlogFile << "Time_ms,AX,AY,AZ,GX,GY,GZ\n";
        
        int LEDtimer = 0;
        float ax, ay, az, gx, gy, gz;
        float g = 9.815;
        int recordingTimer=0;
        uint8_t raw[14];
        uint8_t config[4];
        IMUdata IMUoutput{}; 

        // Perform self test and setup read register
        uint8_t configReg = 0x1c;
        uint8_t reg = 0x3B;
        write(fd, &configReg, 1);
        read(fd, config, 1);
        
        // setup GPS output
        char buffer[256];
        std::vector<char> lineBuffer; // “folder” for current letter
        std::vector<char> GPSoutASCII;
        bool GPSGAfound=false;
        static int matchIndex = 0;
        const char target[] = "GPGGA";

        while (true){
                write(fd, &reg, 1);
                read(fd, raw, 14);
                int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::high_resolution_clock::now().time_since_epoch())
                                  .count();

                

                accelConvertData(raw,now,IMUoutput,sensorError);
        
                IMUlogFile << now << ',' <<  IMUoutput.xAccel << "," << IMUoutput.yAccel << "," << IMUoutput.zAccel << ","
                        << IMUoutput.xGyro << "," << IMUoutput.yGyro << "," << IMUoutput.zGyro << "\n";
                IMUlogFile.flush(); 

                //consoleOutput(IMUoutput);
                
                // every 50 IMU readings, extract GPS reading
                // recordingTimer+=1;
                // if(recordingTimer==50){
                //         recordingTimer=0;
                //         int n = read(GPSfd, buffer,sizeof(buffer));
                //         for(int i=0;i<n;i++){
                //                 char c = buffer[i];
                //                 if(c=='\n'){
                //                         std::string sentence(lineBuffer.begin(), lineBuffer.end());
                //                         std::cout<<sentence;
                //                         lineBuffer.clear();        // ready for the next sentence
                //                 }
                //                 if(c!='\r'){
                //                         lineBuffer.push_back(c);
                //                 }
                //         }
                // }

                recordingTimer+=1;
                if(recordingTimer==50){
                        recordingTimer=0;
                        int n = read(GPSfd, buffer,sizeof(buffer));
                        for(int i=0;i<n;i++){
                                char c = buffer[i];
                                if(GPSGAfound){
                                        if(c=='*'){
                                                lineBuffer.push_back(buffer[i]);
                                                lineBuffer.push_back(buffer[i+1]);
                                                lineBuffer.push_back(buffer[i+2]);
                                                GPSGAfound=false;
                                                std::string sentence(lineBuffer.begin(), lineBuffer.end());
                                                GPSlogFile << sentence << std::endl;  
                                                GPSlogFile.flush();
                                                lineBuffer.clear();
                                        }
                                        lineBuffer.push_back(c);
                                }
                                if (c == target[matchIndex]) {
                                        // std::cout << c << " " << target[matchIndex] <<std::endl;
                                        matchIndex++;
                                        if (matchIndex == 5) {
                                                GPSGAfound = true;
                                                lineBuffer.clear();
                                                // for(int j=0;j<5;j++){
                                                //         lineBuffer.push_back(target[i]);
                                                // }
                                                matchIndex = 0;
                                                std::cout << "GPGGA found\n";
                                        }
                                }else {
                                        matchIndex = 0;
                                }
                                
                        }
                }


                LEDtimer +=1;
                if(LEDtimer == 95){
                        gpioWrite(LED, 1);
                }
                if(LEDtimer == 100){
                        gpioWrite(LED, 0);
                        LEDtimer=0;
                }

                
                usleep(10000); // 100ms
                
        }
        std::cout << sensorError.xAccelOffset;
        close(fd);

        return 0;
}
