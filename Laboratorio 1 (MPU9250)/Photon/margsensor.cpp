// This #include statement was automatically added by the Particle IDE.
#include "mpu9250.h"
#include <application.h>
#include "math.h"

#define GYRO_CONVERSION_FS_SEL_0    (250*2)/65535 
#define GYRO_CONVERSION_FS_SEL_1    (500*2)/65535 
#define GYRO_CONVERSION_FS_SEL_2    (1000*2)/65535 
#define GYRO_CONVERSION_FS_SEL_3    (2000*2)/65535 

#define ACCEL_CONVERSION_FS_SEL_0    (2*2)/65535
#define ACCEL_CONVERSION_FS_SEL_1    (4*2)/65535
#define ACCEL_CONVERSION_FS_SEL_2    (8*2)/65535
#define ACCEL_CONVERSION_FS_SEL_3    (16*2)/65535

#define MAG_CONVERSION_16_BITS      (4912*2)/65535

float DegradToRad=0.0005321125768275396;// pi/(180*32.8)

//---------------- WIFI --------------
UDP Udp;
unsigned int localPort = 8888;
IPAddress remoteIP(192,168,0,12);
int port = 8889;

#define NUM_NODOS 1
MPU9250 margSensor(0);
int16_t IMU[1][10]; //[NumNodos][NumSensors] (3gyros - 3 Accel - 1 Temp - 3 Mag)

float 
    gx,gy,gz,
    ax,ay,az,
    mx,my,mz;

void setup() {
    //Congifuro y comienzo la comunicacion i2c y Serial
     Wire.setSpeed(CLOCK_SPEED_400KHZ);
     Wire.begin();
     Serial.begin(921600);
     
     //Establesco conexion Wifi y espero por un paquete Serial o Udp para seguir con el loop
     // de lo contrario mantengo la comunicacion con la nube Particle
     WiFi.on();
     WiFi.connect();
     waitUntil(WiFi.ready);
     Udp.begin(localPort);
     while(!Serial.available() & Udp.parsePacket() == 0) Particle.process();
     
     // Imprimo por serial la direccion Ip que le fue asignada a Photon
     Serial.println(WiFi.localIP());
     
    //Selecciono el nodo 0 y Establesco la configuracion inicial begin()
    margSensor.Select(0);
    margSensor.begin();
    
    //Set Offsets Giroscopios y acelerometros
    margSensor.setGyroOffsetX(0);
    margSensor.setGyroOffsetY(0);
    margSensor.setGyroOffsetZ(0);
    margSensor.setAccelOffsetX(0);
    margSensor.setAccelOffsetY(0);
    margSensor.setAccelOffsetZ(0);
    
    /* ============================
    Seleccione escala de Giroscopios:
    0 = ±250dps = 
    1 = ±500 dps = 
    2 = ±1000 dps = 
    3 = ±2000 dps =
    ===============================*/
    margSensor.setGyroScale(2);// +-1000 dps
    
    /* ============================
    Seleccione escala de Acelerometros:
    0 = ±2g
    1 = ±4g
    2 = ±8g
    3 = ±16g
    ===============================*/
    margSensor.setAccelScale(1);// +-4g
    /*=======================================================
     Configuracion de Filtro Pasa Bajas para Acelerometros
     Value = 3dB BW[Hz]     Rate[Khz]       Delay(ms)
        0  =    1046            4            0.503
        1  =     218            1            1.88   
        2  =     218            1            1.88
        3  =      99            1            2.88
        4  =    44.8            1            4.88
        5  =    21.2            1            8.87
        6  =    10.2            1           16.83
        7  =    5.05            1           32.48
        8  =     420            1            1.38
    =========================================================*/
    margSensor.setAccelDLPF(2);
}

void loop() {
        margSensor.Read9Axis(IMU[0]);
        Matlab2();
        UnityCalibracionSensores();
}


void SensorConversion(){
    ax=(IMU[0][0])*ACCEL_CONVERSION_FS_SEL_1;
    ay=(IMU[0][1])*ACCEL_CONVERSION_FS_SEL_1;
    az=(IMU[0][2])*ACCEL_CONVERSION_FS_SEL_1; 
    
    gx=(IMU[0][4])*GYRO_CONVERSION_FS_SEL_1;
    gy=(IMU[0][5])*GYRO_CONVERSION_FS_SEL_1;
    gz=(IMU[0][6])*GYRO_CONVERSION_FS_SEL_1; 
    
    mx=(IMU[0][7])*MAG_CONVERSION_16_BITS;
    my=(IMU[0][8])*MAG_CONVERSION_16_BITS;
    mz=(IMU[0][9])*MAG_CONVERSION_16_BITS; 
}
//----------------------------------------------Calibracion de sensores ----------------------------------------------
void SensorCalibrationMatlab(){
     ax =  0.00024492     *IMU[0][0]   - 0.0000044705    *IMU[0][1]  +  0.00000317350  *IMU[0][2]  +  4.76330;
     ay =  0.00000035499  *IMU[0][0]   + 0.0002446700    *IMU[0][1]  -  0.00000212930  *IMU[0][2]  +  5.57190;
     az = -0.00000054721  *IMU[0][0]   + 0.000001355     *IMU[0][1]  +  0.0002387      *IMU[0][2]  -  0.24021;
     
     gx = (IMU[0][4] - 20.315)*DegradToRad;
     gy = (IMU[0][5] - 26.361)*DegradToRad;
     gz = (IMU[0][6] + 25.2663)*DegradToRad; 
     
     mx = (IMU[0][7] - 39.4278)/165.0380;
     my = (IMU[0][8] - 425.3639)/165.4276;
     mz = (IMU[0][9] + 96.4758)/172.1092;
}

//---------------------------- Comunicacion Serial Matlab ------------------------------
void Matlab2(){
    if(Serial.available()>0){
        char inbyte=Serial.read();
        if(inbyte=='a'){
              Serial.print(IMU[0][0]); Serial.print(",");
              Serial.print(IMU[0][1]); Serial.print(",");
              Serial.println(IMU[0][2]);
        }
        
        if(inbyte=='b'){
              Serial.print(IMU[0][4]); Serial.print(",");
              Serial.print(IMU[0][5]); Serial.print(",");
              Serial.println(IMU[0][6]); 
        }
        
        if(inbyte=='c'){
              Serial.print(IMU[0][7]); Serial.print(",");
              Serial.print(IMU[0][8]); Serial.print(",");
              Serial.println(IMU[0][9]); 
        }
    }
}

//-------------------- Transmicion UDP Data RAW Unity ------------------------
void UnityCalibracionSensores(){
    if (Udp.parsePacket() > 0) {
        char inbyte= Udp.read();
        
        if(inbyte=='s'){
            IPAddress ipAddress = Udp.remoteIP();
            int port = Udp.remotePort();
            Udp.beginPacket(ipAddress, port);
            
            for(int i=0;i<NUM_NODOS;i++){
                if((IMU[i][0]+IMU[i][1]+IMU[i][2])!=0)Udp.write(i);
            }
            
            Udp.endPacket();
            
        }else{
            int inbyteNum = inbyte - '0';
            
            uint8_t nodo = inbyteNum/3;
            uint8_t data =  (inbyteNum%3)*3;
            if(data>0)data++;
            
            IPAddress ipAddress = Udp.remoteIP();
            int port = Udp.remotePort();
            Udp.beginPacket(ipAddress, port);
            
            for(int i=data;i<data+3;i++){
                Udp.write((uint8_t)(IMU[nodo][i]>>8));
                Udp.write((uint8_t)IMU[nodo][i]);
            }
            
            Udp.endPacket();
        }
    }
}