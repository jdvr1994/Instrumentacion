// ArduinoSerialMatlab - github.com/jdvr1994
// Copyright Juan David Velasquez 2018
// MIT License

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

void setup() {
     Serial.begin(19200);

     //----- Toda tu configuracion y codigo que tengas en el void setup-----
     //.
     //.
     //.
    //----- Inicializas y configuras tu sensor por i2c etc etc etc-----
    //.
    //.
    //.
    
}

void loop() {
    //------- Lees tus sensores y almacenas en tus variables ax, ay, az, gx, gy, gz
    //.
    //.
    //... Supongamos que tus medidas fuero las siguientes (OJO::: es un ejemplo, tus medidas seras las leidas desde el sensor)
    ax = 5345;
    ay = 12312;
    az = -19853;
    gx = -878;
    gy = 246;
    gz = -18;

    sendDataMatlab();
}


void sendDataMatlab(){
    //Con Serial.available()>0 compruebas si llego algo al buffer serial del Arduino
    if(Serial.available()>0){
        // Almacenas el byte que fue enviado desde Matlab en la variable inbyte
        char inbyte=Serial.read();

        //Compruebas si lo que envio Matlab fue una 'a' , 'g' 
        //::::: Si mandamos 'a' desde Matlab le responderemos con los datos del acelerometro de la siguiente forma:
        //     dataAX,dataAY,dataAZ\n      (\n == salto de linea efectuado por el metodo println )
        if(inbyte=='a'){
              Serial.print(ax); Serial.print(",");
              Serial.print(ay); Serial.print(",");
              Serial.println(az);
        }
        
        //::::: Si mandamos 'g' desde Matlab le responderemos con los datos del giroscopos de la siguiente forma:
        //     dataGX,dataGY,dataGZ\n      (\n == salto de linea efectuado por el metodo println )
        if(inbyte=='g'){
              Serial.print(gx); Serial.print(",");
              Serial.print(gy); Serial.print(",");
              Serial.println(gz); 
        }
    }
}
