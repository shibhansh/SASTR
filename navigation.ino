//this library has 2 independent parts one that will be responsible for motion between 2 points in a straight line and other will cause the turning of the bot
//this is the first part that will cause navigation
//this function will move the motors forward at our decided PWM
float mod(float a){
    if(a>0)
      return a;
    else
      return -a;
}
float distance(float x,float y){
    float temp;
    temp=sqrt((x-prevlat)*(x-prevlat)+(x-prevlong)*(x-prevlong));
    return temp;
}
void reset(){
    prevlat=latitude;
    prevlong=longitude;
}
void forward(){
    //give the responsibel motors required PWM
}
//this will calclate the latitudes and lonitudes using the value of pos calculated by IMU
void latlongIMU(){
    //we find the components of pos along lat and long axis and add them to the initial latitude and longitude
}
float error(float x,float y){
    //calculaate the distance of point from the line
    float distance=0;
    distance = mod(latcoff*x+longcoff*y+constant)/sqrt(latcoff*latcoff+longcoff*longcoff);
    return distance;
}

//this part will be used for the turning of the bot as per the path

int reached(float x,float y){
//this will calculate the distance of our current position form the waypoint
    if(sqrt((x-nextlat)*(x-nextlat)+(x-nextlong)*(x-nextlong))<=1.5)
        return 1;
    else 
        return 0;
}

void turn(float b){
    //we will give the pwm to the motors according to the requirement here  
    float p,currentangle,angle=b;
    //angle has to be calculated using the current and previous latitudes and longitudes
    int maxpwm,minpwm;
    float error=currentangle-angle;
    while(mod(currentangle-angle)>0.1){
         //give pwm corresponding to the current error
         //take the value from the gps magnetometer
         currentangle= gpsyaw();
         Serial.print(currentangle);
         error=currentangle-0;
         if(error>0){
            digitalWrite(0,HIGH);
            digitalWrite(1,LOW);
            analogWrite(5,100);
            digitalWrite(2,LOW);
            digitalWrite(3,HIGH);
            analogWrite(6,100);
         }
         else{
            analogWrite(5,100);
            analogWrite(6,100);
            digitalWrite(1,HIGH);
            digitalWrite(0,LOW);
            digitalWrite(3,LOW);
            digitalWrite(2,HIGH);         
         }
    } 
}
