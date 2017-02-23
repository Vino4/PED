#define INPUT_SIZE 27//expected max input length

#include <Servo.h> //to help operate the ESCs

Servo escFR;   // create servo object to control front right ESC
Servo escFL;  // create servo object to control front left ESC
Servo escBR; // create servo object to control back right ESC
Servo escBL;// create servo object to control back left ESC

char input[INPUT_SIZE + 1];//used to read input from serial, +1 for end character 0
char* command; //to store the command separated from the other commands
byte size; //size of input
int rotationCounter = 0;//used for a mod clock to forward commands to the proper motor
int currentChoice;//rotationCounter's complement

void setup(){
    Serial.begin(115200); //open serial with PAUD 115200
    escFR.writeMicroseconds(700); //set initial front right ESC signal
    escFL.writeMicroseconds(700); //set initial front left ESC signal
    escBR.writeMicroseconds(700); //set initial back right ESC signal
    escBL.writeMicroseconds(700); //set initial back left ESC signal
    escFR.attach(3);  //the pin for the front right ESC control
    escFL.attach(5);  //the pin for the front left ESC control
    escBR.attach(6);  //the pin for the back right ESC control
    escBL.attach(9);  //the pin for the back left ESC control
    Serial.println("Serial and ESC set up is complete."); // show in serial that set up is done
    Serial.println("input must be in the format x:yyyy,x:yyyy,x:yyyy,x:yyyy"); // instructions
    Serial.println("where x is 1 or 0 indicating whether the frequency should be changed or not respectively"); // instructions
    Serial.println("and yyyy is the PWM frequency which must be between 600 and 2000."); // instructions
    Serial.println("the order of frequencies will be forwarded to the motors in the following order: front-left, front-right, back-left, back right."); // instructions
}

void loop() {
    size = Serial.readBytes(input, INPUT_SIZE);//read an input of max 27 characters
    input[size] = 0;// Add the end character 0 to the end of the string
    command = strtok(input, ",");//get the next part of input before "," (next command part).
    while (command != 0){
        char* separator = strchr(command, ':');// split command into two parts (before and after ":")
        if (separator != 0){
            currentChoice = rotationCounter%4+1;//current motor choice (rotates between 1 and 4)
            *separator = 0;// end the command string where ":" is to split it
            int trigger = atoi(command);//parse first part into the trigger int
            ++separator;
            int frequency = atoi(separator);//parse second part into the frequency int
            Serial.print("Received: "); // confirmation of reception
            Serial.print(trigger); // confirmation of reception
            Serial.print(":"); // confirmation of reception
            Serial.println(frequency); // confirmation of reception
            rotationCounter++;//increase the motor selection clock

            switch (currentChoice){//send the frequency to the proper motor
                case 1:
                    if (trigger && frequency >= 600 && frequency <= 2000){
                    escFL.writeMicroseconds(frequency); // forward frequency to front-left motor
                    Serial.print("PWM frequency: "); // confirmation of reception
                    Serial.print(frequency); // confirmation of reception
                    Serial.println(" forwarded to front-left motor."); // confirmation of reception
                    }
                    break;
                case 2:
                    if (trigger && frequency >= 600 && frequency <= 2000){
                    escFR.writeMicroseconds(frequency); // forward frequenc yto front-right motor
                    Serial.print("PWM frequency: "); // confirmation of reception
                    Serial.print(frequency); // confirmation of reception
                    Serial.println(" forwarded to front-right motor."); // confirmation of reception
                    }
                    break;
                case 3:
                    if (trigger && frequency >= 600 && frequency <= 2000){
                    escBL.writeMicroseconds(frequency); //forward frequency to back-left motor
                    Serial.print("PWM frequency: "); // confirmation of reception
                    Serial.print(frequency); // confirmation of reception
                    Serial.println(" forwarded to back-left motor."); // confirmation of reception
                    }
                    break;
                case 4:
                    if (trigger && frequency >= 600 && frequency <= 2000){
                    escBR.writeMicroseconds(frequency); //forward frequency back-right motor
                    Serial.print("PWM frequency: "); // confirmation of reception
                    Serial.print(frequency); // confirmation of reception
                    Serial.println(" forwarded to back-right motor."); // confirmation of reception
                    }
                    break;

                default:
                    break;
            }
        }
        command = strtok(0, ",");//parse the next command
    }
}
