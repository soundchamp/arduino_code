/* four pin module -
with keypad */

#include <Keypad.h>
#include <SoftwareSerial.h>


const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = { {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'} };
byte rowPins[ROWS] = {22, 24, 26, 28};
byte colPins[COLS] = {30, 32, 34};

// Exercise 3 Setting
int numberHolder [255]; //hold inputs
int counter = 0;  //hold # of inputs


Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
SoftwareSerial bluetooth(10, 11);

void setup() {
  
  
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Keypad Live...");
  Serial.println("");
  bluetooth.begin(9600);
}

void loop() {

  if (bluetooth.available()) {
    Serial.write(bluetooth.read());
  }
  if (Serial.available()) {
  
    int input = Serial.parseInt();
    
    
    bluetooth.write(input);
  }


  digitalKeypad();


}


void digitalKeypad()
{
  char key = keypad.getKey();

  if (key != NO_KEY) {
      Serial.print(key);
      if ((key != '#') && (key != '*')) {
        if (numberHolder[counter] == NULL || numberHolder[counter] == 0){
          numberHolder[counter] = (int)key - 48;    //Convert from ASCII to Integer
        }
        else{
          numberHolder[counter] *= 10;
          numberHolder[counter] += (int)key - 48;
        }
      }

      if (key == '#'){  //Equals Sign
        int output = 1;
        
        for (int i = 0; i <= counter; i++){
          output *= numberHolder[i];
          Serial.println("");
          Serial.print("You entered ");
          Serial.print(output);
          Serial.println("");
          bluetooth.write(output);
          numberHolder[i] = 0;
        }
        
        counter = 0;
      }
  }
}
