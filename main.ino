#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>
#include <QueueList.h>

Servo LServo;  // Declare Left servo
Servo RServo;  // Declare right servo

enum Execute {MAP, GOAL};

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

#define RED 0x1
#define GREEN 0x2
#define BLUE 0x4
#define YELLOW 0x3
#define WHITE 0x7

//ENCODERS
int lEncPin = 10;
int rEncPin = 11;
int rEncCount = 0;
int lEncCount = 0;
int lEncLast = 0;
int rEncLast = 0;
int lEncVal = 0;
int rEncVal = 0;

//each cell will contain the following format:
// { (visited bit), (path to goal bit), (previous cell num), (west wall bit), (north wall bit), (east wall bit), (south wall bit)}
int maze[16][7] = {{0,0,0,1,1,0,0}, {0,0,0,0,1,0,0}, {0,0,0,0,1,0,0}, {0,0,0,0,1,1,0},    
                   {0,0,0,1,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,1,0},
                   {0,0,0,1,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,1,0},
                   {0,0,0,1,0,0,1}, {0,0,0,0,0,0,1}, {0,0,0,0,0,0,1}, {0,0,0,0,0,1,1}};

const int SSensors[3] = {A1, A0, A2};
const char* erraseString  = "  ";

Execute e = (Execute)MAP;
int Front_Readings[20];
int Left_Readings[20];
int Right_Readings[20];
int Sensor_Count = 0;

int Min = 0;
int i = 0, j = 0, temp = 0;
int Medians[3];
float Distances[3] = {0, 0, 0};

bool Stop = true;
String Color = "";

float LSpeed = 0;
float RSpeed = 0;

int Red[20];
int Green[20];
int Blue[20];
int RGB[3] = {0,0,0};

uint8_t buttons;
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

int grid = 0;
int start = -1;
int goal = -1;
int dir = -1;
int compass[4] = {0,0,0,0};

void setup() {
  // set color sensor 
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // set frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  pinMode(lEncPin, INPUT_PULLUP);
  pinMode(rEncPin, INPUT_PULLUP);
  
  LServo.attach(2);  // Attach right signal to pin 2
  RServo.attach(3);  // Attach right signal to pin 3
  
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Start Locat.");
  lcd.setCursor(0, 1);
  lcd.print("1");
  
  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);
 
  i = 0;
  while (1) {
    buttons = lcd.readButtons();

    if (buttons & BUTTON_SELECT) {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0,0);
      if(start < 0) {
        start = i;
        lcd.print("Ending Locat.");
        lcd.setCursor(0,1);
        lcd.print("1");
        i = 0;
      } else if (goal < 0) {
        goal = i;
        lcd.print("Orientation");
        lcd.setCursor(0,1);
        lcd.print("West");
        i = 0;
      } else if (dir < 0) {
        dir = i;
        lcd.print("Exit");
        lcd.setCursor(0,1);
        lcd.print("            ");
      } else {
        break;
      }
      delay(100);
    } else if (buttons & BUTTON_UP) {
      i += 1;
      lcd.setCursor(0,1);
      lcd.print("           ");
      lcd.setCursor(0,1);  
      if (start < 0 || goal < 0) {
        if (i > 15) {
          i = 0;
        }
        lcd.print(i + 1);
      } else if (dir < 0) {
        if (i > 3) {
          i = 0;
        }
        switch (i) {
          case 0:
            lcd.print("West");
            break;
          case 1:
            lcd.print("North");
            break;
          case 2:
            lcd.print("East");
            break;
          case 3:
            lcd.print("South");
            break;
        }
      }
      delay(100);
    } else if (buttons & BUTTON_DOWN) {
      i -= 1;
      lcd.setCursor(0,1);
      lcd.print("           ");
      lcd.setCursor(0,1);  
      if (start < 0 || goal < 0) {
        if (i < 0) {
          i = 15;
        }
        lcd.print(i + 1);
      } else if (dir < 0) {
        if (i < 0) {
          i = 3;
        }
        switch (i) {
          case 0:
            lcd.print("West");
            break;
          case 1:
            lcd.print("North");
            break;
          case 2:
            lcd.print("East");
            break;
          case 3:
            lcd.print("South");
            break;
        }
      }
      delay(100);
    }
  }

  lcd.setCursor(0, 0);

  for (i = 0; i < 16; i++) {
    if (i != start) {
      maze[i][0] = 0;
      lcd.print("X");
    } else {
      grid = start;
      maze[i][0] = 1;
      lcd.print("0");
    }
  }
  lcd.setCursor(0, 1);
  lcd.print("G1  W  N  E  S ");

  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);

  while (Stop) {
    buttons = lcd.readButtons();
    if (buttons && BUTTON_SELECT) {
      Stop = false;
    }
    
    if (Sensor_Count < 20) {
      UpdateSensors(); 
    } else {
      UpdateSensors();
      UpdateCell();
    }
    delay(50);
  }
  DetermineDirection();
}

bool line = false;
bool finished = false;

void loop() {
  if (finished) {
    LServo.writeMicroseconds(1500);
    RServo.writeMicroseconds(1500);
    switch(e) {
      case MAP:
        finished = false;
        grid = start;
        for (i = 0; i < 16; i++) {
          if (i != grid) {
            maze[i][0] = 0;
          } else {
            maze[i][0] = 1;
          }
        }
        
        e = GOAL;
        FindPath(grid);

        lcd.setCursor(0, 0);
        for (i = 0; i < 16; i++) {
          if (i != grid) {
            maze[i][0] = 0;
            lcd.print("X");
          } else {
            maze[i][0] = 1;
            lcd.print("0");
          }
        }
        
        lcd.setCursor(1,1);
        lcd.print("  ");
        lcd.setCursor(1,1);
        lcd.print(grid + 1);
        while (1) {
          buttons = lcd.readButtons();
          if (buttons && BUTTON_SELECT) {
            delay(100);
            break;
          }

          if (Sensor_Count < 20) {
            UpdateSensors(); 
          } else {
            UpdateSensors();
            UpdateCell();
          }
          delay(100);
        }
        
        DetermineDirection();
        break;
        
      case GOAL:
        Stop = true;
        break; 
    }

    LServo.writeMicroseconds(1500);
    RServo.writeMicroseconds(1500);
    Sensor_Count = 0;
    while (Sensor_Count < 20) {
      UpdateSensors();
      delay(100);
    }
  }

  UpdateSensors();
  Movement();
  
  if (LSpeed > 1600) {
    LSpeed = 1600;
  }

  if (RSpeed < 1400) {
    RSpeed = 1400;
  }

  LServo.writeMicroseconds(LSpeed);
  RServo.writeMicroseconds(RSpeed);
  delay(10);
}

bool CheckForFinish() {
  switch(e) {
    case MAP:
      for (i = 0; i < 16; i++) {
        if (maze[i][0] == 0) {
          return false;
        }
      }
      return true;
      
    case GOAL:
      if(grid != goal)
      {
        return false;
      }
      return true;
  }
}

void FindPath(int g) {
  QueueList<int> q;
  maze[g][0] = 1;
  maze[g][1] = 1;
  maze[g][2] = -1;

  q.push(g);
  while (!q.isEmpty()) {
    g = q.pop();
    maze[g][0] = 1;

    for (i = 0; i < 4; i++) {
      if (!maze[g][i + 3]) {
        switch(i) {
          case 0:
            if (!maze[g-1][0]) {
              q.push(g - 1);
              maze[g - 1][2] = g;
            }
            break;
          case 1:
            if (!maze[g - 4][0]) {
              q.push(g - 4);
              maze[g - 4][2] = g;
            }
            break;
          case 2:
            if (!maze[g+1][0]) {
              q.push(g+1);
              maze[g+1][2] = g;
            }
            break;
          case 3:
            if (!maze[g+4][0]) {
              q.push(g+4);
              maze[g+4][2] = g;
            }
            break;
        }
      }
    }
  }

  switch (e) {
    case GOAL:
      g = goal;
      break;
  }
  
  while (maze[g][2] != -1) {
    maze[g][1] = 1;
    g = maze[g][2];
  }
  maze[g][1] = 1;
}

void Movement() {
  if (Stop) {
    LSpeed = 1500;
    RSpeed = 1500;
    UpdateCell();
  } else {
    if (Color != "Floor") {
      line = true;
      lEncCount = 0;
      rEncCount = 0;
    }
    
    if (line) {
      if (lEncCount < 58 || rEncCount < 58) {
        LSpeed = 1520;
        RSpeed = 1482;
        if ((Distances[0] < 6 && Distances[0] != 0)) {
          LSpeed += 5;
          //LSpeed += 6 - Distances[0];
        }
        if ((Distances[2] < 6 && Distances[2] != 0)) {
          RSpeed -= 5;
          //RSpeed -= 6 - Distances[2];
        }
      } else {
        LSpeed = 1500;
        RSpeed = 1500;
        LServo.writeMicroseconds(LSpeed);
        RServo.writeMicroseconds(RSpeed);

        Sensor_Count = 0;
        
        while (Sensor_Count < 20) {
          UpdateSensors();
          delay(100);
        }
        
        UpdateSensors();
        UpdateCell();
        if (CheckForFinish()) {
          finished = true;
          line = false;
          return;
        }
        DetermineDirection(); 
        line = false;
      }
    } else {
      LSpeed = 1520;
      RSpeed = 1482;
      if ((Distances[0] < 6 && Distances[0] != 0)) {
        LSpeed += 5;
        //LSpeed += 6 - Distances[0];
      }
      if ((Distances[2] < 6 && Distances[2] != 0)) {
        RSpeed -= 5;
        //RSpeed -= 6 - Distances[2];
      }
    }
  }
  lEncVal = digitalRead(lEncPin);
  rEncVal = digitalRead(rEncPin);

  if (lEncVal != lEncLast) lEncCount++;
  if (rEncVal != rEncLast) rEncCount++;
  lEncLast = lEncVal;
  rEncLast = rEncVal;
  delay(10);
}

void left (int num) {
  for (i = 0; i < num; i++) {
    lEncCount = 0;
    rEncCount = 0;
    while (lEncCount < 25 || rEncCount < 25) {
      LServo.writeMicroseconds(1480);
      RServo.writeMicroseconds(1480);
    
      lEncVal = digitalRead(lEncPin);
      rEncVal = digitalRead(rEncPin);

      if (lEncVal != lEncLast) lEncCount++;
      if (rEncVal != rEncLast) rEncCount++;
      lEncLast = lEncVal;
      rEncLast = rEncVal;
    }
    dir--;
    if (dir < 0) {
      dir = 3;
    }
  }
}

void right(int num) {
  for (i = 0; i < num; i++) {
    lEncCount = 0;
    rEncCount = 0;
    while (lEncCount < 25 || rEncCount < 25) {
      LServo.writeMicroseconds(1520);
      RServo.writeMicroseconds(1520);
    
      lEncVal = digitalRead(lEncPin);
      rEncVal = digitalRead(rEncPin);

      if (lEncVal != lEncLast) lEncCount++;
      if (rEncVal != rEncLast) rEncCount++;
      lEncLast = lEncVal;
      rEncLast = rEncVal;
    }
    dir++;
    if (dir > 3) {
      dir = 0;
    }
  }
}

int dirTesting;
void DetermineDirection() {
  switch (e) {
    case MAP:
      dirTesting = dir - 1;
      if (dirTesting < 0) {
        dirTesting = 3;
      }
      for (i = 0; i < 4; i++) {
        if (compass[dirTesting] == 0 && !maze[grid][dirTesting + 3]) {
          switch (dirTesting) {
            case 0:
              if (!maze[grid - 1][0] && (grid % 4 != 0)) {
                if (dir == 3) {
                  right(1);
                } else if (dir != dirTesting) {
                  left(dir - dirTesting);
                }
                return;
              }
              break;
            case 1:
              if (!maze[grid - 4][0] && grid / 4 != 0) {
                if (dir == 0) {
                  right(1);
                } else if (dir != dirTesting) {
                  left(dir - dirTesting);
                }
                return;
              }
              break;
            case 2:
              if (!maze[grid + 1][0] && grid % 4 != 3) {
                if (dir == 1) {
                  right(1);
                } else if (dir != dirTesting) {
                  if (dir == 0) {
                    left(2);
                  } else {
                    left(1);
                  }
                }
                return;
              }
              break;
            case 3:
              if (!maze[grid + 4][0] && grid / 4 != 3) {
                if (dir == 2) {
                  right(1);
                } else if (dir != dirTesting) {
                  if (dir == 0){
                    left(1);
                  } else {
                    left(2);
                  }
                }
                return;
              }
              break;
          }
        }
        dirTesting++;
        if (dirTesting > 3) {
          dirTesting = 0;
        }
      }

      dirTesting = dir - 1;
      if (dirTesting < 0) {
        dirTesting = 3;
      }

      for (i = 0; i < 4; i++) {
        if (compass[dirTesting] == 0 && !maze[grid][dirTesting + 3]) {
          switch (dirTesting) {
            case 0:
              if (grid % 4 == 0) {
                break;
              }
              if (dir == 3) {
                right(1);
              } else if (dir != dirTesting) {
                left(dir - dirTesting);
              }
              return;
              break;
            case 1:
              if (grid / 4 == 0) {
                break;
              }
              if (dir == 0) {
                right(1);
              } else if (dir != dirTesting) {
                left(dir - dirTesting);
              }
              return;
              break;
            case 2:
              if (grid % 4 == 3) {
                break;
              }
              if (dir == 1) {
                right(1);
              } else if (dir != dirTesting) {
                if (dir == 0) {
                  left(2);
                } else
                {
                  left(1);
                }
              }
              return;
              break;
            case 3:
              if (grid / 4 == 3) {
                break;
              }
              if (dir == 2) {
                right(1);
              } else if (dir != dirTesting) {
                if (dir == 0) {
                  left(1);
                } else {
                  left(2);
                }
              } 
              return;
              break;
          }
        }
        dirTesting++;
        if (dirTesting > 3) {
          dirTesting = 0;
        }
      }
      left(2);
      break;
      
    case GOAL:
      dirTesting = -1;
      for (i = 0; i < 4; i++) {
        if (!maze[grid][i + 3]) {
          switch(i) {
            case 0:
              if (maze[grid - 1][1] && !maze[grid - 1][0]) {
                dirTesting = 0;
              }
              break;
            case 1:
              if (maze[grid - 4][1] && !maze[grid - 4][0]) {
                dirTesting = 1;
              }
              break;
            case 2:
              if (maze[grid + 1][1] && !maze[grid + 1][0]) {
                dirTesting = 2;
              }
              break;
            case 3:
              if (maze[grid + 4][1] && !maze[grid + 4][0]) {
                dirTesting = 3;
              }
              break;
          }

          if (dirTesting != -1) {
            break;
          }
        }
      }
  
      switch (dirTesting) {
        case 0:
          if (dir == 3) {
            right(1);
          } else if (dir != dirTesting) {
            left(dir - dirTesting);
          }
          break;
        case 1:
        case 2:
          if (dir < dirTesting) {
            right(dirTesting - dir);
          } else if (dir > dirTesting) {
            left(dir - dirTesting);
          }
          break;
        case 3:
          if (dir == 0) {
            left(1);
          } else if (dir != dirTesting) {
            right(dirTesting - dir);
          }
          break;
      }
      break;
    }
}

void UpdateCell() { //locality update

  dirTesting = dir;
  dirTesting--;
  if (dirTesting < 0) {
    dirTesting = 3;
  }
  for (i = 0; i < 3; i++) {
    if (Distances[i] != 0 && Distances[i] < 8) {
      compass[dirTesting] = 1;
      if (e == MAP) {
        maze[grid][dirTesting + 3] = 1;
        switch (dirTesting) {
          case 0:
            if (grid % 4 != 0) {
              maze[grid - 1][5] = 1;
            }
            break;
          case 1:
            if (grid / 4 != 0) {
              maze[grid - 4][6] = 1;
            }
            break;
          case 2:
            if (grid % 4 != 3) {
              maze[grid + 1][3] = 1;
            }
            break;
          case 3:
            if (grid / 4 != 3) {
              maze[grid + 4][4] = 1;
            }
            break;
        }
      }
    } else {
      compass[dirTesting] = 0;
    }
    
    dirTesting++;
    if (dirTesting > 3) {
      dirTesting = 0;
    }
  }

  compass[dirTesting] = -1;
  if (e == MAP) {
    maze[grid][dirTesting + 3] = 0;
  }
  
  for (i = 0; i < 4; i++) {
    lcd.setCursor(5 + 3 * i, 1);
    switch (compass[i]) {
      case 0:
        lcd.print("0");
        break;
      case 1:
        lcd.print("X");
        break;
      case -1:
        lcd.print("U");
        break;
    }
  }
}

void UpdateSensors() {
  if (Sensor_Count < 20) {
    Front_Readings[Sensor_Count] = analogRead(SSensors[1]);
    Right_Readings[Sensor_Count] = analogRead(SSensors[2]);
    Left_Readings[Sensor_Count] = analogRead(SSensors[0]);
    // get red filtered photodiodes 
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    Red[Sensor_Count] = pulseIn(sensorOut, LOW);
  
    // get Green filtered photodiodes
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    Green[Sensor_Count] = pulseIn(sensorOut, LOW);
  
    // get Blue filtered photodiodes
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    Blue[Sensor_Count] = pulseIn(sensorOut, LOW);
    Sensor_Count++;
  } else {
    Sensor_Count = 0;
    Medians[1] = median(Front_Readings);
    Medians[2] = median(Right_Readings);
    Medians[0] = median(Left_Readings);
    RGB[0] = median(Red);
    RGB[1] = median(Green);
    RGB[2] = median(Blue);

    convert();
    Color = color();
    
    if (Color == "Red" || Color == "Blue") {
      switch (dir) {
        case 0: //west
          lcd.setBacklight(GREEN);
          break;
        case 1: //north
          lcd.setBacklight(BLUE);
          break;
        case 2: //east
          lcd.setBacklight(RED);
          break;
        case 3: //south
          lcd.setBacklight(YELLOW);
          break;
        default:
          //should never reach here
          break;
      }
    } else {
      lcd.setBacklight(WHITE);
    }
  }
}

String color() {
  if ((RGB[0] <= 150)) {
    // red
    if(Color == "Floor") {
      if (dir == 1) {
        grid -= 4;
      } else if (dir == 3) {
        grid += 4;
      }
      maze[grid][0] = 1;
      lcd.setCursor(0,0);
      for (i = 0; i < 16; i++) {
        if (maze[i][0]) {
          lcd.print("0"); 
        } else {
          lcd.print("X");
        }
      }
      lcd.setCursor(1, 1);
      lcd.print(erraseString);
      lcd.setCursor(1, 1);
      lcd.print(grid + 1);
    }
    return "Red";
  } else if (RGB[1] <= 150) {
    // blue
    if (Color == "Floor") {
      if (dir == 0) {
        grid -= 1;
      } else if(dir == 2) {
        grid += 1;
      }
      maze[grid][0] = 1;
      lcd.setCursor(0,0);
      for (i = 0; i < 16; i++) {
        if (maze[i][0]) {
          lcd.print("0"); 
        } else {
          lcd.print("X");
        }
      }
      lcd.setCursor(1, 1);
      lcd.print(erraseString);
      lcd.setCursor(1, 1);
      lcd.print(grid + 1);
    }
    return "Blue";
  } else {
    // floor
    return "Floor";
  }
}

int median (int a[20]) {
  for (i = 0; i < 11; i++) {
    Min = i;
    for (j = i + 1; j < 20; j++) {
      if (a[j] < a[Min]) {
        Min = j;
      }
    }
    temp = a[i];
    a[i] = a[Min];
    a[Min] = temp;
  }
  return (a[9] + a[10]) / 2;
}

void convert() {
  for (i = 0; i < 3; i++) {
    Distances[i] = 10 * pow(Medians[i] * 0.0048828125, -1.15) * 0.394;
    if (Distances[i] > 10) {
      Distances[i] = 0;
    }
  }
}

