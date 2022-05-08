#include "SparkFunLSM9DS1.h"

#define MAX_CALIB_GYRO_DEV 200 // deg^2
#define MAX_ACCEL_CALIB_DEV 0.1 // g

// Note that these calibration procedures were originally from https://github.com/FemmeVerbeek/Arduino_LSM9DS1
//**********************************************************************************************************************************
//*********************************************              Gyroscope                **********************************************
//**********************************************************************************************************************************

boolean gyroOffsetOK = false;
boolean gyroSlopeOK[3] = {false, false, false};
char xyz[3] = {'X', 'Y', 'Z'};

char readChar()
{
    char ch;
    while (!Serial.available())
        ; // wait for character to be entered
    ch = toupper(Serial.read());
    delay(10);
    while (Serial.available())
    {
        Serial.read();
        delay(1);
    } // empty readbuffer
    return ch;
}

void readAnswer(char msg[], uint16_t &param)
{
    char ch = 0;
    byte count = 0;
    const byte NofChars = 8;
    char ans[NofChars];
    //  float val;
    while (Serial.available())
    {
        Serial.read();
    } // empty read buffer
    Serial.print(msg);
    Serial.print(param);
    Serial.print(F(" Enter new value "));
    while (byte(ch) != 10 && byte(ch) != 13 && count < (NofChars - 1))
    {
        if (Serial.available())
        {
            ch = Serial.read();
            ans[count] = ch;
            count++;
        }
    }
    ans[count] = 0;
    Serial.println(ans);
    if (count > 1)
        param = atoi(ans);
    while (Serial.available())
    {
        Serial.read();
    }
    Serial.println("\n\n\n\n\n\n\n");
}

void printSetParam(char txt[], float param[3])
{
    Serial.print(txt);
    Serial.print("");
    Serial.print(param[0], 6);
    Serial.print(", ");
    Serial.print(param[1], 6);
    Serial.print(", ");
    Serial.print(param[2], 6);
    Serial.print(");");
}

void LSM9DS1::openGyroCalibration()
{
    char incomingByte = 0;
    uint16_t turnangle = 180;
    uint16_t NofCalibrationSamples = 5000;
    setGyroSlopeBiases(1, 1, 1);
    while (1)
    {
        if (!gyroOffsetOK)
        {
            Serial.println(F("\n\nStep 1       CALIBRATE GYROSCOPE OFFSET "));
            Serial.println(F("First choose the sample frequency (ODR) and the Full Scale value. The accelerometer and the gyroscope"));
            Serial.println(F("share their ODR, so the setting here must be the same as in the DIY_Calibration_Gyroscope sketch."));
            Serial.println(F("This is far more important for the Gyroscope than for the accelerometer. "));
            Serial.println(F("Next enter \"O\" to start the gyroscope offset measurement. \nDuring this offset measurement the sensor must be kept still.\n"));
        }
        else
        {
            Serial.println(F("\n\nStep 2       CALIBRATE GYROSCOPE SLOPE"));
            Serial.println(F("During a slope calibration the sensor must be rotated calmly about one axis over a known angle."));
            Serial.println(F("Change the angle to your convenience. A larger angle is more accurate, but more difficult to do."));
            Serial.println(F("The rotation must be pure, without much rotation about the other two axes. It can be done by hand."));
            Serial.println(F("Keeping the board on a flat surface with the rotation axis vertical while turning is good enough."));
            Serial.println(F("For an accurate result you can start and end with its side pushed against a non moving object."));
            Serial.println(F("When you're done turning the sensor, press (Enter) to stop measuring. Each of the axes X,Y and Z "));
            Serial.println(F("must be measured. The program automatically detects which. \n"));

            Serial.println(F(" (A) Change the measuring angle to turn the board"));
            Serial.print(F(" (C) Calibrate Slope ==>> Turn the board over "));
            Serial.print(turnangle);
            Serial.println(F("° and press enter when finished "));
        }
        Serial.print(F(" (N) Number of calibration samples: "));
        Serial.println(NofCalibrationSamples);
        Serial.println(F(" (O) Calibrate Offset (keep board still during measurement)"));
        Serial.println(F(" (X) Exit"));

        Serial.println(F("\nOffset calibration ( -OK- )"));
        Serial.print(F("Slope calibration axis  "));
        for (int i = 0; i <= 2; i++)
        {
            Serial.print(xyz[i]);
            if (gyroSlopeOK[i])
                Serial.print(F("= ( -OK- ) "));
            else
                Serial.print(F("= not done "));
        }
        Serial.println();
        printSetParam("   imu.setGyroOffsetBiases(", gOffsetBias);
        Serial.println();
        printSetParam("   imu.setGyroSlopeBiases(", gSlopeBias);
        Serial.println();
        incomingByte = readChar();
        switch (incomingByte)
        {
        case 'A':
        {
            readAnswer("\n\n\n\n\n\nMeasurement turnangle for the board: ", turnangle);
            break;
        }
        case 'C':
        {
            Serial.print(F("\n\n\n\n\n\nMeasuring. Turn the sensor over "));
            Serial.print(turnangle);
            Serial.println(F(" degrees\n"));
            Serial.println(F("Press Enter when finished."));
            calibrateGyroSlope(turnangle);
            break;
        }
        case 'N':
        {
            readAnswer("\n\n\n\n\n\nThe number of calibration samples: ", NofCalibrationSamples);
            break;
        }
        case 'O':
        {
            calibrateGyroOffset(NofCalibrationSamples);
            break;
        }
        case 'X':
        {
            return;
        }
        }
        Serial.println("");
    }
}

void LSM9DS1::calibrateGyroOffset(uint16_t sampleCount) // don't move the board during calibration
{
    float x, y, z; // , addX=0, addY=0, addZ=0  ;
    Serial.println(F("\n\n\n\nMeasuring offset. Just a moment."));
    Serial.println(F("\n\nKeep the board still during measurement"));
    readGyroForAvg(sampleCount, x, y, z);
    setGyroOffsetBiases(x, y, z); // Store the average measurements as offset
    Serial.print("\n\n\n\n\n");
    gyroOffsetOK = true;
}

void LSM9DS1::calibrateGyroSlope(uint8_t turnangle) // rotate board over known angle
{
    boolean validMmt = false;
    float dirX = 0, dirY = 0, dirZ = 0, sigmaX2 = 0, sigmaY2 = 0, sigmaZ2 = 0;
    float maxXYZ = 0;
    unsigned int count = 0;
    while (!Serial.available()) // measure until enter key pressed
    {
        while (!gyroAvailable())
            ;
        readGyro(true);
        dirX += (gx - gOffsetBias[0]) / getGyroODRFloat(); // slope is still raw but offset must already be calibrated
        dirY += (gy - gOffsetBias[1]) / getGyroODRFloat();
        dirZ += (gz - gOffsetBias[2]) / getGyroODRFloat();
        sigmaX2 += gx * gx;
        sigmaY2 += gy * gy;
        sigmaZ2 += gz * gz;
        if (count == 0)
            maxXYZ = abs(gx);
        maxXYZ = max(maxXYZ, abs(gx));
        maxXYZ = max(maxXYZ, abs(gy));
        maxXYZ = max(maxXYZ, abs(gz));
        count++;
        if ((count % 30) == 0)
            Serial.print('.');
        digitalWrite(LED_BUILTIN, (millis() / 125) % 2); // blink onboard led every 250ms
    }
    digitalWrite(LED_BUILTIN, 0); // led off
    Serial.readStringUntil(13);   // Empty read buffer
    Serial.print(F("\n\n\nMeasured direction change X "));
    Serial.print(dirX, 6);
    Serial.print("°\tY ");
    Serial.print(dirY, 6);
    Serial.print("°\t Z ");
    Serial.println(dirZ, 6);
    Serial.print("°");
    sigmaX2 /= count;
    sigmaY2 /= count;
    sigmaZ2 /= count;
    Serial.print(F("Std.dev. "));
    Serial.print(sigmaX2, 6);
    Serial.print('\t');
    Serial.print(sigmaY2, 6);
    Serial.print('\t');
    Serial.println(sigmaZ2, 6);
    Serial.print(F("percentage of Full Scale "));
    Serial.print(100 * maxXYZ / getGyroScaleInt());
    Serial.println('%');
    if (maxXYZ / getGyroScaleInt() > 0.95)
        Serial.print(F("Maximum rotation speed reached. Choose a different FS setting or turn more slowly."));
    else
    {
        dirX = abs(dirX);
        dirY = abs(dirY);
        dirZ = abs(dirZ);
        if (dirX > max(dirY, dirZ))
        {
            if (sigmaY2 < MAX_CALIB_GYRO_DEV && sigmaZ2 < MAX_CALIB_GYRO_DEV)
            {
                validMmt = true;
                gSlopeBias[0] = turnangle / dirX;
                gyroSlopeOK[0] = true;
                Serial.print("Valid measurement, X detected, slope ");
                Serial.println(gSlopeBias[0]);
            }
        }
        if (dirY > max(dirX, dirZ))
        {
            if (sigmaX2 < MAX_CALIB_GYRO_DEV && sigmaZ2 < MAX_CALIB_GYRO_DEV)
            {
                validMmt = true;
                gSlopeBias[1] = turnangle / dirY;
                gyroSlopeOK[1] = true;
                Serial.print("Valid measurement, Y detected, slope ");
                Serial.println(gSlopeBias[1]);
            }
        }
        if (dirZ > max(dirY, dirX))
        {
            if (sigmaY2 < MAX_CALIB_GYRO_DEV && sigmaX2 < MAX_CALIB_GYRO_DEV)
            {
                validMmt = true;
                gSlopeBias[2] = turnangle / dirZ;
                gyroSlopeOK[2] = true;
                Serial.print("Valid measurement, Z detected, slope ");
                Serial.println(gSlopeBias[2]);
            }
        }
    }
    if (!validMmt)
    {
        Serial.println(F("\n\nNot a valid measurement!"));
        delay(2000);
    }
    else
        Serial.println("\n\n");
}

void LSM9DS1::readGyroForAvg(int16_t sampleCount, float &averX, float &averY, float &averZ)
{
    float x, y, z;
    averX = 0;
    averY = 0;
    averZ = 0;
    for (int16_t i = 1; i <= sampleCount; i++)
    {
        while (!gyroAvailable())
            ;
        readGyro(false);
        averX += gx;
        averY += gy;
        averZ += gz;
        digitalWrite(LED_BUILTIN, (millis() / 125) % 2); // blink onboard led every 250ms
        if ((i % 30) == 0)
            Serial.print('.');
    }
    averX /= sampleCount;
    averY /= sampleCount;
    averZ /= sampleCount;
    digitalWrite(LED_BUILTIN, 0); // led off
}

//**********************************************************************************************************************************
//*********************************************              Accelerometer            **********************************************
//**********************************************************************************************************************************

void LSM9DS1::openAccelCalibration()
{
    setAccelSlopeBiases(1, 1, 1);

    char incomingByte = 0;
    uint16_t NofCalibrationSamples = 5000;
    while (1)
    {
        Serial.println(F("\n\n"));
        Serial.println(F(" Calibrate Accelerometer Offset and Slope"));
        Serial.println(F(" Before calibrating choose the Full Scale (FS) setting and Output Data Rate (ODR)"));
        Serial.println(F(" Place the board on a horizontal surface with one of its axes vertical and hit enter to start a calibration"));
        Serial.println(F(" measurement. Each of the axes must be measured pointing up and pointing down, so a total of 6 measurements."));
        Serial.println(F(" The program recognizes which axis is vertical and shows which were measured successfully. If the angle is too"));
        Serial.println(F(" far expected the measurement is not valid.\n  "));
        Serial.println(F(" (enter)  Start a calibration measurement. "));
        Serial.print(F("   (N)    Number of calibration samples: "));
        Serial.println(NofCalibrationSamples);
        Serial.println(F("   (X) Exit"));


        Serial.print(F(" Measured status of axis \n "));
        for (int i = 0; i <= 2; i++)
        {
            Serial.print(xyz[i]);
            if (bitRead(acceMMlOK, i) == 1)
                Serial.print("+ = ( -OK- ) ");
            else
                Serial.print("+ = not done ");
        }
        Serial.print("\n ");
        for (int i = 0; i <= 2; i++)
        {
            Serial.print(xyz[i]);
            if (bitRead(acceMMlOK, i + 3) == 1)
                Serial.print("- = ( -OK- ) ");
            else
                Serial.print("- = not done ");
        }
        Serial.println();

        printSetParam("   imu.setAccelOffsetBiases(", aOffsetBias);
        Serial.println();
        printSetParam("   imu.setAccelSlopeBiases(", aSlopeBias);
        Serial.println("\n\n");
        incomingByte = readChar(); // wait for and get keyboard input
        switch (incomingByte)
        {
        case 'N':
        {
            readAnswer("\n\n\n\n\n\nThe number of calibration samples ", NofCalibrationSamples);
            break;
        }
        case 'C':
        default:
            calibrateAccel(NofCalibrationSamples);
        }
    }
}

void LSM9DS1::calibrateAccel(uint16_t sampleCount)
{
    boolean validMmt = false;
    float x, y, z;
    Serial.println(F("Measuring"));
    readAccelForAvg(sampleCount, x, y, z);
    if (abs(x) > max(abs(y), abs(z)))
    {
        Serial.println(F("\nX detected"));
        if (sqrt(y * y + z * z) / x < MAX_ACCEL_CALIB_DEV)
        {
            validMmt = true;
            if (x > 0)
            {
                maxAX = x;
                acceMMlOK = acceMMlOK | 0b00000001;
            }
            else
            {
                minAX = x;
                acceMMlOK = acceMMlOK | 0b00001000;
            }
        }
    }
    if (abs(y) > max(abs(x), abs(z)))
    {
        Serial.println(F("\nY detected"));
        if (sqrt(x * x + z * z) / y < MAX_ACCEL_CALIB_DEV)
        {
            validMmt = true;
            if (y > 0)
            {
                maxAY = y;
                acceMMlOK = acceMMlOK | 0b00000010;
            }
            else
            {
                minAY = y;
                acceMMlOK = acceMMlOK | 0b00010000;
            }
        }
    }
    if (abs(z) > max(abs(x), abs(y)))
    {
        Serial.println(F("\nZ detected"));
        if (sqrt(x * x + y * y) / z < MAX_ACCEL_CALIB_DEV)
        {
            validMmt = true;
            if (z > 0)
            {
                maxAZ = z;
                acceMMlOK = acceMMlOK | 0b00000100;
            }
            else
            {
                minAZ = z;
                acceMMlOK = acceMMlOK | 0b00100000;
            }
        }
    }
    setAccelOffsetBiases((maxAX + minAX) / 2, (maxAY + minAY) / 2, (maxAZ + minAZ) / 2);
    setAccelSlopeBiases((maxAX - minAX) / 2, (maxAY - minAY) / 2, (maxAZ - minAZ) / 2);

    if (!validMmt)
    {
        Serial.println(F("\n\n\nNot a valid measurement!  "));
        Serial.print(" x=");
        Serial.print(x);
        Serial.print("  y=");
        Serial.print(y);
        Serial.print("  z=");
        Serial.print(z);
        Serial.println();
    }
}

void LSM9DS1::readAccelForAvg(uint16_t sampleCount, float &averX, float &averY, float &averZ)
{
    averX = 0;
    averY = 0;
    averZ = 0;
    for (uint16_t i = 1; i <= sampleCount; i++)
    {
        while (!accelAvailable());
        readAccel(false);
        averX += ax;
        averY += ay;
        averZ += az;
        digitalWrite(LED_BUILTIN, (millis() / 125) % 2); // blink onboard led every 250ms
        if ((i % 30) == 0)
            Serial.print('.');
    }
    averX /= sampleCount;
    averY /= sampleCount;
    averZ /= sampleCount;
    digitalWrite(LED_BUILTIN, 0); // led off
}