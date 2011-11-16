/*
 * IMC-PID auto tuner
 * PID with optional CO filter
 * Based on article: Controller Tuning Using Set Point Driven Data by Doug Cooper
 * http://www.controlguru.com/wp/p89.html
 */

#define IMC_PID_TUNE_CO_START 40
#define IMC_PID_TUNE_CO_INTERVAL 5
#define CHANGE_THRESHOLD 4
#define PROCESS_TIME_THRESHOLD 0.63
#define STABLE_MS_REQUIRED 50000
#define CAL_SMOOTHFACTOR SMOOTHFACTOR
// 1 sample per 20 seconds for 20 minutes = 60
#define NUM_SAMPLES_TO_STORE 60
#define MS_PER_SAMPLE 20000

void ImcPidTune (int targetTemp, boolean setWhenDone)
{
  int tempReadings[NUM_SAMPLES_TO_STORE][2] = {
    0        }; // Element 0 - raw temperature, Element 1 time in ms
  analogWrite(HEATER_0_PIN, 0); // Turn off heater
  digitalWrite(FAN_PIN, HIGH); // Turn on fan
  analogWrite(FAN_PIN, 255);
  Serial.println("echo: Auto tuning PID values...");
  Serial.println("echo: Please keep watch over your printer and be ready to shut it down if it acts strangely.");

  // If temperature goes past this point, abort test. It means IMC_PID_TUNE_TEST_CO is set too high!
  // It has to be high enough to move t he PV to somewhere in the range of the desired printing SP (raw temp),
  // but low enough that it doesn't keep heating past that point.
  int maxRaw = temp2analog(MAXTEMP);
  // Get base value first...
  int rawAvg = analogRead(TEMP_0_PIN);
  int lastRawAvg = rawAvg;
  int startingTemp = rawAvg;

  // First lets find out what controller output we need to hit the desired temperature
  int targetRaw = temp2analog(targetTemp);
  int lastTargetRawCO = IMC_PID_TUNE_CO_START;
  int targetRawCO = IMC_PID_TUNE_CO_START - IMC_PID_TUNE_CO_INTERVAL;
  targetRawCO = 105; // DEBUG
  int highestTemp = 0;

  Serial.print("echo: Discovering what controller output is required to reach ");
  Serial.print(targetTemp);
  Serial.println("C");
  unsigned long startMillis;
  unsigned long endMillis; // Potential end time of test
  unsigned long lastTempMillis = millis(); // Last time the temperature was printed
  while (highestTemp < targetRaw) // Keep looping till we hit the target temp
  { 
     break;     // DEBUG 
    targetRawCO += IMC_PID_TUNE_CO_INTERVAL;
    analogWrite(HEATER_0_PIN, targetRawCO); // Turn on heater
    endMillis = millis(); // Potential end time of test
    while ((millis() - startMillis) <= STABLE_MS_REQUIRED) // Spin to temp stabalises
    {
      delay(1000);

      rawAvg = (rawAvg * (SMOOTHFACTOR - 1) + analogRead(TEMP_0_PIN)) / SMOOTHFACTOR;
      if (rawAvg > highestTemp)
      {
        // Temp is still rising. Record and reset timer
        highestTemp = rawAvg; // record new highest temp
        endMillis = millis(); // reset timer
      }
      else if ((highestTemp - rawAvg) > CHANGE_THRESHOLD)
      {
        // Temperature hasn't stabalised yet.
        endMillis = millis(); // reset timer
      }
      else if (rawAvg >= maxRaw)
      {
        // IMC_PID_TUNE_TEST_CO is probably too high. Printer will overheat. Abort!
        analogWrite(HEATER_0_PIN, 0); // Turn off heater! But leave fan on.
        Serial.println("echo: ERROR! Heater pin set too high! Aborting PID tuning routine.");
        return;
      }

      if ((millis() - lastTempMillis) >= 3000) // Print temperature once every 3s
      {
        Serial.print("ok T:");
        Serial.print(analog2temp(rawAvg)); 
        Serial.print(", raw:");
        Serial.println(rawAvg);       
        lastTempMillis = millis();
        Serial.print("echo: Highest temp: ");
        Serial.println(highestTemp);
      }
    }
    Serial.print("echo: Temperature stabalised at: ");
    Serial.print(analog2temp(rawAvg));
    Serial.print(" (CO: ");
    Serial.print(targetRawCO);
    Serial.println(")");
  }

  analogWrite(HEATER_0_PIN, 0); // Turn off heater and start cooling down

  // Next check to see if the printer is cooling or heating.
  // To be considered successful, it has to stay at a stable temp for 20s
  Serial.println("echo: Attempting to establish ambient temperature");
  boolean warnedUser = false;
  int lowestTemp = rawAvg;
  startMillis = millis();
  while ((millis() - startMillis) <= STABLE_MS_REQUIRED) // Spin till temp stabalises before taking baseline readings
  {
    delay(5000); // 5 seconds
    rawAvg = (rawAvg * (SMOOTHFACTOR - 1) + analogRead(TEMP_0_PIN)) / SMOOTHFACTOR;
    Serial.print("ok T:");
    Serial.print(analog2temp(rawAvg)); 
    Serial.print(", raw:");
    Serial.println(rawAvg);       
    if (rawAvg < lowestTemp)
    {
      // Temp is still lowering
      lowestTemp = rawAvg; // set new lowest temp
      startMillis = millis(); // reset timer
    }
    else if ((rawAvg - lowestTemp) > CHANGE_THRESHOLD)
    {
      // Temp is oscilliating outside of acceptable change boundaries
      startMillis = millis(); // reset timer
    }
    else if (rawAvg >= maxRaw)
    {
      // Something is wrong. It's hitting the max temp! Abort!
      Serial.println("echo: It appears that the printer is somehow heating... Aborting process!");
      analogWrite(HEATER_0_PIN, 0); // Turn off heater (just to be safe)
      return;
    }
    Serial.print("echo: Lowest temp: ");
    Serial.println(lowestTemp);
  }
  int roomTempRaw = rawAvg;
  Serial.print("echo: Room temperature: ");
  Serial.print(analog2temp(roomTempRaw));
  Serial.print("C, raw: ");
  Serial.println(roomTempRaw);

  // Now we need to find the Process Dead Time, Process Time, and Process Gain (in this order)

  // Set controller output to known value
  Serial.println("echo: Determining base tuning constants.");
  analogWrite(HEATER_0_PIN, targetRawCO);
  Serial.print("echo: Heating printer. Heater pin set to ");
  Serial.println(targetRawCO);

  //digitalWrite(FAN_PIN, LOW); // Turn off fan... supposedly we are not supposed to model disturbance...?!?!
  //analogWrite(FAN_PIN, 0);

  // Find the Process dead time
  startMillis = millis();
  while ((rawAvg - roomTempRaw) <= (CHANGE_THRESHOLD * 2)) // Keep spinning till temp raw increases past the threshold * 2.
  {
    delay(HEATER_CHECK_INTERVAL);
    rawAvg = (rawAvg * (CAL_SMOOTHFACTOR - 1) + analogRead(TEMP_0_PIN)) / CAL_SMOOTHFACTOR;
  }
  unsigned long deadTimeMs = millis() - startMillis; // In ms
  float deadTime = deadTimeMs / 1000; // convert to seconds
  Serial.print("echo: Process dead time: ");
  Serial.print(deadTime);
  Serial.println("s");

  // Find the Process Time
  highestTemp = rawAvg;
  //int tempReadings[NUM_SAMPLES_TO_STORE][2] = {0}; // Element 0 - raw temperature, Element 1 time in ms
  int arrayIndex = 0;
  startMillis = millis();
  lastTempMillis = millis(); // Last time the temperature was printed
  unsigned long lastStoredMillis = millis(); // Last time a temp reading was stored
  endMillis = millis(); // Potential end time of test
  Serial.print("echo: startMillis: ");
  Serial.println(startMillis);
  while((millis() - endMillis) < STABLE_MS_REQUIRED) // Keep spinning till temperature has been stable for 20s
  {
    //delay(HEATER_CHECK_INTERVAL * 100); // Checking slower to allow temperature changes to be more pronounced
    delay(1000);
    rawAvg = (rawAvg * (CAL_SMOOTHFACTOR - 1) + analogRead(TEMP_0_PIN)) / CAL_SMOOTHFACTOR;

    // Record temp and time so that we can attempt to determine when in time we've hit hte 63% PV.
    if ((millis() - lastStoredMillis) >= MS_PER_SAMPLE)
    {
      tempReadings[arrayIndex][0] = rawAvg;
      tempReadings[arrayIndex++][1] = millis();
      arrayIndex = arrayIndex % NUM_SAMPLES_TO_STORE;
      lastStoredMillis = millis();
    }

    if (rawAvg > highestTemp)
    {
      // Temp is still rising. Record and reset timer
      highestTemp = rawAvg; // record new highest temp
      endMillis = millis(); // reset timer
    }
    else if ((highestTemp - rawAvg) > CHANGE_THRESHOLD)
    {
      // Temperature hasn't stabalised yet.
      endMillis = millis(); // reset timer
    }
    else if (rawAvg >= maxRaw)
    {
      // IMC_PID_TUNE_TEST_CO is probably too high. Printer will overheat. Abort!
      analogWrite(HEATER_0_PIN, 0); // Turn off heater! But leave fan on.
      Serial.println("echo: ERROR! Heater pin set too high! Aborting PID tuning routine.");
      return;
    }

    if ((millis() - lastTempMillis) >= 3000) // Print temperature once every 3s
    {
      Serial.print("ok T:");
      Serial.print(analog2temp(rawAvg)); 
      Serial.print(", raw:");
      Serial.println(rawAvg);       
      lastTempMillis = millis();
      Serial.print("echo: Highest temp: ");
      Serial.println(highestTemp);
    }
  }
  Serial.print("echo: Final temperature: ");
  Serial.println(analog2temp(rawAvg));

  // The Process time is 63% of the time it takes for the temperature to stabalise.
  // Look backwards through the array of temps to determine when that was.
  float pv63percent = rawAvg * PROCESS_TIME_THRESHOLD;
  int runCount = NUM_SAMPLES_TO_STORE; // This is to prevent a runaway loop if the array was not sufficently large.
  float processTime = -1;
  int currentReading = 0;
  int lastReading;
  while (runCount-- > 0)
  {
    arrayIndex--;
    if (arrayIndex < 0)
    {
      arrayIndex += NUM_SAMPLES_TO_STORE; // Cycle back to end
    }
    lastReading = currentReading;
    currentReading = tempReadings[arrayIndex][0];
    if (currentReading < pv63percent)
    {
      // We've found the 63% point... kinda
      // This processTime will be an aproximate. Due to the way we've arrived here, it'll be earlier than the 63% mark.
      processTime = tempReadings[arrayIndex][1];
      // Compensate for the above fact...
      processTime += (pv63percent - currentReading) / (lastReading - currentReading) * MS_PER_SAMPLE;
      processTime /= 1000; // convert to seconds
      break;
    }
  }
  if (processTime == -1)
  {
    Serial.println("echo: Error! Temperature array was not large enough to store all the values required to determine the process time!");
    Serial.println("echo: Error! Process time will be approximated as if the PV was growing linearly.");
    Serial.println("echo: Error! To get a more accurate value, either increase the number of array elements or sample time in the firmware.");
    processTime = (endMillis - startMillis)  * PROCESS_TIME_THRESHOLD;
  }

  Serial.print("echo: Process time: ");
  Serial.print(processTime);
  Serial.println("s");
  Serial.print("echo: ");
  /*
  Serial.println((endMillis));
  Serial.print("echo: ");
  Serial.println((startMillis));
  Serial.print("echo: ");
  Serial.println((endMillis - startMillis));
  Serial.print("echo: ");
  Serial.println((endMillis - startMillis)  * PROCESS_TIME_THRESHOLD);
  */
  // Process gain
  float processGain = (rawAvg - roomTempRaw) / targetRawCO; // The unit is C/pwmUnit
  Serial.print("echo: Process gain: ");
  Serial.print(processGain);
  Serial.print(" (deltaPV: ");
  Serial.print(rawAvg - roomTempRaw);
  Serial.println(")");

  // Completed data gathering. Turn off heater and fan.
  analogWrite(HEATER_0_PIN, 0);
  digitalWrite(FAN_PIN, LOW); // Turn off fan
  analogWrite(FAN_PIN, 0);


  float Tc = max(processTime, 8 * deadTime); // Moderate settings
  //Tc /= 10; // Aggressive settings
  // TC *= 10; // Conservative settings

  float Ti = processTime + 0.5 * deadTime;
  float Td = processTime * deadTime / (2 * processTime + deadTime);

  Serial.println("echo: Computed values:");
  Serial.print("echo:   Kp: ");
  Serial.println(processGain, 9);
  Serial.print("echo:   Tc: ");
  Serial.println(Tc, 9);
  Serial.print("echo:   Ti: ");
  Serial.println(Ti, 9);
  Serial.print("echo:   Td: ");
  Serial.println(Td, 9);

  // Calculating PID from IMC tuning correlations
  // With optional CO filtering (A term)
  float P = (Ti / (Tc + deadTime)) / processGain; // (processTime + 0.5 * deadTime) / (Tc + deadTime);
  float I = P / Ti;
  float D = P * Td;
  float A = Tc * Ti / (processTime * (Tc + deadTime)); // aka "alpha" or "CO filter"

  // This is due to Marlin scaling up the PID constants by 100
  P *= 100;
  I *= 100;
  D *= 100;
  A *= 100;

  Serial.print("echo: Calculated P:");
  Serial.print(P, 9);
  Serial.print(" ,I: ");
  Serial.print(I, 9);
  Serial.print(", D:");
  Serial.print(D, 9);
  Serial.print(" A:");
  Serial.print(A);

  Serial.println();

  if (setWhenDone)
  {
    Kp = P;
    Ki = I * PID_dT;
    Kd = D / PID_dT;
    // TODO Add "A"
    Serial.println("echo: Calculated PID settings entered into running state. Please update firmware or store new values into EEPROM.");
  }
  else
  {
    Serial.println("echo:");
    Serial.println("echo: WARNING! PID settings were merely computed. To use them, please enter the following command:");
    Serial.print("echo: M301 P");
    Serial.print(P, 9);
    Serial.print(" I");
    Serial.print(I, 9);
    Serial.print(" D");
    Serial.print(D, 9);
    Serial.print(" A");
    Serial.println(A, 9);
    Serial.println();
  }

}







