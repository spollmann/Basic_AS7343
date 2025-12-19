/* Flash an LED on a arduino nano 33 ble sense rev2 (nrf chip).  Rate can be set using serial input. */
/* This code only tested on an nordic chip */

#define LED_PIN_BIT (1 << 13) 
#define EXTERNAL_LED_PIN_BIT (1 << 14)

// Use a variable for frequency so it can be changed
uint32_t currentFrequency_Hz = 2000;
uint32_t computedMicroDelay = 0;

// Counters
uint32_t onCounter = 0;
uint32_t elapsedMillis = 0;
uint32_t elapsedMicros = 0;
bool turnLedOn = true;

// NEW: Flag to control the "always on" state
bool ledAlwaysOn = false;

void setup() {
  Serial.begin(115200);
  
  pinMode(6, OUTPUT);

  // Calculate the initial delay
  computedMicroDelay = 1000000 / (2 * currentFrequency_Hz);
  
  Serial.println("Starting High Speed Toggle...");
  Serial.print("Initial Frequency: ");
  Serial.print(currentFrequency_Hz);
  Serial.println(" Hz");
  Serial.println("Enter a new frequency (e.g., 500) or 0 for 'Always ON' and press Enter:");
  
  elapsedMillis = millis();
  elapsedMicros = micros();
}

/**
 * @brief Checks for new serial input and updates the frequency.
 * This is non-blocking and safe to call in a fast loop.
 */
void checkSerialInput() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim(); // Remove any whitespace

    if (inputString.length() > 0) {
      long newFreq = inputString.toInt(); // Use long to parse
      
      // Validate the input
      if (newFreq > 0) {
        // --- STANDARD TOGGLING ---
        ledAlwaysOn = false; // We are now toggling
        currentFrequency_Hz = (uint32_t)newFreq;
        
        computedMicroDelay = 1000000 / (2 * currentFrequency_Hz);

        if (computedMicroDelay == 0) {
          computedMicroDelay = 1; 
        }

        Serial.print("--> Set new frequency: ");
        Serial.print(currentFrequency_Hz);
        Serial.print(" Hz (New Delay: ");
        Serial.print(computedMicroDelay);
        Serial.println(" us)");
        
      } else if (newFreq == 0) {
        // --- NEW: ALWAYS ON STATE ---
        ledAlwaysOn = true; // Stop toggling
        currentFrequency_Hz = 0;
        
        // Manually turn the LED ON
        NRF_P1->OUTSET = EXTERNAL_LED_PIN_BIT;
        turnLedOn = false; // Set state so if we resume, it starts with OFF
        onCounter = 0;     // Reset counter
        
        Serial.println("--> Set new frequency: 0 Hz (LED Always ON)");
        
      } else {
        Serial.println("Invalid input. Frequency must be a positive number or 0.");
      }
    }
  }
}

void loop() {
  while(1){
    // Always check for serial commands
    checkSerialInput();

    // NEW: If in "always on" mode, skip all timing logic
    if (ledAlwaysOn) {
      continue; // Go back to the start of while(1)
    }

    // --- This code only runs if ledAlwaysOn is false ---

    // LOGGING
    if (millis() - elapsedMillis > 10000){
      Serial.print("Toggles in last 10s: ");
      Serial.println(onCounter);
      Serial.print("Avg Hz: ");
      Serial.println((float)onCounter / 10);
      onCounter = 0;
      elapsedMillis = millis();
    }

    // TIMING
    uint32_t currentMicros = micros();
    
    if (currentMicros - elapsedMicros >= computedMicroDelay){
      
      if (turnLedOn){
        NRF_P1->OUTSET = EXTERNAL_LED_PIN_BIT;
        onCounter++;
        turnLedOn = false;
      }
      else{
        NRF_P1->OUTCLR = EXTERNAL_LED_PIN_BIT;
        turnLedOn = true;
      }
      
      elapsedMicros += computedMicroDelay; 
    }
  }
}