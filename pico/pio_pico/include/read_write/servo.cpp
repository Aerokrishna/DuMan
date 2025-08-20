// Function to move servos to target angles
bool setServoAngles(int targetAngles[NUM_JOINTS]) {

    bool allReached = true;
    static unsigned long prev_millis = 0;

    unsigned long current_millis = millis();

    if (current_millis - prev_millis > 20){
        for (int i = 0; i < NUM_JOINTS; i++) {

            if (currentAngles[i] < targetAngles[i]) {
            currentAngles[i]++;
            allReached = false;
            }

            else if (currentAngles[i] > targetAngles[i]) {
            currentAngles[i]--;
            allReached = false;
            }
            servos[i].write(currentAngles[i]);

        }
        return allReached;
    }
    return false;
}
