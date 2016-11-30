
int range (int sensorval) {
    if (sensorval > 250) {
        return 0;
    }
    else if (sensorval > 150) {
        return 1;
    }
    else if (sensorval > 70) {
        return 2;
    }
    else if (sensorval > 45) {
        return 3;
    }
    else if (sensorval > 30) {
        return 4;
    }
    else {
        return 5;
    }
}

int angle (int left, int center, int right) {
    int rrange = range (right);
    int lrange = range (left);
    int crange = range (center);
    if (rrange >= 3 && lrange >= 3 && crange >= 3) {
        
    }
    if (lrange < 2 && rrange < 2 && crange < 2) {
        if (lrange > 0 && rrange == 0 && crange == 0) {
            return -45;
        }
        else if (lrange == 0 && rrange > 0 && crange == 0) {
            return 30;
        }
        else if (lrange == 0 && rrange == 0 && crange > 0) {
            return 0;
        }
        else if (lrange == 0 && rrange > 0 && crange > 0) {
            return 30;
        }
        else if (lrange > 0 && rrange == 0 && crange > 0) {
            return -30;
        }
    }
}