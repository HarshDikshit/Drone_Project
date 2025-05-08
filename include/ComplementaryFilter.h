#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

class ComplementaryFilter {
public:
    ComplementaryFilter(float sampleFreq, float alphaVal);
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    void getEulerAngles(float& rollOut, float& pitchOut, float& yawOut);
    void resetAngles(float ax, float ay, float az); // Declared resetAngles
private:
    bool isValidData(float ax, float ay, float az, float gx, float gy, float gz);
    float roll, pitch, yaw;
    float dt;
    float alpha;
    bool isInitialized;
};

#endif