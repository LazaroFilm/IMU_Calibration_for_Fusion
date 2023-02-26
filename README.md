# IMU calibration for Fusion

## Let's start a conversation about calibration of LSMxx IMU modules to be used with [Fusion](https://github.com/xioTechnologies/Fusion)

> First I'll note that the LSM9... sensor suffers from nasty random spikes in the gyro that were skewing my tests. I have moved to the Nano RP2040 Connect which has a LSM6DSOX sensor (no compas) which has less gyro noise and no spikes. My project doesnt require compas.

In the Fusion library, options to enter calibration values are present but with no documentation on calibration methods. Notably the misalignment matrix. 

The calibration parameters for each the gyroscope and accelerometer are:
- Offset: aligns the resting position to zero *(default:`FUSION_VECTOR_ZERO`)* `raw - offset`
- Scale/sensitivity *(default:`FUSION_VECTOR_ONES`)* `raw * sensitivity`
- Misalignment matrix *(default: 3x3 matrix as follows:)* ``

```
{
  1.0f, 0.0f, 0.0f,  //  xx   xy  xz
  0.0f, 1.0f, 0.0f,  //  yx   yy  yz
  0.0f, 0.0f, 1.0f   //  zx   zy  zz
}
```

Here is the function used in `FusionCalibrations.h` line 26: 
``` 
static inline FusionVector FusionCalibrationInertial(const FusionVector uncalibrated, const FusionMatrix misalignment, const FusionVector sensitivity, const FusionVector offset) {
    return FusionMatrixMultiplyVector(misalignment, FusionVectorHadamardProduct(FusionVectorSubtract(uncalibrated, offset), sensitivity));
};
```
In FusionMath:
- Offset:
```c
static inline FusionVector FusionVectorSubtract(const FusionVector vectorA, const FusionVector vectorB) {
    const FusionVector result = {.axis = {
            .x = vectorA.axis.x - vectorB.axis.x,
            .y = vectorA.axis.y - vectorB.axis.y,
            .z = vectorA.axis.z - vectorB.axis.z,
    }};
    return result;
}
```
- Sensitivity:
```c
static inline FusionVector FusionVectorHadamardProduct(const FusionVector vectorA, const FusionVector vectorB) {
    const FusionVector result = {.axis = {
            .x = vectorA.axis.x * vectorB.axis.x,
            .y = vectorA.axis.y * vectorB.axis.y,
            .z = vectorA.axis.z * vectorB.axis.z,
    }};
    return result;
}
```
- Matrix:
```c
static inline FusionVector FusionMatrixMultiplyVector(const FusionMatrix matrix, const FusionVector vector) {
#define R matrix.element
    const FusionVector result = {.axis = {
            .x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z,
            .y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z,
            .z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z,
    }};
    return result;
#undef R
}
```


---

## What I discovered *(the hard way)*:

### Offset
Values are passed first before rescaling the RAW IMU inputs, that means that, reading the Fusion output of accelerometer.axis.x and so on will not allow for calculation of offset unless all other calibrations are set to initial values (zeros or ones, depending). 

### Misalignment matrix
It allows you to change the vector direction of each axis (X, Y, Z). I have found this paper regarding gyro alignment, but it’s a bit past my math capabilities. [MEMS IMU/Gyroscope Alignment | Analog Devices](https://www.analog.com/en/analog-dialogue/articles/mems-imu-gyroscope-alignment.html) I have also found a rotation matrix calculator ([3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/)) that I was able to use to find some sort of value that worked to flatten X and Y when rotating around Z to a certain degree (pun intended) I have built a jig with cinema camera parts that rotates the Nano in a single axis. Rotation is by hand and not extremely precise (I need to build a better jig).

## TO DO: 
- [ ] create a calibration procedures to be executed in a specific order to get proper values for each calibration parameter to gather the proper values.
- [ ] script the calibration in a sketch using serial input to pass to the next test.
- [ ] automate the system with a servo controlled 3 axis gimbal jig.
