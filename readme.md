# Install

copy `xsensdeviceapi-2025.0.0-cp38-none-win_amd64.whl` from the `MT Software Suite 2025.0` folder(in my case: `F:\Program Files\Xsens\MT Software Suite 2025.0\MT SDK\Python\x64`), then

```bash
python -m venv venv
venv\Scripts\activate
pip install xsensdeviceapi-2025.0.0-cp38-none-win_amd64.whl
pip install scipy
```

# Orientation Correction Algorithm for Xsens IMU

## Overview

This document explains the orientation correction algorithm implemented in the provided code, which is designed to work with Xsens MTi devices. The algorithm establishes a reference frame by correcting the raw orientation data from the IMU sensor.

## Purpose

The primary purpose of this algorithm is to allow users to define a custom reference orientation. This is particularly useful when:

- The sensor cannot be physically mounted in the desired orientation
- A specific orientation needs to be defined as "zero" or "forward"
- Consistent alignment is required across multiple recording sessions

## Mathematical Foundation

### Quaternion Representation

The algorithm uses quaternions to represent orientations. A quaternion is a four-dimensional complex number represented as:

$$q = w + xi + yj + zk$$

where $w$ is the scalar part, and $x$, $y$, $z$ form the vector part.

### The Correction Formula

The core of the algorithm applies the following transformation:

$$q_{corrected} = q_{raw} \otimes q_{rotSensor}$$

Where:
- $q_{raw}$ is the quaternion representing the raw orientation from the sensor
- $q_{rotSensor}$ is the correction quaternion computed during initialization
- $\otimes$ represents quaternion multiplication

### Post-Multiplication Significance

The correction quaternion $q_{rotSensor}$ appears on the right side of the multiplication because:

1. Quaternion multiplication is non-commutative: $q_1 \otimes q_2 \neq q_2 \otimes q_1$
2. Post-multiplication (applying $q_{rotSensor}$ on the right) applies the correction in the global reference frame
3. This ordering ensures that the correction is applied after the sensor's orientation is determined

In geometric terms, if we think of quaternions as representing rotations, then $q_{raw} \otimes q_{rotSensor}$ means "first rotate by $q_{raw}$, then rotate by $q_{rotSensor}$."

### Computing the Correction Quaternion

The correction quaternion is elegantly derived as the conjugate of the initial orientation quaternion:

$$q_{rotSensor} = q_{initial}^* = w - xi - yj - zk$$

Since quaternions representing rotations are unit quaternions, the conjugate is equivalent to the inverse. This $q_{rotSensor}$ effectively "undoes" the initial orientation, making it the new zero reference.

## Implementation Details

The algorithm leverages the Xsens Device API (XDA) for quaternion operations with a remarkably simple approach:

```python
def initialize_from_first_frame(self, q_raw):
    # Simply use the conjugate of the initial quaternion as our correction
    self.q_rotSensor = q_raw.conjugate()
    
    # Display the initial orientation for reference
    euler = xda.XsEuler()
    euler.fromQuaternion(q_raw)
    # ... display code ...
```

For each subsequent orientation reading, the correction is applied:

```python
def correct_orientation(self, q_raw):
    if not self.initialized:
        self.initialize_from_first_frame(q_raw)
        return q_raw
    
    # Apply correction: q_raw * q_rotSensor
    q_corrected = xda.XsQuaternion()
    q_corrected.multiply(q_raw, self.q_rotSensor)
    
    return q_corrected
```

This elegant solution avoids unnecessary conversions to Euler angles and back, making it both efficient and robust against singularities that can occur with Euler angle representations.

## Usage

The algorithm is implemented in the `OrientationCorrector` class:

1. Create an instance of the corrector: `corrector = OrientationCorrector()`
2. For each new orientation reading, apply the correction: `corrected = corrector.correct_orientation(raw)`
3. The first call to `correct_orientation()` will automatically initialize the correction

## Applications

This orientation correction is valuable for:

- Motion capture systems where sensors cannot be physically aligned with the desired coordinate system
- Virtual reality applications requiring a calibrated "forward" direction
- Robotics and drone navigation systems where a reference orientation needs to be established
- Biomechanical analysis where anatomical reference frames must be aligned

## Benefits

- **Ease of use**: No need for precise physical alignment of sensors
- **Consistency**: Provides a stable reference frame across recording sessions
- **Flexibility**: Can redefine the reference orientation at any time by reinitializing the corrector
- **Efficiency**: Direct quaternion operations avoid unnecessary conversions and potential singularities

## Implementation Notes

- The algorithm uses the native Xsens Device API for all quaternion operations, ensuring compatibility and optimal performance
- The correction is performed in quaternion space to avoid gimbal lock issues that can occur with Euler angles

## Requirements

- Xsens Device API (XDA) library
- An Xsens MTi device