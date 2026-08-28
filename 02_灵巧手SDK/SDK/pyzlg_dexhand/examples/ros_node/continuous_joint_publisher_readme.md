## Features of the Continuous Joint Publisher

### Motion Patterns
The publisher supports several motion patterns:

1. **Sine Wave** (default): Smooth oscillating motion
2. **Triangle Wave**: Linear transitions between min and max positions
3. **Square Wave**: Immediate transitions between min and max positions
4. **Hold**: Fixed position at maximum amplitude

### Customization
You can customize the behavior with command-line arguments:

```bash
python continuous_joint_publisher.py --pattern sine --amplitude 30 --period 5 --offset 0.2
```

- `--pattern`: Motion pattern (sine, triangle, square, hold)
- `--amplitude`: Maximum joint angle in degrees
- `--period`: Time for one complete motion cycle in seconds
- `--offset`: Phase offset between fingers in radians (creates wave effects)

### Smart Scaling
The publisher applies intelligent scaling based on joint type:

- Metacarpophalangeal joints (joint*_2): 100% of specified amplitude
- Proximal interphalangeal joints (joint*_3): 70% of amplitude
- Distal interphalangeal joints (joint*_4): 50% of amplitude

This creates more natural-looking finger motions that respect the anatomical constraints of real fingers.

### Automatic Reset
The publisher:
- Calls the reset service before starting
- Resets hands when shutting down (Ctrl+C)

## Usage Examples

### Basic Sine Wave
```bash
python continuous_joint_publisher.py
```

### Slow Triangle Wave (10-second period)
```bash
python continuous_joint_publisher.py --pattern triangle --period 10
```

### Larger Amplitude Motion (60 degrees)
```bash
python continuous_joint_publisher.py --amplitude 60
```

### Mexican Wave Effect (increased finger-to-finger offset)
```bash
python continuous_joint_publisher.py --offset 0.6
```

### Hold at Maximum Position
```bash
python continuous_joint_publisher.py --pattern hold
```

This continuous publisher is perfect for testing the DexHand's full range of motion, validating the feedback system, and demonstrating capabilities. It ensures all joints are exercised at the proper rate specified in your configuration.
