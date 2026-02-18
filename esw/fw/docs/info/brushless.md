# Brushless DC Motors

Brushless DC motors (BLDC) are electric motors that operate without the use of brushes for commutation.
They provide higher efficiency and better performance compared to brushed motors, making them ideal for applications
requiring precision and power. They are, however, more complex to control, requiring us to use specialized controllers.

BLDC motors are used troughout the rover, such as:

- All drive wheels
- Joint A
- Joint C
- Joint DE

## Moteus

The BLDC motors are controlled using [moteus r4.11](https://mjbots.com/products/moteus-r4-11) and
[moteus n1](https://mjbots.com/products/moteus-n1) controllers. The r4.11 controller is a bit simpler, but it is
cheaper, so we use them for the drive motors. The n1 controller features more I/O, so we use them for the arm joints.

These controllers are designed by mjbots (Josh Pieper) and are fully open source. Here are some useful links:

- [moteus GitHub repository](https://github.com/mjbots/moteus)
- [moteus documentation](https://mjbots.github.io/moteus)

The moteus Discord server invite can be found in the documentation above, and is a **_great_** place to ask questions;
Josh Pieper is very active and responsive there.
