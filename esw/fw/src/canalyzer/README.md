# CANalyzer

***DEVELOPMENT FOR THIS PROJECT TAKES PLACE ON THE `canalyzer` BRANCH***

CANalyzer will be a custom CAN bus analyzer tool device. The end-goal is to have a fully embedded
device that can be used to monitor and log CAN bus traffic. Its main inspiration is the PEAK System
[diagnostic tool](https://www.peak-system.com/PCAN-Diag-FD.456.0.html?&L=1). Custom hardware will
be designed by EHW in parallel with the software development.

## Project Goals

- **Real-time Monitoring:** Display CAN bus messages in a structured and easy-to-read format
- **Custom CAN Message Handling:** Implement support for our team's custom CAN message format
- **Filter and Search:** Implement filters to allow users to focus on specific CAN IDs or data patterns
- **Logging:** Enable saving of CAN bus data to a file for later analysis
