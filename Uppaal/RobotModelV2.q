//This file was generated from (Commercial) UPPAAL 4.0.14 (rev. 5615), May 2014

/*
A block sensor at pad one leads to input robt move block
*/
A[] !(aConveyorSensors.endSensor == 1 && aConveyor.On)
