//This file was generated from (Commercial) UPPAAL 4.0.14 (rev. 5615), May 2014

/*
When the controller is paused, the conveyor will pause eventually.
*/
aController.Paused --> aConveyor.Paused

/*

*/
E<> aController.Paused

/*

*/
A[] !deadlock
