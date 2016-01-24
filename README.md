# SDP-milestone-one-tester
Code for testing the communications task of SDP milestone 1

```randomFileGenerator <file size(bytes)>```<br />
generates a file of random bytes with the specified size, for testing

The arduino code just forwards data received to the serial port

```SDP-milestone-one-tester <port ("/dev/ttyACM0")> <filename ("randomFile.bin")> ```<br />
listens for incoming data and compares it to the input file

