#include "utils.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <vector>
#include <math.h>
#include <deque>
#include <fstream>
#include <iostream>

#define RECV_TIMEOUT	2.0  		// timeout from last i2c receive until end of trial

using namespace std;

// taken from http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
	return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

vector<float> diff(vector<float> numbers){
	vector<float> diffs;
	for (unsigned int ni = 1; ni < numbers.size(); ni++)
		diffs.push_back(numbers[ni] - numbers[ni-1]);
	return diffs;
}

float vectorMean(vector<float> numbers){
	float total = 0;
	for (vector<float>::iterator ni = numbers.begin(); ni != numbers.end(); ni++){
		total += *ni;
		//printf("%.3f ", *ni);
	}
	return total/numbers.size();
}

float vectorSD(vector<float> numbers){
	float mean = vectorMean(numbers);
	float sumSqDev = 0;
	for (vector<float>::iterator ni = numbers.begin(); ni != numbers.end(); ni++)
		sumSqDev += (*ni - mean)*(*ni - mean);
	return sqrt(sumSqDev / numbers.size());
}


int main(int argc, char *argv[]){
	vector<uint8_t> received;
	vector<float>	tReceived;
	vector<uint8_t> expected, matched;
	float tReceiveStart = gettime(), tLastRecv;
	bool receiving = false;
	printf("Beginning Receiver Program\n\n");

	char const *portname;
	if (argc < 2){
		portname = "/dev/ttyACM0";	
	} else {
		portname = argv[1];
	}

	char const *inputfilename;
	if (argc < 3){
		inputfilename = "randomFile.bin";
	} else {
		inputfilename = argv[2];
	}
	

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0){
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        return 0;
    }

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    setbuf(stdout, NULL);			// turn off buffering to stdout, so data appears in real time

    // demo of expected data. Replace with file read.
    for (int i=0; i<50; i++){
    	expected.push_back((uint8_t)i);
    }


    // get expected data from file
    ifstream inputFile(inputfilename, ifstream::in);
    if (!inputFile.is_open()){
    	cout << "Error, could not open file" << inputfilename << "\n";
    	return -1;
    }
    uint8_t ebyte = inputFile.get();

    {
    	int inFileCounter = 0;
	    printf("Input file contents: \n");
	    while (inputFile.good()){
	    	printf("%2X ", ebyte);
	    	//expected.push_back(ebyte);
	    	ebyte = inputFile.get();
	    	inFileCounter++;
	    	if (!(inFileCounter%20)) printf("\n");
	    }
	    inputFile.close();
	    printf("\n");
	}

    // main read loop
    // interprets multiple bytes received as ASCII data to be displayed
    // single byte as a received byte

    char buf [100];
    while (1){
        int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
        if (n > 1){
        	for (int i=0; i<n; i++)
        		printf("%c", buf[i]);
        } else if (n ==1){
        	if (!receiving){
        		receiving = true;
        		tReceiveStart = gettime();
        		printf("Receiving Data:\n");
        	}
        	printf("%02X ", (uint8_t)buf[0]);
        	received.push_back(buf[0]);
        	tReceived.push_back(gettime() - tReceiveStart);
        	tLastRecv = gettime();
        	if (!(tReceived.size()%20)) printf("\n");
        } else if (receiving){
        	if (gettime() - tLastRecv > RECV_TIMEOUT){
        		receiving = false;
        		printf("\nReceiving Finished\n");
        		printf("Total Time: %.3f\n", tReceived.back());
        		printf("Expected bytes: %d\t Received bytes: %d (%.1f %%)\n", (int)expected.size(), (int)received.size(), ((float)received.size()/(float)expected.size())*100);
        		printf("Time/expected bytes: %.3f\n", tReceived.back()/max((int)expected.size()-1, 1));
        		vector<float> tIntervals = diff(tReceived);
        		printf("Mean period: %.3f\n", vectorMean(tIntervals));
        		printf("Standard Deviation: %.3f\n", vectorSD(tIntervals));

        		// do a quick n' dirty match between the expected and the received data
        		// can cope with gaps and noise in the data
        		// this algorithm is kind, i.e. forgives errors in byte order. 
        		// It will probably overestimate by a couple of percent on a really noisy data set
        		deque<uint8_t> dReceived(received.begin(), received.end());
        		int rindex = 0;
        		for (vector<uint8_t>::iterator di = expected.begin(); di != expected.end(); di++){
        			for (int ri = max(rindex-5,0); ri<rindex+5 && ri<(int)dReceived.size(); ri++){
        				if (dReceived[ri] == *di){
        					matched.push_back(*di);
        					rindex = max(rindex,ri);
        					dReceived.erase(dReceived.begin()+ri);		// delete byte to prevent multiple matching
        					//dReceived.erase(dReceived.begin(), dReceived.begin()+ri);	// this disnae work. throws away good data
        					break;
        				}
        			}
        		}
        		printf("Matched bytes: %d (%.1f%%)\n", (int)matched.size(), ((float)matched.size()/(float)expected.size())*100);

        		//for (vector<uint8_t>::iterator mi = matched.begin(); mi != matched.end(); mi++)
        		//	printf("%02X ", *mi);

        		// print out the expected and matched data for visual comparison
        		// this ignores received data that was not successfully matched
        		vector<uint8_t>::iterator mi = matched.begin();
        		for (unsigned int di=0; di<expected.size(); di++){
        			printf("%02X ", expected[di]);
        			if ((di && (di+1)%20 == 0) || di == expected.size()-1){
        				printf("\n");
        				int ri = 19;
        				if (di == expected.size()-1) ri = di%20;
        				for (; ri>=0; ri--){
        					//printf("di: %d\t ri: %d\n", di, ri);
        					if (mi != matched.end() && expected[di-ri] == *mi){
        						printf("%02X ", *mi);
        						mi++;
        					} else {
        						printf("   ");
        					}
        				}
        				printf("\n\n");
        			}
        		}

        		// reset ready for next data set.
        		received.clear();
        		tReceived.clear();
        		matched.clear();
        		dReceived.clear();
        		printf("Ready for new data\n");
        		
        	}
        }
    }
}