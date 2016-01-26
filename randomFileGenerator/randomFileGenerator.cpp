#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <random>
#include <chrono>

using namespace std;

int main(int argc, char *argv[]){
  if (argc != 2){
    cout<<"usage: " << argv[0] << " <file size (bytes)>\n";
    return -1;
  }
  int filesize = atoi(argv[1]);

  unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();

  std::default_random_engine generator(seed1);
	std::uniform_int_distribution<int> distribution(0,255);

	fstream fs;
	fs.open("randomFile.bin", fstream::out | fstream::trunc);

	for (int i=0; i<filesize; i++)
		fs << (uint8_t) distribution(generator);;

	fs.close();


	return 0;
}
