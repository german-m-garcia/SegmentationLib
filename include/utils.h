/*
 * utils.h
 *
 *  Created on: 5 Nov 2015
 *      Author: martin
 */

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include <chrono>
#include <string>
#include <ctime>
#include <ratio>
#include <iostream>

using namespace std;
using namespace std::chrono;

class Utils {
public:
	Utils() {
	}
	virtual ~Utils() {
	}

	void tick() {
		t0 = high_resolution_clock::now();
	}

	void tock(string& text) {
		tf = std::chrono::high_resolution_clock::now();
		duration<double> time_span = duration_cast<duration<double>>(tf - t0);

		cout << text << " took: " << time_span.count() * 1000 << " ms" << endl;
	}
private:
	high_resolution_clock::time_point t0, tf;
};

#endif /* INCLUDE_UTILS_H_ */
