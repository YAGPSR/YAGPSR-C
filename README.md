# YAGPSR-C
Soft-GPS in C++ (basic functionality)


## Description

Basic soft GPS implementation based on 

	- "Global Positioning System Standard Positioning Service Signal Specification", 2nd Ed., June 2, 1995 
	
	- "Understanding GPS: Principles and Applications, Second Edition", 2005, Elliott D. Kaplan, Christopher J. Hegarty
	
This implmentation was coded entirely based on written descriptions, and no reference to any existing soft GPS code was made during this implementation.

This implementation only covers basic GPS, and is neither well tested nor optimized. This was a personal project to understand the fundamentals of GPS. It is slow and not suitable for use.

If you are interested in soft GPS, I recommend looking instead at the "GNSS-SDR" Open Source Project.


## Requirements

1. Windows or Linux

	- Tested on Windows 10 and 11
	- Tested on Debian 11, running on WSL 2 on Windows 11

2. C++ compiler (with pthread support)

	- Tested with Microsoft Visual Studio 2017/2022 on Windows,
	- Tested with clang version 11.0.1-2 (x86_64-pc-linux-gnu) on Debian 11

3. FFTW

	- Tested with FFTW v3.3.5 on Windows 
	- Tested with FFTW v3.3.8-2 on Linux

4. (When using git) Git and Git LFS

	- Git LFS is required to support the large binary of test data

## Building

Obtain the codebase from GitHub using `git clone`, or by downloading from GitHub. Note that Git LFS must be installed for the test data to be included.

### Linux

With multithreading (recommended):

`make`

Without multithreading (slower, only if you experience problems with pthread):

`make nomultithread`


### Windows

	- Create an empty Visual Studio Project, add all *.cpp and *.h files are source and headers respectively.
	- Install and add FFTW to the project (header, library, dll).
	- Build.


### WSL on Windows

1. Install a linux distribution on WSL (this example is for Debian)

	  - Ensure Virtualization support is enabled for your CPU in your BIOS (called SVM on AMD)

	  - Open Powershell (Run as administrator)

	  - At the shell, install Debian

	      `wsl --install -d Debian`

	    - You can also install a different distrobution, but these instructions assume Debian:
	      
	      `wsl --list --online`
	  
	      `wsl --install -d <Distribution Name>`
 
	- Reboot the PC.
		- On restart, the install will be completed and a username/password will be set - if there is an error, check that virtualization is enabled.

2. Ensure that all the required packages for development are installed

	On Debian 11 (WSL):

	`sudo apt install clang`

	`sudo apt install make`

	`sudo apt install libfftw3-dev`

	`sudo apt install git`
	
	`sudo apt install git-lfs`


3. Place the codebase into a directory of your choice (using `git clone` for example), and while in the directory containing "Makefile"

	`make`


## Testing

Once the code has been successfully compiled use the command

`./run_yagpsr_test.sh`

which will execute with appropriate command-line paramters on `TestGPS.bin` writing out the results in `TestGPS_Lat_Lon.kml`. File size limitations mean that this is the only binary file included, and only covers a small time window of samples.


# How to use

When called with no command line arguments, usage information is echoed to the command line. In normal use, default values are used for values which are not specified. 

The command line arguments for `yagpsr` are as follows:

`yagpsr -f<infilename> -DopplerSwing <value> -DopplerStep <value> -DetectionThreshold <value> -CorrelationWindow <value> -OSR <value>`

**-f\<infilename\>**

	The filename of the input data stream. 
	Note that there is NO SPACE between -f and the filename. 
	Specifying this as stdin takes input from the console.

**-DopplerLow \<value\>**

	The beginning of the range of Doppler offset frequencies used when searching for each SV. 
	Overwrites the DopplerLow value where placed later on the command line than DopplerSwing.
	Integer only, specified in Hz.

**-DopplerHigh \<value\>**

	The end of the range of Doppler offset frequencies used when searching for each SV. 
	Integer only, specified in Hz.

**-DopplerSwing \<value\>**

	The range of Doppler offset frequencies about the centre frequency (+/-DopplerSwing) used when searching for each SV. 
	Overrides all earlier DopplerLow/DopplerHigh settings on the command line. 
	Positive integer only, specified in Hz.

**-DopplerStep \<value\>**

	The frequency spacing between consecutive Doppler offset frequencies used when searching for each SV. 
	If not an integer divisor of (2 X DopplerSwing) then the maximum Doppler offset is reduced accordingly. 
	Integer only, specified in Hz.

**-DetectionThreshold \<value\>**

	The threshold value to use in detection of SVs. 
	Positive integer only.

**-CorrelationWindow \<value\>**

	The length of time over which to correlate in SV detection in ms. 
	Positive integer only.

**-CorrelationMode \<value\>**

	The mode in which to operate the correlation used in the initial acquisition satellites. 
	Valid values: MultiPhase, FFT, Full.

**-OSR \<value\>**

	The oversampling ratio at which to process data (during the tracking phase, but also used to simplify data selection during acquisition). 
	Positive integer multiple of 4 only.

**-SleepTime \<value\>**

	The length of time to wait after failing to acquire a satellite before trying again. 
	Positive integer only, specified in ms.


## Data Format

The input data is binary data stored in a `.bin` file. There is also an associated `.dat` file containing information about the data in the `.bin`.

Example contents of `.dat`:

	4000000			% FsData, the sampling rate of the input, in Hz
	
	7500			% Fc, the centre frequency of the input, in Hz
	
	4			% The default/suggested oversampling rate at which to perform acquisition (may be overwritten at the command line)
	
	DT_16_BIT_COMPLEX	% Data format in the input



# FAQs

### What is it and what is it not?

It is a basic implementation of GPS receiver functionality, written for my own edification as a personal project over a number of weekends. It is not a complete GPS receiver and is not suitable for general use.

### What does it do and not do?

It does:
* Read an IF signal stored as a binary stream in a file
* Shift and process as near-zero-IF
* Search for SVs from 1 to 31
* Detect Doppler offsets
* Receive data packets from each SV detected
* Read ephemeris data and predict SV locations at specific time instants
* Use either direct correlation (slow) or FFT-based correlation (much better for software)
* Solve the GPS equations using Newton's method
* Track the code phase using early/late correlation outputs
* Track the carrier using a PLL
* Return a set of location fixes in a .kml file that may be read by Google Earth (or other)
* Echo information to command line as it is running

It does not:
* have any a priori information about the satellites
* use almanac/ephemeris information to intelligently look for satellites or predict Doppler before starting
* detect phase slipping
* give a continually updating location
* have any optimization
* have any analysis of performance
* have any guarantees


### Why did you stop working on it, when it's clearly not finished?

Real life and lack of test data.
This was a something was worked on in my spare time, and eventually real-life took over with other priorities. Also, testing GPS requires a lot of test data, and since I had no way of capturing such data to IF, this is based only on what was available online at the time it was written.

