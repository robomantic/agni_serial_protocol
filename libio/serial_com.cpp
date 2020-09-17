/* ============================================================
 * Modified by Guillaume Walck 2019
 * 
 * Copyright (C) 2015 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the "LGPL"),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CITEC, "Cognitive Interaction Technology" Excellence Cluster
 *     Bielefeld University
 *
 * ============================================================ */
#include "serial_com.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>



using namespace std;

namespace serial_protocol {


const char *SerialCom::timeout_error::what() const throw()
{
	return "serial communication timed out";
}


SerialCom::SerialCom() :
 connected(false), verbose(false)
{
	setTimeOut(100);
}
SerialCom::~SerialCom() {
	disconnect();
}

void SerialCom::setTimeOut(unsigned int msec)
{
	if (msec >= 1000)
	{
		timeout.tv_sec = msec/1000;
		unsigned int remsec = 
		timeout.tv_nsec = (msec%1000) * 1e6;
	}
	else
	{
		timeout.tv_sec = 0;
		timeout.tv_nsec = msec * 1e6;
	}
}

void SerialCom::connect(const std::string &sDevice)
{
	if (connected) return;
	if ((fd = open (sDevice.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
		throw std::runtime_error(string("sc: Connection failed: ") + strerror(errno));

	tcgetattr(fd,&oldtio); /* save current port settings */
	bzero(&newtio, sizeof(newtio));

	cfsetospeed (&newtio, B115200);
	cfsetispeed (&newtio, B115200);

	// set up raw mode / no echo / binary
	newtio.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
	newtio.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN); //|ECHOPRT

	newtio.c_oflag &= (tcflag_t) ~(OPOST);
	newtio.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
	newtio.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
	newtio.c_iflag &= (tcflag_t) ~PARMRK;
#endif

	// setup char len = CS8
	newtio.c_cflag |= CS8;
	// setup one stopbit
	newtio.c_cflag &= (tcflag_t) ~(CSTOPB);
	// setup parity: no parity
	newtio.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
	newtio.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
	// setup flow control: none
	newtio.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
	newtio.c_cflag &= (tcflag_t) ~(CRTSCTS);

	newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	newtio.c_cc[VMIN]     = 0;   /* no minimum chars to be read */

	tcsetattr(fd,TCSANOW,&newtio);
	tcflush(fd, TCIFLUSH);

	FD_ZERO (&fdset);
	FD_SET (fd,&fdset);

	connected = true;
  flush();
}

void SerialCom::disconnect()
{
	if (!connected) return;
	connected = false;
	tcsetattr(fd,TCSANOW,&oldtio);
	tcflush(fd, TCIOFLUSH);
	close(fd);
}

size_t SerialCom::readFrame(uint8_t *buf, size_t len)
{
	if (verbose)
		printf("sc: trying to read %lu bytes\n", len);
	if (!connected) throw std::runtime_error("sc: not connected to read");

	//unsigned char buf[PACKET_SIZE_BYTES]; // receive buffer
	if (!buf)
		throw std::runtime_error("sc: unintialized buffer");

	size_t index = 0;
	while(index < len)
	{
		FD_ZERO (&fdset);
		FD_SET (fd,&fdset);
		int res = pselect (fd+1, &fdset, NULL, NULL, &timeout, NULL);
		if (verbose)
			printf("sc: pselect result %d \n", res);
		if (res == -1) throw std::runtime_error(strerror(errno));
		if (res == 0) return index; // timeout
		
		// read a maximum of len bytes into buf (actual read count is in res)
		if (verbose)
			printf("sc: reading %lu bytes\n", len - index);
		res = read(fd, buf + index, len - index);
		if (verbose)
			printf("sc: read result %d \n", res);
		if (res < 0)
			throw std::runtime_error(std::string("sc: serial read error"));
		index += res;
	}
	if (verbose)
		printf("sc: read total of %lu bytes\n", index);
	return index;
}

size_t SerialCom::writeFrame(const uint8_t *buf, size_t len)
{
	if (!connected) throw std::runtime_error("sc: not connected to write");

	if (!buf)
		throw std::runtime_error("sc: unintialized buffer");
	if (verbose)
	{
		printf("sc: writing :");
		for (unsigned int i=0; i< len;i++)
			printf("%x ", buf[i]);
		printf("\n");
	}
	size_t res = write(fd, buf, len);
	if (res < 0)
		throw std::runtime_error(strerror(errno));
		//throw std::runtime_error(std::string("sc: serial write error"));
	if (verbose)
		printf("sc: written %lu bytes\n", res);

	return res;
}

void SerialCom::flush()
{
	unsigned char buf[256];
	size_t read_len;
	try{
		read_len = readFrame(buf, 256); // read possibly incomplete frame
	}
	catch (const std::exception &e) {
		std::cerr << e.what() << std::endl;
	}
	if (verbose)
		printf("sc: flushed %lu chars\n", read_len);
}

}
