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
#pragma once
#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <stdint.h>
#include <exception>
#include <string>


namespace serial_protocol {
class SerialCom
{
public:
	class timeout_error : public std::exception {
	public:
		const char* what() const throw();
	};

	SerialCom();
	~SerialCom();

	void setTimeOut(unsigned int msec);
	void setVerbose(bool verb) { verbose = verb; }
	void connect(const std::string &sDevice);
	void disconnect();
  void flush();
	size_t readFrame (uint8_t *buf, size_t len);
	size_t writeFrame (const uint8_t *buf, size_t len);

private:
	//void sync(unsigned char buf[]) const;

private:
	struct termios oldtio,newtio;
	struct timespec timeout;
	int    fd;
	fd_set fdset;
	bool   connected;
	bool   verbose;
};
}
