#include "serial.hpp"

bool serial::init(const std::string& port, unsigned int baudrate)
{
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
 	if (fd < 0) {
		return false;
 	}

	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if(tcgetattr(fd, &tty) != 0)
	{
		return false;
	}

	cfsetospeed(&tty, baudrate);
	cfsetispeed(&tty, baudrate);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	//8bit
	tty.c_iflag &= ~IGNBRK;	//Set break condition not to be ignored
	tty.c_iflag &= ~ICRNL;	//Disable convert CR to LF
	tty.c_lflag = 0;		//Disable canonical mode, echo,signal processing
	tty.c_oflag = 0;		//Output without any
	tty.c_cc[VMIN]  = 1;	//minimum number of characters to read
	tty.c_cc[VTIME] = 1;	//read timeout 0.1s
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);	//Disable XOM/XOFF
	tty.c_cflag |= (CLOCAL | CREAD);		//Enable local connetion and reception
	tty.c_cflag &= ~(PARENB | PARODD);		//Disable parity bits
	tty.c_cflag &= ~CSTOPB;					//Stopbit 1
	tty.c_cflag &= ~CRTSCTS;				//Disable RTC/CTS

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		return false;
	}
	return true;
}

void serial::write(std::string word)
{
	::write(fd, word.c_str(), word.size());
}

void serial::write(uint8_t *wBuf, uint8_t size)
{
	::write(fd, wBuf, size);
}

std::string serial::read()
{
	char buf[256];
	memset(buf, 0, sizeof(buf));
	::read(fd, buf, sizeof(buf));
	return std::string(buf);
}

ssize_t serial::read(uint8_t *rBuf, uint8_t size)
{
	memset(rBuf, 0, size);
	return ::read(fd, rBuf, size);
}

void serial::clearReadBuf(){
	tcflush(fd, TCIFLUSH);
}

//void serial::bind(){}

serial::~serial()
{
	close(fd);
}

