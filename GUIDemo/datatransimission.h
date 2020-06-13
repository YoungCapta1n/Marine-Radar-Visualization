#ifndef DATATRANSIMISSION_H
#define DATATRANSIMISSION_H

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>

#include <ctype.h>
#include <stdarg.h>

unsigned long long int pack754(long double f, unsigned bits, unsigned expbits);

long double unpack754(unsigned long long int i, unsigned bits,
                      unsigned expbits);

void packi16(unsigned char *buf, unsigned int i);

void packi32(unsigned char *buf, unsigned long int i);

void packi64(unsigned char *buf, unsigned long long int i);

int unpacki16(unsigned char *buf);

unsigned int unpacku16(unsigned char *buf);

long int unpacki32(unsigned char *buf);

unsigned long int unpacku32(unsigned char *buf);

long long int unpacki64(unsigned char *buf);

unsigned long long int unpacku64(unsigned char *buf);

unsigned int pack(unsigned char *buf, const char *format, ...);

void unpack(unsigned char *buf, const char *format, ...);

class datatransimission {
 public:
  explicit datatransimission(const std::string &_port);
  virtual ~datatransimission() = default;

  void selectserver(unsigned char *recv_buffer,
                    const unsigned char *send_buffer, size_t recv_size,
                    size_t send_size);

  int getsocketresults() const noexcept { return results; }
  int getconnectioncount();

 private:
  fd_set master;    // master file descriptor list
  fd_set read_fds;  // temp file descriptor list for select()
  int fdmax;        // maximum file descriptor number
  int listener;     // listening socket descriptor

  struct sockaddr_storage remoteaddr;  // client address

  char remoteIP[INET6_ADDRSTRLEN];

  std::string port;  // port we're listening on
  int results;
  // get sockaddr, IPv4 or IPv6:
  void *get_in_addr(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
      return &(((struct sockaddr_in *)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }

  void initializesocket();
};

#endif  // DATATRANSIMISSION_H
