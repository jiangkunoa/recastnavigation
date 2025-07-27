#pragma once
#include <string>
#include <string.h>
#include <list>
#include <vector>
#include <set>
#include <cassert>
#include <algorithm>
#include <map>
#include <iostream>
#include <fstream>
#include <stdarg.h>
#include <time.h>
#include <stdio.h>
#include <functional>
#include <unordered_map>
#include <memory>
#include <queue>
#include <deque>
#include <assert.h>
#include <limits>
#include <cmath>
#include <errno.h>

#if defined(WIN32) || defined(WIN64)
#define PLATFORM_WINDOWS
#elif defined(LINUX32) || defined(LINUX64)
#define PLATFORM_LINUX
#endif

#ifdef _WIN32
	#include <io.h>      // Windows
	#include <direct.h>   // Windows
	#include <process.h>
	#include <ws2tcpip.h>
	#include <Windows.h>
	#include <mswsock.h>
	#include "Mstcpip.h"
	#include <time.h>
	#include <stdarg.h>
#else
	#include <unistd.h>  // Linux/macOS
	#include <sys/stat.h> // Linux/macOS
	#include <pthread.h>  // Linux/macOS
#endif
#ifdef PLATFORM_WINDOWS


#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <string.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <dirent.h>
#include <netdb.h>
#endif



