#ifndef __WIFI_H__
#define __WIFI_H__

#include <stdint.h>

class WiFi {
public:
    WiFi(const char* ssid, const char* password);

private:
    const char* ssid;
    const char* password;
};

#endif
