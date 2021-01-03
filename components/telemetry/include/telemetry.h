#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#include <esp_http_client.h>

class Telemetry {
public:
    Telemetry();

    void postTelemetry(const char* data);

    static esp_err_t httpEventHandler(esp_http_client_event_t* event);

protected:
    void initConnection();

private:
    esp_http_client_handle_t httpClient;
};

#endif
